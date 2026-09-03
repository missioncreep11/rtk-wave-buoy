#include "OpenLog_Artemis.h"
// OpenLog_Artemis_GNSS_Logging Modified

// Global variable definitions
TwoWire qwiic(PIN_QWIIC_SDA, PIN_QWIIC_SCL);
ICM_20948_SPI myICM;
SFE_UBLOX_GNSS gpsSensor_ublox;
Apollo3RTC myRTC;

#if SD_FAT_TYPE == 1
SdFat32 sd;
File32 gnssDataFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile gnssDataFile;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile gnssDataFile;
FsFile imuDataFile;
#else
SdFat sd;
File gnssDataFile;
File imuDataFile;
#endif

char gnssDataFileName[30] = "";
char imuDataFileName[30] = "";
const int sdPowerDownDelay = 100;

struct_settings settings;
struct_online online;
struct_QwiicSensors qwiicAvailable = { .uBlox = false };
struct_QwiicSensors qwiicOnline = { .uBlox = false };

uint64_t measurementStartTime = 0;
unsigned long lastReadTime = 0;
unsigned long lastDataLogSyncTime = 0;
const byte menuTimeout = 15;
bool rtcHasBeenSyncd = false;
bool rtcNeedsSync = true;
bool gnssSettingsChanged = false;
volatile bool stopLoggingSeen = false;
int lowBatteryReadings = 0;
const int lowBatteryReadingsLimit = 1000;
bool ignorePowerLossInterrupt = true;
unsigned long lastIMUReadTime = 0;

void setup() {
  //If 3.3V rail drops below 3V, system will power down and maintain RTC
  pinMode(PIN_POWER_LOSS, INPUT); // BD49K30G-TL has CMOS output and does not need a pull-up

  delay(1); // Let PIN_POWER_LOSS stabilize

  if (digitalRead(PIN_POWER_LOSS) == LOW) powerDown(); //Check PIN_POWER_LOSS just in case we missed the falling edge
  ignorePowerLossInterrupt = true; // Ignore the power loss interrupt - when attaching the interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_POWER_LOSS), powerDown, FALLING);
  ignorePowerLossInterrupt = false;

  powerLEDOn(); // Turn the power LED on - if the hardware supports it
  
  pinMode(PIN_STAT_LED, OUTPUT);
  digitalWrite(PIN_STAT_LED, HIGH); // Turn the STAT LED on while we configure everything

  if (PIN_LOGIC_DEBUG >= 0)
  {
    pinMode(PIN_LOGIC_DEBUG, OUTPUT); //Debug pin
    digitalWrite(PIN_LOGIC_DEBUG, HIGH); //Make this high, trigger debug on falling edge
  }

  Serial.begin(115200); //Default for initial debug messages if necessary
  // Don't wait for Serial connection - start logging immediately
  Serial.setTimeout(100); // Short timeout for any serial operations
  Serial.println();

  EEPROM.init();

  SPI.begin(); //Needed if SD is disabled

  beginSD(); //285 - 293ms
  
  loadSettings(); //50 - 250ms

  Serial.flush(); //Complete any previous prints
  Serial.begin(settings.serialTerminalBaudRate);
  Serial.printf("Artemis OpenLog GNSS v%d.%d\r\n", FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR);
  Serial.flush();

  beginSerialOutput(); // If required, output data on the TX pin. Do this after loadSettings and before beginQwiic

  if (settings.useGPIO32ForStopLogging == true)
  {
    Serial.println("Stop Logging is enabled. Pull GPIO pin 32 to GND to stop logging.");
    pinMode(PIN_STOP_LOGGING, INPUT_PULLUP);
    delay(1); // Let the pin stabilize
    attachInterrupt(digitalPinToInterrupt(PIN_STOP_LOGGING), stopLoggingISR, FALLING); // Enable the interrupt
    pinMode(PIN_STOP_LOGGING, INPUT_PULLUP); //Re-attach the pull-up (bug in v2.1.0 of the core)
    stopLoggingSeen = false; // Make sure the flag is clear (attachInterrupt will trigger an immediate interrupt with v2.1.0 of the core)
  }

  beginQwiic();

  for (int i = 0; i < 250; i++) // Allow extra time for the qwiic sensors to power up
  {
    checkBattery(); // Check for low battery
    delay(1);
  }

  analogReadResolution(14); //Increase from default of 10

  beginDataLogging(); //180ms

  // disableIMU(); //Disable IMU
  beginIMU(); // instead of disable IMU - Amara

  if (online.microSD == true) Serial.println("SD card online");
  else Serial.println("SD card offline");

  if (online.dataLogging == true) Serial.println("Data logging online");
  else Serial.println("Datalogging offline");

  if (settings.enableTerminalOutput == false && settings.logData == true) Serial.println(F("Logging to microSD card with no terminal output"));

  if ((online.microSD == false) || (online.dataLogging == false))
  {
    // If we're not using the SD card, everything will have happened much qwicker than usual.
    // Allow extra time for the u-blox module to start. It seems to need 1sec total.
    for (int i = 0; i < 750; i++)
    {
      checkBattery(); // Check for low battery
      delay(1);
    }
  }

  if (beginSensors() == true) Serial.println(F("GNSS online"));
  else
  {
    Serial.println(F("GNSS offline"));
    // No GPS available — set RTC from compile time so IMU logs have approximate UTC timestamps
    setRTCFromCompileTime();
  }

  //If we are sleeping between readings then we cannot rely on millis() as it is powered down. Used RTC instead.
  measurementStartTime = rtcMillis();

  digitalWrite(PIN_STAT_LED, LOW); // Turn the STAT LED off now that everything is configured

//  //If we are immediately going to go to sleep after the first reading then
//  //first present the user with the config menu in case they need to change something
//  if (settings.usBetweenReadings == settings.usLoggingDuration)
//    menuMain();
}

void loop() {
  
  checkBattery(); // Check for low battery

  if (Serial.available()) menuMain(); //Present user menu

  storeData();

  if ((settings.useGPIO32ForStopLogging == true) && (stopLoggingSeen == true)) // Has the user pressed the stop logging button?
  {
    stopLogging();
  }

  uint64_t timeNow = rtcMillis();

  // Is sleep enabled and is it time to go to sleep?
  if ((settings.usSleepDuration > 0) && (timeNow > (measurementStartTime + (settings.usLoggingDuration / 1000ULL))))
  {
    if (settings.printMajorDebugMessages == true)
    {
      Serial.println(F("Going to sleep..."));
    }

    goToSleep();

    if (settings.printMajorDebugMessages == true)
    {
      Serial.println(F("I'm awake!"));
    }

    //Update measurementStartTime so we know when to go back to sleep
    measurementStartTime = measurementStartTime + (settings.usLoggingDuration / 1000ULL) + (settings.usSleepDuration / 1000ULL);
    
    rtcNeedsSync = true; //Let's re-sync the RTC after sleep
  }
}

void beginQwiic()
{
  pinMode(PIN_QWIIC_POWER, OUTPUT);
  qwiicPowerOn();
  qwiic.begin();
  setQwiicPullups(); //Just to make it really clear what pull-ups are being used, set pullups here.
}

void setQwiicPullups()
{
  //Change SCL and SDA pull-ups manually using pin_config
  am_hal_gpio_pincfg_t sclPinCfg = g_AM_BSP_GPIO_IOM1_SCL;
  am_hal_gpio_pincfg_t sdaPinCfg = g_AM_BSP_GPIO_IOM1_SDA;

  if (settings.qwiicBusPullUps == 0)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE; // No pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_NONE;
  }
  else if (settings.qwiicBusPullUps == 1)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K; // Use 1K5 pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  }
  else if (settings.qwiicBusPullUps == 6)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K; // Use 6K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_6K;
  }
  else if (settings.qwiicBusPullUps == 12)
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K; // Use 12K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_12K;
  }
  else
  {
    sclPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K; // Use 24K pull-ups
    sdaPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_24K;
  }

  pin_config(PinName(PIN_QWIIC_SCL), sclPinCfg);
  pin_config(PinName(PIN_QWIIC_SDA), sdaPinCfg);
}

void beginSD()
{
  pinMode(PIN_MICROSD_POWER, OUTPUT);
  pinMode(PIN_MICROSD_CHIP_SELECT, OUTPUT);
  digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected

  if (settings.enableSD == true)
  {
    microSDPowerOn();

    //Max power up time is 250ms: https://www.kingston.com/datasheets/SDCIT-specsheet-64gb_en.pdf
    //Max current is 200mA average across 1s, peak 300mA
    for (int i = 0; i < 10; i++) //Wait
    {
      checkBattery(); // Check for low battery
      delay(1);
    }

    if (sd.begin(PIN_MICROSD_CHIP_SELECT, SD_SCK_MHZ(24)) == false) //Standard SdFat
    {
      Serial.println(F("SD init failed (first attempt). Trying again...\r\n"));
      for (int i = 0; i < 250; i++) //Give SD more time to power up, then try again
      {
        checkBattery(); // Check for low battery
        delay(1);
      }
      if (sd.begin(PIN_MICROSD_CHIP_SELECT, SD_SCK_MHZ(24)) == false) //Standard SdFat
      {
        Serial.println(F("SD init failed. Is card present? Formatted?"));
        digitalWrite(PIN_MICROSD_CHIP_SELECT, HIGH); //Be sure SD is deselected
        online.microSD = false;
        return;
      }
    }

    //Change to root directory. All new file creation will be in root.
    if (sd.chdir() == false)
    {
      Serial.println(F("SD change directory failed"));
      online.microSD = false;
      return;
    }

    online.microSD = true;
  }
  else
  {
    microSDPowerOff();
    online.microSD = false;
  }
}

void disableIMU()
{
  pinMode(PIN_IMU_POWER, OUTPUT);
  pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
  digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); //Be sure IMU is deselected

  imuPowerOff();
}
// Instead of disable IMU - Amara

void beginIMU() {
  if (settings.sensor_IMU.log == false) {
    // If IMU logging is disabled, power it off
    disableIMU();
    return;
  }

  Serial.println(F("Starting IMU initialization...")); // ADDED: Debug message

  pinMode(PIN_IMU_POWER, OUTPUT);
  pinMode(PIN_IMU_CHIP_SELECT, OUTPUT);
  digitalWrite(PIN_IMU_CHIP_SELECT, HIGH); // Deselect IMU
  
  // Power on IMU
  imuPowerOn();
  delay(100); // Wait for IMU to power up

  SPI.begin();
  if (!enableCIPOpullUp()) { // Enable CIPO pull-up
    Serial.println(F("WARNING: Failed to enable CIPO pull-up. SPI may be unreliable."));
  }

  bool initialized = false;
  int attempts = 0;
  while (!initialized && attempts < 5) {
    myICM.begin(PIN_IMU_CHIP_SELECT, SPI);

    if (settings.printMajorDebugMessages) {
      Serial.print(F("IMU initialization attempt "));
      Serial.print(attempts + 1);
      Serial.print(F(": "));
      Serial.println(myICM.statusString());
    }
    
    if (myICM.status == ICM_20948_Stat_Ok) {
      initialized = true;
      online.imu = true;
      
      // ADDED: Test if we can actually read data
      if (myICM.dataReady()) {
        myICM.getAGMT(); // Try to get data
        Serial.print(F("IMU test reading - AccX: "));
        Serial.println(myICM.accX());
      } else {
        Serial.println(F("IMU initialized but no data ready"));
      }
      
      if (settings.printMajorDebugMessages) {
        Serial.println(F("IMU online"));
      }
    } else {
      attempts++;
      delay(500);
    }
  }
  
  if (!initialized) {
    if (settings.printMajorDebugMessages) {
      Serial.println(F("IMU initialization failed"));
    }
    online.imu = false;
    disableIMU();
  }
}

// Function to read and log IMU data - Amara
void readIMUData() {
  if (!online.imu || !settings.sensor_IMU.log) {
    // ADDED: Debug message when IMU is not available
    if (settings.printMinorDebugMessages) {
      Serial.println(F("IMU not online or logging disabled"));
    }
    return;
  }
  
  unsigned long currentTime = millis();
  if (currentTime - lastIMUReadTime < settings.sensor_IMU.logRateMs) {
    return; // Not time to read IMU yet
  }
  
  // ADDED: Check if IMU has data ready with debug message
  if (!myICM.dataReady()) {
    if (settings.printMinorDebugMessages) {
      Serial.println(F("IMU data not ready"));
    }
    return;
  }
  
  // Get fresh data
  myICM.getAGMT(); // Get Accelerometer, Gyroscope, Magnetometer, Temperature
  
  // ADDED: Debug raw values to help troubleshoot
  if (settings.printMinorDebugMessages) {
    Serial.print(F("IMU Raw - AccX: "));
    Serial.print(myICM.accX());
    Serial.print(F(", AccY: "));
    Serial.print(myICM.accY());
    Serial.print(F(", AccZ: "));
    Serial.println(myICM.accZ());
  }
  
  // Create IMU data string in a stack buffer (no heap allocation)
  char imuDataBuf[200];
  int imuDataLen = createIMUDataString(imuDataBuf, sizeof(imuDataBuf));

  if (settings.printMinorDebugMessages) {
    Serial.print(F("IMU Data String: "));
    Serial.print(imuDataBuf);
  }

  // Log to SD card if enabled
  if (settings.logData && online.microSD && online.dataLogging) {
    if (imuDataFile) {
      size_t bytesWritten = imuDataFile.write(imuDataBuf, imuDataLen);
      if (bytesWritten > 0) {
        if (settings.printMinorDebugMessages) {
          Serial.print(F("Written "));
          Serial.print(bytesWritten);
          Serial.println(F(" bytes to IMU file"));
        }
      } else {
        Serial.println(F("Failed to write to IMU file"));
      }
    } else {
      Serial.println(F("IMU file not open"));
    }
  }

  // Output to terminal if enabled
  if (settings.enableTerminalOutput) {
    Serial.print(imuDataBuf);
  }
  
  lastIMUReadTime = currentTime;
}

// Apollo3 snprintf/dtostrf don't support floats. Manual conversion.
// Writes a float as "-123.45" into buf, returns chars written.
int floatToStr(char *buf, int bufSize, float val, int decimals) {
  int pos = 0;
  if (val < 0.0f) {
    if (pos < bufSize - 1) buf[pos++] = '-';
    val = -val;
  }
  // Round
  float rounding = 0.5f;
  for (int i = 0; i < decimals; i++) rounding /= 10.0f;
  val += rounding;

  unsigned long intPart = (unsigned long)val;
  float remainder = val - (float)intPart;

  // Write integer part
  pos += snprintf(buf + pos, bufSize - pos, "%lu", intPart);

  if (decimals > 0 && pos < bufSize - 1) {
    buf[pos++] = '.';
    for (int i = 0; i < decimals && pos < bufSize - 1; i++) {
      remainder *= 10.0f;
      int digit = (int)remainder;
      buf[pos++] = '0' + digit;
      remainder -= digit;
    }
  }
  buf[pos] = '\0';
  return pos;
}

// Uses a static char buffer instead of String to avoid heap fragmentation.
// Parse __DATE__ ("Apr  8 2026") and __TIME__ ("16:56:45") to set the RTC
void setRTCFromCompileTime() {
  const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char monthStr[4];
  int day, year, hour, minute, second;

  // __DATE__ format: "Mmm dd yyyy" (dd may have leading space)
  sscanf(__DATE__, "%3s %d %d", monthStr, &day, &year);
  sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);

  // Convert month string to number (1-12)
  const char *found = strstr(months, monthStr);
  int month = ((found - months) / 3) + 1;

  myRTC.setTime(0, second, minute, hour, day, month, (year - 2000));
  rtcHasBeenSyncd = true;
  rtcNeedsSync = false;

  Serial.print(F("RTC set from compile time: "));
  char timeString[40];
  getTimeString(timeString);
  Serial.println(timeString);
}

int createIMUDataString(char *buf, int bufSize) {
  int pos = 0;

  // Add timestamp and millis
  char timeString[40];
  getTimeString(timeString);
  pos += snprintf(buf + pos, bufSize - pos, "%s,IMU", timeString);

  if (settings.sensor_IMU.enableAccel) {
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.accX(), 2);
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.accY(), 2);
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.accZ(), 2);
  }

  if (settings.sensor_IMU.enableGyro) {
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.gyrX(), 2);
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.gyrY(), 2);
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.gyrZ(), 2);
  }

  if (settings.sensor_IMU.enableMag) {
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.magX(), 2);
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.magY(), 2);
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.magZ(), 2);
  }

  if (settings.sensor_IMU.enableTemp) {
    buf[pos++] = ','; pos += floatToStr(buf + pos, bufSize - pos, myICM.temp(), 1);
  }

  pos += snprintf(buf + pos, bufSize - pos, "\r\n");
  return pos;
}

void configureSerial1TxRx(void) // Configure pins 12 and 13 for UART1 TX and RX
{
  am_hal_gpio_pincfg_t pinConfigTx = g_AM_BSP_GPIO_COM_UART_TX;
  pinConfigTx.uFuncSel = AM_HAL_PIN_12_UART1TX;
  pin_config(PinName(BREAKOUT_PIN_TX), pinConfigTx);
  am_hal_gpio_pincfg_t pinConfigRx = g_AM_BSP_GPIO_COM_UART_RX;
  pinConfigRx.uFuncSel = AM_HAL_PIN_13_UART1RX;
  pinConfigRx.ePullup = AM_HAL_GPIO_PIN_PULLUP_WEAK; // Put a weak pull-up on the Rx pin
  pin_config(PinName(BREAKOUT_PIN_RX), pinConfigRx);
}


void beginDataLogging()
{
  if (online.microSD == true && settings.logData == true)
  {
    //If we don't have a file yet, create one. Otherwise, re-open the last used file
    if ((strlen(gnssDataFileName) == 0) || (settings.openNewLogFile == true))
      strcpy(gnssDataFileName, findNextAvailableLog(settings.nextDataLogNumber, "dataLog"));

    // O_CREAT - create the file if it does not exist
    // O_APPEND - seek to the end of the file prior to each write
    // O_WRITE - open for write
    if (gnssDataFile.open(gnssDataFileName, O_CREAT | O_APPEND | O_WRITE) == false)
    {
      Serial.println(F("Failed to create sensor data file"));
      online.dataLogging = false;
      return;
    }

    // Creation of IMU CSV File with better error handling
    if (settings.sensor_IMU.log) { // Only create IMU file if logging is enabled
      // Use (nextDataLogNumber - 1) to match the GNSS log number that was just assigned
      strcpy(imuDataFileName, findNextAvailableIMULog(settings.nextDataLogNumber - 1, "imuLog"));
      
      // ADDED: Debug message showing which file we're trying to open
      Serial.print(F("Opening IMU file: "));
      Serial.println(imuDataFileName);
      
      if (imuDataFile.open(imuDataFileName, O_CREAT | O_APPEND | O_WRITE) == false)
      {
        Serial.println(F("Failed to create IMU data file"));
        // Continue with GNSS logging even if IMU file fails
      } else {
        Serial.println(F("IMU file opened successfully")); // ADDED: Success confirmation
        
        // ADDED: Write CSV header if file is empty
        if (imuDataFile.fileSize() == 0) {
          String header = "Timestamp,Sensor,";
          if (settings.sensor_IMU.enableAccel) {
            header += "AccX,AccY,AccZ,";
          }
          if (settings.sensor_IMU.enableGyro) {
            header += "GyrX,GyrY,GyrZ,";
          }
          if (settings.sensor_IMU.enableMag) {
            header += "MagX,MagY,MagZ,";
          }
          if (settings.sensor_IMU.enableTemp) {
            header += "Temp";
          }
          header += "\r\n";
          imuDataFile.print(header);
          imuDataFile.sync();
          Serial.println(F("IMU CSV header written")); // ADDED: Header confirmation
        }
        
        updateDataFileCreate(&imuDataFile); // ADDED: This was missing
      }
    }

    updateDataFileCreate(&gnssDataFile); //File creation stamp

    online.dataLogging = true;
  }
  else
    online.dataLogging = false;
}

void beginSerialOutput()
{
  if ((settings.outputUBX == true) || (settings.outputNMEA == true))
  {
    //We need to manually restore the Serial1 TX and RX pins
    configureSerial1TxRx();

    Serial1.begin(settings.serialTXBaudRate); // (Re)start the serial port
  }
}

#if SD_FAT_TYPE == 1
void updateDataFileCreate(File32 *dataFile)
#elif SD_FAT_TYPE == 2
void updateDataFileCreate(ExFile *dataFile)
#elif SD_FAT_TYPE == 3
void updateDataFileCreate(FsFile *dataFile)
#else // SD_FAT_TYPE == 0
void updateDataFileCreate(File *dataFile)
#endif  // SD_FAT_TYPE
{
  //if (rtcHasBeenSyncd == true) //Update the create time stamp only if the RTC is valid
  {
    myRTC.getTime(); //Get the RTC time so we can use it to update the create time
    //Update the file create time
    dataFile->timestamp(T_CREATE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
  }
}

#if SD_FAT_TYPE == 1
void updateDataFileAccess(File32 *dataFile)
#elif SD_FAT_TYPE == 2
void updateDataFileAccess(ExFile *dataFile)
#elif SD_FAT_TYPE == 3
void updateDataFileAccess(FsFile *dataFile)
#else // SD_FAT_TYPE == 0
void updateDataFileAccess(File *dataFile)
#endif  // SD_FAT_TYPE
{
  //if (rtcHasBeenSyncd == true) //Update the write and access time stamps only if RTC is valid
  {
    myRTC.getTime(); //Get the RTC time so we can use it to update the last modified time
    //Update the file access time
    dataFile->timestamp(T_ACCESS, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
    //Update the file write time
    dataFile->timestamp(T_WRITE, (myRTC.year + 2000), myRTC.month, myRTC.dayOfMonth, myRTC.hour, myRTC.minute, myRTC.seconds);
  }
}

void printUint64(uint64_t val)
{
  Serial.print("0x");
  uint8_t Byte = (val >> 56) & 0xFF;
  Serial.print(Byte, HEX);
  Byte = (val >> 48) & 0xFF;
  Serial.print(Byte, HEX);
  Byte = (val >> 40) & 0xFF;
  Serial.print(Byte, HEX);
  Byte = (val >> 32) & 0xFF;
  Serial.print(Byte, HEX);
  Byte = (val >> 24) & 0xFF;
  Serial.print(Byte, HEX);
  Byte = (val >> 16) & 0xFF;
  Serial.print(Byte, HEX);
  Byte = (val >> 8) & 0xFF;
  Serial.print(Byte, HEX);
  Byte = (val >> 0) & 0xFF;
  Serial.println(Byte, HEX);
}

//Called once number of milliseconds has passed
extern "C" void am_stimer_cmpr6_isr(void)
{
  uint32_t ui32Status = am_hal_stimer_int_status_get(false);
  if (ui32Status & AM_HAL_STIMER_INT_COMPAREG)
  {
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREG);
  }
}

//Stop Logging ISR
void stopLoggingISR(void)
{
  stopLoggingSeen = true;
}

// From example code - Amara
#if defined(ARDUINO_ARCH_MBED) // updated for v2.1.0 of the Apollo3 core
bool enableCIPOpullUp()
{
  //Add 1K5 pull-up on CIPO
  am_hal_gpio_pincfg_t cipoPinCfg = g_AM_BSP_GPIO_IOM0_MISO;
  cipoPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  pin_config(PinName(PIN_SPI_CIPO), cipoPinCfg);
  return (true);
}
#else
bool enableCIPOpullUp()
{
  //Add CIPO pull-up
  ap3_err_t retval = AP3_OK;
  am_hal_gpio_pincfg_t cipoPinCfg = AP3_GPIO_DEFAULT_PINCFG;
  cipoPinCfg.uFuncSel = AM_HAL_PIN_6_M0MISO;
  cipoPinCfg.ePullup = AM_HAL_GPIO_PIN_PULLUP_1_5K;
  cipoPinCfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA;
  cipoPinCfg.eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL;
  cipoPinCfg.uIOMnum = AP3_SPI_IOM;
  padMode(MISO, cipoPinCfg, &retval);
  return (retval == AP3_OK);
}
#endif
// ADD these new functions anywhere in your main tab

// Function to check IMU file status
void checkIMUFileStatus() {
  if (settings.sensor_IMU.log && online.microSD) {
    if (imuDataFile) {
      Serial.print(F("IMU file size: "));
      Serial.print(imuDataFile.fileSize());
      Serial.println(F(" bytes"));
    } else {
      Serial.println(F("IMU file not open!"));
    }
  }
}

// Comprehensive IMU debug function
void debugIMU() {
  Serial.println(F("=== IMU Debug Information ==="));
  Serial.print(F("IMU Logging Enabled: "));
  Serial.println(settings.sensor_IMU.log ? "YES" : "NO");
  Serial.print(F("IMU Online: "));
  Serial.println(online.imu ? "YES" : "NO");
  Serial.print(F("IMU Log Rate (ms): "));
  Serial.println(settings.sensor_IMU.logRateMs);
  Serial.print(F("Last IMU Read: "));
  Serial.print((millis() - lastIMUReadTime) / 1000);
  Serial.println(F(" seconds ago"));
  
  if (online.imu) {
    Serial.println(F("Testing IMU data read..."));
    if (myICM.dataReady()) {
      myICM.getAGMT();
      Serial.print(F("AccX: "));
      Serial.print(myICM.accX());
      Serial.print(F(", AccY: "));
      Serial.print(myICM.accY());
      Serial.print(F(", AccZ: "));
      Serial.println(myICM.accZ());
    } else {
      Serial.println(F("IMU data not ready"));
    }
  }
  
  checkIMUFileStatus();
  Serial.println(F("=== End IMU Debug ==="));
}