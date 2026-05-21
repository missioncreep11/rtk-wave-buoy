/* buoy_combo.ino */
#include "BotleticsSIM7000.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif
#include <Arduino.h>
#include "BluetoothSerial.h"

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#include "secrets.h" 
#include "buoy_combo.h"

// Pin Definitions
#define SIMCOM_7000
#define BOTLETICS_PWRKEY 18
#define RST 5
#define TX_MODEM 17  // ESP32 TX1 to Modem RX
#define RX_MODEM 16  // ESP32 RX1 to Modem TX

// Global Objects
HardwareSerial modemSS(1);     // UART1 to modem
HardwareSerial gpsSerial(2);   // UART2 to GPS
Botletics_modem_LTE modem = Botletics_modem_LTE();
SFE_UBLOX_GNSS myGNSS;

// State
enum SystemState {
  MODEM_INIT,
  START_CYCLE,         // Power on modem, start init, wait for RSSI and Registration
  AWAIT_RTK_FIX,       // NTRIP is streaming, waiting for carrSoln == 2
  IDLE_CYCLE           // Solution achieved! Graceful shutdown, then wait for next cycle
};

SystemState currState = START_CYCLE;

// Flags
bool isBooting = false;
bool hasStarted = false;
bool networkConnected = false;
bool gprsEnabled = false;
bool gpsEnabled = false;
bool ntripConnected = false;
bool gpsUARTOnline = false;
volatile bool shutdownRequested = false;

// Timing
long lastReceivedRTCM_ms = 0;
int maxTimeBeforeHangup_ms = 100000;

const unsigned long ntripRetryInterval = 30000;
unsigned long lastNTRIPAttempt = 0;

long lastGPSPrint = 0;
unsigned long lastFixStatusPrint = 0;

unsigned long rtkFixedTimestamp_ms = 0;
const unsigned long rtkFixedWaitTime_ms = 30000;

unsigned long lteShutdownTimestamp_ms = 0;
const unsigned long lteShutdownWaitTime_ms = 60000;


// Configuration
uint8_t type;
char replybuffer[255];
char imei[16] = {0};

// Bluetooth Name
String device_name = "RTK-GPS-BT";

void setup() {
  // USB Debug Serial
  Serial.begin(9600);
  SerialBT.begin(device_name);
  Serial.println("Connecting to bluetooth...");
  
  delay(2000);
  Serial.println(F("\n=== Buoy Combo - UART GPS ==="));
  SerialBT.println("\n=== Buoy Combo - UART GPS ===");

  // Setting Pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(SHUTDOWN_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHUTDOWN_BTN), shutdownISR, FALLING);

  initialize_gnss_uart_f();

  // Initialize Modem Pins
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);

  Serial.println(F("Setup complete!\n"));
  SerialBT.println(F("Setup complete\n"));
}

void loop() {
  // Handle user AT commands
  if (Serial.available()) {
    Serial.print(F("modem> "));
    SerialBT.println(F("modem>" ));
    while (Serial.available()) {
      modemSS.write(Serial.read());
    }
    delay(100);
    while (modemSS.available()) {
      Serial.write(modemSS.read());
    }
    return;
  }
  
  switch (currState)
  {
    case SystemState::MODEM_INIT:
      if (!isBooting)
      {
        isBooting = true;

        Serial.println(F("Powering on modem..."));
        SerialBT.println(F("Powering on modem..."));
        modem.powerOn(BOTLETICS_PWRKEY);

        // Start modem serial
        modemSS.begin(9600, SERIAL_8N1, TX_MODEM, RX_MODEM);   
      }

      if (!modem.begin(modemSS)) 
      {
        Serial.println(F("Couldn't find modem"));
      }
      else
      {
        currState = SystemState::START_CYCLE;
      }
      break;
    case SystemState::START_CYCLE:
      if (!hasStarted)
      {
        hasStarted = true;

        type = modem.type();
        Serial.println(F("Modem is OK"));
        Serial.println(F("SIM7000 detected"));
        
        // Print module IMEI number
        uint8_t imeiLen = modem.getIMEI(imei);
        if (imeiLen > 0) {
          Serial.print(F("Module IMEI: ")); 
          Serial.println(imei);
        }
        
        modem.setFunctionality(1); // AT+CFUN=1
        modem.setNetworkSettings(F("hologram")); // For Hologram SIM card
        
        Serial.println(F("Modem setup complete!"));
      }

      if (!networkConnected)
      {
        Serial.println(F("Awaiting network connection..."));
        SerialBT.println(F("Awaiting network connection..."));
      }
      else if (!gprsEnabled)
      {
        Serial.println(F("Awaiting GPRS..."));
        SerialBT.println(F("Awaiting GPRS..."));
      }
      else if (!ntripConnected)
      {
        Serial.println(F("Awaiting NTRIP connection..."));
        SerialBT.println(F("Awaiting NTRIP connection..."));
      }
      else if (myGNSS.getCarrierSolutionType() != 2)
      {
        Serial.println(F("Awaiting RTK Fixed status..."));
        SerialBT.println(F("Awaiting RTK Fixed status..."));
      }
      else
      {
        currState = SystemState::AWAIT_RTK_FIX;
      }
      break;

    case SystemState::AWAIT_RTK_FIX:
      if (myGNSS.getCarrierSolutionType() == 2)
      {
        if (rtkFixedTimestamp_ms == 0) rtkFixedTimestamp_ms = millis();

        if (millis() - rtkFixedTimestamp_ms >= rtkFixedWaitTime_ms)
        {
          rtkFixedTimestamp_ms = 0;
          currState = SystemState::IDLE_CYCLE;
        }
      }
      break;

    case SystemState::IDLE_CYCLE:
      // Perform one-time graceful shutdown
      if (lteShutdownTimestamp_ms == 0) {
        Serial.println(F("Closing NTRIP connection..."));
        SerialBT.println(F("Closing NTRIP connection..."));
        modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
        ntripConnected = false;
        delay(1000);
      
        // Disable GPRS
        Serial.println(F("Disabling GPRS..."));
        SerialBT.println(F("Disabling GPRS..."));
        modem.enableGPRS(false);
        gprsEnabled = false;
        delay(1000);
      
        // Power down modem
        Serial.println(F("Powering down modem..."));
        SerialBT.println(F("Powering down modem..."));
        modem.sendCheckReply(F("AT+CPOWD=1"), F("NORMAL POWER DOWN"), 5000);
        delay(2000);

        // reset all flags
        ntripConnected = false;
        gprsEnabled = false;
        networkConnected = false;
        hasStarted = false;
        isBooting = false;

        lteShutdownTimestamp_ms = millis();
      }

      // wait 1 minute
      if (millis() - lteShutdownTimestamp_ms >= lteShutdownWaitTime_ms)
      {
        lteShutdownTimestamp_ms = 0;
        currState = SystemState::MODEM_INIT;
      }
      break;
    default:
      Serial.println(F("ERROR: State exception reached"));
      SerialBT.println(F("ERROR: State exception reached"));
      currState = SystemState::MODEM_INIT;
      break;
  }

  // background tasks
  if (currState == SystemState::START_CYCLE || currState == SystemState::AWAIT_RTK_FIX)
  {
    // Network management
    network_status_check_f();
    monitor_connection_health();
    enable_gprs_f();
    
    // NTRIP connection management
    if (gprsEnabled && !ntripConnected && 
        (millis() - lastNTRIPAttempt > ntripRetryInterval)) {
      beginNTRIPClient();
      lastNTRIPAttempt = millis();
    }

    // Handle NTRIP data (receives RTCM and sends to GPS via UART)
    if (ntripConnected) {
      handleNTRIPData();
    }
  }

  // Print GPS fix status every 5 seconds
  if (gpsUARTOnline && (millis() - lastFixStatusPrint > 5000)) {
    lastFixStatusPrint = millis();
    if (myGNSS.getPVT()) {  // single poll, populates everything below
      uint8_t fixType  = myGNSS.getFixType();           // 0=no, 2=2D, 3=3D, 4=GNSS+DR
      uint8_t carrSoln = myGNSS.getCarrierSolutionType(); // 0=none, 1=float, 2=FIXED
      uint8_t sats     = myGNSS.getSIV();

      const char* rtkStr = (carrSoln == 2) ? "FIXED" :
                           (carrSoln == 1) ? "float" : "none";

      Serial.print(F("[GPS] fix="));    Serial.print(fixType);
      Serial.print(F(" rtk="));         Serial.print(rtkStr);
      Serial.print(F(" sats="));        Serial.println(sats);

      SerialBT.print(F("[GPS] fix="));  SerialBT.print(fixType);
      SerialBT.print(F(" rtk="));       SerialBT.print(rtkStr);
      SerialBT.print(F(" sats="));      SerialBT.println(sats);
    }
  }
  
  updateStatusLED();

  // Check for shutdown request
  if (shutdownRequested) {
    gracefulShutdown();
  }

  delay(10);  // Reduced from 1000ms for better responsiveness
}

// =============================================================================
// Helper Functions (Implementations)
// =============================================================================

void initialize_gnss_uart_f() {
  Serial.println(F("=== Initializing ZED-F9P via UART ==="));
  SerialBT.println(F("=== Initializing ZED-F9P via UART ==="));
  
  gpsSerial.begin(115200, SERIAL_8N1, RX_GPS, TX_GPS);
  delay(1000);  
  
  if (myGNSS.begin(gpsSerial)) {
    Serial.println(F("GPS UART connected!"));
    SerialBT.println(F("GPS UART connected!"));
    gpsUARTOnline = true;

    myGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
    myGNSS.setUART1Output(COM_TYPE_UBX);
    myGNSS.saveConfiguration();
  } else {
    Serial.println(F("ERROR: GPS UART failed!"));
    SerialBT.println(F("ERROR: GPS UART failed!"));
    gpsUARTOnline = false;
  }
}

void network_status_check_f() {
  if (!networkConnected) {
    uint8_t n = modem.getNetworkStatus();
    if (n == 1 || n == 5) {
      networkConnected = true;
      Serial.println(F("Network connection confirmed!"));
    }
  }
}

void enable_gprs_f() {
  if (networkConnected && !gprsEnabled) {
    if (modem.enableGPRS(true)) {
      gprsEnabled = true;
      Serial.println(F("GPRS enabled successfully!"));
    }
  }
}

void beginNTRIPClient() {
  Serial.println(F("Attempting NTRIP connection via LTE TCP..."));
  if (!modem.TCPconnect((char*)casterHost, casterPort)) return;
  
  delay(1000);
  String ntripRequest = "GET /" + String(mountPoint) + " HTTP/1.0\r\n";
  ntripRequest += "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n";
  
  if (strlen(casterUser) > 0) {
    base64 b;
    ntripRequest += "Authorization: Basic " + b.encode(String(casterUser) + ":" + String(casterUserPW)) + "\r\n";
  }
  ntripRequest += "\r\n";
  
  if (modem.TCPsend((char*)ntripRequest.c_str(), ntripRequest.length())) {
    ntripConnected = true;
    lastReceivedRTCM_ms = millis();
    Serial.println(F("NTRIP request sent!"));
  }
}

void handleNTRIPData() {
  uint16_t available = modem.TCPavailable();
  if (available > 0) {
    uint8_t rtcmBuffer[256];
    uint16_t bytesRead = modem.TCPread(rtcmBuffer, 250);
    if (bytesRead > 0 && gpsUARTOnline) {
      gpsSerial.write(rtcmBuffer, bytesRead);
      lastReceivedRTCM_ms = millis();
    }
  }  
  
  if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
    Serial.println(F("RTCM timeout. Disconnecting..."));
    ntripConnected = false;
  }
}

void monitor_connection_health() {
  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck > 30000) {
    lastHealthCheck = millis();
    uint8_t status = modem.getNetworkStatus();
    if (status != 1 && status != 5) {
      networkConnected = false;
      gprsEnabled = false;
      ntripConnected = false;
    }
  }
}

void updateStatusLED() {
  digitalWrite(STATUS_LED, ntripConnected ? HIGH : LOW);
}

void IRAM_ATTR shutdownISR() {
  shutdownRequested = true;
}

void gracefulShutdown() {
  Serial.println(F("\n=== SHUTDOWN REQUESTED ==="));
  modem.sendCheckReply(F("AT+CPOWD=1"), F("NORMAL POWER DOWN"), 5000);
  digitalWrite(STATUS_LED, LOW);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  esp_light_sleep_start();
}
