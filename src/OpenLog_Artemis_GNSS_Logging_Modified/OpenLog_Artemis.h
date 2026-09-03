#ifndef OPENLOG_ARTEMIS_H
#define OPENLOG_ARTEMIS_H

#include <Arduino.h>

// Board identifier — must be defined before settings.h uses it
#define OLA_IDENTIFIER 0x232

#include "settings.h"
#include "ICM_20948.h"
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <SdFat.h>
#include "RTC.h"

// Firmware version
#define FIRMWARE_VERSION_MAJOR 3
#define FIRMWARE_VERSION_MINOR 2

// Hardware version
#define HARDWARE_VERSION_MAJOR 1
#define HARDWARE_VERSION_MINOR 0

// Pin definitions
#if (HARDWARE_VERSION_MAJOR == 0 && HARDWARE_VERSION_MINOR == 4)
  const byte PIN_MICROSD_CHIP_SELECT = 10;
  const byte PIN_IMU_POWER = 22;
#elif (HARDWARE_VERSION_MAJOR == 1 && HARDWARE_VERSION_MINOR == 0)
  const byte PIN_MICROSD_CHIP_SELECT = 23;
  const byte PIN_IMU_POWER = 27;
  const byte PIN_PWR_LED = 29;
  const byte PIN_VREG_ENABLE = 25;
  const byte PIN_VIN_MONITOR = 34;
#endif

const byte PIN_POWER_LOSS = 3;
const int8_t PIN_LOGIC_DEBUG = -1;
const byte PIN_MICROSD_POWER = 15;
const byte PIN_QWIIC_POWER = 18;
const byte PIN_STAT_LED = 19;
const byte PIN_IMU_INT = 37;
const byte PIN_IMU_CHIP_SELECT = 44;
const byte PIN_STOP_LOGGING = 32;
const byte PIN_QWIIC_SCL = 8;
const byte PIN_QWIIC_SDA = 9;
const byte PIN_SPI_SCK = 5;
const byte PIN_SPI_CIPO = 6;
const byte PIN_SPI_COPI = 7;
const byte BREAKOUT_PIN_32 = 32;
const byte BREAKOUT_PIN_TX = 12;
const byte BREAKOUT_PIN_RX = 13;
const byte BREAKOUT_PIN_11 = 11;

// Return status enum
enum returnStatus {
  STATUS_GETBYTE_TIMEOUT = 255,
  STATUS_GETNUMBER_TIMEOUT = -123455555,
  STATUS_PRESSED_X,
};

// SD card configuration
#define SD_FAT_TYPE 3
#define SD_CONFIG SdSpiConfig(PIN_MICROSD_CHIP_SELECT, SHARED_SPI, SD_SCK_MHZ(24))

#if SD_FAT_TYPE == 1
  extern SdFat32 sd;
  extern File32 gnssDataFile;
#elif SD_FAT_TYPE == 2
  extern SdExFat sd;
  extern ExFile gnssDataFile;
#elif SD_FAT_TYPE == 3
  extern SdFs sd;
  extern FsFile gnssDataFile;
  extern FsFile imuDataFile;
#else
  extern SdFat sd;
  extern File gnssDataFile;
  extern File imuDataFile;
#endif

// GNSS buffer sizes — define before SparkFun library so ours takes precedence
#undef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE 384
#define FILE_BUFFER_SIZE 32768

#include "SparkFun_u-blox_GNSS_v3.h"

// Debug macro
#define DUMP(varname) { Serial.printf("%s: %d\r\n", #varname, varname); }

// Qwiic bus
extern TwoWire qwiic;

// RTC
extern Apollo3RTC myRTC;

// GNSS
extern SFE_UBLOX_GNSS gpsSensor_ublox;

// IMU
extern ICM_20948_SPI myICM;

// Global variables
extern struct_settings settings;
extern struct_online online;
extern struct_QwiicSensors qwiicAvailable;
extern struct_QwiicSensors qwiicOnline;
extern uint64_t measurementStartTime;
extern unsigned long lastReadTime;
extern unsigned long lastDataLogSyncTime;
extern const byte menuTimeout;
extern bool rtcHasBeenSyncd;
extern bool rtcNeedsSync;
extern bool gnssSettingsChanged;
extern volatile bool stopLoggingSeen;
extern int lowBatteryReadings;
extern const int lowBatteryReadingsLimit;
extern bool ignorePowerLossInterrupt;
extern unsigned long lastIMUReadTime;
extern char gnssDataFileName[30];
extern char imuDataFileName[30];
extern const int sdPowerDownDelay;

// ============================================================
// Function prototypes
// ============================================================

// main.cpp
void beginQwiic();
void setQwiicPullups();
void beginSD();
void disableIMU();
void beginIMU();
void readIMUData();
int floatToStr(char *buf, int bufSize, float val, int decimals);
void setRTCFromCompileTime();
int createIMUDataString(char *buf, int bufSize);
void configureSerial1TxRx(void);
void beginDataLogging();
void beginSerialOutput();
#if SD_FAT_TYPE == 1
void updateDataFileCreate(File32 *dataFile);
void updateDataFileAccess(File32 *dataFile);
#elif SD_FAT_TYPE == 2
void updateDataFileCreate(ExFile *dataFile);
void updateDataFileAccess(ExFile *dataFile);
#elif SD_FAT_TYPE == 3
void updateDataFileCreate(FsFile *dataFile);
void updateDataFileAccess(FsFile *dataFile);
#else
void updateDataFileCreate(File *dataFile);
void updateDataFileAccess(File *dataFile);
#endif
void printUint64(uint64_t val);
extern "C" void am_stimer_cmpr6_isr(void);
void stopLoggingISR(void);
bool enableCIPOpullUp();
void checkIMUFileStatus();
void debugIMU();

// callbacks.cpp
void SerialPrintTimeString();
void callbackNAVPOSECEF(UBX_NAV_POSECEF_data_t *ubxDataStruct);
void callbackNAVSTATUS(UBX_NAV_STATUS_data_t *ubxDataStruct);
void callbackNAVDOP(UBX_NAV_DOP_data_t *ubxDataStruct);
void callbackNAVATT(UBX_NAV_ATT_data_t *ubxDataStruct);
void callbackNAVPVT(UBX_NAV_PVT_data_t *ubxDataStruct);
void callbackNAVODO(UBX_NAV_ODO_data_t *ubxDataStruct);
void callbackNAVVELECEF(UBX_NAV_VELECEF_data_t *ubxDataStruct);
void callbackNAVVELNED(UBX_NAV_VELNED_data_t *ubxDataStruct);
void callbackNAVHPPOSECEF(UBX_NAV_HPPOSECEF_data_t *ubxDataStruct);
void callbackNAVHPPOSLLH(UBX_NAV_HPPOSLLH_data_t *ubxDataStruct);
void callbackNAVCLOCK(UBX_NAV_CLOCK_data_t *ubxDataStruct);
void callbackNAVRELPOSNED(UBX_NAV_RELPOSNED_data_t *ubxDataStruct);
void callbackRXMSFRBX(UBX_RXM_SFRBX_data_t *ubxDataStruct);
void callbackRXMRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct);
void callbackRXMMEASX(UBX_RXM_MEASX_data_t *ubxDataStruct);
void callbackTIMTM2(UBX_TIM_TM2_data_t *ubxDataStruct);
void callbackESFALG(UBX_ESF_ALG_data_t *ubxDataStruct);
void callbackESFINS(UBX_ESF_INS_data_t *ubxDataStruct);
void callbackESFMEAS(UBX_ESF_MEAS_data_t *ubxDataStruct);
void callbackESFRAW(UBX_ESF_RAW_data_t *ubxDataStruct);
void callbackESFSTATUS(UBX_ESF_STATUS_data_t *ubxDataStruct);
void callbackHNRPVT(UBX_HNR_PVT_data_t *ubxDataStruct);
void callbackHNRATT(UBX_HNR_ATT_data_t *ubxDataStruct);
void callbackHNRINS(UBX_HNR_INS_data_t *ubxDataStruct);

// logging.cpp
char* findNextAvailableLog(int &newFileNumber, const char *fileLeader);
char* findNextAvailableIMULog(int fileNumber, const char *fileLeader);

// lowerPower.cpp
void checkBattery(void);
void powerDown();
void goToSleep();
void wakeFromSleep();
void stopLogging(void);
void qwiicPowerOn();
void qwiicPowerOff();
void microSDPowerOn();
void microSDPowerOff();
void imuPowerOn();
void imuPowerOff();
void powerLEDOn();
void powerLEDOff();
uint64_t rtcMillis();
int calculateDayOfYear(int day, int month, int year);

// menuAttachedDevices.cpp
void menuConfigure_uBlox();
void menuConfigure_uBloxUBX();
void menuConfigure_uBloxNMEA();
void setLogRate(uint8_t *rate);

// menuDebug.cpp
void menuDebug(bool *printMajorDebugMessages, bool *printMinorDebugMessages);

// menuIMU.cpp
void menuConfigure_IMU();

// menuMain.cpp
void menuMain();
void menuConfigure_QwiicBus();
void menuSetUTCTime();

// menuPower.cpp
void menuPower();

// menuTerminal.cpp
void menuLogRate(bool *prevTerminalOutput);

// nvm.cpp
void loadSettings();
void recordSettings();
void recordSettingsToFile();
bool loadSettingsFromFile();
bool parseLine(char* str);

// Sensors.cpp
bool beginSensors();
bool detectQwiicDevices();
void openNewLogFile();
void closeLogFile();
void resetGNSS();
void disableMessages(uint16_t maxWait);
void enableMessages(uint16_t maxWait);
boolean enableConstellations(uint16_t maxWait);
boolean powerManagementTask(uint32_t duration, uint16_t maxWait);
void determineMaxI2CSpeed();
float readVIN();

// storeData.cpp
void storeData(void);
void storeFinalData(void);

// support.cpp
void printMajorDebug(String thingToPrint);
void printMinorDebug(String thingToPrint);
void printUnknown(uint8_t unknownChoice);
void printUnknown(int unknownValue);
void waitForInput();
uint8_t getByteChoice(int numberOfSeconds);
int64_t getNumber(int numberOfSeconds);
double getDouble(int numberOfSeconds);
int olaftoa(float fValue, char *pcBuf, int iPrecision, int bufSize);
void getTimeString(char timeStringBuffer[]);

#endif // OPENLOG_ARTEMIS_H
