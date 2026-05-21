/* buoy_combo.h */
#ifndef BUOY_COMBO_H
#define BUOY_COMBO_H

#include "BotleticsSIM7000.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>

// Add base64 library
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"
#else
#include <Base64.h>
#endif

#define STATUS_LED 13  // Built-in LED for status
#define TX_GPS 12      // ESP32 TX2 to GPS RX  
#define RX_GPS 27      // ESP32 RX2 to GPS TX
#define SHUTDOWN_BTN 0 // for shutdown

// Global variables defined in .ino
extern bool networkConnected;
extern bool gprsEnabled;
extern bool gpsEnabled;
extern bool ntripConnected;
extern bool gpsUARTOnline;
extern unsigned long lastNTRIPAttempt;
extern long lastReceivedRTCM_ms;
extern int maxTimeBeforeHangup_ms;
extern const unsigned long ntripRetryInterval;
extern long lastGPSPrint;
extern Botletics_modem_LTE modem;
extern SFE_UBLOX_GNSS myGNSS;
extern HardwareSerial gpsSerial;
extern volatile bool shutdownRequested;

// Function declarations (Prototypes)
void network_status_check_f();
void enable_gprs_f();
void initialize_gnss_uart_f();
void beginNTRIPClient();
void handleNTRIPData();
void monitor_connection_health();
void printDebugStatus();
void updateStatusLED();
void shutdownISR();
void gracefulShutdown();

#endif
