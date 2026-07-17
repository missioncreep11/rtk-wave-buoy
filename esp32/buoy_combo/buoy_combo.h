/* buoy_combo.h */
#ifndef BUOY_COMBO_H
#define BUOY_COMBO_H

#include "BotleticsSIM7000.h"
#include <Adafruit_INA228.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <cstring>

// BLE forward declarations (defined in buoy_combo.ino)
void buoyPrint(const String& msg);
void buoyPrintln(const String& msg);

inline void buoyPrint(int v)                  { buoyPrint(String(v)); }
inline void buoyPrint(unsigned int v)         { buoyPrint(String(v)); }
inline void buoyPrint(long v)                 { buoyPrint(String(v)); }
inline void buoyPrint(unsigned long v)        { buoyPrint(String(v)); }
inline void buoyPrint(uint8_t v)              { buoyPrint(String((unsigned int)v)); }
inline void buoyPrint(uint16_t v)             { buoyPrint(String((unsigned int)v)); }
inline void buoyPrint(float v, int p = 2)     { buoyPrint(String(v, p)); }
inline void buoyPrint(double v, int p = 2)    { buoyPrint(String(v, p)); }

inline void buoyPrintln(int v)                { buoyPrintln(String(v)); }
inline void buoyPrintln(unsigned int v)       { buoyPrintln(String(v)); }
inline void buoyPrintln(long v)               { buoyPrintln(String(v)); }
inline void buoyPrintln(unsigned long v)      { buoyPrintln(String(v)); }
inline void buoyPrintln(uint8_t v)            { buoyPrintln(String((unsigned int)v)); }
inline void buoyPrintln(uint16_t v)           { buoyPrintln(String((unsigned int)v)); }
inline void buoyPrintln(float v, int p = 2)   { buoyPrintln(String(v, p)); }
inline void buoyPrintln(double v, int p = 2)  { buoyPrintln(String(v, p)); }

extern bool bleConnected;

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_system.h>
#endif

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
#define I2C_SDA 23     // ESP32 Thing Plus (WRL-15663) Qwiic SDA
#define I2C_SCL 22     // ESP32 Thing Plus Qwiic SCL
#ifndef BOTLETICS_PWRKEY
#define BOTLETICS_PWRKEY 18
#endif
#ifndef MODEM_RST_PIN
#define MODEM_RST_PIN 5
#endif

// Zombie PDP recovery (override before #include "buoy_combo.h" if needed)
#ifndef DATA_PATH_STALE_MS
#define DATA_PATH_STALE_MS (5UL * 60UL * 1000UL)
#endif
#ifndef GPRS_REFRESH_COOLDOWN_MS
#define GPRS_REFRESH_COOLDOWN_MS (2UL * 60UL * 1000UL)
#endif
#ifndef NTRIP_FAILURES_BEFORE_HARD_RESET
#define NTRIP_FAILURES_BEFORE_HARD_RESET 10
#endif
#ifndef MODEM_HARD_RECOVER_COOLDOWN_MS
#define MODEM_HARD_RECOVER_COOLDOWN_MS (10UL * 60UL * 1000UL)
#endif
#ifndef NETWORK_RECHECK_MS
#define NETWORK_RECHECK_MS 30000UL
#endif
#ifndef CGREG_BAD_STREAK_LIMIT
#define CGREG_BAD_STREAK_LIMIT 2
#endif
#ifndef CELLULAR_LINK_ALIVE_MS
#define CELLULAR_LINK_ALIVE_MS 120000UL
#endif
#ifndef UNREGISTERED_HARD_RECOVER_MS
#define UNREGISTERED_HARD_RECOVER_MS (5UL * 60UL * 1000UL)
#endif
#ifndef UNREGISTERED_SEARCHING_GRACE_MS
#define UNREGISTERED_SEARCHING_GRACE_MS (10UL * 60UL * 1000UL)
#endif
#ifndef MODEM_POWER_CYCLE_COOLDOWN_MS
#define MODEM_POWER_CYCLE_COOLDOWN_MS (15UL * 60UL * 1000UL)
#endif
#ifndef MODEM_PWRKEY_OFF_MS
#define MODEM_PWRKEY_OFF_MS 1600UL
#endif
#ifndef MODEM_POWER_OFF_SETTLE_MS
#define MODEM_POWER_OFF_SETTLE_MS 8000UL
#endif
#ifndef MODEM_POST_POWER_ON_MS
#define MODEM_POST_POWER_ON_MS 5000UL
#endif
#ifndef MODEM_BOOT_POWER_ON_MS
#define MODEM_BOOT_POWER_ON_MS 5000UL
#endif
#ifndef MODEM_POST_RST_MS
#define MODEM_POST_RST_MS 5000UL
#endif
#ifndef MODEM_FULL_POWER_OFF_SETTLE_MS
#define MODEM_FULL_POWER_OFF_SETTLE_MS (20UL * 1000UL)
#endif
#ifndef REGISTERED_STABLE_MS
#define REGISTERED_STABLE_MS (2UL * 60UL * 1000UL)
#endif
#ifndef MODEM_STUCK_FORCE_CYCLE_MS
#define MODEM_STUCK_FORCE_CYCLE_MS (30UL * 60UL * 1000UL)
#endif
#ifndef MODEM_AT_READY_TIMEOUT_MS
#define MODEM_AT_READY_TIMEOUT_MS 20000UL
#endif

// UART1 to SIM7000 (override in sketch before #include if needed)
#ifndef TX_MODEM
#define TX_MODEM 17
#endif
#ifndef RX_MODEM
#define RX_MODEM 16
#endif

// Hologram / US LTE CAT-M: 12 = AT&T/T-Mobile, 13 = Verizon
#ifndef LTE_CATM_BAND
#define LTE_CATM_BAND 12
#endif
#ifndef LTE_CATM_US_FALLBACK
#define LTE_CATM_US_FALLBACK F("AT+CBANDCFG=\"CAT-M\",2,4,12,13")
#endif

// Plain AT+CIP* TCP for NTRIP. Botletics TCPconnect() uses SSL (AT+CACID/CAOPEN) when
// BOTLETICS_SSL is 1 in the installed library, which fails on plain NTRIP port 2101.
class BuoyModem : public Botletics_modem_LTE {
public:
  bool ensurePdpActive();
  bool bringUpCipStack();
  bool tcpConnectPlain(const char *server, uint16_t port);
  bool tcpConnectedPlain();
  bool tcpSendPlain(const char *packet, uint16_t len);
  bool sendHologramCloudMessage(const char *msg, uint16_t len);
  void printDiagnostics();
  bool waitModemAtReady(uint32_t timeoutMs = MODEM_AT_READY_TIMEOUT_MS);
  bool ensureRadioOn();
  bool applyLteCatMBandSettings();
  bool configureLteCatM(bool afterRecover = false);
  void invalidateCipStack();
  bool cnactHasIp();
  bool configureNetwork(bool afterRecover = false);

private:
  bool _cipStackUp = false;
};

extern bool networkConnected;
extern bool gprsEnabled;
extern bool gpsEnabled;
extern bool ntripConnected;
extern bool gpsUARTOnline;
extern unsigned long lastNTRIPAttempt;
extern unsigned long lastCellularActivity_ms;
extern unsigned long lastGprsEnabled_ms;
extern uint8_t consecutiveNtripFailures;
extern long lastReceivedRTCM_ms;
extern int maxTimeBeforeHangup_ms;
extern const unsigned long ntripRetryInterval;
extern long lastGPSPrint;
extern BuoyModem modem;
extern HardwareSerial modemSS;
extern SFE_UBLOX_GNSS myGNSS;
extern Adafruit_INA228 ina228;
extern HardwareSerial gpsSerial;
extern volatile bool shutdownRequested;
extern bool ina228Online;

// Function declarations
void network_status_check_f();
void enable_gprs_f();
void initialize_gnss_uart_f();
void initialize_ina228_f();
void print_power_status_f();
void beginNTRIPClient();
void handleNTRIPData();
void monitor_connection_health();
void noteCellularActivity();
void invalidateDataPath(const __FlashStringHelper *reason);
void refreshGprs_f(const __FlashStringHelper *reason);
bool modemHardRecover_f(const __FlashStringHelper *reason);
bool modemPowerCycleRecover_f(const __FlashStringHelper *reason,
                              bool bypassCooldown = false);
void modemRecoverEscalated_f(const __FlashStringHelper *reason);
void post_telemetry_f();
void printDebugStatus();
void updateStatusLED();
void shutdownISR();

extern const char hologramDeviceKey[];
extern char imei[];

#endif