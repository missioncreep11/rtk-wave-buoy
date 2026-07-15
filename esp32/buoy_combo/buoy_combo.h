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

  void printDiagnostics() {
    const uint16_t t = 3000;
    buoyPrint(F("[DIAG] CPIN: "));
    buoyPrintln(replybuffer);
    getReply(F("AT+CFUN?"), t);
    buoyPrint(F("[DIAG] CFUN: "));
    buoyPrintln(replybuffer);
    getReply(F("AT+CREG?"), t);
    buoyPrint(F("[DIAG] CREG (circuit): "));
    buoyPrintln(replybuffer);
    getReply(F("AT+CGREG?"), t);
    buoyPrint(F("[DIAG] CGREG (LTE data — used by [NET]): "));
    buoyPrintln(replybuffer);
    getReply(F("AT+CSQ"), t);
    buoyPrint(F("[DIAG] CSQ: "));
    buoyPrintln(replybuffer);
    getReply(F("AT+CGATT?"), t);
    buoyPrint(F("[DIAG] CGATT: "));
    buoyPrintln(replybuffer);
    getReply(F("AT+COPS?"), t);
    buoyPrint(F("[DIAG] COPS: "));
    buoyPrintln(replybuffer);
    getReply(F("AT+CNACT?"), t);
    buoyPrint(F("[DIAG] CNACT: "));
    buoyPrintln(replybuffer);
  }

  bool waitModemAtReady(uint32_t timeoutMs = MODEM_AT_READY_TIMEOUT_MS) {
    const uint32_t deadline = millis() + timeoutMs;
    while ((int32_t)(deadline - millis()) > 0) {
      if (sendCheckReply(F("AT"), ok_reply, 2000)) {
        getReply(F("AT+CPIN?"), (uint16_t)3000);
        if (strstr(replybuffer, "READY") != nullptr ||
            strstr(replybuffer, "SIM PIN") != nullptr) {
          return true;
        }
      }
      delay(500);
    }
    return false;
  }

  bool simPinReady() {
    getReply(F("AT+CPIN?"), (uint16_t)3000);
    return (strstr(replybuffer, "READY") != nullptr ||
            strstr(replybuffer, "SIM PIN") != nullptr);
  }

  bool ensureCfun1() {
    getReply(F("AT+CFUN?"), (uint16_t)3000);
    if (strstr(replybuffer, ": 1") != nullptr) {
      return true;
    }
    if (sendCheckReply(F("AT+CFUN=1"), ok_reply, 30000)) {
      delay(2000);
      return true;
    }
    buoyPrintln("[MODEM] CFUN=1 failed");
    return false;
  }

  bool cfunIs0() {
    getReply(F("AT+CFUN?"), (uint16_t)3000);
    return (strstr(replybuffer, ": 0") != nullptr);
  }

  bool applyLteCatMBandSettings() {
    bool ok = true;
    if (!setPreferredMode(38)) {
      buoyPrintln("[MODEM] setPreferredMode(38) failed");
      ok = false;
    }
    if (!setPreferredLTEMode(1)) {
      buoyPrintln("[MODEM] setPreferredLTEMode(1) failed");
      ok = false;
    }

    char bandCmd[48];
    snprintf(bandCmd, sizeof(bandCmd), "AT+CBANDCFG=\"CAT-M\",%d", LTE_CATM_BAND);
    if (!sendCheckReply(bandCmd, ok_reply, 8000)) {
      buoyPrintln("[MODEM] CBANDCFG band " + String(LTE_CATM_BAND) + " failed — trying US 2,4,12,13");
      buoyPrintln("[MODEM] CBANDCFG fallback 2,4,12,13");
      if (!sendCheckReply(LTE_CATM_US_FALLBACK, ok_reply, 8000)) {
        ok = false;
      }
    }

    sendCheckReply(F("AT+CGREG=2"), ok_reply, 3000);

    getReply(F("AT+CBANDCFG?"), (uint16_t)3000);
    buoyPrintln("[MODEM] CBANDCFG: " + String(replybuffer));

    return ok;
  }

  // afterRecover: CFUN=0 band cycle (post-RST/PWRKEY). Boot keeps radio on.
  bool configureLteCatM(bool afterRecover = false) {
    buoyPrintln("[MODEM] LTE CAT-M, band " + String(LTE_CATM_BAND) + (afterRecover ? " (recover)" : " (boot)"));

    if (!simPinReady()) {
      getReply(F("AT+CPIN?"), (uint16_t)3000);
      buoyPrint("[MODEM] SKIP band config — CPIN: ");
      buoyPrintln(replybuffer);
      return false;
    }

    if (afterRecover) {
      if (!sendCheckReply(F("AT+CFUN=0"), ok_reply, 10000) && !cfunIs0()) {
        buoyPrintln("[MODEM] CFUN=0 failed (recover band config)");
        ensureCfun1();
        return false;
      }
      delay(1500);
    }

    const bool ok = applyLteCatMBandSettings();
    ensureCfun1();
    return ok;
  }

  void invalidateCipStack() { _cipStackUp = false; }

  bool cnactHasIp() {
    getReply(F("AT+CNACT?"), (uint16_t)3000);
    return strchr(replybuffer, '.') != nullptr && strstr(replybuffer, "0.0.0.0") == nullptr;
  }

  bool configureNetwork(bool afterRecover = false) {
    if (!waitModemAtReady()) {
      buoyPrintln("[MODEM] WARN: modem not AT-ready before config");
    }
 
    if (!afterRecover) {
      setFunctionality(1);
      delay(2000);
    }
 
    setNetworkSettings(F("hologram"));
 
    bool ok = configureLteCatM(afterRecover);
    if (!ok) {
      buoyPrintln("[MODEM] WARN: LTE CAT-M band config failed");
    }
 
    ensureCfun1();
 
    sendCheckReply(F("AT+CGATT=1"), ok_reply, 15000);
    sendCheckReply(F("AT+COPS=0"), ok_reply, 60000);
    sendCheckReply(F("AT+CMEE=2"), ok_reply, 3000);
    sendCheckReply(F("AT+CDNSCFG=1,\"8.8.8.8\",\"1.1.1.1\""), ok_reply, 5000);
 
    buoyPrintln("[MODEM] post-config diagnostics:");
    printDiagnostics();
    return ok;
  }

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
// void print_gps_status_f();
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

extern const char hologramDeviceKey[];
extern char imei[];



// SAPBR is the legacy GPRS bearer the B03/B05 firmware needs for AT+HTTP* + AT+HTTPSSL.
// It is a separate bearer from CNACT and CIP, and on SIM7000 these can coexist.
bool BuoyModem::ensurePdpActive() {
  // GPRS uses AT+CNACT; activating again when already active returns "operation not allowed".
  if (wirelessConnStatus()) return true;
  if (!openWirelessConnection(true)) return false;
  return wirelessConnStatus();
}

bool BuoyModem::bringUpCipStack() {
  // The CIPSTART/CIPSEND/CIPRXGET stack is independent of CNACT.
  // Required order on SIM7000:
  //   CIPSHUT  -> IP INITIAL  (so CIPMUX/CIPRXGET can be set)
  //   CIPMUX=0
  //   CIPRXGET=1
  //   CSTT="<apn>"
  //   CIICR
  //   CIFSR    (must return an IP literal)
  if (_cipStackUp) return true;

  // CIPSHUT may deactivate CNACT; caller re-activates after.
  sendCheckReply(F("AT+CIPSHUT"), F("SHUT OK"), 20000);

  // CIPMUX / CIPRXGET only accepted in IP INITIAL state; tolerate either way.
  sendCheckReply(F("AT+CIPMUX=0"),   ok_reply, 5000);
  sendCheckReply(F("AT+CIPRXGET=1"), ok_reply, 5000);

  // CSTT may already be set from a prior bring-up; tolerate ERROR.
  sendCheckReply(F("AT+CSTT=\"hologram\""), ok_reply, 10000);

  if (!sendCheckReply(F("AT+CIICR"), ok_reply, 85000)) return false;

  // CIFSR returns just the IP literal on success (no OK), or ERROR.
  getReply(F("AT+CIFSR"), (uint16_t)5000);
  if (strstr(replybuffer, "ERROR") || !strchr(replybuffer, '.')) return false;

  _cipStackUp = true;
  return true;
}

bool BuoyModem::tcpConnectPlain(const char *server, uint16_t port) {
  // Best-effort socket cleanup; ignore errors when no socket is open.
  getReply(F("AT+CIPCLOSE"), (uint16_t)2000);

  // Bring up the legacy CIPSTART stack. CIPSHUT inside may kill CNACT, restored below.
  if (!bringUpCipStack()) return false;
  ensurePdpActive();

  char cmd[128];
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", server, port);
  if (!sendCheckReply(cmd, ok_reply, 60000)) return false;

  // CIPSTART returns OK first, then CONNECT OK / ALREADY CONNECT / CONNECT FAIL / STATE: PDP DEACT.
  uint32_t deadline = millis() + 75000;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(2000);
    if (replybuffer[0] == 0) continue;
    if (strstr(replybuffer, "CONNECT OK") || strstr(replybuffer, "ALREADY CONNECT")) return true;
    if (strstr(replybuffer, "PDP DEACT") || strstr(replybuffer, "CONNECT FAIL") ||
        strstr(replybuffer, "ERROR")) {
      _cipStackUp = false;  // force re-bring-up next attempt
      return false;
    }
  }
  return false;
}

bool BuoyModem::tcpConnectedPlain() {
  if (!sendCheckReply(F("AT+CIPSTATUS"), ok_reply, 100)) return false;
  readline(100);
  return (strcmp(replybuffer, "STATE: CONNECT OK") == 0);
}

bool BuoyModem::tcpSendPlain(const char *packet, uint16_t len) {
  flushInput();

  // AT+CIPSEND=<len> -- modem replies with ">" then waits for exactly <len> bytes.
  // Cannot use sendCheckReply() here: it expects "OK" but the response is "> ".
  mySerial->print(F("AT+CIPSEND="));
  mySerial->println(len);

  uint32_t deadline = millis() + 5000;
  bool gotPrompt = false;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(1000);
    if (strchr(replybuffer, '>')) { gotPrompt = true; break; }
    if (strstr(replybuffer, "ERROR")) break;
  }
  if (!gotPrompt) {
    // ESC (0x1B) aborts the pending CIPSEND so the modem returns to AT mode.
    mySerial->write((uint8_t)0x1B);
    delay(200);
    flushInput();
    return false;
  }

  // Write the payload exactly once. Modem is now counting bytes.
  mySerial->write((const uint8_t *)packet, len);

  deadline = millis() + 15000;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(2000);
    if (replybuffer[0] == 0) continue;
    if (strstr(replybuffer, "SEND OK")) return true;
    if (strstr(replybuffer, "SEND FAIL") ||
        strstr(replybuffer, "ERROR") ||
        strstr(replybuffer, "CLOSED")) return false;
  }
  return false;
}

bool BuoyModem::sendHologramCloudMessage(const char *msg, uint16_t len) {
  buoyPrintln("[HOLO] CIPSTART cloudsocket.hologram.io:9999");
  if (!tcpConnectPlain("cloudsocket.hologram.io", 9999)) {
    buoyPrintln("[HOLO] CIPSTART FAILED");
    return false;
  }
  buoyPrintln("[HOLO] CIPSEND " + String(len) + " bytes");
  if (!tcpSendPlain(msg, len)) {
    buoyPrintln("[HOLO] CIPSEND FAILED");
    sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    return false;
  }

  // The CIP stack is in CIPRXGET=1 ("manual receive") mode globally so NTRIP
  // can pull RTCM bytes on demand. In that mode incoming TCP bytes are NOT
  // delivered as +IPD URCs — we see only a "+CIPRXGET: 1" hint, then have to
  // pull the data ourselves. Botletics' TCPavailable()/TCPread() wrap the
  // required AT+CIPRXGET=2,<n> sequence; reuse the same path RTCM uses.
  char respBuf[80];
  uint16_t respLen = 0;
  respBuf[0] = '\0';
  bool ok = false;
  const uint32_t deadline = millis() + 8000;
  while ((int32_t)(deadline - millis()) > 0) {
    uint16_t avail = TCPavailable();
    if (avail > 0) {
      uint16_t room = (uint16_t)(sizeof(respBuf) - 1 - respLen);
      if (room == 0) break;
      uint16_t want = (avail < room) ? avail : room;
      uint16_t got = TCPread((uint8_t *)(respBuf + respLen), want);
      respLen += got;
      respBuf[respLen] = '\0';
      if (strstr(respBuf, "[0,0]")) { ok = true; break; }
    } else {
      delay(100);
    }
  }

  buoyPrintln("[HOLO] response (" + String(respLen) + " bytes): '" + String(respBuf) + "'");

  sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  return ok;
}

void initialize_gnss_uart_f() {
  buoyPrintln("=== Initializing ZED-F9P via UART ===");
  buoyPrintln("TX_GPS pin: " + String (TX_GPS));
  buoyPrintln("RX_GPS pin: " + String (RX_GPS));
  
  const long baudRates[] = {115200, 115200, 115200, 115200, 115200};
  const int numRates = 5;
  
  for (int i = 0; i < numRates; i++) {
    buoyPrint("Trying ");
    buoyPrint(baudRates[i]);
    buoyPrintln(" baud...");
    
    gpsSerial.begin(baudRates[i], SERIAL_8N1, RX_GPS, TX_GPS);
    delay(1000);  // Give more time
    
    // Try to get any response
    buoyPrintln("  Attempting myGNSS.begin()...");
    
    if (myGNSS.begin(gpsSerial)) {
      buoyPrint("SUCCESS at ");
      buoyPrint(baudRates[i]);
      buoyPrintln(" baud!");
      gpsUARTOnline = true;
      
      buoyPrintln("GPS UART connected!");

      // Configure the UART we're talking to: accept RTCM3 in, send UBX out
      myGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
      myGNSS.setUART1Output(COM_TYPE_UBX);
      // Persist to flash so future boots don't depend on this reconfigure
      myGNSS.saveConfiguration();

      buoyPrintln("ZED-F9P: RTCM3 input enabled on UART1");
      return;
      
    } else {
      buoyPrintln("  Failed");
    }
    
    gpsSerial.end();
    delay(100);
  }
  
  buoyPrintln("ERROR: GPS UART failed at all baud rates!");
  gpsUARTOnline = false;
}

void initialize_ina228_f() {
  buoyPrintln("=== Initializing INA228 (I2C) ===");
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!ina228.begin()) {
    buoyPrintln("INA228 not found — power logging disabled");
    ina228Online = false;
    return;
  }

  ina228.setShunt(0.015, 10.0);  // Adafruit breakout: 15 mΩ, 10 A max
  ina228Online = true;
  buoyPrintln("INA228 OK");
}

void print_power_status_f() {
  if (!ina228Online) {
    return;
  }

  float currentMa = ina228.getCurrent_mA();
  float busV = ina228.getBusVoltage_V();
  float powerMw = ina228.getPower_mW();

  if (busV < 0.5f) {
    buoyPrintln("[PWR] (bench/USB — INA228 not on active battery rail)");
    return;
  }

  buoyPrint("[PWR] I=");
  buoyPrint(currentMa, 2);
  buoyPrint(" mA  V=");
  buoyPrint(busV, 3);
  buoyPrint(" V  P=");
  buoyPrint(powerMw, 1);
  buoyPrintln(" mW");
}


static void ntripAttemptFailed() {
  if (consecutiveNtripFailures < 255) {
    consecutiveNtripFailures++;
  }
  buoyPrint("[NTRIP] fail streak=");
  buoyPrintln(consecutiveNtripFailures);
}

void noteCellularActivity() {
  lastCellularActivity_ms = millis();
}

void invalidateDataPath(const __FlashStringHelper *reason) {
  buoyPrint("[DATA] invalidate: ");
  buoyPrintln(reason);

  if (ntripConnected) {
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    ntripConnected = false;
  }
  modem.invalidateCipStack();
  if (gprsEnabled) {
    modem.enableGPRS(false);
    gprsEnabled = false;
  }
  lastNTRIPAttempt = 0;
}

void refreshGprs_f(const __FlashStringHelper *reason) {
  static unsigned long lastRefreshMs = 0;

  if (millis() - lastRefreshMs < GPRS_REFRESH_COOLDOWN_MS) {
    buoyPrintln("[GPRS] refresh skipped (cooldown)");
    return;
  }
  lastRefreshMs = millis();

  buoyPrint("[GPRS] refresh: ");
  buoyPrintln(reason);

  invalidateDataPath(reason);
}

static bool modemLinkBegin() {
  modemSS.begin(115200, SERIAL_8N1, TX_MODEM, RX_MODEM);
  modemSS.println(F("AT+IPR=9600"));
  delay(1000);
  modemSS.begin(9600, SERIAL_8N1, TX_MODEM, RX_MODEM);
  return modem.begin(modemSS);
}

static void modemUartFlush() {
  const uint32_t deadline = millis() + 500;
  while ((int32_t)(deadline - millis()) > 0) {
    while (modemSS.available()) {
      (void)modemSS.read();
    }
    delay(10);
  }
}

// Match setup(): UART re-probe + boot-style network config (not CFUN=0 recover path).
static bool modemColdBootSequence() {
  modem.invalidateCipStack();
  modemUartFlush();
  if (!modemLinkBegin()) {
    buoyPrintln("[MODEM] UART/begin failed");
    return false;
  }
  modem.configureNetwork(false);
  return true;
}

static void modemPwrkeyPowerOff() {
  pinMode(BOTLETICS_PWRKEY, OUTPUT);
  digitalWrite(BOTLETICS_PWRKEY, HIGH);
  delay(100);
  digitalWrite(BOTLETICS_PWRKEY, LOW);
  delay(MODEM_PWRKEY_OFF_MS);
  digitalWrite(BOTLETICS_PWRKEY, HIGH);
}

bool modemHardRecover_f(const __FlashStringHelper *reason) {
  static unsigned long lastHardMs = 0;

  if (millis() - lastHardMs < MODEM_HARD_RECOVER_COOLDOWN_MS) {
    buoyPrintln("[MODEM] hard recover skipped (cooldown)");
    return false;
  }
  lastHardMs = millis();

  buoyPrint("[MODEM] hard recover: ");
  buoyPrintln(reason);

  invalidateDataPath(reason);
  networkConnected = false;
  consecutiveNtripFailures = 0;

  modem.sendCheckReply(F("AT+CIPSHUT"), F("SHUT OK"), 20000);
  delay(500);

  pinMode(MODEM_RST_PIN, OUTPUT);
  digitalWrite(MODEM_RST_PIN, LOW);
  delay(300);
  digitalWrite(MODEM_RST_PIN, HIGH);
  delay(MODEM_POST_RST_MS);

  if (modem.configureNetwork(true)) {
    buoyPrintln("[MODEM] hard recover done — waiting for CGREG");
    return true;
  }
  return false;
}

bool modemPowerCycleRecover_f(const __FlashStringHelper *reason,
                              bool bypassCooldown) {
  static unsigned long lastPowerCycleMs = 0;

  if (millis() - lastPowerCycleMs < MODEM_POWER_CYCLE_COOLDOWN_MS) {
    buoyPrintln("[MODEM] power cycle skipped (cooldown)");
    return false;
  }
  lastPowerCycleMs = millis();

  buoyPrint("[MODEM] power cycle: ");
  buoyPrintln(reason);

  invalidateDataPath(reason);
  networkConnected = false;
  consecutiveNtripFailures = 0;

  modem.sendCheckReply(F("AT+CIPSHUT"), F("SHUT OK"), 20000);
  delay(500);
  modem.sendCheckReply(F("AT+CPOWD=1"), F("NORMAL POWER DOWN"), 5000);
  delay(1000);
  modemPwrkeyPowerOff();
  buoyPrintln("[MODEM] modem off — settling");
  delay(MODEM_FULL_POWER_OFF_SETTLE_MS);

  buoyPrintln("[MODEM] PWRKEY power on...");
  pinMode(MODEM_RST_PIN, OUTPUT);
  digitalWrite(MODEM_RST_PIN, HIGH);
  modem.powerOn(BOTLETICS_PWRKEY);
  delay(MODEM_POST_POWER_ON_MS);

  if (!modemLinkBegin()) {
    buoyPrintln("[MODEM] begin failed after power cycle");
    return false;
  }

  if (modem.configureNetwork(true)) {
    buoyPrintln("[MODEM] power cycle done — waiting for CGREG");
    return true;
  }
  return false;
}

static bool s_modemRecoverNextPowerCycle = false;

void modemRecoverEscalated_f(const __FlashStringHelper *reason) {
  if (modemPowerCycleRecover_f(reason, false)) {
    return;
  }
  buoyPrintln("[MODEM] power cycle failed — trying RST recover");
  modemHardRecover_f(reason);
}

static void modemRecoverEscalationReset() {
  s_modemRecoverNextPowerCycle = false;
}

static bool cgregRegistered(uint8_t n) { return n == 1 || n == 5; }

static void modemRecoverEscalationMaybeReset(uint8_t cgregStat) {
  static unsigned long registeredSinceMs = 0;

  if (!cgregRegistered(cgregStat)) {
    registeredSinceMs = 0;
    return;
  }
  if (registeredSinceMs == 0) {
    registeredSinceMs = millis();
    return;
  }
  if (millis() - registeredSinceMs >= REGISTERED_STABLE_MS) {
    modemRecoverEscalationReset();
  }
}

static bool cellularLinkAlive() {
  return (ntripConnected &&
          (millis() - lastReceivedRTCM_ms < (long)CELLULAR_LINK_ALIVE_MS)) ||
         (lastCellularActivity_ms > 0 &&
          (millis() - lastCellularActivity_ms < CELLULAR_LINK_ALIVE_MS));
}

// CGREG can read 0 transiently while the CIP/NTRIP socket is still delivering RTCM.
static bool cgregLossConfirmed(uint8_t n) {
  static uint8_t badStreak = 0;

  if (cgregRegistered(n)) {
    badStreak = 0;
    return false;
  }
  if (cellularLinkAlive()) {
    return false;
  }
  badStreak++;
  return badStreak >= CGREG_BAD_STREAK_LIMIT;
}

void network_status_check_f() {
  static unsigned long lastCheckMs = 0;
  static unsigned long lastDiagMs = 0;
  static unsigned long lastCgregIgnoreLogMs = 0;
  const unsigned long checkIntervalMs = networkConnected ? NETWORK_RECHECK_MS : 5000UL;

  if (millis() - lastCheckMs < checkIntervalMs) return;
  lastCheckMs = millis();

  uint8_t rssi = modem.getRSSI();
  uint8_t n    = modem.getNetworkStatus();

  const __FlashStringHelper *label =
      (n == 1) ? F("home") :
      (n == 5) ? F("roaming") :
      (n == 2) ? F("searching") :
      (n == 3) ? F("denied") :
                 F("not registered");

  buoyPrint("[NET] CSQ=");
  buoyPrint(rssi);
  buoyPrint(" CGREG=");
  buoyPrint(n);
  buoyPrint(" (");
  buoyPrint(label);
  buoyPrintln(")");

  if (n == 0 && millis() - lastDiagMs > 30000) {
    lastDiagMs = millis();
    modem.printDiagnostics();
  }

  if (n == 1 || n == 5) {
    modemRecoverEscalationMaybeReset(n);
    if (!networkConnected) {
      networkConnected = true;
      buoyPrintln("[NET] connected");
    }
  } else if (networkConnected) {
    if (cellularLinkAlive()) {
      if (millis() - lastCgregIgnoreLogMs > 60000) {
        lastCgregIgnoreLogMs = millis();
        buoyPrint("[NET] CGREG=");
        buoyPrint(n);
        buoyPrintln(" (ignored, RTCM active)");
      }
    } else if (cgregLossConfirmed(n)) {
      buoyPrintln("[NET] registration lost");
      invalidateDataPath(F("CGREG lost"));
      networkConnected = false;
    }
  }

  // monitor_connection_health() returns immediately when !networkConnected, so
  // a modem stuck at CSQ=0 / CGREG=0 would never reach hard-recover otherwise.
  static unsigned long unregisteredSinceMs = 0;
  static unsigned long lastForcedFullCycleMs = 0;
  if (cgregRegistered(n)) {
    unregisteredSinceMs = 0;
  } else {
    if (unregisteredSinceMs == 0) {
      unregisteredSinceMs = millis();
    } else {
      const unsigned long unregDuration = millis() - unregisteredSinceMs;
      if (unregDuration >= MODEM_STUCK_FORCE_CYCLE_MS &&
          millis() - lastForcedFullCycleMs >= MODEM_STUCK_FORCE_CYCLE_MS) {
        lastForcedFullCycleMs = millis();
        buoyPrintln("[MODEM] prolonged unregistered — forced power cycle");
        modemPowerCycleRecover_f(F("prolonged unregistered"), true);
        unregisteredSinceMs = millis();
      } else {
        const unsigned long limit =
            (n == 2 && rssi != 0 && rssi != 99)
                ? UNREGISTERED_SEARCHING_GRACE_MS
                : UNREGISTERED_HARD_RECOVER_MS;
        if (unregDuration >= limit) {
          unregisteredSinceMs = millis();
          modemRecoverEscalated_f(F("registration timeout"));
        }
      }
    }
  }
}

void post_telemetry_f() {
  if (hologramDeviceKey[0] == '\0') {
    return;
  }
  if (!gprsEnabled) {
    return;
  }

#ifndef TELEMETRY_INTERVAL_MS
#define TELEMETRY_INTERVAL_MS 60000UL
#endif

  static unsigned long lastTelemetryMs = 0;
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryMs = millis();

  uint8_t fixType = 0;
  uint8_t carrSoln = 0;
  uint8_t sats = 0;
  bool havePvt = false;
  double lat = 0.0;
  double lon = 0.0;
  double altM = 0.0;

  if (gpsUARTOnline && ntripConnected && myGNSS.getPVT()) {
    havePvt = true;
    fixType = myGNSS.getFixType();
    carrSoln = myGNSS.getCarrierSolutionType();
    sats = myGNSS.getSIV();
    lat = myGNSS.getLatitude() / 10000000.0;
    lon = myGNSS.getLongitude() / 10000000.0;
    altM = myGNSS.getAltitudeMSL() / 1000.0;
  }

  const char *rtkStr =
      (carrSoln == 2) ? "FIXED" : (carrSoln == 1) ? "float" : "none";

  float busV = 0.0f;
  float powerMw = 0.0f;
  if (ina228Online) {
    busV = ina228.getBusVoltage_V();
    if (busV >= 0.5f) {
      powerMw = ina228.getPower_mW();
    } else {
      busV = 0.0f;
    }
  }

  uint8_t rssi = modem.getRSSI();
  int ntrip = ntripConnected ? 1 : 0;

  char body[300];
  int n;
  if (havePvt && fixType >= 2) {
    n = snprintf(
        body, sizeof(body),
        "{\"id\":\"%s\",\"fix\":%u,\"rtk\":\"%s\",\"sats\":%u,"
        "\"lat\":%.7f,\"lon\":%.7f,\"alt_m\":%.2f,"
        "\"bus_v\":%.3f,\"power_mw\":%.1f,\"rssi\":%u,\"ntrip\":%d}",
        imei, fixType, rtkStr, sats, lat, lon, altM, busV, powerMw, rssi, ntrip);
  } else {
    n = snprintf(
        body, sizeof(body),
        "{\"id\":\"%s\",\"fix\":%u,\"rtk\":\"%s\",\"sats\":%u,"
        "\"bus_v\":%.3f,\"power_mw\":%.1f,\"rssi\":%u,\"ntrip\":%d}",
        imei, fixType, rtkStr, sats, busV, powerMw, rssi, ntrip);
  }

  if (n <= 0 || n >= (int)sizeof(body)) {
    buoyPrintln("[TELEM] JSON build failed");
    return;
  }

  char inner[300];
  int innerLen = 0;
  for (int i = 0; i < n && innerLen < (int)sizeof(inner) - 2; i++) {
    char c = body[i];
    if (c == '"') {
      inner[innerLen++] = '\\';
      inner[innerLen++] = '"';
    } else {
      inner[innerLen++] = c;
    }
  }
  inner[innerLen] = '\0';

  char msg[380];
  n = snprintf(msg, sizeof(msg), "{\"k\":\"%s\",\"d\":\"%s\"}\n\n",
               hologramDeviceKey, inner);
  if (n <= 0 || n >= (int)sizeof(msg)) {
    return;
  }

  // Match the local-portal branch's CIP cleanup: closing the NTRIP socket
  // with just a 500 ms delay was not enough — the modem still treated the
  // CIP stack as busy when sendHologramCloudMessage tried to CIPSTART again,
  // producing intermittent [TELEM] Hologram failed even when the Hologram
  // device key is valid.
  bool wasNtrip = ntripConnected;
  if (wasNtrip) {
    buoyPrintln("[TELEM] closing NTRIP for Hologram send...");
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    ntripConnected = false;
    modem.invalidateCipStack();
    delay(2000);
    buoyPrintln("[TELEM] ensurePdpActive...");
    modem.ensurePdpActive();
  }

  buoyPrintln("[TELEM] Hologram cloud...");
  bool ok = modem.sendHologramCloudMessage(msg, (uint16_t)n);
  buoyPrintln(ok ? "[TELEM] Hologram OK" : "[TELEM] Hologram failed");
  if (ok) {
    noteCellularActivity();
  }
  if (wasNtrip) {
    lastNTRIPAttempt = 0;
  }
}

void enable_gprs_f() {
  if (!networkConnected || gprsEnabled) return;

  // Poor signal: try again next loop
  uint8_t rssi = modem.getRSSI();
  if (rssi == 0 || rssi == 99) {
    return;
  }

  // Network may have dropped between checks
  uint8_t n = modem.getNetworkStatus();
  if (n != 1 && n != 5) {
    networkConnected = false;
    return;
  }

  modem.enableGPRS(false);
  delay(2000);

  for (int attempt = 1; attempt <= 3; attempt++) {
    if (modem.enableGPRS(true)) {
      gprsEnabled = true;
      lastGprsEnabled_ms = millis();
      noteCellularActivity();
      consecutiveNtripFailures = 0;
      buoyPrintln("[GPRS] enabled");
      delay(2000);
      return;
    }
    buoyPrint("[GPRS] attempt "); buoyPrint(attempt); buoyPrintln("/3 failed");
    if (attempt < 3) delay(attempt * 5000);  // 5s, 10s
  }

  buoyPrintln("[GPRS] all attempts failed");
  delay(10000);
}

void beginNTRIPClient() {
  buoyPrint("[NTRIP] connecting to "); buoyPrint(casterHost);
  buoyPrint(":"); buoyPrintln(casterPort);

  if (!modem.tcpConnectPlain(casterHost, casterPort)) {
    buoyPrintln("[NTRIP] TCP connect failed");
    ntripAttemptFailed();
    return;
  }
  delay(1000);

  // Build HTTP GET request
  String ntripRequest = "GET /" + String(mountPoint) + " HTTP/1.0\r\n";
  ntripRequest += "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n";
  if (strlen(casterUser) > 0) {
    String creds = String(casterUser) + ":" + String(casterUserPW);
    base64 b;
    ntripRequest += "Authorization: Basic " + b.encode(creds) + "\r\n";
  } else {
    ntripRequest += "Accept: */*\r\n";
  }
  ntripRequest += "\r\n";

  if (!modem.tcpSendPlain(ntripRequest.c_str(), ntripRequest.length())) {
    buoyPrintln("[NTRIP] send failed");
    ntripAttemptFailed();
    return;
  }

  delay(2000);  // give caster time to reply

  uint16_t available = modem.TCPavailable();
  if (available == 0) {
    buoyPrintln("[NTRIP] no response from caster");
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    ntripAttemptFailed();
    return;
  }

  // Read first chunk to check status line
  uint8_t responseBuffer[128];
  uint16_t bytesToRead = min(available, (uint16_t)sizeof(responseBuffer));
  uint16_t bytesRead = modem.TCPread(responseBuffer, bytesToRead);
  if (bytesRead == 0) {
    delay(1000);
    bytesRead = modem.TCPread(responseBuffer, bytesToRead);
  }
  if (bytesRead == 0) {
    buoyPrintln("[NTRIP] empty read");
    ntripAttemptFailed();
    return;
  }

  // Null-terminate for substring search
  responseBuffer[(bytesRead < sizeof(responseBuffer)) ? bytesRead : sizeof(responseBuffer) - 1] = '\0';
  const char *rb = (const char *)responseBuffer;

  bool ok = strstr(rb, "ICY 200") || strstr(rb, "HTTP/1.0 200") || strstr(rb, "HTTP/1.1 200");
  bool unauth = strstr(rb, " 401") != nullptr;
  bool notfound = strstr(rb, " 404") != nullptr;

  if (ok) {
    buoyPrintln("[NTRIP] connected");
    ntripConnected = true;
    consecutiveNtripFailures = 0;
    lastReceivedRTCM_ms = millis();
    noteCellularActivity();
  } else if (unauth) {
    buoyPrintln("[NTRIP] 401 unauthorized");
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    ntripAttemptFailed();
  } else if (notfound) {
    buoyPrintln("[NTRIP] 404 mount not found");
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    ntripAttemptFailed();
  } else {
    buoyPrintln("[NTRIP] unrecognized response");
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    ntripAttemptFailed();
  }
}



void handleNTRIPData() {
  uint16_t available = modem.TCPavailable();

  if (available == 0) {
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      buoyPrintln("[NTRIP] RTCM timeout, disconnecting");
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      ntripConnected = false;
      ntripAttemptFailed();
    }
    return;
  }

  uint8_t rtcmBuffer[256];
  uint32_t totalSent = 0;
  int readCount = 0;

  while (modem.TCPavailable() > 0 && readCount < 40) {
    uint16_t bytesRead = modem.TCPread(rtcmBuffer, 250);
    if (bytesRead > 0 && gpsUARTOnline) {
      gpsSerial.write(rtcmBuffer, bytesRead);
      totalSent += bytesRead;
      readCount++;
    } else {
      break;
    }
  }

  if (totalSent > 0) {
    lastReceivedRTCM_ms = millis();
    noteCellularActivity();
  }

  // Periodic throughput line so we know RTCM is flowing without spamming every loop.
  static unsigned long lastRtcmReport = 0;
  static uint32_t bytesSinceReport = 0;
  bytesSinceReport += totalSent;
  if (millis() - lastRtcmReport > 10000) {
    lastRtcmReport = millis();
    buoyPrint("[RTCM] "); buoyPrint(bytesSinceReport);
    buoyPrint(" B/10s backlog="); buoyPrintln(modem.TCPavailable());
    bytesSinceReport = 0;
  }
}

void monitor_connection_health() {
  if (!networkConnected) return;

  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck < 30000) return;
  lastHealthCheck = millis();

  uint8_t status = modem.getNetworkStatus();
  uint8_t rssi   = modem.getRSSI();
  buoyPrint("[HEALTH] net="); buoyPrint(status);
  buoyPrint(" rssi="); buoyPrint(rssi);
  buoyPrint(" gprs="); buoyPrint(gprsEnabled ? 1 : 0);
  buoyPrint(" ntrip="); buoyPrint(ntripConnected ? 1 : 0);
  buoyPrint(" fail="); buoyPrintln(consecutiveNtripFailures);

  if (!cgregRegistered(status)) {
    if (cellularLinkAlive()) {
      static unsigned long lastIgnoreLogMs = 0;
      if (millis() - lastIgnoreLogMs > 60000) {
        lastIgnoreLogMs = millis();
        buoyPrint("[HEALTH] CGREG=");
        buoyPrint(status);
        buoyPrintln(" ignored (RTCM active)");
      }
    } else if (cgregLossConfirmed(status)) {
      buoyPrintln("[HEALTH] network lost");
      invalidateDataPath(F("CGREG health"));
      networkConnected = false;
      return;
    } else {
      return;
    }
  }

  // CNACT (wirelessConnStatus) is not the same path as NTRIP's CIP stack — CNACT can
  // read 0.0.0.0 while RTCM is flowing. Only refresh when there is no recent payload.
  if (gprsEnabled && !modem.wirelessConnStatus()) {
    if (!cellularLinkAlive()) {
      refreshGprs_f(F("PDP inactive"));
      return;
    }
  }

  if (gprsEnabled) {
    unsigned long activityMs = lastCellularActivity_ms;
    if (activityMs == 0) {
      activityMs = lastGprsEnabled_ms;
    }
    const bool expectingData =
        ntripConnected ||
        (millis() - lastNTRIPAttempt < ntripRetryInterval * 2UL);
    if (expectingData && activityMs > 0 &&
        millis() - activityMs > DATA_PATH_STALE_MS) {
      refreshGprs_f(F("data path stale"));
      return;
    }
  }

  if (!ntripConnected && gprsEnabled &&
      consecutiveNtripFailures >= NTRIP_FAILURES_BEFORE_HARD_RESET) {
    modemRecoverEscalated_f(F("NTRIP failures"));
  }
}

// DEBUG help
void printDebugStatus() {
  buoyPrintln("=== DEBUG STATUS ===");
  buoyPrint("networkConnected = ");
  buoyPrintln(networkConnected ? "true" : "false");
  buoyPrint("gprsEnabled = ");
  buoyPrintln(gprsEnabled ? "true" : "false");
  buoyPrint("gpsEnabled = ");
  buoyPrintln(gpsEnabled ? "true" : "false");
  buoyPrint("ntripConnected = ");
  buoyPrintln(ntripConnected ? "true" : "false");
  buoyPrint("lastNTRIPAttempt = ");
  buoyPrintln(lastNTRIPAttempt);
  buoyPrint("millis() = ");
  buoyPrintln(millis());
  buoyPrint("Time since last attempt = ");
  buoyPrintln(millis() - lastNTRIPAttempt);
  buoyPrintln("=== END DEBUG STATUS ===");
}

void updateStatusLED() {

  if (ntripConnected) {
    digitalWrite(STATUS_LED, HIGH);  // Solid = NTRIP active
    return;
  }
  
  digitalWrite(STATUS_LED, LOW);
}

void IRAM_ATTR shutdownISR() {
  shutdownRequested = true;
}

void gracefulShutdown() {
  buoyPrintln("\n=== SHUTDOWN REQUESTED ===");
  
  // Blink LED 3 times to confirm shutdown
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  
  // Close NTRIP/TCP connection
  if (ntripConnected) {
    buoyPrintln("Closing NTRIP connection...");
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    ntripConnected = false;
    delay(1000);
  }
  
  // Disable GPRS
  if (gprsEnabled) {
    buoyPrintln("Disabling GPRS...");
    modem.enableGPRS(false);
    gprsEnabled = false;
    delay(1000);
  }
  
  // Power down modem
  buoyPrintln("Powering down modem...");
  modem.sendCheckReply(F("AT+CPOWD=1"), F("NORMAL POWER DOWN"), 5000);
  delay(2000);
  
  // Turn off LED
  digitalWrite(STATUS_LED, LOW);
  
  // Configure wake-up source
  buoyPrintln("Entering deep sleep...");
  buoyPrintln("Press button again to wake up.");
  Serial.flush();  // Make sure message prints before sleep
  
  // Configure pin 0 to wake on LOW (button pressed)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  
  // Enter deep sleep // this saves more power but it turns off bluetooth
  // esp_deep_sleep_start();
  // enter light sleep // uses more power but keeps bluetooth on
  esp_light_sleep_start();
  
}

#endif