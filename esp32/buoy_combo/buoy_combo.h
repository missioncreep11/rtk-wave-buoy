/* buoy_combo.h */
#ifndef BUOY_COMBO_H
#define BUOY_COMBO_H

#include "BotleticsSIM7000.h"
#include <Adafruit_INA228.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <cstring>

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

// Hologram / US LTE CAT-M: 12 = AT&T/T-Mobile, 13 = Verizon
#ifndef LTE_CATM_BAND
#define LTE_CATM_BAND 12
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
  bool httpPostJson(const char *fullUrl, const char *body);
  bool tcpHttpPost(const char *host, uint16_t port, const char *path, const char *body);

  void printDiagnostics() {
    const uint16_t t = 3000;
    getReply(F("AT+CPIN?"), t);
    Serial.print(F("[DIAG] CPIN: "));
    Serial.println(replybuffer);
    getReply(F("AT+CFUN?"), t);
    Serial.print(F("[DIAG] CFUN: "));
    Serial.println(replybuffer);
    getReply(F("AT+CREG?"), t);
    Serial.print(F("[DIAG] CREG (circuit): "));
    Serial.println(replybuffer);
    getReply(F("AT+CGREG?"), t);
    Serial.print(F("[DIAG] CGREG (LTE data — used by [NET]): "));
    Serial.println(replybuffer);
    getReply(F("AT+CSQ"), t);
    Serial.print(F("[DIAG] CSQ: "));
    Serial.println(replybuffer);
    getReply(F("AT+CGATT?"), t);
    Serial.print(F("[DIAG] CGATT: "));
    Serial.println(replybuffer);
    getReply(F("AT+COPS?"), t);
    Serial.print(F("[DIAG] COPS: "));
    Serial.println(replybuffer);
    getReply(F("AT+CNACT?"), t);
    Serial.print(F("[DIAG] CNACT: "));
    Serial.println(replybuffer);
  }

  bool configureLteCatM() {
    Serial.print(F("[MODEM] LTE CAT-M only, band "));
    Serial.println(LTE_CATM_BAND);
    if (!setPreferredMode(38)) return false;
    if (!setPreferredLTEMode(1)) return false;
    if (!setOperatingBand("CAT-M", LTE_CATM_BAND)) return false;
    sendCheckReply(F("AT+CGREG=2"), ok_reply, 3000);
    return true;
  }

  void invalidateCipStack() { _cipStackUp = false; }

  bool cnactHasIp() {
    getReply(F("AT+CNACT?"), (uint16_t)3000);
    return strchr(replybuffer, '.') != nullptr && strstr(replybuffer, "0.0.0.0") == nullptr;
  }

  void resetHttpService() {
    sendCheckReply(F("AT+HTTPTERM"), ok_reply, 2000);
    sendCheckReply(F("AT+SHDISC"), ok_reply, 3000);
    sendCheckReply(F("AT+SHDISC"), ok_reply, 3000);
    delay(200);
  }

  bool syncClockFromGnss();
  bool prepareForHttps();
  bool sapbrUp();
  bool sapbrHttpsPost(const char *host, const char *path, const char *body);

  void configureNetwork() {
    setFunctionality(1);
    delay(1000);
    setNetworkSettings(F("hologram"));

    if (!configureLteCatM()) {
      Serial.println(F("[MODEM] WARN: LTE CAT-M band config failed"));
      SerialBT.println(F("[MODEM] WARN: LTE CAT-M band config failed"));
    }

    sendCheckReply(F("AT+CGATT=1"), ok_reply, 15000);
    sendCheckReply(F("AT+COPS=0"), ok_reply, 60000);
    sendCheckReply(F("AT+CMEE=2"), ok_reply, 3000);
    sendCheckReply(F("AT+CDNSCFG=1,\"8.8.8.8\",\"1.1.1.1\""), ok_reply, 5000);

    Serial.println(F("[MODEM] post-config diagnostics:"));
    SerialBT.println(F("[MODEM] post-config diagnostics:"));
    printDiagnostics();
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
void modemHardRecover_f(const __FlashStringHelper *reason);
void post_telemetry_f();
void printDebugStatus();
void updateStatusLED();

extern const char *telemetryUrl;
extern const char *telemetrySecret;
extern const char hologramDeviceKey[];
extern char imei[];

bool BuoyModem::syncClockFromGnss() {
  if (!gpsUARTOnline || !myGNSS.getPVT()) return false;
  if (myGNSS.getFixType() < 2) return false;

  const int y = myGNSS.getYear();
  const int mo = myGNSS.getMonth();
  const int d = myGNSS.getDay();
  const int h = myGNSS.getHour();
  const int mi = myGNSS.getMinute();
  const int s = myGNSS.getSecond();
  if (y < 2024 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31) return false;

  char cmd[56];
  snprintf(cmd, sizeof(cmd), "AT+CCLK=\"%02d/%02d/%02d,%02d:%02d:%02d+00\"",
           y % 100, mo, d, h, mi, s);
  if (sendCheckReply(cmd, ok_reply, 5000)) {
    Serial.println(F("[TELEM] modem clock synced from GPS"));
    return true;
  }
  return false;
}

bool BuoyModem::prepareForHttps() {
  static bool loggedFw = false;
  if (!loggedFw) {
    getReply(F("AT+CGMR"), (uint16_t)3000);
    Serial.print(F("[TELEM] modem FW: "));
    Serial.println(replybuffer);
    loggedFw = true;
  }

  // Close NTRIP socket (CIP) so the HTTP/SAPBR stack has the modem to itself.
  sendCheckReply(F("AT+CIPCLOSE"), ok_reply, 5000);
  invalidateCipStack();
  delay(1500);

  if (!ensurePdpActive()) {
    Serial.println(F("[TELEM] PDP lost after NTRIP close"));
    return false;
  }

  resetHttpService();
  syncClockFromGnss();
  return true;
}

// SAPBR is the legacy GPRS bearer the B03/B05 firmware needs for AT+HTTP* + AT+HTTPSSL.
// It is a separate bearer from CNACT and CIP, and on SIM7000 these can coexist.
bool BuoyModem::sapbrUp() {
  sendCheckReply(F("AT+SAPBR=3,1,\"Contype\",\"GPRS\""), ok_reply, 5000);
  sendCheckReply(F("AT+SAPBR=3,1,\"APN\",\"hologram\""), ok_reply, 5000);
  // Force public DNS — Hologram on T-Mobile sometimes does not push DNS via PCO.
  sendCheckReply(F("AT+SAPBR=3,1,\"DNS1\",\"8.8.8.8\""), ok_reply, 5000);
  sendCheckReply(F("AT+SAPBR=3,1,\"DNS2\",\"1.1.1.1\""), ok_reply, 5000);

  // SAPBR=2,1 returns: +SAPBR: 1,<status>,"<ip>"  ; status==1 means activated.
  getReply(F("AT+SAPBR=2,1"), (uint16_t)5000);
  bool active = (strstr(replybuffer, "+SAPBR: 1,1,") != nullptr);

  if (!active) {
    sendCheckReply(F("AT+SAPBR=1,1"), ok_reply, 60000);
    getReply(F("AT+SAPBR=2,1"), (uint16_t)5000);
    active = (strstr(replybuffer, "+SAPBR: 1,1,") != nullptr);
  }

  if (active) {
    Serial.print(F("[TELEM] SAPBR: "));
    Serial.println(replybuffer);
    // Explicit DNS for the legacy stack (separate from SAPBR DNS option).
    sendCheckReply(F("AT+CDNSCFG=\"8.8.8.8\",\"1.1.1.1\""), ok_reply, 5000);
  }
  return active;
}

bool BuoyModem::sapbrHttpsPost(const char *host, const char *path, const char *body) {
  if (!host || !path || !body) return false;

  const size_t bodyLen = strlen(body);

  if (!sapbrUp()) {
    Serial.println(F("[TELEM] SAPBR bearer down"));
    return false;
  }

  // Diagnostic DNS lookup on the SAPBR bearer.
  // CDNSGIP responds OK first, then "+CDNSGIP: 1,\"<host>\",\"<ip>\"" on a later line.
  {
    char dnsCmd[160];
    snprintf(dnsCmd, sizeof(dnsCmd), "AT+CDNSGIP=\"%s\"", host);
    flushInput();
    mySerial->println(dnsCmd);
    uint32_t dnsDeadline = millis() + 20000;
    bool gotDns = false;
    while ((int32_t)(dnsDeadline - millis()) > 0) {
      readline(2000);
      if (replybuffer[0] == 0) continue;
      if (strstr(replybuffer, "+CDNSGIP") || strstr(replybuffer, "ERROR")) {
        Serial.print(F("[TELEM] DNS: "));
        Serial.println(replybuffer);
        gotDns = true;
        break;
      }
    }
    if (!gotDns) Serial.println(F("[TELEM] DNS: (no response)"));
  }

  if (!sendCheckReply(F("AT+HTTPINIT"), ok_reply, 5000)) {
    Serial.print(F("[TELEM] HTTPINIT failed: "));
    Serial.println(replybuffer);
    return false;
  }

  sendCheckReply(F("AT+HTTPPARA=\"CID\",1"), ok_reply, 5000);
  sendCheckReply(F("AT+HTTPSSL=1"), ok_reply, 5000);
  sendCheckReply(F("AT+HTTPPARA=\"REDIR\",1"), ok_reply, 5000);

  char cmd[256];
  snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"https://%s%s\"", host, path);
  if (!sendCheckReply(cmd, ok_reply, 10000)) {
    Serial.print(F("[TELEM] URL failed: "));
    Serial.println(replybuffer);
    sendCheckReply(F("AT+HTTPTERM"), ok_reply, 5000);
    return false;
  }

  sendCheckReply(F("AT+HTTPPARA=\"CONTENT\",\"application/json\""), ok_reply, 5000);

  if (telemetrySecret && telemetrySecret[0] != '\0') {
    char ud[200];
    int p = snprintf(ud, sizeof(ud), "AT+HTTPPARA=\"USERDATA\",\"BUOY_SECRET: ");
    for (size_t i = 0; telemetrySecret[i] && p < (int)sizeof(ud) - 8; i++) {
      const char c = telemetrySecret[i];
      if (c == '"' || c == '\\') {
        ud[p++] = '\\';
      }
      ud[p++] = c;
    }
    p += snprintf(ud + p, sizeof(ud) - p, "\\r\\n\"");
    sendCheckReply(ud, ok_reply, 5000);
  }

  flushInput();
  mySerial->print(F("AT+HTTPDATA="));
  mySerial->print((unsigned)bodyLen);
  mySerial->println(F(",30000"));

  uint32_t deadline = millis() + 15000;
  bool ready = false;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(1000);
    if (strstr(replybuffer, "DOWNLOAD")) {
      ready = true;
      break;
    }
    if (strstr(replybuffer, "ERROR")) break;
  }
  if (!ready) {
    Serial.println(F("[TELEM] HTTPDATA prompt failed"));
    sendCheckReply(F("AT+HTTPTERM"), ok_reply, 5000);
    return false;
  }

  mySerial->write((const uint8_t *)body, bodyLen);

  deadline = millis() + 30000;
  bool uploaded = false;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(2000);
    if (replybuffer[0] == 0) continue;
    if (strcmp(replybuffer, "OK") == 0) {
      uploaded = true;
      break;
    }
    if (strstr(replybuffer, "ERROR")) break;
  }
  if (!uploaded) {
    Serial.println(F("[TELEM] HTTPDATA upload failed"));
    sendCheckReply(F("AT+HTTPTERM"), ok_reply, 5000);
    return false;
  }

  if (!sendCheckReply(F("AT+HTTPACTION=1"), ok_reply, 10000)) {
    Serial.print(F("[TELEM] HTTPACTION failed: "));
    Serial.println(replybuffer);
    sendCheckReply(F("AT+HTTPTERM"), ok_reply, 5000);
    return false;
  }

  bool ok = false;
  char lastLine[256] = {0};
  deadline = millis() + 120000;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(3000);
    if (replybuffer[0] == 0) continue;
    strncpy(lastLine, replybuffer, sizeof(lastLine) - 1);
    lastLine[sizeof(lastLine) - 1] = '\0';

    if (strstr(replybuffer, "HTTPACTION")) {
      int method = 0, status = 0, datalen = 0;
      const char *colon = strchr(replybuffer, ':');
      if (colon) {
        sscanf(colon + 1, " %d,%d,%d", &method, &status, &datalen);
      }
      if (status >= 200 && status < 300) {
        ok = true;
      } else if (status > 0) {
        Serial.print(F("[TELEM] HTTP status "));
        Serial.println(status);
      }
      break;
    }
    if (strstr(replybuffer, "ERROR") || strstr(replybuffer, "CME ERROR")) {
      break;
    }
  }

  if (!ok) {
    Serial.print(F("[TELEM] modem: "));
    Serial.println(lastLine[0] ? lastLine : "(no response)");
  }

  sendCheckReply(F("AT+HTTPTERM"), ok_reply, 5000);
  return ok;
}

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

bool BuoyModem::httpPostJson(const char *fullUrl, const char *body) {
  if (!fullUrl || !body || fullUrl[0] == '\0') return false;

  // tcp:// — raw HTTP/1.1 over a plain CIPSTART socket. Designed for ngrok TCP
  // tunnel → local_portal Flask. This is the ONLY HTTP path proven to work on
  // SIM7000A firmware B03 (the AT+SH*, AT+HTTP* + HTTPSSL, and AT+CAOPEN stacks
  // all fail on this firmware after an NTRIP socket has been open).
  //
  // URL format: tcp://<host>:<port>/<path>     e.g. tcp://0.tcp.ngrok.io:14723/api/ingest
  const char *p = fullUrl;
  bool ssl = false;
  bool plainTcp = false;
  if (strncmp(p, "tcp://", 6) == 0) {
    p += 6;
    plainTcp = true;
  } else if (strncmp(p, "https://", 8) == 0) {
    p += 8;
    ssl = true;
  } else if (strncmp(p, "http://", 7) == 0) {
    p += 7;
  }

  char host[96];
  uint16_t port = ssl ? 443 : 80;
  const char *path = "/";
  const char *slash = strchr(p, '/');
  size_t hostPortLen = slash ? (size_t)(slash - p) : strlen(p);
  if (hostPortLen == 0 || hostPortLen >= sizeof(host)) return false;
  memcpy(host, p, hostPortLen);
  host[hostPortLen] = '\0';
  if (slash) path = slash;

  // Pull explicit ":<port>" off the host string if present.
  char *colon = strchr(host, ':');
  if (colon) {
    *colon = '\0';
    long parsedPort = strtol(colon + 1, nullptr, 10);
    if (parsedPort > 0 && parsedPort <= 65535) port = (uint16_t)parsedPort;
  }

  if (plainTcp) {
    return tcpHttpPost(host, port, path, body);
  }

  if (!ssl) {
    char url[192];
    snprintf(url, sizeof(url), "http://%s%s", host, path);
    return postData("POST", url, body);
  }

  // SIM7000 firmware B03 only supports legacy SAPBR + AT+HTTPSSL=1 for HTTPS.
  // (AT+SH*, AT+CAOPEN, and CNACT-bound HTTPS were added in B05+.)
  if (!ensurePdpActive()) {
    Serial.println(F("[TELEM] PDP not active"));
    return false;
  }

  if (!prepareForHttps()) {
    return false;
  }

  return sapbrHttpsPost(host, path, body);
}

bool BuoyModem::tcpHttpPost(const char *host, uint16_t port, const char *path, const char *body) {
  if (!host || !path || !body) return false;

  const size_t bodyLen = strlen(body);

  // Compose request header. Host header uses the public ngrok address so any
  // future virtual-hosted server can route correctly; Flask itself ignores it.
  // BUOY_SECRET is sent only if configured.
  char req[768];
  int n;
  if (telemetrySecret && telemetrySecret[0]) {
    n = snprintf(req, sizeof(req),
                 "POST %s HTTP/1.1\r\n"
                 "Host: %s:%u\r\n"
                 "User-Agent: rtk-wave-buoy/1\r\n"
                 "Content-Type: application/json\r\n"
                 "BUOY_SECRET: %s\r\n"
                 "Connection: close\r\n"
                 "Content-Length: %u\r\n\r\n",
                 path, host, port, telemetrySecret, (unsigned)bodyLen);
  } else {
    n = snprintf(req, sizeof(req),
                 "POST %s HTTP/1.1\r\n"
                 "Host: %s:%u\r\n"
                 "User-Agent: rtk-wave-buoy/1\r\n"
                 "Content-Type: application/json\r\n"
                 "Connection: close\r\n"
                 "Content-Length: %u\r\n\r\n",
                 path, host, port, (unsigned)bodyLen);
  }
  if (n <= 0 || (size_t)n + bodyLen >= sizeof(req)) {
    Serial.println(F("[TELEM] request too large"));
    return false;
  }
  memcpy(req + n, body, bodyLen);
  const uint16_t total = (uint16_t)((size_t)n + bodyLen);

  if (!tcpConnectPlain(host, port)) {
    Serial.println(F("[TELEM] TCP connect failed"));
    return false;
  }

  bool sent = tcpSendPlain(req, total);
  if (!sent) {
    Serial.println(F("[TELEM] TCP send failed"));
    sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    return false;
  }

  // Give the server a moment to read the body and reply. We don't strictly need
  // to parse the response — Flask processes the POST as soon as Content-Length
  // bytes arrive, so SEND OK already proves the request was delivered.
  delay(1500);

  sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  return true;
}

bool BuoyModem::sendHologramCloudMessage(const char *msg, uint16_t len) {
  if (!tcpConnectPlain("cloudsocket.hologram.io", 9999)) return false;
  if (!tcpSendPlain(msg, len)) {
    sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    return false;
  }
  delay(1500);
  readline(3000);
  bool ok = (strstr(replybuffer, "[0,0]") != nullptr);
  sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
  return ok;
}

void initialize_gnss_uart_f() {
  Serial.println(F("=== Initializing ZED-F9P via UART ==="));
  SerialBT.println(F("=== Initializing ZED-F9P via UART ==="));
  Serial.print(F("TX_GPS pin: "));
  SerialBT.print(F("TX_GPS pin: "));
  Serial.println(TX_GPS);
  SerialBT.println(TX_GPS);
  Serial.print(F("RX_GPS pin: "));
  SerialBT.print(F("RX_GPS pin: "));
  Serial.println(RX_GPS);
  SerialBT.println(RX_GPS);
  
  const long baudRates[] = {115200, 115200, 115200, 115200, 115200};
  const int numRates = 5;
  
  for (int i = 0; i < numRates; i++) {
    Serial.print(F("Trying "));
    SerialBT.print(F("Trying "));
    Serial.print(baudRates[i]);
    SerialBT.print(baudRates[i]);
    Serial.println(F(" baud..."));
    SerialBT.println(F(" baud..."));
    
    gpsSerial.begin(baudRates[i], SERIAL_8N1, RX_GPS, TX_GPS);
    delay(1000);  // Give more time
    
    // Try to get any response
    Serial.println(F("  Attempting myGNSS.begin()..."));
    SerialBT.println(F("  Attempting myGNSS.begin()..."));
    
    if (myGNSS.begin(gpsSerial)) {
      Serial.print(F("SUCCESS at "));
      SerialBT.print(F("SUCCESS at "));
      Serial.print(baudRates[i]);
      SerialBT.print(baudRates[i]);
      Serial.println(F(" baud!"));
      SerialBT.println(F(" baud!"));
      gpsUARTOnline = true;
      
      Serial.println(F("GPS UART connected!"));
      SerialBT.println(F("GPS UART connected!"));

      // Configure the UART we're talking to: accept RTCM3 in, send UBX out
      myGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
      myGNSS.setUART1Output(COM_TYPE_UBX);
      // Persist to flash so future boots don't depend on this reconfigure
      myGNSS.saveConfiguration();

      Serial.println(F("ZED-F9P: RTCM3 input enabled on UART1"));
      SerialBT.println(F("ZED-F9P: RTCM3 input enabled on UART1"));
      return;
      
    } else {
      Serial.println(F("  Failed"));
      SerialBT.println(F("GPS UART connected!"));
    }
    
    gpsSerial.end();
    delay(100);
  }
  
  Serial.println(F("ERROR: GPS UART failed at all baud rates!"));
  SerialBT.println(F("ERROR: GPS UART failed at all baud rates!"));
  gpsUARTOnline = false;
}

void initialize_ina228_f() {
  Serial.println(F("=== Initializing INA228 (I2C) ==="));
  SerialBT.println(F("=== Initializing INA228 (I2C) ==="));
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!ina228.begin()) {
    Serial.println(F("INA228 not found — power logging disabled"));
    SerialBT.println(F("INA228 not found — power logging disabled"));
    ina228Online = false;
    return;
  }

  ina228.setShunt(0.015, 10.0);  // Adafruit breakout: 15 mΩ, 10 A max
  ina228Online = true;
  Serial.println(F("INA228 OK"));
  SerialBT.println(F("INA228 OK"));
}

void print_power_status_f() {
  if (!ina228Online) {
    return;
  }

  float currentMa = ina228.getCurrent_mA();
  float busV = ina228.getBusVoltage_V();
  float powerMw = ina228.getPower_mW();

  if (busV < 0.5f) {
    Serial.println(F("[PWR] (bench/USB — INA228 not on active battery rail)"));
    SerialBT.println(F("[PWR] (bench/USB — INA228 not on active battery rail)"));
    return;
  }

  Serial.print(F("[PWR] I="));
  Serial.print(currentMa, 2);
  Serial.print(F(" mA  V="));
  Serial.print(busV, 3);
  Serial.print(F(" V  P="));
  Serial.print(powerMw, 1);
  Serial.println(F(" mW"));

  SerialBT.print(F("[PWR] I="));
  SerialBT.print(currentMa, 2);
  SerialBT.print(F(" mA  V="));
  SerialBT.print(busV, 3);
  SerialBT.print(F(" V  P="));
  SerialBT.print(powerMw, 1);
  SerialBT.println(F(" mW"));
}


static void ntripAttemptFailed() {
  if (consecutiveNtripFailures < 255) {
    consecutiveNtripFailures++;
  }
  Serial.print(F("[NTRIP] fail streak="));
  Serial.println(consecutiveNtripFailures);
  SerialBT.print(F("[NTRIP] fail streak="));
  SerialBT.println(consecutiveNtripFailures);
}

void noteCellularActivity() {
  lastCellularActivity_ms = millis();
}

void invalidateDataPath(const __FlashStringHelper *reason) {
  Serial.print(F("[DATA] invalidate: "));
  Serial.println(reason);
  SerialBT.print(F("[DATA] invalidate: "));
  SerialBT.println(reason);

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
    Serial.println(F("[GPRS] refresh skipped (cooldown)"));
    SerialBT.println(F("[GPRS] refresh skipped (cooldown)"));
    return;
  }
  lastRefreshMs = millis();

  Serial.print(F("[GPRS] refresh: "));
  Serial.println(reason);
  SerialBT.print(F("[GPRS] refresh: "));
  SerialBT.println(reason);

  invalidateDataPath(reason);
}

void modemHardRecover_f(const __FlashStringHelper *reason) {
  static unsigned long lastHardMs = 0;

  if (millis() - lastHardMs < MODEM_HARD_RECOVER_COOLDOWN_MS) {
    Serial.println(F("[MODEM] hard recover skipped (cooldown)"));
    SerialBT.println(F("[MODEM] hard recover skipped (cooldown)"));
    return;
  }
  lastHardMs = millis();

  Serial.print(F("[MODEM] hard recover: "));
  Serial.println(reason);
  SerialBT.print(F("[MODEM] hard recover: "));
  SerialBT.println(reason);

  invalidateDataPath(reason);
  networkConnected = false;
  consecutiveNtripFailures = 0;

  modem.sendCheckReply(F("AT+CIPSHUT"), F("SHUT OK"), 20000);
  delay(500);

  pinMode(MODEM_RST_PIN, OUTPUT);
  digitalWrite(MODEM_RST_PIN, LOW);
  delay(300);
  digitalWrite(MODEM_RST_PIN, HIGH);
  delay(3000);

  modem.configureNetwork();
  Serial.println(F("[MODEM] hard recover done — waiting for CGREG"));
  SerialBT.println(F("[MODEM] hard recover done — waiting for CGREG"));
}

static bool cgregRegistered(uint8_t n) { return n == 1 || n == 5; }

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

  Serial.print(F("[NET] CSQ="));
  Serial.print(rssi);
  Serial.print(F(" CGREG="));
  Serial.print(n);
  Serial.print(F(" ("));
  Serial.print(label);
  Serial.println(F(")"));
  SerialBT.print(F("[NET] CSQ="));
  SerialBT.print(rssi);
  SerialBT.print(F(" CGREG="));
  SerialBT.print(n);
  SerialBT.print(F(" ("));
  SerialBT.print(label);
  SerialBT.println(F(")"));

  if (n == 0 && millis() - lastDiagMs > 30000) {
    lastDiagMs = millis();
    modem.printDiagnostics();
  }

  if (n == 1 || n == 5) {
    if (!networkConnected) {
      networkConnected = true;
      Serial.println(F("[NET] connected"));
      SerialBT.println(F("[NET] connected"));
    }
  } else if (networkConnected) {
    if (cellularLinkAlive()) {
      if (millis() - lastCgregIgnoreLogMs > 60000) {
        lastCgregIgnoreLogMs = millis();
        Serial.print(F("[NET] CGREG="));
        Serial.print(n);
        Serial.println(F(" (ignored, RTCM active)"));
        SerialBT.print(F("[NET] CGREG="));
        SerialBT.print(n);
        SerialBT.println(F(" (ignored, RTCM active)"));
      }
    } else if (cgregLossConfirmed(n)) {
      Serial.println(F("[NET] registration lost"));
      SerialBT.println(F("[NET] registration lost"));
      invalidateDataPath(F("CGREG lost"));
      networkConnected = false;
    }
  }
}

void post_telemetry_f() {
  const bool useHttp =
      (telemetryUrl != nullptr && telemetryUrl[0] != '\0');
  const bool useHologram = (hologramDeviceKey[0] != '\0');
  if (!useHttp && !useHologram) {
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

  if (gpsUARTOnline && myGNSS.getPVT()) {
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
    Serial.println(F("[TELEM] JSON build failed"));
    return;
  }

  // HTTP POST to local portal (ngrok). Pause NTRIP briefly — SIM7000 HTTP stack
  // often conflicts with an open CIP NTRIP socket.
  if (useHttp) {
    bool wasNtrip = ntripConnected;
    if (wasNtrip) {
      modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
      ntripConnected = false;
      modem.invalidateCipStack();
      delay(2000);
      modem.ensurePdpActive();
    }

    Serial.println(F("[TELEM] HTTP POST..."));
    if (modem.httpPostJson(telemetryUrl, body)) {
      Serial.println(F("[TELEM] POST OK"));
      SerialBT.println(F("[TELEM] POST OK"));
      noteCellularActivity();
    } else {
      Serial.println(F("[TELEM] POST failed"));
      SerialBT.println(F("[TELEM] POST failed"));
    }

    if (wasNtrip) {
      lastNTRIPAttempt = 0;
    }
    return;
  }

  // Optional: Hologram Cloud socket only (view in Hologram dashboard)
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

  bool wasNtrip = ntripConnected;
  if (wasNtrip) {
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    ntripConnected = false;
    delay(500);
  }

  Serial.println(F("[TELEM] Hologram cloud..."));
  bool ok = modem.sendHologramCloudMessage(msg, (uint16_t)n);
  Serial.println(ok ? F("[TELEM] Hologram OK") : F("[TELEM] Hologram failed"));
  SerialBT.println(ok ? F("[TELEM] Hologram OK") : F("[TELEM] Hologram failed"));
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
      Serial.println(F("[GPRS] enabled"));
      SerialBT.println(F("[GPRS] enabled"));
      delay(2000);
      return;
    }
    Serial.print(F("[GPRS] attempt "));    Serial.print(attempt);    Serial.println(F("/3 failed"));
    SerialBT.print(F("[GPRS] attempt "));  SerialBT.print(attempt);  SerialBT.println(F("/3 failed"));
    if (attempt < 3) delay(attempt * 5000);  // 5s, 10s
  }

  Serial.println(F("[GPRS] all attempts failed"));
  SerialBT.println(F("[GPRS] all attempts failed"));
  delay(10000);
}

void beginNTRIPClient() {
  Serial.print(F("[NTRIP] connecting to "));    Serial.print(casterHost);
  Serial.print(F(":"));                          Serial.println(casterPort);
  SerialBT.print(F("[NTRIP] connecting to "));  SerialBT.print(casterHost);
  SerialBT.print(F(":"));                        SerialBT.println(casterPort);

  if (!modem.tcpConnectPlain(casterHost, casterPort)) {
    Serial.println(F("[NTRIP] TCP connect failed"));
    SerialBT.println(F("[NTRIP] TCP connect failed"));
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
    Serial.println(F("[NTRIP] send failed"));
    SerialBT.println(F("[NTRIP] send failed"));
    ntripAttemptFailed();
    return;
  }

  delay(2000);  // give caster time to reply

  uint16_t available = modem.TCPavailable();
  if (available == 0) {
    Serial.println(F("[NTRIP] no response from caster"));
    SerialBT.println(F("[NTRIP] no response from caster"));
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
    Serial.println(F("[NTRIP] empty read"));
    SerialBT.println(F("[NTRIP] empty read"));
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
    Serial.println(F("[NTRIP] connected"));
    SerialBT.println(F("[NTRIP] connected"));
    ntripConnected = true;
    consecutiveNtripFailures = 0;
    lastReceivedRTCM_ms = millis();
    noteCellularActivity();
  } else if (unauth) {
    Serial.println(F("[NTRIP] 401 unauthorized"));
    SerialBT.println(F("[NTRIP] 401 unauthorized"));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    ntripAttemptFailed();
  } else if (notfound) {
    Serial.println(F("[NTRIP] 404 mount not found"));
    SerialBT.println(F("[NTRIP] 404 mount not found"));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    ntripAttemptFailed();
  } else {
    Serial.println(F("[NTRIP] unrecognized response"));
    SerialBT.println(F("[NTRIP] unrecognized response"));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
    ntripAttemptFailed();
  }
}



void handleNTRIPData() {
  uint16_t available = modem.TCPavailable();

  if (available == 0) {
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("[NTRIP] RTCM timeout, disconnecting"));
      SerialBT.println(F("[NTRIP] RTCM timeout, disconnecting"));
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
    Serial.print(F("[RTCM] "));    Serial.print(bytesSinceReport);
    Serial.print(F(" B/10s backlog=")); Serial.println(modem.TCPavailable());
    SerialBT.print(F("[RTCM] "));  SerialBT.print(bytesSinceReport);
    SerialBT.print(F(" B/10s backlog=")); SerialBT.println(modem.TCPavailable());
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
  Serial.print(F("[HEALTH] net="));    Serial.print(status);
  Serial.print(F(" rssi="));            Serial.print(rssi);
  Serial.print(F(" gprs="));            Serial.print(gprsEnabled ? 1 : 0);
  Serial.print(F(" ntrip="));           Serial.print(ntripConnected ? 1 : 0);
  Serial.print(F(" fail="));            Serial.println(consecutiveNtripFailures);
  SerialBT.print(F("[HEALTH] net="));  SerialBT.print(status);
  SerialBT.print(F(" rssi="));          SerialBT.print(rssi);
  SerialBT.print(F(" gprs="));          SerialBT.print(gprsEnabled ? 1 : 0);
  SerialBT.print(F(" ntrip="));         SerialBT.print(ntripConnected ? 1 : 0);
  SerialBT.print(F(" fail="));          SerialBT.println(consecutiveNtripFailures);

  if (!cgregRegistered(status)) {
    if (cellularLinkAlive()) {
      static unsigned long lastIgnoreLogMs = 0;
      if (millis() - lastIgnoreLogMs > 60000) {
        lastIgnoreLogMs = millis();
        Serial.print(F("[HEALTH] CGREG="));
        Serial.print(status);
        Serial.println(F(" ignored (RTCM active)"));
        SerialBT.print(F("[HEALTH] CGREG="));
        SerialBT.print(status);
        SerialBT.println(F(" ignored (RTCM active)"));
      }
    } else if (cgregLossConfirmed(status)) {
      Serial.println(F("[HEALTH] network lost"));
      SerialBT.println(F("[HEALTH] network lost"));
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
    modemHardRecover_f(F("NTRIP failures"));
  }
}

// DEBUG help
void printDebugStatus() {
  Serial.println(F("=== DEBUG STATUS ==="));
  SerialBT.println(F("=== DEBUG STATUS ==="));
  Serial.print(F("networkConnected = "));
  SerialBT.print(F("networkConnected = "));
  Serial.println(networkConnected ? "true" : "false");
  SerialBT.println(networkConnected ? "true" : "false");
  Serial.print(F("gprsEnabled = "));
  SerialBT.print(F("gprsEnabled = "));
  Serial.println(gprsEnabled ? "true" : "false");
  SerialBT.println(gprsEnabled ? "true" : "false");
  Serial.print(F("gpsEnabled = "));
  SerialBT.print(F("gpsEnabled = "));
  Serial.println(gpsEnabled ? "true" : "false");
  SerialBT.println(gpsEnabled ? "true" : "false");
  Serial.print(F("ntripConnected = "));
  SerialBT.print(F("ntripConnected = "));
  Serial.println(ntripConnected ? "true" : "false");
  SerialBT.println(ntripConnected ? "true" : "false");
  Serial.print(F("lastNTRIPAttempt = "));
  SerialBT.print(F("lastNTRIPAttempt = "));
  Serial.println(lastNTRIPAttempt);
  SerialBT.println(lastNTRIPAttempt);
  Serial.print(F("millis() = "));
  SerialBT.print(F("millis() = "));
  Serial.println(millis());
  SerialBT.println(millis());
  Serial.print(F("Time since last attempt = "));
  SerialBT.print(F("Time since last attempt = "));
  Serial.println(millis() - lastNTRIPAttempt);
  SerialBT.println(millis() - lastNTRIPAttempt);
  Serial.println(F("=== END DEBUG STATUS ==="));
  SerialBT.println(F("=== END DEBUG STATUS ==="));
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
  Serial.println(F("\n=== SHUTDOWN REQUESTED ==="));
  SerialBT.println(F("\n=== SHUTDOWN REQUESTED ==="));
  
  // Blink LED 3 times to confirm shutdown
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  
  // Close NTRIP/TCP connection
  if (ntripConnected) {
    Serial.println(F("Closing NTRIP connection..."));
    SerialBT.println(F("Closing NTRIP connection..."));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 5000);
    ntripConnected = false;
    delay(1000);
  }
  
  // Disable GPRS
  if (gprsEnabled) {
    Serial.println(F("Disabling GPRS..."));
    SerialBT.println(F("Disabling GPRS..."));
    modem.enableGPRS(false);
    gprsEnabled = false;
    delay(1000);
  }
  
  // Power down modem
  Serial.println(F("Powering down modem..."));
  SerialBT.println(F("Powering down modem..."));
  modem.sendCheckReply(F("AT+CPOWD=1"), F("NORMAL POWER DOWN"), 5000);
  delay(2000);
  
  // Turn off LED
  digitalWrite(STATUS_LED, LOW);
  
  // Configure wake-up source
  Serial.println(F("Entering deep sleep..."));
  SerialBT.println(F("Entering deep sleep..."));
  Serial.println(F("Press button again to wake up."));
  SerialBT.println(F("Press button again to wake up."));
  Serial.flush();  // Make sure message prints before sleep
  
  // Configure pin 0 to wake on LOW (button pressed)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  
  // Enter deep sleep // this saves more power but it turns off bluetooth
  // esp_deep_sleep_start();
  // enter light sleep // uses more power but keeps bluetooth on
  esp_light_sleep_start();
  
}

#endif