/* buoy_combo.ino */
// NTRIP casters use plain TCP (e.g. port 2101). BotleticsSIM7000.h defaults BOTLETICS_SSL=1.
#ifndef BOTLETICS_SSL
#define BOTLETICS_SSL 0
#endif

#include <Arduino.h>

// BLE - Nordic UART Service (NUS), compatible with nRF Connect
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_mac.h"

#define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> buoy
#define BLE_CHAR_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // buoy -> phone

BLEServer* pServer = NULL;
BLECharacteristic* pTxChar = NULL;
bool bleConnected = false;
char bleRxBuf[256] = {0};
volatile bool bleDataReady = false;

#include "secrets.h"
#include "buoy_combo.h"

// ========================================================
// Style: camelCase for functions and vars, m_camelCase for
// members, ALL_CAPS for macros, PascalCase for classes,
// 2 space indentation
// ========================================================

// BLE output helpers — echo to Serial and BLE simultaneously
void buoyPrint(const String& msg) {
  Serial.print(msg);
  if (bleConnected && pTxChar != NULL) {
    int len = msg.length();
    int offset = 0;
    while (offset < len) {
      int chunkSize = min(20, len - offset);
      String chunk = msg.substring(offset, offset + chunkSize);
      pTxChar->setValue(chunk.c_str());
      pTxChar->notify();
      offset += chunkSize;
      delay(10);
    }
  }
}
void buoyPrintln(const String& msg) { buoyPrint(msg + "\n"); }

#if !defined(HAS_HOLOGRAM_DEVICE_KEY)
const char hologramDeviceKey[] = "";
#endif

// Pin Definitions
#define SIMCOM_7000
#define BOTLETICS_PWRKEY 18
#define RST 5
#define TX_MODEM 17  // ESP32 TX1 to Modem RX
#define RX_MODEM 16  // ESP32 RX1 to Modem TX
// #define TX_GPS 12    // ESP32 TX2 to GPS RX
// #define RX_GPS 27    // ESP32 RX2 to GPS TX

// Global Objects
HardwareSerial modemSS(1);     // UART1 to modem
HardwareSerial gpsSerial(2);   // UART2 to GPS
BuoyModem modem;
SFE_UBLOX_GNSS myGNSS;
Adafruit_INA228 ina228;

// Flags
bool networkConnected = false;
bool gprsEnabled = false;
bool ntripConnected = false;
bool gpsUARTOnline = false;
bool ina228Online = false;
volatile bool shutdownRequested = false;

// Timing
unsigned long lastReceivedRtcmMs = 0;
int maxTimeBeforeHangupMs = 100000;
const unsigned long ntripRetryInterval = 30000;
unsigned long lastNtripAttempt = 0;
unsigned long lastCellularActivityMs = 0;
unsigned long lastGprsEnabledMs = 0;
uint8_t consecutiveNtripFailures = 0;
unsigned long lastFixStatusPrint = 0;
const unsigned long ggaIntervalMs = 10000;
unsigned long lastGgaSentMs = 0;

// NTRIP chunked stream buffer. Bytes pulled from the modem with block
// tcpRead() calls land here; the chunked decoder peels one byte at a time off
// the buffer exactly like the WiFi reference does off a WiFiClient. The buffer
// is seeded with any overflow captured while reading the HTTP response headers.
static uint8_t ntripStreamBuf[2048];
static uint16_t ntripStreamLen = 0;
static uint16_t ntripStreamPos = 0;

// Configuration
char imei[16] = {0};

// ============================================================
// BuoyModem method implementations
// ============================================================

// TODO: Make actual print msgs and replies more readable
void BuoyModem::printDiagnostics() {
  const uint16_t t = 3000;
  // KH -- SIM PIN status: READY if no password needed, SIM PIN if awaiting password
  getReply(F("AT+CPIN?"), t);
  buoyPrint(F("[DIAG] CPIN: "));
  buoyPrintln(replybuffer);
  // KH -- Modem Functionality Level: 0 if minimal, 1 if full (this is what you want)
  getReply(F("AT+CFUN?"), t);
  buoyPrint(F("[DIAG] CFUN: "));
  buoyPrintln(replybuffer);
  // KH -- network registration: 0 if not registered, 1 if registered at home, 5 if roaming
  getReply(F("AT+CREG?"), t);
  buoyPrint(F("[DIAG] CREG (circuit): "));
  buoyPrintln(replybuffer);
  // KH -- GPRS network registration: 0 if not registered, 1 if registered at home, 5 if roaming (typical)
  getReply(F("AT+CGREG?"), t);
  buoyPrint(F("[DIAG] CGREG (LTE data — used by [NET]): "));
  buoyPrintln(replybuffer);
  // KH -- Signal quality: 2-31 is proper
  getReply(F("AT+CSQ"), t);
  buoyPrint(F("[DIAG] CSQ: "));
  buoyPrintln(replybuffer);
  // KH -- GPRS attachment status: 0 if not attached, 1 if attached
  getReply(F("AT+CGATT?"), t);
  buoyPrint(F("[DIAG] CGATT: "));
  buoyPrintln(replybuffer);
  // KH -- Operator Selection: format is <mode> (0 is auto select, 1 is manual), <format> (0 is long alphanum, 1 is short),
  // <operator> (Verizon, AT&T, etc), <access technology> (7 for LTE)
  getReply(F("AT+COPS?"), t);
  buoyPrint(F("[DIAG] COPS: "));
  buoyPrintln(replybuffer);
  // KH -- APP Network Status: format is <mode> (0 for inactive, 1 for active), <access point name> (will give a real IP if connected)
  getReply(F("AT+CNACT?"), t);
  buoyPrint(F("[DIAG] CNACT: "));
  buoyPrintln(replybuffer);
}

// KH -- checks modem activity with AT+CPIN for a specified amount of time, pauses ESP32 for 500ms if not
bool BuoyModem::waitModemAtReady(uint32_t timeoutMs) {
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

// KH -- Returns true if modem is operating at full functionality via AT+CFUN, sets to full functionality if not,
// returns false if fails
bool BuoyModem::ensureRadioOn() {
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

// KH -- tries to set CAT-M band to preferred settings and tries fallbacks, returns true if successful
bool BuoyModem::applyLteCatMBandSettings() {
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

// KH -- if booting, applies CAT-M band settings and ensures radio is on. if recovering, checks 
// if functionality is minimal (required for band reconfig) beforehand
bool BuoyModem::configureLteCatM(bool afterRecover) {
  buoyPrintln("[MODEM] LTE CAT-M, band " + String(LTE_CATM_BAND) + (afterRecover ? " (recover)" : " (boot)"));
  if (afterRecover) {
    if (!sendCheckReply(F("AT+CFUN=0"), ok_reply, 10000) && ensureRadioOn()) {
      buoyPrintln("[MODEM] CFUN=0 failed (recover band config)");
      return false;
    }
    delay(1500);
  }

  const bool ok = applyLteCatMBandSettings();
  ensureRadioOn();
  return ok;
}

// KH -- forces CIP stack rebuild
void BuoyModem::invalidateCipStack() { m_CipStackUp = false; }

// KH -- First checks modem operational status, then configures functionality, provider, LTE band,
// GPS attachment, error reporting, and DNS
bool BuoyModem::configureNetwork(bool afterRecover) {
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

  ensureRadioOn();

  sendCheckReply(F("AT+CGATT=1"), ok_reply, 15000);
  sendCheckReply(F("AT+COPS=0"), ok_reply, 60000);
  sendCheckReply(F("AT+CMEE=2"), ok_reply, 3000);
  sendCheckReply(F("AT+CDNSCFG=1,\"8.8.8.8\",\"1.1.1.1\""), ok_reply, 5000);

  buoyPrintln("[MODEM] post-config diagnostics:");
  printDiagnostics();
  return ok;
}

// SAPBR is the legacy GPRS bearer the B03/B05 firmware needs for AT+HTTP* + AT+HTTPSSL.
// It is a separate bearer from CNACT and CIP, and on SIM7000 these can coexist.
bool BuoyModem::ensurePdpActive() {
  // GPRS uses AT+CNACT; activating again when already active returns "operation not allowed".
  if (wirelessConnStatus()) return true;
  if (!openWirelessConnection(true)) return false;
  return wirelessConnStatus();
}

// KH -- If the boolean flag returns false, the GPRS PDP (Packet Data Protocol) context is shut down, 
// an IP connection is started and contained, access point is brought up, then GPRS wireless, then 
// IP is checked for success 
bool BuoyModem::bringUpCipStack() {
  // The CIPSTART/CIPSEND/CIPRXGET stack is independent of CNACT.
  // Required order on SIM7000:
  //   CIPSHUT  -> IP INITIAL  (so CIPMUX/CIPRXGET can be set)
  //   CIPMUX=1   (multi-connection: 0=NTRIP RTCM, 1=Hologram telemetry)
  //   CIPRXGET=1
  //   CSTT="<apn>"
  //   CIICR
  //   CIFSR    (must return an IP literal)
  if (m_CipStackUp) return true;

  // CIPSHUT may deactivate CNACT; caller re-activates after.
  sendCheckReply(F("AT+CIPSHUT"), F("SHUT OK"), 20000);
  // SIM7000 only accepts CIPMUX/CIPRXGET in IP INITIAL; sending them in the
  // instant after CIPSHUT often returns ERROR, so let the stack settle first.
  delay(300);

  // CIPMUX / CIPRXGET only stick in IP INITIAL. A CIPRXGET=0 socket delivers
  // incoming bytes as +IPD URCs that get discarded by flushInput() — the
  // classic "connected but zero RTCM" failure — so this MUST be verified, not
  // tolerated.
  bool muxSet = false;
  for (int i = 0; i < 3 && !muxSet; i++) {
    muxSet = sendCheckReply(F("AT+CIPMUX=1"), ok_reply, 5000);
    if (!muxSet) delay(200);
  }
  sendCheckReply(F("AT+CIPRXGET=1"), ok_reply, 5000);

  getReply(F("AT+CIPMUX?"), (uint16_t)3000);
  char muxResp[24];
  snprintf(muxResp, sizeof(muxResp), "%s", replybuffer);
  const bool muxOk = (strstr(replybuffer, ": 1") != nullptr);

  getReply(F("AT+CIPRXGET?"), (uint16_t)3000);
  const bool rxGetOk = (strstr(replybuffer, ": 1") != nullptr);

  buoyPrint("[CIP] MUX="); buoyPrintln(muxResp);
  buoyPrint("[CIP] RXGET="); buoyPrintln(replybuffer);
  if (!muxOk || !rxGetOk) {
    buoyPrintln("[CIP] CIPMUX/CIPRXGET not applied — invalidating stack for retry");
    invalidateCipStack();
    return false;
  }

  // CSTT may already be set from a prior bring-up; tolerate ERROR.
  sendCheckReply(F("AT+CSTT=\"hologram\""), ok_reply, 10000);

  if (!sendCheckReply(F("AT+CIICR"), ok_reply, 60000)) return false;

  // CIFSR returns just the IP literal on success (no OK), or ERROR.
  getReply(F("AT+CIFSR"), (uint16_t)5000);
  if (strstr(replybuffer, "ERROR") || !strchr(replybuffer, '.')) return false;

  m_CipStackUp = true;
  return true;
}

bool BuoyModem::tcpConnectPlain(uint8_t linkId, const char *server, uint16_t port) {
  // Best-effort socket cleanup; ignore errors when no socket is open.
  getReply(F("AT+CIPCLOSE="), linkId, 2000);

  // Bring up the legacy CIPSTART stack. CIPSHUT inside may kill CNACT, restored below.
  if (!bringUpCipStack()) return false;
  ensurePdpActive();

  char cmd[128];
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=%u,\"TCP\",\"%s\",%u", linkId, server, port);
  if (!sendCheckReply(cmd, ok_reply, 60000)) return false;

  // CIPSTART returns OK first, then <id>, CONNECT OK / ALREADY CONNECT / CONNECT FAIL / STATE: PDP DEACT.
  uint32_t deadline = millis() + 75000;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(2000);
    if (replybuffer[0] == 0) continue;
    if (strstr(replybuffer, "CONNECT OK") || strstr(replybuffer, "ALREADY CONNECT")) return true;
    if (strstr(replybuffer, "PDP DEACT") || strstr(replybuffer, "CONNECT FAIL") ||
        strstr(replybuffer, "ERROR")) {
      invalidateCipStack();  // force re-bring-up next attempt
      return false;
    }
  }
  return false;
}

bool BuoyModem::tcpSendPlain(uint8_t linkId, const char *packet, uint16_t len) {
  flushInput();

  // AT+CIPSEND=<id>,<len> -- modem replies with ">" then waits for exactly <len> bytes.
  // Cannot use sendCheckReply() here: it expects "OK" but the response is "> ".
  mySerial->print(F("AT+CIPSEND="));
  mySerial->print(linkId);
  mySerial->print(',');
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
    mySerial->write(0x1B);
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

bool BuoyModem::tcpClosePlain(uint8_t linkId) {
  flushInput();
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%u", linkId);
  mySerial->println(cmd);

  // Multi-connection close replies "<id>,CLOSE OK" (or ERROR if link never opened).
  uint32_t deadline = millis() + 5000;
  while ((int32_t)(deadline - millis()) > 0) {
    readline(1000);
    if (replybuffer[0] == 0) continue;
    if (strstr(replybuffer, "CLOSE OK")) return true;
    if (strstr(replybuffer, "ERROR")) return true;  // already closed is fine
  }
  return false;
}

uint16_t BuoyModem::tcpAvailable(uint8_t linkId) {
  // CIPRXGET=4,<id> -> "+CIPRXGET: 4,<id>,<len>" then OK.
  uint16_t avail = 0;
  getReply(F("AT+CIPRXGET=4,"), linkId, 500);
  if (!parseReply(F("+CIPRXGET: 4,"), &avail, ',', 1)) return 0;
  return avail;
}

uint16_t BuoyModem::tcpRead(uint8_t linkId, uint8_t *buff, uint16_t len) {
  // CIPRXGET=2,<id>,<len> -> "+CIPRXGET: 2,<id>,<len>,<cnflen>" then raw data then OK.
  uint16_t avail = 0;
  getReply(F("AT+CIPRXGET=2,"), linkId, len, 1000);
  if (!parseReply(F("+CIPRXGET: 2,"), &avail, ',', 1)) return 0;
  if (avail > len) avail = len;

  // Pull exactly `avail` payload bytes straight off the UART. This bypasses the
  // 254-byte replybuffer cap in readRaw() AND the no-timeout readRaw() that can
  // return short and let the tail bytes get eaten by the trailing "OK" read.
  uint16_t got = 0;
  uint32_t deadline = millis() + 1000;
  while (got < avail && (int32_t)(deadline - millis()) > 0) {
    if (mySerial->available()) {
      buff[got++] = (uint8_t)mySerial->read();
    }
  }

  readline(1000);  // eat trailing "OK"
  return got;
}

// ============================================================
// NTRIP chunked-stream buffer
// Bytes are pulled from the modem in block tcpRead() calls so the per-byte AT
// CIPRXGET cost is amortized ~256x; the chunked decoder consumes them one at a
// time from the buffer with the same blocking-with-timeout semantics the WiFi
// reference gets for free from WiFiClient.
// ============================================================

void ntripStreamReset() {
  ntripStreamLen = 0;
  ntripStreamPos = 0;
}

void ntripStreamSeed(const uint8_t *data, uint16_t len) {
  ntripStreamLen = (len > (uint16_t)sizeof(ntripStreamBuf)) ? (uint16_t)sizeof(ntripStreamBuf) : len;
  ntripStreamPos = 0;
  if (ntripStreamLen > 0) {
    memcpy(ntripStreamBuf, data, ntripStreamLen);
  }
}

// Refill the buffer from the modem when it is fully drained. Returns the number
// of bytes newly buffered (0 = nothing available right now).
uint16_t ntripStreamRefill() {
  if (ntripStreamPos < ntripStreamLen) {
    return ntripStreamLen - ntripStreamPos;
  }
  ntripStreamLen = 0;
  ntripStreamPos = 0;

  uint16_t avail = modem.tcpAvailable(BuoyModem::LINK_NTRIP);
  if (avail == 0) return 0;

  uint16_t want = min(avail, (uint16_t)sizeof(ntripStreamBuf));
  uint16_t got = modem.tcpRead(BuoyModem::LINK_NTRIP, ntripStreamBuf, want);
  ntripStreamLen = got;
  return got;
}

// Blocking single-byte read from the stream buffer. Returns -1 on timeout.
// Keeps the chunked decoder in sync when a chunk-size line or payload straddles
// a refill boundary, exactly like the WiFi reference's readByteBlocking().
int ntripStreamReadByte(uint32_t timeoutMs) {
  uint32_t start = millis();
  while ((int32_t)(millis() - start) < (int32_t)timeoutMs) {
    if (ntripStreamPos < ntripStreamLen) {
      return ntripStreamBuf[ntripStreamPos++];
    }
    if (ntripStreamRefill() > 0) continue;
    delay(1);
  }
  return -1;
}

bool BuoyModem::sendHologramCloudMessage(const char *msg, uint16_t len) {
  buoyPrintln("[HOLO] CIPSTART cloudsocket.hologram.io:9999");
  if (!tcpConnectPlain(LINK_HOLOGRAM, "cloudsocket.hologram.io", 9999)) {
    buoyPrintln("[HOLO] CIPSTART FAILED");
    return false;
  }
  buoyPrintln("[HOLO] CIPSEND " + String(len) + " bytes");
  if (!tcpSendPlain(LINK_HOLOGRAM, msg, len)) {
    buoyPrintln("[HOLO] CIPSEND FAILED");
    tcpClosePlain(LINK_HOLOGRAM);
    return false;
  }

  // The CIP stack is in CIPRXGET=1 ("manual receive") mode globally so NTRIP
  // can pull RTCM bytes on demand. In that mode incoming TCP bytes are NOT
  // delivered as +IPD URCs — we see only a "+CIPRXGET: 1" hint, then have to
  // pull the data ourselves. tcpAvailable()/tcpRead() wrap the required
  // AT+CIPRXGET=4,<id> / AT+CIPRXGET=2,<id>,<len> sequence for link 1.
  char respBuf[80];
  uint16_t respLen = 0;
  respBuf[0] = '\0';
  bool ok = false;
  const uint32_t deadline = millis() + 8000;
  while ((int32_t)(deadline - millis()) > 0) {
    uint16_t avail = tcpAvailable(LINK_HOLOGRAM);
    if (avail > 0) {
      uint16_t room = (uint16_t)(sizeof(respBuf) - 1 - respLen);
      if (room == 0) break;
      uint16_t want = (avail < room) ? avail : room;
      uint16_t got = tcpRead(LINK_HOLOGRAM, (uint8_t *)(respBuf + respLen), want);
      respLen += got;
      respBuf[respLen] = '\0';
      if (strstr(respBuf, "[0,0]")) { ok = true; break; }
    } else {
      delay(100);
    }
  }

  buoyPrintln("[HOLO] response (" + String(respLen) + " bytes): '" + String(respBuf) + "'");

  tcpClosePlain(LINK_HOLOGRAM);
  return ok;
}

String BuoyModem::buildGGA() {
  // One PVT poll caches every NAV-PVT field used below; subsequent getters return
  // cached values instead of issuing eight separate UART polls that each block
  // RTCM injection to the F9P.
  myGNSS.getPVT();

  double lat = myGNSS.getLatitude()    / 10000000.0;
  double lon = myGNSS.getLongitude()   / 10000000.0;
  double alt = myGNSS.getAltitudeMSL() / 1000.0;
  uint8_t fix     = myGNSS.getFixType();
  uint8_t siv     = myGNSS.getSIV();
  uint8_t carrier = myGNSS.getCarrierSolutionType();
  uint8_t h = myGNSS.getHour();
  uint8_t m = myGNSS.getMinute();
  uint8_t s = myGNSS.getSecond();

  // GGA quality indicator: 0=no fix, 1=GPS, 4=RTK Fixed, 5=RTK Float
  int quality = 0;
  if (fix >= 2) {
    if      (carrier == 2) quality = 4;
    else if (carrier == 1) quality = 5;
    else                   quality = 1;
  }

  char latDir = (lat >= 0) ? 'N' : 'S';
  double absLat = fabs(lat);
  int latDeg    = (int)absLat;
  double latMin = (absLat - latDeg) * 60.0;

  char lonDir = (lon >= 0) ? 'E' : 'W';
  double absLon = fabs(lon);
  int lonDeg    = (int)absLon;
  double lonMin = (absLon - lonDeg) * 60.0;

  char body[128];
  snprintf(body, sizeof(body),
    "GPGGA,%02d%02d%02d.00,%02d%07.4f,%c,%03d%07.4f,%c,%d,%02d,1.0,%.2f,M,0.0,M,,",
    h, m, s, latDeg, latMin, latDir, lonDeg, lonMin, lonDir, quality, siv, alt);

  uint8_t checksum = 0;
  for (int i = 0; body[i]; i++) checksum ^= (uint8_t)body[i];

  char sentence[140];
  snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, checksum);
  return (String)sentence;
}

// ============================================================
// Free function implementations
// ============================================================

void initializeGnssUart() {
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

void initializeIna228() {
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

void printPowerStatus() {
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

void ntripAttemptFailed() {
  if (consecutiveNtripFailures < 255) {
    consecutiveNtripFailures++;
  }
  buoyPrint("[NTRIP] fail streak=");
  buoyPrintln(consecutiveNtripFailures);
}

void noteCellularActivity() {
  lastCellularActivityMs = millis();
}

void invalidateDataPath(const __FlashStringHelper *reason) {
  buoyPrint("[DATA] invalidate: ");
  buoyPrintln(reason);

  if (ntripConnected) {
    modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
    ntripConnected = false;
  }
  modem.invalidateCipStack();
  if (gprsEnabled) {
    modem.enableGPRS(false);
    gprsEnabled = false;
  }
  lastNtripAttempt = 0;
}

void refreshGprs(const __FlashStringHelper *reason) {
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

// Sends "AT" at the current modemSS baud and returns true if the modem answers
// (OK with echo off, or the echoed command with echo on).
bool modemRespondsAt(uint16_t timeoutMs) {
  while (modemSS.available()) modemSS.read();
  const uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(deadline - millis()) > 0) {
    modemSS.println(F("AT"));
    char line[32];
    uint8_t idx = 0;
    line[0] = '\0';
    const uint32_t lineDeadline = millis() + 500;
    while ((int32_t)(lineDeadline - millis()) > 0) {
      if (modemSS.available()) {
        char c = (char)modemSS.read();
        if (c == '\n') break;
        if (c != '\r' && idx < sizeof(line) - 1) line[idx++] = c;
      }
    }
    line[idx] = '\0';
    if (idx > 0 && (strstr(line, "OK") != nullptr || strstr(line, "AT") != nullptr)) {
      return true;
    }
  }
  return false;
}

bool modemLinkBegin() {
  // SIM7000 persists AT+IPR in NVRAM; after a session at 9600 it can cold-boot
  // at 9600 even though the factory default is 115200. Probe AT at 115200 first,
  // then fall back to 9600 and re-lock the modem to 115200 for this session.
  const unsigned long candidateRates[] = {115200UL, 9600UL};
  for (uint8_t i = 0; i < 2; i++) {
    const unsigned long rate = candidateRates[i];
    modemSS.begin(rate, SERIAL_8N1, TX_MODEM, RX_MODEM);
    delay(200);
    if (modemRespondsAt(2000)) {
      if (rate != 115200UL) {
        // Modem was at 9600 — switch it to 115200 for this session.
        modemSS.println(F("AT+IPR=115200"));
        delay(1000);
        modemSS.begin(115200UL, SERIAL_8N1, TX_MODEM, RX_MODEM);
        delay(100);
        if (!modemRespondsAt(2000)) {
          return false;
        }
      }
      return modem.begin(modemSS);
    }
  }
  return false;
}

void modemUartFlush() {
  const uint32_t deadline = millis() + 500;
  while ((int32_t)(deadline - millis()) > 0) {
    while (modemSS.available()) {
      (void)modemSS.read();
    }
    delay(10);
  }
}

// Match setup(): UART re-probe + boot-style network config (not CFUN=0 recover path).
void modemPwrkeyPowerOff() {
  pinMode(BOTLETICS_PWRKEY, OUTPUT);
  digitalWrite(BOTLETICS_PWRKEY, HIGH);
  delay(100);
  digitalWrite(BOTLETICS_PWRKEY, LOW);
  delay(MODEM_PWRKEY_OFF_MS);
  digitalWrite(BOTLETICS_PWRKEY, HIGH);
}

bool modemHardRecover(const __FlashStringHelper *reason) {
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

bool modemPowerCycleRecover(const __FlashStringHelper *reason,
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

void modemRecoverEscalated(const __FlashStringHelper *reason) {
  if (modemPowerCycleRecover(reason, false)) {
    return;
  }
  buoyPrintln("[MODEM] power cycle failed — trying RST recover");
  modemHardRecover(reason);
}

bool cgregRegistered(uint8_t n) { return n == 1 || n == 5; }

bool cellularLinkAlive() {
  return (ntripConnected &&
          (millis() - lastReceivedRtcmMs < (long)CELLULAR_LINK_ALIVE_MS)) ||
         (lastCellularActivityMs > 0 &&
          (millis() - lastCellularActivityMs < CELLULAR_LINK_ALIVE_MS));
}

// CGREG can read 0 transiently while the CIP/NTRIP socket is still delivering RTCM.
bool cgregLossConfirmed(uint8_t n) {
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

void networkStatusCheck() {
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

  // monitorConnectionHealth() returns immediately when !networkConnected, so
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
        modemPowerCycleRecover(F("prolonged unregistered"), true);
        unregisteredSinceMs = millis();
      } else {
        const unsigned long limit =
            (n == 2 && rssi != 0 && rssi != 99)
                ? UNREGISTERED_SEARCHING_GRACE_MS
                : UNREGISTERED_HARD_RECOVER_MS;
        if (unregDuration >= limit) {
          unregisteredSinceMs = millis();
          modemRecoverEscalated(F("registration timeout"));
        }
      }
    }
  }
}

void postTelemetry() {
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

  // CIPMUX=1 keeps the NTRIP socket (link 0) open while Hologram (link 1)
  // sends telemetry. No more closing NTRIP / invalidating the CIP stack here.
  buoyPrintln("[TELEM] Hologram cloud...");
  bool ok = modem.sendHologramCloudMessage(msg, (uint16_t)n);
  buoyPrintln(ok ? "[TELEM] Hologram OK" : "[TELEM] Hologram failed");
  if (ok) {
    noteCellularActivity();
  }
  if (ntripConnected) {
    lastNtripAttempt = 0;
  }
}

void setupGprs() {
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
  for (int i = 0; i < 5; i++)
  {
    delay(1000);
    if (modem.sendCheckReply(F("AT+CGATT?"), F("+CGATT: 0"), 5000))
    {
      buoyPrint("[GPRS] detached on try ");
      buoyPrintln(i + 1);
      break;
    }
  }

  modem.enableGPRS(true);
  for (int attempt = 0; attempt < 5; attempt++) 
  {
    if (modem.sendCheckReply(F("AT+CGATT?"), F("+CGATT: 1"), 5000)) 
    {
      gprsEnabled = true;
      lastGprsEnabledMs = millis();
      noteCellularActivity();
      consecutiveNtripFailures = 0;
      buoyPrintln("[GPRS] enabled");
      delay(1000);
      return;
    }
    buoyPrint("[GPRS] attempt "); buoyPrint(attempt + 1); buoyPrintln("/5 failed");
    if ((attempt + 1) < 5) delay(1000);  // 1s between polls
  }

  buoyPrintln("[GPRS] all attempts failed");
  delay(10000);
}

void beginNTRIPClient() {
  buoyPrint("[NTRIP] connecting to "); buoyPrint(casterHost);
  buoyPrint(":"); buoyPrintln(casterPort);

  if (!modem.tcpConnectPlain(BuoyModem::LINK_NTRIP, casterHost, casterPort)) {
    buoyPrintln("[NTRIP] TCP connect failed");
    ntripAttemptFailed();
    return;
  }
  delay(1000);

  // Build HTTP GET request
  // String ntripRequest = "GET /" + String(mountPoint) + " HTTP/1.1\r\n";
  // ntripRequest += "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n";
  // HTTP/1.1 with Ntrip-Version: Ntrip/2.0 — Polaris uses chunked transfer encoding
  const int SERVER_BUFFER_SIZE = 512;
  char serverRequest[SERVER_BUFFER_SIZE];
  snprintf(serverRequest, SERVER_BUFFER_SIZE,
    "GET /%s HTTP/1.1\r\n"
    "Host: %s\r\n"
    "Ntrip-Version: Ntrip/2.0\r\n"
    "User-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
    mountPoint, casterHost);
  if (strlen(casterUser) > 0) {
    String creds = String(casterUser) + ":" + String(casterUserPW);
    base64 b;
    String printCreds = "Authorization: Basic " + b.encode(creds) + "\r\n";
    strncat(serverRequest, printCreds.c_str(), SERVER_BUFFER_SIZE - strlen(serverRequest) - 1);
  } else {
    strncat(serverRequest, "Accept: */*\r\n", SERVER_BUFFER_SIZE - strlen(serverRequest) - 1);
  }
  strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE - strlen(serverRequest) - 1);

  if (!modem.tcpSendPlain(BuoyModem::LINK_NTRIP, serverRequest, strlen(serverRequest))) {
    buoyPrintln("[NTRIP] send failed");
    ntripAttemptFailed();
    return;
  }

  delay(2000);  // give caster time to reply

  uint16_t available = modem.tcpAvailable(BuoyModem::LINK_NTRIP);
  if (available == 0) {
    buoyPrintln("[NTRIP] no response from caster");
    modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
    ntripAttemptFailed();
    return;
  }

  // Read response headers in fast block reads until \r\n\r\n
  char responseBuffer[1024];
  uint16_t responseLen = 0;
  bool foundHeaderEnd = false;
  uint32_t deadline = millis() + 10000; // 10s timeout
  ntripStreamReset();

  while (millis() < deadline && !foundHeaderEnd) {
    uint16_t available = modem.tcpAvailable(BuoyModem::LINK_NTRIP);
    if (available > 0) {
      uint16_t toRead = min(available, (uint16_t)(sizeof(responseBuffer) - 1 - responseLen));
      if (toRead > 0) {
        uint16_t got = modem.tcpRead(BuoyModem::LINK_NTRIP, (uint8_t *)(responseBuffer + responseLen), toRead);
        if (got > 0) {
          responseLen += got;
          responseBuffer[responseLen] = '\0';

          // Check if we've received the full HTTP headers (\r\n\r\n)
          const char *headerEnd = strstr(responseBuffer, "\r\n\r\n");
          if (headerEnd != nullptr) {
            foundHeaderEnd = true;
            // Seed the chunked decoder with any body overflow that arrived in
            // the same read as the headers — without this the first chunk
            // header is lost and the decoder desyncs immediately.
            const uint8_t *payloadStart = (const uint8_t *)(headerEnd + 4);
            uint16_t overflow = responseLen - (payloadStart - (const uint8_t *)responseBuffer);
            if (overflow > 0) {
              ntripStreamSeed(payloadStart, overflow);
              buoyPrintln("[NTRIP] Handoff: " + String(overflow) + " bytes preloaded");
            }
            break;
          }
        }
      }
    } else {
      delay(5);
    }
  }

  if (!foundHeaderEnd) {
    buoyPrintln("[NTRIP] Failed to read full HTTP headers / timeout");
    modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
    ntripAttemptFailed();
    return;
  }

  bool ok = strstr(responseBuffer, "ICY 200") != nullptr || 
            strstr(responseBuffer, "HTTP/1.0 200") != nullptr || 
            strstr(responseBuffer, "HTTP/1.1 200") != nullptr;

  bool unauth = strstr(responseBuffer, " 401") != nullptr;
  bool notfound = strstr(responseBuffer, " 404") != nullptr;

  if (ok) {
    buoyPrintln("[NTRIP] connected");
    ntripConnected = true;
    consecutiveNtripFailures = 0;
    lastReceivedRtcmMs = millis();
    noteCellularActivity();

    String gga = modem.buildGGA();
    modem.tcpSendPlain(BuoyModem::LINK_NTRIP, gga.c_str(), gga.length());
    lastGgaSentMs = millis();
    buoyPrintln("[NTRIP] GGA Sent");
  } else if (unauth) {
    buoyPrintln("[NTRIP] 401 unauthorized");
    modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
    ntripAttemptFailed();
  } else if (notfound) {
    buoyPrintln("[NTRIP] 404 mount not found");
    modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
    ntripAttemptFailed();
  } else {
    buoyPrintln("[NTRIP] unrecognized response");
    modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
    ntripAttemptFailed();
  }
}

void handleNTRIPData() {
  // If not connected, or if no data is expected, just send GGA and return.
  if (!ntripConnected) {
    if (millis() - lastGgaSentMs > ggaIntervalMs) {
      String gga = modem.buildGGA();
      modem.tcpSendPlain(BuoyModem::LINK_NTRIP, gga.c_str(), gga.length());
      lastGgaSentMs = millis();
      buoyPrintln("[NTRIP] GGA Sent (idle)");
    }
    return;
  }

  // Handle NTRIP data (receives RTCM and sends to GPS via UART).
  // Bytes come from ntripStreamBuf, refilled with block tcpRead() calls so the
  // AT CIPRXGET cost is amortized instead of one round-trip per byte.
  // Process as many chunks as possible within a reasonable timeframe (e.g., 200ms).
  uint32_t startTime = millis();
  uint16_t forwarded = 0;

  while ((int32_t)(millis() - startTime) < 200) {

    // Check for GGA interval
    if (millis() - lastGgaSentMs > ggaIntervalMs) {
      String gga = modem.buildGGA();
      modem.tcpSendPlain(BuoyModem::LINK_NTRIP, gga.c_str(), gga.length());
      lastGgaSentMs = millis();
      buoyPrintln("[NTRIP] GGA Sent");
    }

    // No data buffered locally and nothing waiting in the modem socket.
    if (ntripStreamPos >= ntripStreamLen &&
        modem.tcpAvailable(BuoyModem::LINK_NTRIP) == 0) {
      // If no data, check for timeout
      if (millis() - lastReceivedRtcmMs > maxTimeBeforeHangupMs) {
        buoyPrintln("[NTRIP] RTCM timeout, disconnecting");
        modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
        ntripConnected = false;
        ntripAttemptFailed();
        return;
      }
      break; // No more data to read for now
    }

    // --- Chunked Stream Decoder ---
    // Read hex chunk-size line, terminated by CRLF.
    // Polaris paces corrections in ~1Hz bursts; the 5s blocking timeout matches
    // the proven WiFi reference so a normal inter-chunk gap doesn't drop us.
    char chunkSizeBuf[12];
    int idx = 0;
    bool sizeReadOk = true;
    while (idx < (int)sizeof(chunkSizeBuf) - 1) {
      int b = ntripStreamReadByte(5000); // 5s timeout (matches WiFi reference)
      if (b < 0) { sizeReadOk = false; break; }
      if (b == '\n') break;
      if (b != '\r') chunkSizeBuf[idx++] = (char)b;
    }
    chunkSizeBuf[idx] = '\0';

    if (!sizeReadOk) {
      buoyPrintln("[NTRIP] timeout reading chunk size — dropping socket");
      modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
      ntripConnected = false;
      return;
    }

    long chunkSize = strtol(chunkSizeBuf, NULL, 16);

    if (chunkSize == 0) {
      buoyPrintln("[NTRIP] Chunked stream ended (0-size)");
      modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
      ntripConnected = false;
      return;
    }

    if (chunkSize < 0 || chunkSize > 4096) {
      buoyPrintln("[NTRIP] Oversized chunk, likely desync (chunkSize=" + String(chunkSize) + ")");
      modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
      ntripConnected = false;
      return;
    }

    // Consume exactly chunkSize bytes
    long remaining = chunkSize;
    uint8_t buffer[128];
    while (remaining > 0) {
      int want = (remaining > (long)sizeof(buffer)) ? (int)sizeof(buffer) : (int)remaining;
      int got = 0;
      while (got < want) {
        int b = ntripStreamReadByte(5000); // 5s timeout (matches WiFi reference)
        if (b < 0) {
          buoyPrintln("[NTRIP] Payload read failed");
          modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
          ntripConnected = false;
          return;
        }
        buffer[got++] = (uint8_t)b;
      }

      if (gpsUARTOnline) gpsSerial.write(buffer, got);
      forwarded += got;
      remaining -= got;
      lastReceivedRtcmMs = millis();
      noteCellularActivity();
    }

    // Consume trailing CRLF
    int b1 = ntripStreamReadByte(5000); // 5s timeout (matches WiFi reference)
    int b2 = ntripStreamReadByte(5000); // 5s timeout (matches WiFi reference)
    if (b1 != '\r' || b2 != '\n') {
      buoyPrintln("[NTRIP] Trailing CRLF mismatch");
      modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
      ntripConnected = false;
      return;
    }
  }

  // Throughput + backlog telemetry so a healthy stream is distinguishable from
  // a desync from a dead socket without guessing.
  static unsigned long lastRtcmReport = 0;
  static uint32_t bytesSinceReport = 0;
  bytesSinceReport += forwarded;
  if (millis() - lastRtcmReport > 10000) {
    lastRtcmReport = millis();
    buoyPrint("[RTCM] "); buoyPrint(bytesSinceReport);
    buoyPrint(" B/10s backlog="); buoyPrintln(modem.tcpAvailable(BuoyModem::LINK_NTRIP));
    bytesSinceReport = 0;
  }
  if (forwarded > 0) {
    buoyPrint("[RTCM] to ZED: "); buoyPrintln(forwarded);
  }
}

void monitorConnectionHealth() {
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
      refreshGprs(F("PDP inactive"));
      return;
    }
  }

  if (gprsEnabled) {
    unsigned long activityMs = lastCellularActivityMs;
    if (activityMs == 0) {
      activityMs = lastGprsEnabledMs;
    }
    const bool expectingData =
        ntripConnected ||
        (millis() - lastNtripAttempt < ntripRetryInterval * 2UL);
    if (expectingData && activityMs > 0 &&
        millis() - activityMs > DATA_PATH_STALE_MS) {
      refreshGprs(F("data path stale"));
      return;
    }
  }

  if (!ntripConnected && gprsEnabled &&
      consecutiveNtripFailures >= NTRIP_FAILURES_BEFORE_HARD_RESET) {
    modemRecoverEscalated(F("NTRIP failures"));
  }
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

  // reset shutdown flag
  shutdownRequested = false;
  
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
    modem.tcpClosePlain(BuoyModem::LINK_NTRIP);
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
  buoyPrintln("Entering sleep...");
  buoyPrintln("Press button again to wake up.");
  Serial.flush();  // Make sure message prints before sleep
  
  // Configure pin 0 to wake on LOW (button pressed)
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  
  // Enter deep sleep // this saves more power but it turns off bluetooth
  // esp_deep_sleep_start();
  // enter light sleep // uses more power but keeps bluetooth on
  esp_light_sleep_start();
  
}

// ============================================================
// BLE helpers
// ============================================================

// ============================================================
// BLE server callbacks
// ============================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    buoyPrintln("BLE connected.");
  }
  void onDisconnect(BLEServer* pServer) {
    bleConnected = false;
    buoyPrintln("BLE disconnected - restarting advertising");
    pServer->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    strncpy(bleRxBuf, pChar->getValue().c_str(), sizeof(bleRxBuf) - 1);
    bleRxBuf[sizeof(bleRxBuf) - 1] = '\0';
    bleDataReady = true;
  }
};

// ============================================================
// Print GPS status over BLE
// ============================================================
void broadcastGPS() {
  if (!bleConnected) return;
  if (!myGNSS.getPVT()) return;
  
  float lat     = myGNSS.getLatitude()        / 10000000.0;
  float lon     = myGNSS.getLongitude()       / 10000000.0;
  float alt     = myGNSS.getAltitudeMSL()     / 1000.0;
  float hAcc    = myGNSS.getHorizontalAccEst()/ 1000.0;
  uint8_t sats  = myGNSS.getSIV();
  uint8_t carrier = myGNSS.getCarrierSolutionType();
  String rtk = (carrier == 2) ? "FIX" : (carrier == 1) ? "FLOAT" : "NONE";
  buoyPrintln("GPS " + String(lat, 7) + " " + String(lon, 7) + "\n"
              "alt=" + String(alt, 1) + "m\n" + 
              "RTK=" + rtk + "\n" +
              "horizAcc=" + String(hAcc, 3) + "m\n" + 
              "sats=" + String(sats));
}

// ============================================================
// setup() and loop()
// ============================================================

void setup() {
  // USB Debug Serial
  Serial.begin(115200);

  // BLE NUS init — name includes MAC suffix for multi-buoy ID
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char bleName[20];
  snprintf(bleName, sizeof(bleName), "RTK-Buoy-%02X%02X", mac[4], mac[5]);

  BLEDevice::init(bleName);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  pTxChar = pService->createCharacteristic(BLE_CHAR_TX_UUID,
              BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());

  BLECharacteristic* pRxChar = pService->createCharacteristic(BLE_CHAR_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pRxChar->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  buoyPrintln("BLE advertising as: " + String(bleName));

  delay(2000);
  buoyPrintln("\n=== Buoy Combo - UART GPS ===");

  // Setting Pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(SHUTDOWN_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHUTDOWN_BTN), shutdownISR, FALLING);

  initializeIna228();
  initializeGnssUart();

  // Initialize Modem
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  
  buoyPrintln("Powering on modem...");
  modem.powerOn(BOTLETICS_PWRKEY);
  delay(5000);

  buoyPrintln("Configuring modem to 115200 baud");
  if (!modemLinkBegin()) {
    buoyPrintln("Couldn't find modem");
    while (1);
  }

  buoyPrintln("SIM7000 detected");
  
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    buoyPrint("Module IMEI: ");
    buoyPrintln(imei);
  }

  modem.configureNetwork();

  buoyPrintln("Setup complete — waiting for CGREG registration\n");
}

void loop() {
  // Handle user AT commands
  if (Serial.available()) {
    buoyPrint("modem> ");
    while (Serial.available()) {
      modemSS.write(Serial.read());
    }
    delay(100);
    while (modemSS.available()) {
      Serial.write(modemSS.read());
    }
    return;
  }
  
  // Network management
  networkStatusCheck();
  setupGprs();
  
  // NTRIP connection management
  if (gprsEnabled && !ntripConnected && 
      (millis() - lastNtripAttempt > ntripRetryInterval)) {
    beginNTRIPClient();
    lastNtripAttempt = millis();
  }

  // Handle NTRIP data (receives RTCM and sends to GPS via UART)
  // check for ntrip connection performed in-function
  handleNTRIPData();
  
  monitorConnectionHealth();

  // telemetry sent every 60s
  // check for gprs conn performed in-function
  postTelemetry();

  // Power + GPS status every 5 seconds
  if (millis() - lastFixStatusPrint > 5000) {
    lastFixStatusPrint = millis();
    printPowerStatus();

    if (gpsUARTOnline) {  // single poll, populates everything below
      broadcastGPS();
    }
  }
  
  updateStatusLED();

  // Check for shutdown request
  if (shutdownRequested) {
    gracefulShutdown();
  }

  delay(10);  // Reduced from 1000ms for better responsiveness
}