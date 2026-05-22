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
void printDebugStatus();
void updateStatusLED();

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


void network_status_check_f() {
  if (networkConnected) return;

  static unsigned long lastCheckMs = 0;
  static unsigned long lastDiagMs = 0;
  const unsigned long checkIntervalMs = 5000;

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
    if (modem.getNetworkStatus() == n) {
      networkConnected = true;
      Serial.println(F("[NET] connected"));
      SerialBT.println(F("[NET] connected"));
    }
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
    return;
  }

  delay(2000);  // give caster time to reply

  uint16_t available = modem.TCPavailable();
  if (available == 0) {
    Serial.println(F("[NTRIP] no response from caster"));
    SerialBT.println(F("[NTRIP] no response from caster"));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
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
    lastReceivedRTCM_ms = millis();
  } else if (unauth) {
    Serial.println(F("[NTRIP] 401 unauthorized"));
    SerialBT.println(F("[NTRIP] 401 unauthorized"));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
  } else if (notfound) {
    Serial.println(F("[NTRIP] 404 mount not found"));
    SerialBT.println(F("[NTRIP] 404 mount not found"));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
  } else {
    Serial.println(F("[NTRIP] unrecognized response"));
    SerialBT.println(F("[NTRIP] unrecognized response"));
    modem.sendCheckReply(F("AT+CIPCLOSE"), F("CLOSE OK"), 10000);
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

  if (totalSent > 0) lastReceivedRTCM_ms = millis();

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
  if (ntripConnected) return;
  if (!networkConnected) return;

  static unsigned long lastHealthCheck = 0;
  if (millis() - lastHealthCheck < 30000) return;
  lastHealthCheck = millis();

  uint8_t status = modem.getNetworkStatus();
  uint8_t rssi   = modem.getRSSI();
  Serial.print(F("[HEALTH] net="));    Serial.print(status);
  Serial.print(F(" rssi="));            Serial.println(rssi);
  SerialBT.print(F("[HEALTH] net="));  SerialBT.print(status);
  SerialBT.print(F(" rssi="));          SerialBT.println(rssi);

  if (status != 1 && status != 5) {
    Serial.println(F("[HEALTH] network lost"));
    SerialBT.println(F("[HEALTH] network lost"));
    networkConnected = false;
    gprsEnabled = false;
    ntripConnected = false;
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