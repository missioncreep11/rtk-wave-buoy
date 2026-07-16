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
bool gpsEnabled = false;
bool ntripConnected = false;
bool gpsUARTOnline = false;
bool ina228Online = false;
volatile bool shutdownRequested = false;

// Timing
long lastReceivedRTCM_ms = 0;
int maxTimeBeforeHangup_ms = 100000;
const unsigned long ntripRetryInterval = 30000;
unsigned long lastNTRIPAttempt = 0;
unsigned long lastCellularActivity_ms = 0;
unsigned long lastGprsEnabled_ms = 0;
uint8_t consecutiveNtripFailures = 0;
long lastGPSPrint = 0;
unsigned long lastFixStatusPrint = 0;

// Configuration
uint8_t type;
char replybuffer[255];
char imei[16] = {0};

// ============================================================
// BLE server callbacks
// ============================================================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleConnected = true;
    buoyPrintln("BLE connected. Commands: STATUS GPS RESET SLEEP");
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
// BLE command handler (buoy_combo variant — no WiFi, no GET/SET)
// ============================================================
void handleBLECommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String cmdUpper = cmd;
  cmdUpper.toUpperCase();

  // --- STATUS ---
  if (cmdUpper == "STATUS") {
    buoyPrintln("=== STATUS ===");
    buoyPrintln("Network: " + String(networkConnected ? "connected" : "disconnected"));
    buoyPrintln("GPRS: "    + String(gprsEnabled    ? "enabled"  : "disabled"));
    buoyPrintln("NTRIP: "   + String(ntripConnected ? "connected" : "disconnected"));
    buoyPrintln("GPS: "     + String(gpsUARTOnline  ? "online"   : "offline"));
    buoyPrintln("INA228: "  + String(ina228Online   ? "online"   : "offline"));
    buoyPrintln("==============");

  // --- GPS ---
  } else if (cmdUpper == "GPS") {
    buoyPrintln("=== GPS ===");
    float lat    = myGNSS.getLatitude()         / 10000000.0;
    float lon    = myGNSS.getLongitude()        / 10000000.0;
    float alt    = myGNSS.getAltitudeMSL()      / 1000.0;
    float hAcc   = myGNSS.getHorizontalAccEst() / 1000.0;
    uint8_t fix      = myGNSS.getFixType();
    uint8_t carrier  = myGNSS.getCarrierSolutionType();
    uint8_t siv      = myGNSS.getSIV();
    String rtk = (carrier == 2) ? "Fixed" : (carrier == 1) ? "Float" : "None";
    buoyPrintln("Lat:  " + String(lat, 7));
    buoyPrintln("Lon:  " + String(lon, 7));
    buoyPrintln("Alt:  " + String(alt, 2) + " m");
    buoyPrintln("Fix:  " + String(fix));
    buoyPrintln("RTK:  " + rtk);
    buoyPrintln("hAcc: " + String(hAcc, 3) + " m");
    buoyPrintln("SIV:  " + String(siv));
    buoyPrintln("===========");

  // --- RESET ---
  } else if (cmdUpper == "RESET") {
    buoyPrintln("Resetting NTRIP connection...");
    ntripConnected = false;
    lastNTRIPAttempt = 0;

  // --- SLEEP ---
  } else if (cmdUpper == "SLEEP") {
    buoyPrintln("Entering light sleep. Press button to wake.");
    ntripConnected = false;
    digitalWrite(STATUS_LED, LOW);
    delay(500);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
    esp_light_sleep_start();
    buoyPrintln("Woke from sleep.");

  } else {
    buoyPrintln("Unknown command: " + cmd);
    buoyPrintln("Commands: STATUS GPS RESET SLEEP");
  }
}

// ============================================================
// Print GPS status over BLE
// ============================================================
void broadcastGPS() {
  if (!bleConnected) return;
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

  initialize_ina228_f();
  initialize_gnss_uart_f();

  // Initialize Modem
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  
  buoyPrintln("Powering on modem...");
  modem.powerOn(BOTLETICS_PWRKEY);
  delay(5000);

  buoyPrintln("Configuring modem to 9600 baud");
  if (!modemLinkBegin()) {
    buoyPrintln("Couldn't find modem");
    while (1);
  }

  type = modem.type();
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
  // Dispatch BLE commands
  if (bleDataReady) {
    bleDataReady = false;
    handleBLECommand(String(bleRxBuf));
  }

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
  network_status_check_f();
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
  
  monitor_connection_health();

  if (gprsEnabled) {
    post_telemetry_f();
  }

  // Power + GPS status every 5 seconds
  if (millis() - lastFixStatusPrint > 5000) {
    lastFixStatusPrint = millis();
    print_power_status_f();

    if (gpsUARTOnline && myGNSS.getPVT()) {  // single poll, populates everything below
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