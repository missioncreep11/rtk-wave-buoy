/* buoy_combo.ino */
// NTRIP casters use plain TCP (e.g. port 2101). BotleticsSIM7000.h defaults BOTLETICS_SSL=1.
#ifndef BOTLETICS_SSL
#define BOTLETICS_SSL 0
#endif
#include "BotleticsSIM7000.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_INA228.h>
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

#if !defined(HAS_TELEMETRY_URL)
const char *telemetryUrl = "";
#endif
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

// Bluetooth Name
String device_name = "RTK-GPS-BT";

void setup() {
  // USB Debug Serial
  Serial.begin(115200);
  SerialBT.begin(device_name);
  Serial.println("Connecting to bluetooth...");
  
  // Use if bluetooth seems to be acting up
  // SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  delay(2000);
  Serial.println(F("\n=== Buoy Combo - UART GPS ==="));
  SerialBT.println("\n=== Buoy Combo - UART GPS ===");

  // Setting Pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(SHUTDOWN_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHUTDOWN_BTN), shutdownISR, FALLING);

  initialize_ina228_f();
  initialize_gnss_uart_f();

  // Initialize Modem
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);
  
  Serial.println(F("Powering on modem..."));
  SerialBT.println(F("Powering on modem..."));
  modem.powerOn(BOTLETICS_PWRKEY);
  delay(5000);

  modemSS.begin(115200, SERIAL_8N1, TX_MODEM, RX_MODEM);
  Serial.println(F("Configuring modem to 9600 baud"));
  SerialBT.println(F("Configuring modem to 9600 baud"));
  modemSS.println("AT+IPR=9600");
  delay(1000);
  modemSS.begin(9600, SERIAL_8N1, TX_MODEM, RX_MODEM);

  if (!modem.begin(modemSS)) {
    Serial.println(F("Couldn't find modem"));
    SerialBT.println(F("Couldn't find modem"));
    while (1);
  }

  type = modem.type();
  Serial.println(F("SIM7000 detected"));
  SerialBT.println(F("SIM7000 detected"));
  
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print(F("Module IMEI: ")); 
    SerialBT.println(F("Module IMEI: "));
    Serial.println(imei);
    SerialBT.println(imei);
  }

  modem.configureNetwork();

  Serial.println(F("Setup complete — waiting for CGREG registration\n"));
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

  // May cause problems and its not really needed unless debugging, if needed maybe use i2c for print_gps_status_f() - Amara
  //  Print GPS status periodically
  // if (gpsUARTOnline && (millis() - lastGPSPrint > 1000)) {
  //   lastGPSPrint = millis();
  //   print_gps_status_f();
  // }
  
  monitor_connection_health();

  if (gprsEnabled) {
    post_telemetry_f();
  }

  // Power + GPS status every 5 seconds
  if (millis() - lastFixStatusPrint > 5000) {
    lastFixStatusPrint = millis();
    print_power_status_f();

    if (gpsUARTOnline && myGNSS.getPVT()) {  // single poll, populates everything below
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