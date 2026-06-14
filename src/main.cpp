// main.cpp: Read PMS5003 sensor on Serial2 (ESP32)

#include <Arduino.h>
#include <PMserial.h>  // Arduino library for PM sensors with serial interface
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Preferences.h>  // ESP32 internal flash key/value storage (NVS)

#include "secrets.h"

LiquidCrystal_I2C lcd(0x3F, 16, 2);


#define DHTPIN 14
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

SerialPM pms(PMSx003, Serial2);

#define BTN_RIGHT 34
#define BTN_LEFT 35
#define BTN_SETTINGS 36
#define BTN_BOOT 39

volatile bool rightPressed = false;
volatile bool leftPressed = false;
volatile bool settingsPressed = false;
volatile bool bootPressed = false;

volatile uint32_t lastRightInterrupt = 0;
volatile uint32_t lastLeftInterrupt = 0;
volatile uint32_t lastSettingsInterrupt = 0;
volatile uint32_t lastBootInterrupt = 0;

const uint32_t DEBOUNCE_MS = 150;




void IRAM_ATTR rightISR() {
  uint32_t now = millis();

  if (now - lastRightInterrupt > DEBOUNCE_MS) {
    rightPressed = true;
    lastRightInterrupt = now;
  }
}

void IRAM_ATTR leftISR() {
  uint32_t now = millis();

  if (now - lastLeftInterrupt > DEBOUNCE_MS) {
    leftPressed = true;
    lastLeftInterrupt = now;
  }
}

void IRAM_ATTR settingsISR() {
  uint32_t now = millis();

  if (now - lastSettingsInterrupt > DEBOUNCE_MS) {
    settingsPressed = true;
    lastSettingsInterrupt = now;
  }
}

void IRAM_ATTR bootISR() {
  uint32_t now = millis();

  if (now - lastBootInterrupt > DEBOUNCE_MS) {
    bootPressed = true;
    lastBootInterrupt = now;
  }
}


// ---------------------------------------------------------------------------
// Persistent configuration (WiFi credentials + location)
//
// The WiFi name/password and the device location (latitude/longitude) are kept
// in the ESP32's internal flash using the Preferences library (NVS = a small
// built-in key/value store). These values survive reboots and power loss.
//
// Flow: the phone sets the values over BLE, then sends "SAVE" to write them to
// flash. On every boot we load them back and use them to connect to WiFi.
// ---------------------------------------------------------------------------

Preferences preferences;                 // handle to the flash key/value store

// All the settings we store, kept together in one place.
struct DeviceConfig {
  String ssid;       // WiFi network name
  String password;   // WiFi password
  double lat;        // location latitude
  double lon;        // location longitude
};

DeviceConfig config;                     // the live working copy, held in RAM

// Read saved settings from flash into `config`. Anything not yet saved comes
// back as the given default ("" or 0).
void loadConfig() {
  preferences.begin("config", true);     // open storage area "config", read-only
  config.ssid     = preferences.getString("ssid", "");
  config.password = preferences.getString("pass", "");
  config.lat      = preferences.getDouble("lat", 0.0);
  config.lon      = preferences.getDouble("lon", 0.0);
  preferences.end();                      // close it (frees the handle)

  Serial.println("Config loaded from flash:");
  Serial.print("  SSID: "); Serial.println(config.ssid.length() ? config.ssid : "(none)");
  Serial.print("  LAT:  "); Serial.println(config.lat, 6);
  Serial.print("  LON:  "); Serial.println(config.lon, 6);
}

// Write the current `config` to flash so it survives a reboot/power loss.
void saveConfig() {
  preferences.begin("config", false);    // open storage area "config", read-write
  preferences.putString("ssid", config.ssid);
  preferences.putString("pass", config.password);
  preferences.putDouble("lat", config.lat);
  preferences.putDouble("lon", config.lon);
  preferences.end();
  Serial.println("Config saved to flash");
}

// Erase saved settings from flash and reset the working copy to blank.
void clearConfig() {
  preferences.begin("config", false);
  preferences.clear();                    // wipe everything in this storage area
  preferences.end();
  config = DeviceConfig{"", "", 0.0, 0.0};
  Serial.println("Config cleared");
}

// Connect to WiFi using the stored credentials. Gives up after `timeoutMs`
// instead of looping forever, so a wrong password can't freeze the device —
// it stays alive and reachable over BLE so you can fix the credentials.
// Returns true only if the connection succeeded.
bool connectWiFi(uint32_t timeoutMs = 15000) {
  if (config.ssid.length() == 0) {
    Serial.println("WiFi: no SSID configured, skipping connect");
    return false;
  }

  Serial.print("WiFi: connecting to '");
  Serial.print(config.ssid);
  Serial.print("' ");

  WiFi.begin(config.ssid.c_str(), config.password.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi: connected, IP = ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("WiFi: connection timed out");
  return false;
}


// ---------------------------------------------------------------------------
// Bluetooth Low Energy (BLE)
//
// We turn the ESP32 into a tiny "wireless serial port" using the Nordic UART
// Service (NUS). A phone connects and:
//   - writes text to the RX characteristic  (phone -> ESP32)
//   - listens on the TX characteristic       (ESP32 -> phone)
// Later we'll send the WiFi name/password over this same channel.
// ---------------------------------------------------------------------------

// The name the phone sees when scanning for Bluetooth devices.
#define BLE_DEVICE_NAME "AirMonitor-Setup"

// Fixed IDs for the Nordic UART Service. These are an industry standard, so
// generic BLE apps already know how to talk to them.
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // the service
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone -> ESP32
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 -> phone

// Pointer to the TX channel so we can push messages out from anywhere.
BLECharacteristic *bleTxCharacteristic = nullptr;
// True while a phone is connected. (volatile: changed inside BLE callbacks.)
volatile bool bleClientConnected = false;

// Send one line of text back to the connected phone.
void bleSend(const String &msg) {
  if (bleClientConnected && bleTxCharacteristic != nullptr) {
    // Put the text into the TX channel, then "notify" so the phone receives it.
    // The (uint8_t*, size_t) form works on both old and new ESP32 BLE versions.
    bleTxCharacteristic->setValue((uint8_t *)msg.c_str(), msg.length());
    bleTxCharacteristic->notify();
    Serial.print("BLE sent: ");
    Serial.println(msg);
  } else {
    // Nothing is connected, so there is nobody to send to.
    Serial.println("BLE send skipped (no client connected)");
  }
}

// Decide what to do with one command the phone sent us.
// For now it's just a simple test protocol; WiFi commands come later.
void processBluetoothCommand(const String &cmd) {
  Serial.print("BLE received: ");
  Serial.println(cmd);

  if (cmd.equalsIgnoreCase("PING")) {
    bleSend("PONG");                       // simple "are you alive?" check

  } else if (cmd.equalsIgnoreCase("STATUS")) {
    String status = "WiFi: ";
    status += (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
    status += " IP: ";
    status += WiFi.localIP().toString();
    bleSend(status);                       // report current WiFi state

  // --- WiFi-credential provisioning -----------------------------------------
  // The phone sends these one at a time to fill in the settings, then "SAVE".
  // substring() keeps everything after the prefix, so passwords containing
  // ':' are handled correctly.

  } else if (cmd.startsWith("SSID:")) {
    config.ssid = cmd.substring(5);        // text after "SSID:"
    bleSend("OK: SSID set to " + config.ssid);

  } else if (cmd.startsWith("PASS:")) {
    config.password = cmd.substring(5);
    bleSend("OK: password set");           // never echo the password back

  } else if (cmd.startsWith("LAT:")) {
    config.lat = cmd.substring(4).toDouble();   // "31.5204" -> 31.5204
    bleSend("OK: LAT set to " + String(config.lat, 6));

  } else if (cmd.startsWith("LON:")) {
    config.lon = cmd.substring(4).toDouble();
    bleSend("OK: LON set to " + String(config.lon, 6));

  } else if (cmd.equalsIgnoreCase("SAVE")) {
    saveConfig();                          // write all four values to flash
    bleSend("SAVED. Reconnecting WiFi...");
    if (connectWiFi()) {                   // try the new credentials right away
      bleSend("WiFi connected: " + WiFi.localIP().toString());
    } else {
      bleSend("WiFi connect FAILED (check SSID/password)");
    }

  } else if (cmd.equalsIgnoreCase("LOAD")) {
    // Report what's currently set (password hidden for safety).
    String out = "SSID=" + config.ssid;
    out += " PASS=" + String(config.password.length() ? "(set)" : "(empty)");
    out += " LAT=" + String(config.lat, 6);
    out += " LON=" + String(config.lon, 6);
    bleSend(out);

  } else if (cmd.equalsIgnoreCase("CLEAR")) {
    clearConfig();                         // wipe stored settings
    bleSend("CLEARED all stored config");

  } else {
    bleSend("echo: " + cmd);               // anything else: just echo it back
  }
}

// These callbacks run automatically when a phone connects or disconnects.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    bleClientConnected = true;
    Serial.println("BLE: client CONNECTED");
  }
  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;
    Serial.println("BLE: client DISCONNECTED, advertising again");
    server->startAdvertising();            // become visible again to reconnect
  }
};

// This callback runs automatically every time the phone writes to RX.
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    // Read the text the phone sent. .c_str() works on both old and new
    // ESP32 BLE versions (one returns std::string, the other Arduino String).
    String value = characteristic->getValue().c_str();
    value.trim();                          // drop stray newlines / spaces
    if (value.length() > 0) {
      processBluetoothCommand(value);
    }
  }
};

// Build and start the whole BLE service. Logs each step so the serial monitor
// shows exactly how far it got if something goes wrong.
void setupBluetooth() {
  Serial.println("BLE: setup start");

  // 1) Start the BLE radio and give the device its name.
  BLEDevice::init(BLE_DEVICE_NAME);
  Serial.print("BLE: radio init done, MAC = ");
  Serial.println(BLEDevice::getAddress().toString().c_str());

  // 2) Create the GATT server (the thing that holds our data).
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  Serial.println("BLE: server created");

  // 3) Create the Nordic UART Service and its two channels.
  BLEService *service = server->createService(NUS_SERVICE_UUID);

  // TX channel: ESP32 -> phone. "NOTIFY" lets us push messages out.
  bleTxCharacteristic = service->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTxCharacteristic->addDescriptor(new BLE2902());  // lets the phone enable notifications

  // RX channel: phone -> ESP32. "WRITE" lets the phone send us text.
  BLECharacteristic *rxCharacteristic = service->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  rxCharacteristic->setCallbacks(new RxCallbacks());

  service->start();
  Serial.println("BLE: service started");

  // 4) Start advertising (broadcasting) so phones can find us.
  //
  // Important: a BLE advertisement is only 31 bytes. The 128-bit service UUID
  // (18 bytes) plus the device name would overflow that and the name gets
  // dropped — so the device shows up unnamed or not at all. Fix: put the NAME
  // in the main packet, and the big UUID in the separate "scan response" packet.
  BLEAdvertising *advertising = BLEDevice::getAdvertising();

  BLEAdvertisementData advData;
  advData.setFlags(0x06);                  // standard "discoverable, BLE-only" flags
  advData.setName(BLE_DEVICE_NAME);        // name goes in the main packet
  advertising->setAdvertisementData(advData);

  BLEAdvertisementData scanResponse;
  scanResponse.setCompleteServices(BLEUUID(NUS_SERVICE_UUID));  // UUID goes here instead
  advertising->setScanResponseData(scanResponse);

  BLEDevice::startAdvertising();
  Serial.println("BLE: advertising as '" BLE_DEVICE_NAME "' — ready to connect");
}


void setup() {

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.print("Starting");


  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);


  pinMode(BTN_RIGHT, INPUT);
  pinMode(BTN_LEFT, INPUT);
  pinMode(BTN_SETTINGS, INPUT);
  pinMode(BTN_BOOT, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(BTN_RIGHT),
    rightISR,
    FALLING);

  attachInterrupt(
    digitalPinToInterrupt(BTN_LEFT),
    leftISR,
    FALLING);

  attachInterrupt(
    digitalPinToInterrupt(BTN_SETTINGS),
    settingsISR,
    FALLING);

  attachInterrupt(
    digitalPinToInterrupt(BTN_BOOT),
    bootISR,
    FALLING);




  Serial.println(F("Booted"));
  pms.init();
  dht.begin();

  setupBluetooth();


  // Load saved WiFi + location from flash. On a brand-new device nothing is
  // stored yet, so fall back to the defaults in secrets.h until the user
  // provisions real credentials over BLE and sends SAVE.
  loadConfig();
  if (config.ssid.length() == 0) {
    Serial.println("No saved WiFi — using secrets.h defaults");
    config.ssid     = WIFI_SSID;
    config.password = WIFI_PASSWORD;
  }

  // Everything below needs the internet, so only run it if WiFi actually came
  // up. If it didn't, the device keeps running and stays reachable over BLE so
  // you can send new credentials.
  if (connectWiFi()) {
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());

    Serial.print("Subnet: ");
    Serial.println(WiFi.subnetMask());

    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());

    // Sync the clock (needed for TLS), but don't block forever if it fails.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("Waiting for time");
    struct tm timeinfo;
    uint32_t timeStart = millis();
    while (!getLocalTime(&timeinfo) && millis() - timeStart < 10000) {
      Serial.print(".");
      delay(500);
    }
    Serial.println("\nTime synced");
    Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");

    HTTPClient http;
    http.begin("http://192.168.1.8:8080/api/v1/air-quality/current");
    int httpCode = http.GET();
    if (httpCode > 0) {
      String payload = http.getString();
      Serial.println("Response:");
      Serial.println(payload);
    } else {
      Serial.printf("HTTP Error: %d\n", httpCode);
      Serial.println(http.errorToString(httpCode));
    }
    http.end();

    WiFiClientSecure client;
    client.setInsecure();  // safe for testing

    HTTPClient https;
    // NOTE: the '/' before '?' is required — HTTPClient's URL parser splits
    // host/path at the first '/', so without it the query string becomes part
    // of the hostname and DNS resolution fails.
    if (https.begin(client, "https://api.ipify.org/?format=json")) {
      int code = https.GET();
      Serial.print("HTTP code: ");
      Serial.println(code);
      if (code > 0) {
        String payload = https.getString();
        Serial.println("Response:");
        Serial.println(payload);
      } else {
        Serial.println("Request failed");
      }
      https.end();
    } else {
      Serial.println("HTTPS begin failed");
    }
  } else {
    Serial.println("WiFi not connected — connect over BLE and send SSID:/PASS:/SAVE");
  }
}

void loop() {
  if (rightPressed) {
    rightPressed = false;

    Serial.println("RIGHT");
    // TODO: Next screen
  }

  if (leftPressed) {
    leftPressed = false;

    Serial.println("LEFT");
    // TODO: Previous screen
  }

  if (settingsPressed) {
    settingsPressed = false;

    Serial.println("SETTINGS");
    // TODO: Open settings menu
  }

  if (bootPressed) {
    bootPressed = false;

    Serial.println("BOOT");
    // TODO: Return to home screen
  }

  float h = dht.readHumidity();
  float t = dht.readTemperature();  // Celsius

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor");
  } else {
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %  |  Temperature: ");
    Serial.print(t);
    Serial.println(" °C");
  }

  pms.read();
  if (pms) {
    // print formatted results
    Serial.printf("PM1.0 %2d, PM2.5 %2d, PM10 %2d [ug/m3]\n",
                  pms.pm01, pms.pm25, pms.pm10);
    lcd.print("PM 2.5 " + pms.pm25);

    if (pms.has_number_concentration()) {

      Serial.printf("N0.3 %4d, N0.5 %3d, N1.0 %2d, N2.5 %2d, N5.0 %2d, N10 %2d [#/100cc]\n",
                    pms.n0p3, pms.n0p5, pms.n1p0, pms.n2p5, pms.n5p0, pms.n10p0);
    }

    if (pms.has_temperature_humidity() || pms.has_formaldehyde())
      Serial.printf("%5.1f °C, %5.1f %%rh, %5.2f mg/m3 HCHO\n",
                    pms.temp, pms.rhum, pms.hcho);

  } else {  // something went wrong
    switch (pms.status) {
      case pms.OK:  // should never come here
        break;      // included to compile without warnings
      case pms.ERROR_TIMEOUT:
        Serial.println(F(PMS_ERROR_TIMEOUT));
        break;
      case pms.ERROR_MSG_UNKNOWN:
        Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
        break;
      case pms.ERROR_MSG_HEADER:
        Serial.println(F(PMS_ERROR_MSG_HEADER));
        break;
      case pms.ERROR_MSG_BODY:
        Serial.println(F(PMS_ERROR_MSG_BODY));
        break;
      case pms.ERROR_MSG_START:
        Serial.println(F(PMS_ERROR_MSG_START));
        break;
      case pms.ERROR_MSG_LENGTH:
        Serial.println(F(PMS_ERROR_MSG_LENGTH));
        break;
      case pms.ERROR_MSG_CKSUM:
        Serial.println(F(PMS_ERROR_MSG_CKSUM));
        break;
      case pms.ERROR_PMS_TYPE:
        Serial.println(F(PMS_ERROR_PMS_TYPE));
        break;
    }
  }

  // wait for 10 seconds
  delay(10000);
}
