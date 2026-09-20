#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include "esp_wpa2.h"
#include <WiFi.h>
#include "ThingSpeak.h"

// ---- FILL THESE IN YOURSELF IN THE IDE ----
#define EAP_IDENTITY "nbk83478"
#define EAP_USERNAME "nbk83478"
#define EAP_PASSWORD "----"
// did not include the password (IOT DOES NOT WORK NEED TO INCLUDE PASSWORD)

unsigned long CHANNEL_ID = 3498129;
const char* THINGSPEAK_WRITE_KEY = "GV8T0MSXMTJGQD8D";

#define MAX_DISCONNECTS 4
const char* ssid = "PAWS-Secure";
unsigned char disconnectNum = 0;
WiFiClient client;

#define DHTPIN 4
#define DHTTYPE DHT22

Adafruit_BMP280 bmp;
DHT dht(DHTPIN, DHTTYPE);

unsigned long prev_millis = 0;
const unsigned long interval = 16000;

void WiFiStationConnected(WiFiEvent_t event) {
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event) {
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event) {
  Serial.println("Disconnected from WiFi access point");
  Serial.println("Trying to Reconnect");
  if (disconnectNum < MAX_DISCONNECTS) {
    WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);
    delay(30000);
  }
  disconnectNum++;
}

void wifiSetup() {
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);
  ThingSpeak.begin(client);
}

void setup() {
  Serial.begin(115200);

  unsigned status = bmp.begin(0x77);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  dht.begin();

  wifiSetup();

  prev_millis = millis();
}

void loop() {
  if (millis() - prev_millis >= interval) {
    prev_millis = millis();

    float bmpTemp = bmp.readTemperature();
    float bmpPressure = bmp.readPressure();
    float humidity = dht.readHumidity();

    Serial.print("BMP Temp: "); Serial.println(bmpTemp);
    Serial.print("BMP Pressure: "); Serial.println(bmpPressure);
    Serial.print("Humidity: "); Serial.println(humidity);

    ThingSpeak.setField(1, bmpTemp);
    ThingSpeak.setField(2, bmpPressure);
    if (!isnan(humidity)) {
      ThingSpeak.setField(3, humidity);
    }
    int result = ThingSpeak.writeFields(CHANNEL_ID, THINGSPEAK_WRITE_KEY);
    if (result == 200) {
      Serial.println("ThingSpeak update successful.");
    } else {
      Serial.println("ThingSpeak update failed, code: " + String(result));
    }
  }
}
