// ELEE 4230/6230 - CP3: thermistor + LDR signal chains, BMP280 calibration, Kalman
// Built on the working CP2 sketch (BMP280, DHT22, PAWS-Secure, ThingSpeak).
// The code undoes each chain one stage at a time (separate inverse functions).

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "DHT.h"
#include "esp_wpa2.h"
#include <WiFi.h>
#include "ThingSpeak.h"

// ---- FILL IN THE PASSWORD SECTION WITH UGA LOGIN DETAILS ----
#define EAP_IDENTITY "nbk83478"
#define EAP_USERNAME "nbk83478"
#define EAP_PASSWORD ""

unsigned long CHANNEL_ID = 3498129;
const char* THINGSPEAK_WRITE_KEY = "GV8T0MSXMTJGQD8D";

#define MAX_DISCONNECTS 4
const char* ssid = "PAWS-Secure";
unsigned char disconnectNum = 0;
WiFiClient client;

#define DHTPIN 4
#define DHTTYPE DHT22
#define THERM_PIN 34            // ADC1 pin: still works with Wi-Fi on
#define LDR_PIN   35            // ADC1 pin

Adafruit_BMP280 bmp;
DHT dht(DHTPIN, DHTTYPE);


// THERMISTOR: MEASURED CIRCUIT CONSTANTS
const float V_SUPPLY = 4.99;     // V   bridge supply (top of R1/R2 to GND)
const float V_A      = 0.152;    // V   INA121 pin 2 to GND (measured)
const float V_REF    = 0.494;    // V   INA121 pin 5 to GND (bias from TL071)
const float GAIN     = 51.0;     //     1 + 50k / 1k
const float R2_TOP   = 10000.0;  // ohm resistor above the thermistor network
const float R_PAR    = 551.5;    // ohm linearizing resistor across thermistor

// Thermistor model (Beta). R0/T0 measured beside the BMP280: 987 ohm at 76.73 F.
const float R0_THERM = 987.0;    // ohm at T0
const float T0_K     = 298.0;    // K (24.85 C)
const float BETA     = 4425.0;   // K (provisional until warm/cool points)

// Calibration to the BMP280:  T_cal = CAL_GAIN * T_therm + CAL_OFFSET  (deg C)
const float CAL_GAIN   = 1.0;
const float CAL_OFFSET = 0.0;

// LDR: MEASURED CIRCUIT CONSTANTS
const float LDR_VS     = 4.99;      // V   supply on the LDR divider
const float LDR_RFIXED = 1000.0;    // ohm fixed resistor, Node L to GND (measure it)

// LDR model (power law), fitted to 7 points against a phone lux meter:
//   R = LDR_A * lux^(-LDR_GAMMA)
const float LDR_A     = 240540.0;   // ohm
const float LDR_GAMMA = 0.623;

// Valid ADC window: outside this the reading is clipped, not real
const float LDR_V_MIN = 0.15;       // V   ADC floor  (darker than about 100 lux)
const float LDR_V_MAX = 3.10;       // V   ADC ceiling (brighter than about 20,000 lux)

// THERMISTOR INVERSE TRANSFER FUNCTIONS - one per stage of the mountain

// Stage 5 inverse: ADC -> volts at GPIO34
float adcToVolts() {
  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogReadMilliVolts(THERM_PIN);
  return (sum / 32.0) / 1000.0;
}

// Stage 4 inverse: in-amp.  V_out = GAIN*(V_B - V_A) + V_REF
float voltsToBridgeDiff(float vOut) {
  return (vOut - V_REF) / GAIN;
}

// Stage 3 inverse: bridge.  V_B = V_SUPPLY * R_eq / (R2_TOP + R_eq)
float bridgeDiffToReq(float vDiff) {
  float vB = V_A + vDiff;
  return R2_TOP * vB / (V_SUPPLY - vB);
}

// Stage 2 inverse: parallel linearizing resistor.  R_eq = R_th*Rp/(R_th+Rp)
float reqToRtherm(float rEq) {
  return rEq * R_PAR / (R_PAR - rEq);
}

// Stage 1 inverse: thermistor physics.  R = R0 * exp(B*(1/T - 1/T0))
float rthermToCelsius(float rTh) {
  float invT = 1.0 / T0_K + log(rTh / R0_THERM) / BETA;
  return 1.0 / invT - 273.15;
}

// Calibration to the BMP280
float calibrate(float tC) {
  return CAL_GAIN * tC + CAL_OFFSET;
}

float cToF(float c) { return c * 9.0 / 5.0 + 32.0; }

// THERMISTOR KALMAN FILTER (1-D, constant-temperature model)

float kal_x = 0;             // estimate (deg C)
float kal_p = 1.0;           // estimate variance
const float KAL_Q = 0.005;   // process noise
const float KAL_R = 0.05;    // measurement noise
bool  kal_init = false;

float kalmanUpdate(float z) {
  if (!kal_init) { kal_x = z; kal_init = true; return kal_x; }
  kal_p += KAL_Q;                       // predict
  float k = kal_p / (kal_p + KAL_R);    // gain
  kal_x += k * (z - kal_x);             // correct
  kal_p *= (1.0 - k);
  return kal_x;
}

// LDR INVERSE TRANSFER FUNCTIONS - one per stage of the mountain

// Stage 3 inverse: ADC -> volts at GPIO35 (buffer and filter have gain 1)
float ldrAdcToVolts() {
  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogReadMilliVolts(LDR_PIN);
  return (sum / 32.0) / 1000.0;
}

// Stage 2 inverse: divider.  V = Vs * Rf / (Rf + R_ldr)
float ldrVoltsToOhms(float v) {
  if (v < 0.01) v = 0.01;                 // avoid divide-by-zero in the dark
  return LDR_RFIXED * (LDR_VS - v) / v;
}

// Stage 1 inverse: LDR physics.  R = A * lux^(-gamma)
float ldrOhmsToLux(float r) {
  return pow(r / LDR_A, -1.0 / LDR_GAMMA);
}

// LDR KALMAN FILTER
// Runs on log10(lux), because the hardware output is linear in log-lux.
float ldr_x = 0, ldr_p = 1.0;
const float LDR_Q = 0.002;   // process noise (decades^2): light changes quickly
const float LDR_R = 0.010;   // measurement noise (decades^2)
bool  ldr_init = false;

/**
It's in kalmanUpdate(). Each sample it predicts (P grows by Q), 
computes a gain K = P / (P + R), moves the estimate toward the new reading by K, then shrinks P. Q = 0.005 is 
how fast I expect the real temperature to drift. R = 0.05 is the variance of a raw reading. 
They settle to a gain of 0.27, so each reading moves the estimate 27% of the way. 
Smoothness against lag. When I pinch the thermistor 
the filtered value trails the raw one by up to about 2 °F for a few seconds.
**/

float ldrKalman(float z) {
  if (!ldr_init) { ldr_x = z; ldr_init = true; return ldr_x; }
  ldr_p += LDR_Q;
  float k = ldr_p / (ldr_p + LDR_R);
  ldr_x += k * (z - ldr_x);
  ldr_p *= (1.0 - k);
  return ldr_x;
}
/**

adcToVolts(): averages 32 readings, converts to volts.
voltsToBridgeDiff(): undoes the in-amp gain and bias.
bridgeDiffToReq(): undoes the bridge.
reqToRtherm(): undoes the parallel resistor.
rthermToCelsius(): undoes the thermistor physics.

**/
// SENSOR SUBFUNCTIONS (project rule: one function per sensor)
float readThermistorRawC() {
  float vOut  = adcToVolts();
  float vDiff = voltsToBridgeDiff(vOut);
  float rEq   = bridgeDiffToReq(vDiff);
  float rTh   = reqToRtherm(rEq);
  return calibrate(rthermToCelsius(rTh));
}
/**
It's in kalmanUpdate(). Each sample it predicts (P grows by Q), 
computes a gain K = P / (P + R), moves the estimate toward the new reading by K, then shrinks P. Q = 0.005 is 
how fast I expect the real temperature to drift. R = 0.05 is the variance of a raw reading. 
They settle to a gain of 0.27, so each reading moves the estimate 27% of the way.
**/
float readThermistorC() {            // calibrated + Kalman filtered
  return kalmanUpdate(readThermistorRawC());
}

float readLightLuxRaw() {
  return ldrOhmsToLux(ldrVoltsToOhms(ldrAdcToVolts()));
}

float readLightLux() {               // Kalman filtered
  return pow(10.0, ldrKalman(log10(readLightLuxRaw())));
}

float readBmpTempC()    { return bmp.readTemperature(); }
float readBmpPressure() { return bmp.readPressure(); }
float readHumidity()    { return dht.readHumidity(); }

// WIFI (unchanged from CP2)
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

unsigned long prevUpload = 0;
unsigned long prevSample = 0;
const unsigned long UPLOAD_MS = 16000;
const unsigned long SAMPLE_MS = 500;

void setup() {
  Serial.begin(115200);
  analogSetPinAttenuation(THERM_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_PIN, ADC_11db);

  if (!bmp.begin(0x77)) {
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
}

void loop() {
  // ---- fast loop: show every step of both mountains ----
  if (millis() - prevSample >= SAMPLE_MS) {
    prevSample = millis();

    // Thermistor
    float vOut  = adcToVolts();
    float vDiff = voltsToBridgeDiff(vOut);
    float rEq   = bridgeDiffToReq(vDiff);
    float rTh   = reqToRtherm(rEq);
    float rawC  = calibrate(rthermToCelsius(rTh));
    float filtC = kalmanUpdate(rawC);
    float bmpC  = readBmpTempC();

    Serial.print("THERM  Vout: ");  Serial.print(vOut, 3);         Serial.print(" V | ");
    Serial.print("Bridge: ");       Serial.print(vDiff * 1000, 2); Serial.print(" mV | ");
    Serial.print("Req: ");          Serial.print(rEq, 1);          Serial.print(" ohm | ");
    Serial.print("Rtherm: ");       Serial.print(rTh, 1);          Serial.print(" ohm | ");
    Serial.print("Raw: ");          Serial.print(cToF(rawC), 2);   Serial.print(" F | ");
    Serial.print("Kalman: ");       Serial.print(cToF(filtC), 2);  Serial.print(" F | ");
    Serial.print("BMP280: ");       Serial.print(cToF(bmpC), 2);   Serial.println(" F");

    // LDR
    float lv      = ldrAdcToVolts();
    float lr      = ldrVoltsToOhms(lv);
    float luxRaw  = ldrOhmsToLux(lr);
    float luxFilt = pow(10.0, ldrKalman(log10(luxRaw)));

    Serial.print("LDR    Vout: ");  Serial.print(lv, 3);       Serial.print(" V | ");
    Serial.print("Rldr: ");         Serial.print(lr, 0);       Serial.print(" ohm | ");
    Serial.print("Lux raw: ");      Serial.print(luxRaw, 0);   Serial.print(" | ");
    Serial.print("Lux Kalman: ");   Serial.print(luxFilt, 0);
    if (lv < LDR_V_MIN)      Serial.println("  (too dark: below range)");
    else if (lv > LDR_V_MAX) Serial.println("  (too bright: above range)");
    else                     Serial.println();
  }

  // ---- slow loop: ThingSpeak ----
  if (millis() - prevUpload >= UPLOAD_MS) {
    prevUpload = millis();

    float humidity = readHumidity();
    ThingSpeak.setField(1, readBmpTempC());
    ThingSpeak.setField(2, readBmpPressure());
    if (!isnan(humidity)) ThingSpeak.setField(3, humidity);
    ThingSpeak.setField(4, cToF(kal_x));
    ThingSpeak.setField(5, (float)pow(10.0, ldr_x));

    int result = ThingSpeak.writeFields(CHANNEL_ID, THINGSPEAK_WRITE_KEY);
    if (result != 200) Serial.println("# ThingSpeak failed, code: " + String(result));
  }
}
