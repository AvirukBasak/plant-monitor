#include "secrets.hpp"

#include <cmath>
#include "DHT.h"
#include "BlynkSimpleEsp32.h"

#define PIN_SOILMOIST_A (34)
#define PIN_LIGHT_SIG (35)
#define PIN_DHT22_DATA (25)
#define PIN_PUMP_ENB (19)
#define PIN_PUMP_ISON (18)

#define WINDOW_SIZE (100)
#define WINDOW_EMAALPHA (0.2f)

DHT dht(PIN_DHT22_DATA, DHT22);

// Windows used to run EMA over last 100 samples
float windowSoilMoist[WINDOW_SIZE];        // v0
float windowLight[WINDOW_SIZE];            // v1
float windowAmbiTempCelsius[WINDOW_SIZE];  // v2
float windowAmbiMoistPercent[WINDOW_SIZE]; // v3

// Pump speed and states received from cloud
bool pumpIsOn = false;               // v4
float pumpSpeed = 0.0;               // v5

// Spikes on cloud chart eery time device boots - count of resets
bool mcuState = true;                // v6

// EMA value (or last sensor reading) - depends
float lastSoilMoist = 0;
float lastLight = 0;
float lastAmbiTempCelsius = 0;
float lastAmbiMoistPercent = 0;

// Timers to trigger certain tasks at intervals
BlynkTimer timer100ms;
BlynkTimer timer1s;
BlynkTimer timer2s;
BlynkTimer timer5s;

// ================ UITILS ===============

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =========== WINDOW MANAGERS ============

void WindowVals_Init(float* window, float val = 0) {
  for (int i = 0; i < WINDOW_SIZE - 1; ++i) {
    window[i] = val;
  }
}

void WindowVals_Push(float* window, float newValue) {
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    window[i] = window[i + 1];
  }
  window[WINDOW_SIZE - 1] = newValue;
}

float WindowVals_GetEMA(float* window, float newValue, float alpha = WINDOW_EMAALPHA) {
  float currentEMA = window[WINDOW_SIZE - 1];
  float newEMA = (alpha * newValue) + ((1.0 - alpha) * currentEMA);
  WindowVals_Push(window, newEMA);
  return newEMA;
}

// ============ SOIL MOISTURE ============

void SoilMoist_Init() {
  pinMode(PIN_SOILMOIST_A, INPUT);
  WindowVals_Init(windowSoilMoist);
}

void SoilMoist_ReadAnalog() {
  uint16_t val = analogRead(PIN_SOILMOIST_A);
  // ignore analog 0 - rarely we will get exact 0 - so 0 means something went wrong
  if (val == 0) {
    // Serial.println("SoilMoist_ReadAnalog: warning: soil moisture is 0");
    return;
  }
  // max value of 4095 indicates 12 bit ADC readings
  // higher analog value means more dry - so 4095 maps to 0%
  lastSoilMoist = map_float(val, 0, 4095, 100, 0);
}

void SoilMoist_BlynkWrite() {
  Blynk.virtualWrite(V0, lastSoilMoist);
}

// ============ LIGHT SENSOR ============

void Light_Init() {
  pinMode(PIN_LIGHT_SIG, INPUT);
  WindowVals_Init(windowLight);
}

void Light_ReadAnalog() {
  uint16_t val = analogRead(PIN_LIGHT_SIG);
  // ignore analog 0 - rarely we will get exact 0 - so 0 means something went wrong
  if (val == 0) {
    // Serial.println("Light_ReadAnalog: warning: light is 0");
    return;
  }
  // Convert 12-bit ADC value (0-4095) to voltage (0-3.3V)
  float voltage = map_float(val, 0, 4095, 0, 3.3);
  // Convert voltage to current, assume the default 10kohm resistor, use Ohm's law
  float current = voltage / 10e3;
  // 2 microamps per lux
  float lux = current * 2e6;
  lastLight = lux;
}

void Light_BlynkWrite() {
  Blynk.virtualWrite(V1, lastLight);
}

// ============ DHT22 (TEMPERATURE & HUMIDITY) ============

void DHT22_Init() {
  dht.begin();
  WindowVals_Init(windowAmbiTempCelsius);
  WindowVals_Init(windowAmbiMoistPercent);
}

void DHT22_ReadTemperature() {
  float val = dht.readTemperature();
  // Do NOT use NaN
  if (isnan(val)) {
    // Serial.println("DHT22_ReadTemperature: error: temperature is NaN");
    return;
  }
  lastAmbiTempCelsius = val;
}

void DHT22_ReadHumidity() {
  float val = dht.readHumidity();
  // Do NOT use NaN
  if (isnan(val)) {
    // Serial.println("DHT22_ReadHumidity: error: humidity is NaN");
    return;
  }
  lastAmbiMoistPercent = val;
}

void DHT22_Read() {
  DHT22_ReadTemperature();
  DHT22_ReadHumidity();
}

void DHT22_BlynkWrite() {
  Blynk.virtualWrite(V2, lastAmbiTempCelsius);
  Blynk.virtualWrite(V3, lastAmbiMoistPercent);
}

// ============ MOTOR ============

void Pump_Init() {
  pinMode(PIN_PUMP_ISON, OUTPUT);
  pinMode(PIN_PUMP_ENB, OUTPUT);
  // Write OFF state to Blynk on startup
  Pump_BlynkWrite();
}

void Pump_SetIsOn(bool state) {
  pumpIsOn = state;
  digitalWrite(PIN_PUMP_ISON, pumpIsOn ? HIGH : LOW);
  // Uses PWM with max res of 8 bits (so max val of 255)
  analogWrite(PIN_PUMP_ENB, pumpIsOn ? (pumpSpeed * 255) : 0);
}

BLYNK_WRITE(V4) {
  int val = !!param.asInt();
  Pump_SetIsOn(val);
  Serial.printf("BLYNK_WRITE(V4): %s\n", val ? "ON" : "OFF");
}

BLYNK_WRITE(V5) {
  int val = constrain(param.asInt(), 0, 100);
  pumpSpeed = (float)val / 100;
  Pump_SetIsOn(pumpIsOn);
  Serial.printf("BLYNK_WRITE(V5): %d\n", val);
}

void Pump_BlynkWrite() {
  Blynk.virtualWrite(V4, pumpIsOn);
  Blynk.virtualWrite(V5, pumpSpeed);
}

// ========= DEVICE STATE ===========

void DeviceState_UploadAndSet() {
  // Write current state or HIGH (after reset)
  Blynk.virtualWrite(V6, mcuState);
  // Set to LOW to indicate normal operation
  if (mcuState == true) {
    mcuState = false;
  }
}

// ============ TIMED FN ============

void Timed_100ms() {
  // State
  DeviceState_UploadAndSet();

  // Soil
  SoilMoist_ReadAnalog();
  lastSoilMoist = WindowVals_GetEMA(windowSoilMoist, lastSoilMoist);
  SoilMoist_BlynkWrite();

  // Light
  Light_ReadAnalog();
  lastLight = WindowVals_GetEMA(windowLight, lastLight);
  Light_BlynkWrite();
}

void Timed_1s() {
}

void Timed_2s() {
}

void Timed_5s() {
  // DHT
  DHT22_Read();
  lastAmbiTempCelsius = WindowVals_GetEMA(windowAmbiTempCelsius, lastAmbiTempCelsius);
  lastAmbiMoistPercent = WindowVals_GetEMA(windowAmbiMoistPercent, lastAmbiMoistPercent);
  DHT22_BlynkWrite();

  // Print
  Serial.printf("\n");
  Serial.printf("Soil Moisture:     %f\n", lastSoilMoist);
  Serial.printf("Light:             %f\n", lastLight);
  Serial.printf("DHT22 Temperature: %f\n", lastAmbiTempCelsius);
  Serial.printf("DHT22 Humidity:    %f\n", lastAmbiMoistPercent);
}

void setup() {
  // Serial
  Serial.begin(115200);
  Serial.println("Serial.begin: done");

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connect: done");
  Serial.println(WiFi.localIP());

  // Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  Serial.print("Connecting to Blynk");
  while (!Blynk.connect()) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nBlynk.connect: done");

  // Sensors
  SoilMoist_Init();
  Serial.println("SoilMoist_Init: done");
  Light_Init();
  Serial.println("Light_Init: done");
  DHT22_Init();
  Serial.println("DHT22_Init: done");
  Pump_Init();
  Serial.println("Pump_Init: done");

  // First state upload - creates the spike on boot or reset
  DeviceState_UploadAndSet();
  Serial.println("DeviceState_UploadAndSet: done");

  // Schedule all tasks
  timer100ms.setInterval(100, Timed_100ms);
  timer1s.setInterval(1000, Timed_1s);
  timer2s.setInterval(2000, Timed_2s);
  timer5s.setInterval(5000, Timed_5s);
  Serial.println("timer.setInterval: done");
}

void loop() {
  Blynk.run();
  timer100ms.run();
  timer1s.run();
  timer2s.run();
  timer5s.run();
  // yield to avoid any WDT resets
  yield();
}
