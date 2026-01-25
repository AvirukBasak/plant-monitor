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

// ================ UTILS ===================

class SensorValue {

  float alpha;

public:

  float Ema;
  float LastVal;

  SensorValue(float alpha = 0.5)
    : alpha{ alpha }, Ema{ 0 }, LastVal{ 0 } {
  }

  void AddVal(float val) {
    this->Ema = (alpha * val) + ((1.0 - alpha) * Ema);
    this->LastVal = val;
  }
};

float MapF(float val, float inMin, float inMax, float outMin, float outMax) {
  return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ================== MAIN CODE ==================

DHT dht(PIN_DHT22_DATA, DHT22);

// EMA value (with last sensor reading)
SensorValue soilMoist(0.8);  // v0
SensorValue light(0.8);      // v1
SensorValue ambiTemp(0.8);   // v2
SensorValue ambiHumid(0.8);  // v3

// Pump speed and states received from cloud
bool pumpIsOn = false;  // v4
float pumpSpeed = 0.0;  // v5

// Spikes on cloud chart eery time device boots - count of resets
bool mcuReset = true;  // v6

// Timers to trigger certain tasks at intervals
BlynkTimer timer100ms;
BlynkTimer timer5s;

// ============ SOIL MOISTURE ============

void SoilMoist_Init() {
  pinMode(PIN_SOILMOIST_A, INPUT);
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
  soilMoist.AddVal(MapF(val, 0, 4095, 100, 0));
}

void SoilMoist_BlynkWrite() {
  Blynk.virtualWrite(V0, soilMoist.Ema);
}

// ============ LIGHT SENSOR ============

void Light_Init() {
  pinMode(PIN_LIGHT_SIG, INPUT);
}

void Light_ReadAnalog() {
  uint16_t val = analogRead(PIN_LIGHT_SIG);
  // ignore analog 0 - rarely we will get exact 0 - so 0 means something went wrong
  if (val == 0) {
    // Serial.println("Light_ReadAnalog: warning: light is 0");
    return;
  }
  // Convert 12-bit ADC value (0-4095) to voltage (0-3.3V)
  float voltage = MapF(val, 0, 4095, 0, 3.3);
  // Convert voltage to current, assume the default 10kohm resistor, use Ohm's law
  float current = voltage / 10e3;
  // 2 microamps per lux
  float lux = current * 2e6;
  light.AddVal(lux);
}

void Light_BlynkWrite() {
  Blynk.virtualWrite(V1, light.Ema);
}

// ============ DHT22 (TEMPERATURE & HUMIDITY) ============

void DHT22_Init() {
  dht.begin();
}

void DHT22_ReadTemperature() {
  float val = dht.readTemperature();
  // Do NOT use NaN
  if (isnan(val)) {
    // Serial.println("DHT22_ReadTemperature: error: temperature is NaN");
    return;
  }
  ambiTemp.AddVal(val);
}

void DHT22_ReadHumidity() {
  float val = dht.readHumidity();
  // Do NOT use NaN
  if (isnan(val)) {
    // Serial.println("DHT22_ReadHumidity: error: humidity is NaN");
    return;
  }
  ambiHumid.AddVal(val);
}

void DHT22_Read() {
  DHT22_ReadTemperature();
  DHT22_ReadHumidity();
}

void DHT22_BlynkWrite() {
  Blynk.virtualWrite(V2, ambiTemp.Ema);
  Blynk.virtualWrite(V3, ambiHumid.Ema);
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
  pumpSpeed = MapF(val, 0, 100, 0, 1);
  Pump_SetIsOn(pumpIsOn);
  Serial.printf("BLYNK_WRITE(V5): %d\n", val);
}

void Pump_BlynkWrite() {
  Blynk.virtualWrite(V4, pumpIsOn);
  Blynk.virtualWrite(V5, pumpSpeed);
}

// ========= DEVICE STATE ===========

void DeviceReset_SetAndUpload() {
  // Write current state or HIGH (after reset)
  Blynk.virtualWrite(V6, mcuReset);
  // Set to LOW to indicate normal operation
  if (mcuReset == true) {
    mcuReset = false;
  }
}

// ============ TIMED FN ============

void Timed_100ms() {
  DeviceReset_SetAndUpload();
}

void Timed_5s() {
  SoilMoist_ReadAnalog();
  SoilMoist_BlynkWrite();

  Light_ReadAnalog();
  Light_BlynkWrite();

  DHT22_Read();
  DHT22_BlynkWrite();

  Serial.printf("\n");
  Serial.printf("Soil Moisture:     (%.2f, %.2f)\n", soilMoist.LastVal, soilMoist.Ema);
  Serial.printf("Light:             (%.2f, %.2f)\n", light.LastVal, light.Ema);
  Serial.printf("DHT22 Temperature: (%.2f, %.2f)\n", ambiTemp.LastVal, ambiTemp.Ema);
  Serial.printf("DHT22 Humidity:    (%.2f, %.2f)\n", ambiHumid.LastVal, ambiHumid.Ema);
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
  DeviceReset_SetAndUpload();
  Serial.println("DeviceReset_SetAndUpload: done");

  // Schedule all tasks
  timer100ms.setInterval(100, Timed_100ms);
  timer5s.setInterval(5000, Timed_5s);
  Serial.println("timer.setInterval: done");
}

void loop() {
  Blynk.run();
  timer100ms.run();
  timer5s.run();
  // yield to avoid any WDT resets
  yield();
}
