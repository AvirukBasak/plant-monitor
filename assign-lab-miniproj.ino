#include "secrets.hpp"

#include <cmath>

#include "DHT.h"
#include "BlynkSimpleEsp32.h"

#define PIN_SOILMOIST_A (34)  // ADC1, i/p only - not an issue
#define PIN_LIGHT_SIG (35)    // ADC1, i/p only - not an issue
#define PIN_DHT22_DATA (32)   // ADC not needed, both i/o needed
#define PIN_PUMP_ISON (26)    // Needs o/p capable pin
#define PIN_PUMP_ENB (27)     // PWM pin, o/p capable needed

#define PWM_FREQ (100)
#define PWM_RES (8)
#define PWM_MAXVAL ((1 << PWM_RES) - 1)

#define PUMP_MAPPEDMIN (0.15)
#define PUMP_MAPPEDMAX (0.8)
#define PUMP_STARTSPEED (0.4)
#define PUMP_STARTDELAY (100)

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
bool pumpIsOn = false;       // v4
int pumpSpeed = 10;          // v5

// Spikes on cloud chart eery time device boots - count of resets
bool mcuReset = true;        // v6

// Auto pump start threshold
int soilMoistThrMin = 50;    // v7
int soilMoistThrMax = 60;    // v8

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
  ledcAttachChannel(PIN_PUMP_ENB, PWM_FREQ, PWM_RES, LEDC_CHANNEL_0);
}

void Pump_SetState(bool state, int speed) {
  // State
  const bool oldState = pumpIsOn;
  pumpIsOn = state;

  // Only if there has been a state change
  if (state != oldState) {
    digitalWrite(PIN_PUMP_ISON, state ? HIGH : LOW);
  }

  // Speed
  const int oldSpeed = pumpSpeed;
  pumpSpeed = speed;

  // Only if there has been a state or speed change
  if (state != oldState || speed != oldSpeed) {
    // Speed % adjusted b/w 0 and 1; hard 0 or map in the MIN-MAX range
    const float mappedSpeed = (speed < 1) ? 0 : MapF(speed, 1, 100, PUMP_MAPPEDMIN, PUMP_MAPPEDMAX);
    // Either motor turned on with non-zero speed or overall speed increased
    if ((!oldState && state && speed > 0) || speed > oldSpeed) {
      // Target speed is below the "start" speed
      if (mappedSpeed < PUMP_STARTSPEED) {
        // Speed "jump" to give motor an inital "push"
        ledcWrite(PIN_PUMP_ENB, state ? (PUMP_STARTSPEED * PWM_MAXVAL) : 0);
        delay(PUMP_STARTDELAY);
      }
    }
    // Stabilize to target speed after delay
    ledcWrite(PIN_PUMP_ENB, state ? (mappedSpeed * PWM_MAXVAL) : 0);
  }
}

BLYNK_WRITE(V4) {
  int val = !!param.asInt();
  Pump_SetState(val, pumpSpeed);
  Serial.printf("BLYNK_WRITE(V4): %s\n", val ? "ON" : "OFF");
}

BLYNK_WRITE(V5) {
  int val = constrain(param.asInt(), 0, 100);
  Pump_SetState(pumpIsOn, val);
  Serial.printf("BLYNK_WRITE(V5): %d\n", val);
}

void Pump_BlynkWrite() {
  Blynk.virtualWrite(V4, pumpIsOn);
  Blynk.virtualWrite(V5, pumpSpeed);
}

// ========= MOISTURE THR ===========

BLYNK_WRITE(V7) {
  int val = constrain(param.asInt(), 0, 100);
  soilMoistThrMin = val;
  Serial.printf("BLYNK_WRITE(V7): %d\n", val);
}

BLYNK_WRITE(V8) {
  int val = constrain(param.asInt(), 0, 100);
  soilMoistThrMax = val;
  Serial.printf("BLYNK_WRITE(V7): %d\n", val);
}

void MoistThr_BlynkWrite() {
  Blynk.virtualWrite(V7, soilMoistThrMin);
  Blynk.virtualWrite(V8, soilMoistThrMax);
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

// ======= AUTO DESCISION LOGIC =========

void AutoDecisionLogic() {
  if (soilMoist.Ema < soilMoistThrMin) {
    Pump_SetState(true, pumpSpeed);
    Blynk.virtualWrite(V4, pumpIsOn);
  }
  if (soilMoist.Ema > soilMoistThrMax) {
    Pump_SetState(false, pumpSpeed);
    Blynk.virtualWrite(V4, pumpIsOn);
  }
}

// ============ TIMED FN ============

void Timed_100ms() {
  DeviceReset_SetAndUpload();
  AutoDecisionLogic();
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

  MoistThr_BlynkWrite();
  Serial.println("MoistThr_BlynkWrite: done");

  Pump_BlynkWrite();
  Serial.println("Pump_BlynkWrite: done");

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
