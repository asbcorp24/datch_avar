#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_VL53L0X.h>
#include <esp_sleep.h>

// LoRa parameters
#define LORA_FREQ       868E6
#define PIN_SCK         5
#define PIN_MISO        19
#define PIN_MOSI        27
#define PIN_SS          18
#define PIN_RST         14
#define PIN_DIO0        26

// Sensor pins
const uint8_t bendPins[5] = {32, 33, 34, 35, 36};   // analog bend sensors
#define VIB_PIN         15                          // digital SW18010P vibration sensor
#define SHOCK_PIN       13                          // shock interrupt sensor
#define ONEWIRE_BUS     25                          // DS18B20 bus

// VL53L0X Time-of-Flight
Adafruit_VL53L0X lox;

// Persistent memory across deep sleep
RTC_DATA_ATTR uint16_t prevBuf[12];
RTC_DATA_ATTR bool hasPrev = false;

// Objects
OneWire oneWire(ONEWIRE_BUS);
DallasTemperature tempSensor(&oneWire);
RTC_DS3231 rtc;
volatile bool shockDetected = false;

// Interrupt handler for shock event
void IRAM_ATTR shockISR() {
  shockDetected = true;
}

// Function to safely read bend sensor or mark as missing
uint16_t readBend(uint8_t pin) {
  int val = analogRead(pin);
  // assume sensor disconnected if reading near 0 (floating) or max
  if (val < 5 || val > 4090) return 0xFFFF; // -1 marker
  return (uint16_t)val;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("--- ESP32 Client with Fault Marking ---");

  // Initialize RTC
  if (!rtc.begin()) { Serial.println("RTC init failed"); while (1) delay(10); }
  if (rtc.lostPower()) { rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); Serial.println("RTC set to compile time"); }

  // Initialize DS18B20
  tempSensor.begin();

  // Initialize VL53L0X
  if (!lox.begin()) { Serial.println("Failed to boot VL53L0X"); while (1) delay(10); }
  Serial.println("VL53L0X initialized");

  // Setup shock sensor interrupt
  pinMode(SHOCK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);

  // Setup vibration sensor
  pinMode(VIB_PIN, INPUT);

  // Initialize LoRa
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
  if (!LoRa.begin(LORA_FREQ)) { Serial.println("LoRa init failed"); while (1) delay(10); }
  Serial.print("LoRa @ "); Serial.print(LORA_FREQ/1e6); Serial.println(" MHz ready");

  // Configure wakeup sources
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SHOCK_PIN, 0);
}

void loop() {
  // Determine wakeup cause
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT0) Serial.println("Wakeup: shock detected");
  else if (cause == ESP_SLEEP_WAKEUP_TIMER) Serial.println("Wakeup: timer");

  // Read sensors into buffer
  uint16_t buf[12];
  // Bend sensors
  for (uint8_t i = 0; i < 5; i++) {
    buf[i] = readBend(bendPins[i]);
  }
  // Extension sensors via VL53L0X
  for (uint8_t i = 0; i < 5; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus == 4) buf[5 + i] = 0xFFFF;
    else buf[5 + i] = measure.RangeMilliMeter;
    delay(10);
  }
  // Temperature
  tempSensor.requestTemperatures();
  buf[10] = (uint16_t)(tempSensor.getTempCByIndex(0) * 100);
  // Vibration
  buf[11] = digitalRead(VIB_PIN) ? 1 : 0;

  // Calculate deltas
  uint16_t delta[12];
  if (hasPrev) {
    for (uint8_t i = 0; i < 12; i++) {
      if (buf[i] == 0xFFFF || prevBuf[i] == 0xFFFF) delta[i] = 0xFFFF;
      else delta[i] = abs((int)buf[i] - (int)prevBuf[i]);
    }
  } else {
    for (uint8_t i = 0; i < 12; i++) delta[i] = 0xFFFF;
    hasPrev = true;
  }
  memcpy(prevBuf, buf, sizeof(buf));

  // Timestamp
  uint32_t ts = rtc.now().unixtime();

  // Transmit absolute + delta + timestamp
  LoRa.beginPacket();
  for (uint8_t i = 0; i < 12; i++) {
    LoRa.write((buf[i] >> 8) & 0xFF);
    LoRa.write(buf[i] & 0xFF);
  }
  for (uint8_t i = 0; i < 12; i++) {
    LoRa.write((delta[i] >> 8) & 0xFF);
    LoRa.write(delta[i] & 0xFF);
  }
  LoRa.write((ts >> 24) & 0xFF);
  LoRa.write((ts >> 16) & 0xFF);
  LoRa.write((ts >> 8) & 0xFF);
  LoRa.write(ts & 0xFF);
  LoRa.endPacket();
  Serial.println("Client: Sent data + deltas");

  // Brief listen for RTC sync
  LoRa.receive(); unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    int sz = LoRa.parsePacket();
    if (sz == 5 && LoRa.read() == 0xFF) {
      uint32_t newTs = 0;
      for (uint8_t j = 0; j < 4; j++) newTs = (newTs << 8) | LoRa.read();
      rtc.adjust(DateTime(newTs)); Serial.println("Client: RTC synced");
      break;
    }
  }

  Serial.println("Client: Entering deep sleep\n");
  esp_deep_sleep_start();
}
