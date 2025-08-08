// src/main.cpp
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <esp_sleep.h>

// === Настройки LoRa и пинов ===
#define CLIENT_ID       CLIENT_ID      // уникальный ID из build_flags
#define LORA_FREQ       868E6
#define PIN_SCK         5
#define PIN_MISO        19
#define PIN_MOSI        27
#define PIN_SS          18
#define PIN_RST         14
#define PIN_DIO0        26

const uint8_t bendPins[5] = {32,33,34,35,36};  // аналоговые датчики изгиба
#define TILT_PIN        15                     // цифровой KS0025 Tilt Sensor
#define SHOCK_PIN       13                     // прерывающийся датчик удара
#define ONEWIRE_BUS     25                     // DS18B20 шина

// === Объекты ===
Adafruit_VL53L0X lox;
Adafruit_HMC5883_Unified mag(12345);
OneWire oneWire(ONEWIRE_BUS);
DallasTemperature tempSensor(&oneWire);
RTC_DS3231 rtc;

// Память между глубокими снами для предыдущих значений (15 каналов)
RTC_DATA_ATTR uint16_t prevBuf[15];
RTC_DATA_ATTR bool hasPrev = false;
volatile bool shock = false;

// Обработчик прерывания от удара
void IRAM_ATTR shockISR() {
  shock = true;
}

// Безопасное чтение датчиков изгиба
uint16_t readBend(uint8_t pin) {
  int v = analogRead(pin);
  return (v < 5 || v > 4090) ? 0xFFFF : (uint16_t)v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // RTC
  if (!rtc.begin()) while (1) delay(10);
  if (rtc.lostPower()) 
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // DS18B20
  tempSensor.begin();

  // VL53L0X
  if (!lox.begin()) { Serial.println("VL53L0X init failed"); while(1) delay(10); }

  // HMC5883L
  if (!mag.begin()) { Serial.println("HMC5883L init failed"); while(1) delay(10); }

  // Tilt Sensor (KS0025) как цифровой вход
  pinMode(TILT_PIN, INPUT_PULLUP);  // подтяжка, LOW при замыкании
  // Прерывание по удару
  pinMode(SHOCK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);

  // LoRa
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
  if (!LoRa.begin(LORA_FREQ)) while (1) delay(10);

  // Deep sleep: таймер и внешний пин (удар или наклон)
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SHOCK_PIN, 0);
}

void loop() {
  // Определяем причину пробуждения
  auto cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT0)       Serial.println("Wakeup: shock");
  else if (cause == ESP_SLEEP_WAKEUP_TIMER) Serial.println("Wakeup: timer");

  // Собираем данные
  uint16_t buf[15], delta[15];
  for (int i = 0; i < 5; i++) {
    buf[i] = readBend(bendPins[i]);
  }
  for (int i = 0; i < 5; i++) {
    VL53L0X_RangingMeasurementData_t m;
    lox.rangingTest(&m, false);
    buf[5+i] = (m.RangeStatus == 4) ? 0xFFFF : m.RangeMilliMeter;
    delay(10);
  }
  tempSensor.requestTemperatures();
  buf[10] = (uint16_t)(tempSensor.getTempCByIndex(0) * 100);
  // Читаем Tilt Sensor: при наклоне/вибрации – LOW (замыкание) :contentReference[oaicite:1]{index=1}
  buf[11] = (digitalRead(TILT_PIN) == LOW) ? 1 : 0;
  // Магнитометр X, Y, Z
  sensors_event_t ev;
  mag.getEvent(&ev);
  buf[12] = (uint16_t)ev.magnetic.x;
  buf[13] = (uint16_t)ev.magnetic.y;
  buf[14] = (uint16_t)ev.magnetic.z;

  // Вычисляем дельты
  for (int i = 0; i < 15; i++) {
    if (!hasPrev || buf[i] == 0xFFFF || prevBuf[i] == 0xFFFF) {
      delta[i] = 0xFFFF;
    } else {
      delta[i] = abs((int)buf[i] - (int)prevBuf[i]);
    }
  }
  hasPrev = true;
  memcpy(prevBuf, buf, sizeof(buf));

  // Метка времени
  uint32_t ts = rtc.now().unixtime();

  // Отправка по LoRa: ID + 15×abs + 15×delta + timestamp
  LoRa.beginPacket();
    LoRa.write(CLIENT_ID);
    for (int i = 0; i < 15; i++) {
      LoRa.write((buf[i] >> 8) & 0xFF);
      LoRa.write(buf[i] & 0xFF);
    }
    for (int i = 0; i < 15; i++) {
      LoRa.write((delta[i] >> 8) & 0xFF);
      LoRa.write(delta[i] & 0xFF);
    }
    LoRa.write((ts >> 24) & 0xFF);
    LoRa.write((ts >> 16) & 0xFF);
    LoRa.write((ts >> 8) & 0xFF);
    LoRa.write(ts & 0xFF);
  LoRa.endPacket();

  // Небольшое окно приёма для синхронизации времени
  LoRa.receive();
  unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    int sz = LoRa.parsePacket();
    if (sz == 5 && LoRa.read() == 0xFF) {
      uint32_t nts = 0;
      for (int j = 0; j < 4; j++) {
        nts = (nts << 8) | LoRa.read();
      }
      rtc.adjust(DateTime(nts));
      break;
    }
  }

  // Возврат в глубокий сон
  esp_deep_sleep_start();
}
