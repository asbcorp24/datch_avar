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
#include <Preferences.h>  
// === LoRa параметры и пины ===
 uint8_t clientId;   
#define LORA_FREQ       868E6
#define PIN_SCK         18
#define PIN_MISO        19
#define PIN_MOSI        23
#define PIN_SS          5
#define PIN_RST         14
#define PIN_DIO0        26

// === Пины сенсоров ===
const uint8_t bendPins[5] = {32, 33, 34, 35, 36};  // аналоговые датчики изгиба
#define TILT_PIN        2                         // KS0025 Tilt Sensor (вибрация)
#define SHOCK_PIN       13                         // детектор удара (прерывание)
#define ONEWIRE_BUS     25                         // шина DS18B20

// === Объекты датчиков ===
Adafruit_VL53L0X lox       = Adafruit_VL53L0X();
Adafruit_HMC5883_Unified mag(12345);
OneWire oneWire(ONEWIRE_BUS);
DallasTemperature tempSensor(&oneWire);
RTC_DS3231 rtc;

// Память между deep-sleep циклами
RTC_DATA_ATTR uint16_t prevBuf[15];
RTC_DATA_ATTR bool     hasPrev = false;
volatile bool          shockDetected = false;

// Обработчик прерывания от удара
void IRAM_ATTR shockISR() {
  shockDetected = true;
}

// Безопасное чтение изгиба (маркер отсутствия датчика = 0xFFFF)
uint16_t readBend(uint8_t pin) {
  int v = analogRead(pin);
  return (v < 5 || v > 4090) ? 0xFFFF : (uint16_t)v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Preferences prefs;
  prefs.begin("client_cfg", false);

  // Попытаться получить ранее сохранённый ID (0 означает «нет»)
  clientId = prefs.getUChar("id", 0);
  if (clientId == 0) {
    // Генерируем случайный ID от 1 до 255
    randomSeed(esp_random());           // инициализируем генератор
    clientId = random(1, 256);
    prefs.putUChar("id", clientId);     // сохраняем в NVS
    Serial.printf("Generated new clientId=%u and saved to NVS\n", clientId);
  } else {
    Serial.printf("Loaded clientId=%u from NVS\n", clientId);
  }

  prefs.end();  // закрываем NVS
  Serial.println("--- ESP32 Client w/ HMC5883L ---");

  // RTC
  if (!rtc.begin())        while (1) delay(10);
  if (rtc.lostPower())     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // DS18B20
  tempSensor.begin();

  // ToF VL53L0X
  if (!lox.begin()) {
    Serial.println("VL53L0X init failed");
    while (1) delay(10);
  }
  Serial.println("VL53L0X initialized");

  // Magnetometer HMC5883L
  if (!mag.begin()) {
    Serial.println("HMC5883L init failed");
    while (1) delay(10);
  }
  Serial.println("HMC5883L initialized");

  // Tilt Sensor (KS0025)
  pinMode(TILT_PIN, INPUT_PULLUP);

  // Shock Interrupt
 // pinMode(SHOCK_PIN, INPUT_PULLUP);
//attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);

  // LoRa init
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed");
    while (1) delay(10);
  }
  Serial.printf("LoRa @ %.1f MHz ready\n", LORA_FREQ / 1e6);

  // Настройка deep sleep (таймер + внешний wakeup по ударам)
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  //esp_sleep_enable_ext0_wakeup((gpio_num_t)SHOCK_PIN, 0);
}

void loop() {
  // Причина пробуждения
  auto cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT0)       Serial.println("Wakeup: shock detected");
  else if (cause == ESP_SLEEP_WAKEUP_TIMER) Serial.println("Wakeup: timer");

  // Сбор данных
  uint16_t buf[15], delta[15];

  // 1–5: изгиб
  for (int i = 0; i < 5; i++) {
    buf[i] = readBend(bendPins[i]);
  }

  // 6–10: расстояние (ToF)
  for (int i = 0; i < 5; i++) {
    VL53L0X_RangingMeasurementData_t m;
    lox.rangingTest(&m, false);
    buf[5 + i] = (m.RangeStatus == 4) ? 0xFFFF : m.RangeMilliMeter;
    delay(10);
  }

  // 11: температура (DS18B20), сотые градуса
  tempSensor.requestTemperatures();
  buf[10] = (uint16_t)(tempSensor.getTempCByIndex(0) * 100);

  // 12: вибрация/наклон (KS0025) — LOW при срабатывании
  buf[11] = (digitalRead(TILT_PIN) == LOW) ? 1 : 0;

  // 13–15: магнитометр X, Y, Z
  sensors_event_t e;
  mag.getEvent(&e);
  buf[12] = (uint16_t)e.magnetic.x;
  buf[13] = (uint16_t)e.magnetic.y;
  buf[14] = (uint16_t)e.magnetic.z;

  // Вычисление дельт
  for (int i = 0; i < 15; i++) {
    if (!hasPrev || buf[i] == 0xFFFF || prevBuf[i] == 0xFFFF) {
      delta[i] = 0xFFFF;
    } else {
      delta[i] = abs((int)buf[i] - (int)prevBuf[i]);
    }
  }
  hasPrev = true;
  memcpy(prevBuf, buf, sizeof(buf));

  // Метка времени UNIX
  uint32_t ts = rtc.now().unixtime();

  // Отправка: CLIENT_ID + 15 abs + 15 delta + timestamp
  LoRa.beginPacket();
        LoRa.write(clientId);
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

  Serial.println("Client: Sent data + deltas");

  // Окно приёма для синхронизации времени (200 мс)
  LoRa.receive();
  unsigned long start = millis();
  while (millis() - start < 200) {
    int sz = LoRa.parsePacket();
    if (sz == 5 && LoRa.read() == 0xFF) {
      uint32_t newTs = 0;
      for (int j = 0; j < 4; j++) {
        newTs = (newTs << 8) | LoRa.read();
      }
      rtc.adjust(DateTime(newTs));
      Serial.println("Client: RTC synced");
      break;
    }
  }

  Serial.println("Client: Entering deep sleep\n");
  esp_deep_sleep_start();
}
