#undef HMC5883L_ADDRESS
#define HMC5883L_ADDRESS 0x2C
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RtcDS1302.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_Sensor.h>
#include <MechaQMC5883.h>     // MechaQMC5883L library
//#include <Adafruit_HMC5883_U.h>
#include <esp_sleep.h>
#include <Preferences.h>  
#define LORA_DIAG 1
#define DEBUG 1
#if DEBUG
  #define DBG(...)    Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
#endif
// === LoRa параметры и пины ===
 uint8_t clientId;   
#define LORA_FREQ       433E6
#define PIN_SCK         18
#define PIN_MISO        19
#define PIN_MOSI        23
#define PIN_SS          16
#define PIN_RST         14
#define PIN_DIO0        26

// === Пины сенсоров ===
const uint8_t bendPins[5] = {32, 37, 34, 35, 36};  // аналоговые датчики изгиба
#define TILT_PIN        39                         // KS0025 Tilt Sensor (вибрация)
#define SHOCK_PIN       27                         // детектор удара (прерывание)
#define ONEWIRE_BUS     33                         // шина DS18B20

// === Объекты датчиков ===
Adafruit_VL53L0X lox       = Adafruit_VL53L0X();
//Adafruit_HMC5883_Unified mag(12345);
MechaQMC5883 qmc;
OneWire oneWire(ONEWIRE_BUS);
DallasTemperature tempSensor(&oneWire);   // ← ЭТОГО НЕ ХВАТАЛО
ThreeWire myWire(4, 5, 17); // DAT=4, CLK=5, RST=17
RtcDS1302<ThreeWire> rtc(myWire);

Preferences prefs;
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
  // --- Serial & debug output ---
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  while (!Serial) delay(1);
  DBG("Serial initialized at 115200 bps\n");
  
  // --- I2C ---
  Serial.println("=== I2C Debug Start ===");
  Wire.begin(21, 22);
  Wire.setClock(100000);
  Serial.println("Wire.begin(21,22) at 100 kHz");
Serial.println("Scanning I2C bus...");
for (uint8_t addr = 1; addr < 127; addr++) {
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) {
    Serial.printf("  Found device at 0x%02X\n", addr);
  }
}

  // --- NVS (Preferences) для clientId ---
  prefs.begin("client_cfg", false);
  clientId = prefs.getUChar("id", 0);
  if (clientId == 0) {
    randomSeed(esp_random());
    clientId = random(1, 256);
    prefs.putUChar("id", clientId);
    DBG("Generated new clientId = %u and saved to NVS\n", clientId);
  } else {
    DBG("Loaded clientId = %u from NVS\n", clientId);
  }
  prefs.end();

  DBG("--- Starting full initialization ---\n");


  // --- RTC
  rtc.Begin();
 
rtc.SetIsWriteProtected(false);
if (!rtc.GetIsRunning()) {
    Serial.println("RTC not running, setting compile time");
    rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));

}
RtcDateTime now = rtc.GetDateTime();
Serial.printf("%04u-%02u-%02u %02u:%02u:%02u\n",
              now.Year(), now.Month(), now.Day(),
              now.Hour(), now.Minute(), now.Second());
 /* if (!rtc.begin()) {
    DBG("ERROR: RTC.begin() failed\n");
    while (1) delay(10);
  }
  DBG("RTC initialized\n");
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    DBG("RTC lost power - time set to compile time\n");
  }
  DateTime now = rtc.now();
  DBG("RTC current time: %04u-%02u-%02u %02u:%02u:%02u\n",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second());
*/
  // --- DS18B20 temperature sensor ---
  tempSensor.begin();
  DBG("DS18B20 OneWire bus initialized\n");
  tempSensor.requestTemperatures();
  float tC = tempSensor.getTempCByIndex(0);
  DBG("DS18B20 first sensor temp: %.2f °C\n", tC);

  // --- VL53L0X ToF sensor ---
  if (!lox.begin()) {
    DBG("ERROR: VL53L0X.begin() failed\n");
    while (1) delay(10);
  }
  DBG("VL53L0X initialized successfully\n");
  // Однократная проверка измерения
  {
    VL53L0X_RangingMeasurementData_t m;
    lox.rangingTest(&m, false);
    DBG("VL53L0X initial RangeStatus=%u, Distance=%u mm\n",
        m.RangeStatus,
        (m.RangeStatus == 4) ? 0 : m.RangeMilliMeter);
  }

  // --- HMC5883L magnetometer ---
 /* if (!mag.begin()) {
    DBG("ERROR: HMC5883L.begin() failed\n");
    while (1) delay(10);
  }
  DBG("HMC5883L magnetometer initialized\n");
  {
    sensors_event_t ev;
    mag.getEvent(&ev);
    DBG("HMC5883L initial mag: X=%d, Y=%d, Z=%d\n",
        (int)ev.magnetic.x,
        (int)ev.magnetic.y,
        (int)ev.magnetic.z);
  }
*/
 
// --- QMC5883L magnetometer ---
  // Если твой модуль виден, как 0x0D — адрес по умолчанию подходит.
  // При другом адресе можно раскомментировать строку ниже.
  // qmc.setAddress(0x0D);
  qmc.init();
  // Непрерывный режим, 50 Гц, диапазон ±2Г, oversampling 512
  qmc.setMode(Mode_Continuous, ODR_50Hz, RNG_2G, OSR_512);
  DBG("QMC5883L initialized (Mode=Continuous, ODR=50Hz, RNG=2G, OSR=512)\n");

  // --- Tilt Sensor (KS0025) ---
  pinMode(TILT_PIN, INPUT_PULLUP);
  DBG("Tilt sensor on GPIO %u configured as INPUT_PULLUP\n", TILT_PIN);

  // --- Shock sensor interrupt & deep-sleep wakeup ---
  pinMode(SHOCK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);
  DBG("Shock sensor on GPIO %u configured and interrupt attached\n", SHOCK_PIN);

  // --- LoRa initialization ---
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    DBG("ERROR: LoRa.begin() at %.1f MHz failed\n", LORA_FREQ/1e6);
    while (1) delay(10);
  }
LoRa.setSyncWord(0xA5);
LoRa.enableCrc();
//LoRa.explicitHeaderMode();      // для явного заголовка (по умолчанию и так explicit, но явно ок)
LoRa.setSignalBandwidth(125E3); // чтобы точно совпасть
LoRa.setSpreadingFactor(7);
LoRa.setCodingRate4(5);
LoRa.setTxPower(17,PA_OUTPUT_PA_BOOST_PIN); 
  DBG("LoRa initialized @ %.1f MHz\n", LORA_FREQ/1e6);
auto rd = [&](uint8_t r){ digitalWrite(PIN_SS,LOW); SPI.transfer(r&0x7F);
                          uint8_t v=SPI.transfer(0); digitalWrite(PIN_SS,HIGH); return v; };
uint8_t mc1=rd(0x1D), mc2=rd(0x1E), mc3=rd(0x26), sw=rd(0x39);
uint8_t frfMsb=rd(0x06), frfMid=rd(0x07), frfLsb=rd(0x08);
Serial.printf("LoRa cfg: 1D=0x%02X 1E=0x%02X 26=0x%02X SW=0x%02X FRF=%02X%02X%02X\n",
              mc1, mc2, mc3, sw, frfMsb, frfMid, frfLsb);

  // --- Deep Sleep configuration ---
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SHOCK_PIN, 0);
  DBG("Deep sleep configured: timer wakeup %u µs, ext0 on GPIO %u low\n",
      SLEEP_INTERVAL_US, SHOCK_PIN);

  DBG("--- Initialization complete, entering loop() ---\n");
}
/*
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
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
  //if (!rtc.begin())        while (1) delay(10);
  //if (rtc.lostPower())     rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

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
  pinMode(SHOCK_PIN, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(SHOCK_PIN), shockISR, FALLING);

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
 esp_sleep_enable_ext0_wakeup((gpio_num_t)SHOCK_PIN, 0);
}
*/
/*
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
*/
void loop() {
   static int tries = 0;
  tries++;
  // 1) Причина пробуждения
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT0) {
    DBG("[Wakeup] by shock (EXT0)\n");
  } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    DBG("[Wakeup] by timer\n");
  } else {
    DBG("[Wakeup] other cause: %d\n", cause);
  }

  // 2) Подготовка буферов
  DBG("Allocating buf[15], delta[15]\n");
  uint16_t buf[15], delta[15];

  // 3) Чтение изгибов (1–5)
  DBG("Reading bend sensors:\n");
  for (int i = 0; i < 5; i++) {
    buf[i] = readBend(bendPins[i]);
    DBG("  bend[%d] raw = %u%s\n",
        i, buf[i],
        buf[i] == 0xFFFF ? " (missing)" : "");
  }

  // 4) Чтение ToF (6–10)
  DBG("Reading VL53L0X distances:\n");
  for (int i = 0; i < 5; i++) {
    VL53L0X_RangingMeasurementData_t m;
    lox.rangingTest(&m, false);
    buf[5 + i] = (m.RangeStatus == 4) ? 0xFFFF : m.RangeMilliMeter;
    DBG("  tof[%d] status=%u, dist=%u mm%s\n",
        i, m.RangeStatus,
        buf[5 + i],
        buf[5 + i] == 0xFFFF ? " (out of range)" : "");
    delay(10);
  }

  // 5) Чтение температуры (11)
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  buf[10] = (uint16_t)(tempC * 100);
  DBG("Temperature: %.2f °C -> buf[10] = %u\n",
      tempC, buf[10]);

  // 6) Чтение Tilt (12)
  uint8_t tilt = (digitalRead(TILT_PIN) == LOW) ? 1 : 0;
  buf[11] = tilt;
  DBG("Tilt sensor: digitalRead = %u -> buf[11] = %u\n",
      digitalRead(TILT_PIN), buf[11]);

  // 7) Чтение магнитометра (13–15)
  /*sensors_event_t ev;
  mag.getEvent(&ev);
  buf[12] = (uint16_t)ev.magnetic.x;
  buf[13] = (uint16_t)ev.magnetic.y;
  buf[14] = (uint16_t)ev.magnetic.z;
  DBG("Magnetometer: X=%d, Y=%d, Z=%d -> buf[12..14] = %u, %u, %u\n",
      (int)ev.magnetic.x, (int)ev.magnetic.y, (int)ev.magnetic.z,
      buf[12], buf[13], buf[14]);
*/
 // 7) Чтение магнитометра QMC5883L (13–15) + азимут
  int x, y, z;
  if (qmc.read(&x, &y, &z) == 0) { // 0 — OK в этой либе
    buf[12] = (uint16_t)x;
    buf[13] = (uint16_t)y;
    buf[14] = (uint16_t)z;
    DBG("QMC5883L: X=%d, Y=%d, Z=%d -> buf[12..14]=%u,%u,%u\n",
        x, y, z, buf[12], buf[13], buf[14]);

    float az = qmc.azimuth(&x, &y);     // азимут в градусах (0–360)
    DBG("QMC5883L azimuth: %.2f°\n", az);
  } else {
    // Если чтение не удалось — помечаем отсутствием
    buf[12] = buf[13] = buf[14] = 0xFFFF;
    DBG("QMC5883L read() failed, mark XYZ as 0xFFFF\n");
  }
        
  // 8) Вычисление дельт
  DBG("Calculating deltas:\n");
  for (int i = 0; i < 15; i++) {
    if (!hasPrev || buf[i] == 0xFFFF || prevBuf[i] == 0xFFFF) {
      delta[i] = 0xFFFF;
      DBG("  delta[%d] = 0xFFFF (no prev or bad data)\n", i);
    } else {
      delta[i] = abs((int)buf[i] - (int)prevBuf[i]);
      DBG("  delta[%d] = |%u - %u| = %u\n", i, buf[i], prevBuf[i], delta[i]);
    }
  }
  hasPrev = true;
  memcpy(prevBuf, buf, sizeof(buf));
  DBG("Copied current buf[] to prevBuf[]\n");

  // 9) Метка времени
RtcDateTime now = rtc.GetDateTime();
uint32_t ts = now.Unix32Time();           // вместо Epoch32Time()
DBG("Current UNIX timestamp: %lu\n", ts);

  // 10) Формирование и отправка LoRa-пакета
  DBG("LoRa.beginPacket()\n");
  LoRa.beginPacket();
    DBG("  Writing clientId = %u\n", clientId);
    LoRa.write(clientId);
    DBG("  Writing absolute values...\n");
    for (int i = 0; i < 15; i++) {
      LoRa.write((buf[i] >> 8) & 0xFF);
      LoRa.write(buf[i] & 0xFF);
      DBG("    abs[%d] = %u -> bytes 0x%02X 0x%02X\n",
          i, buf[i], (buf[i]>>8)&0xFF, buf[i]&0xFF);
    }
    DBG("  Writing delta values...\n");
    for (int i = 0; i < 15; i++) {
      LoRa.write((delta[i] >> 8) & 0xFF);
      LoRa.write(delta[i] & 0xFF);
      DBG("    del[%d] = %u -> bytes 0x%02X 0x%02X\n",
          i, delta[i], (delta[i]>>8)&0xFF, delta[i]&0xFF);
    }
    DBG("  Writing timestamp bytes...\n");
    LoRa.write((ts >> 24) & 0xFF); DBG("    ts>>24 = 0x%02X\n", (ts>>24)&0xFF);
    LoRa.write((ts >> 16) & 0xFF); DBG("    ts>>16 = 0x%02X\n", (ts>>16)&0xFF);
    LoRa.write((ts >> 8) & 0xFF);  DBG("    ts>>8  = 0x%02X\n", (ts>>8)&0xFF);
    LoRa.write(ts & 0xFF);         DBG("    ts&0xFF= 0x%02X\n", ts&0xFF);
  LoRa.endPacket();
  DBG("LoRa packet sent\n");

  // 11) Окно приёма для синхронизации RTC (200 мс)
  DBG("Entering receive mode for 200 ms to sync time\n");
  LoRa.receive();
  unsigned long start = millis();
  while (millis() - start < 200) {
    int sz = LoRa.parsePacket();
    if (sz > 0) {
      DBG("Received %d bytes in sync window\n", sz);
      int hdr = LoRa.read();
      if (hdr == 0xFF) {
        uint32_t newTs = 0;
        DBG("  Sync header OK (0xFF)\n");
        for (int j = 0; j < 4; j++) {
          int b = LoRa.read();
          newTs = (newTs << 8) | (uint8_t)b;
          DBG("    ts byte[%d] = 0x%02X\n", j, b);
        }
       rtc.SetDateTime(RtcDateTime(newTs));
DBG("DS1302 synced to %lu\n", newTs);
        DBG("RTC synced to %lu\n", newTs);
        break;
      } else {
        DBG("  Unexpected header: 0x%02X\n", hdr);
      }
    }
  }
 
  // 12) Переход в глубокий сон
  DBG("Client: entering deep sleep now\n\n");
  esp_deep_sleep_start();
}