
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <RTClib.h>
#include <WiFi.h>
#include <ESP_WiFiManager.h>
#include <PubSubClient.h>

#define LORA_FREQ 868E6
#define PIN_SCK   5
#define PIN_MISO  19
#define PIN_MOSI  27
#define PIN_SS    18
#define PIN_RST   14
#define PIN_DIO0  26

typedef struct {
  char mqtt_server[40];
  char mqtt_port[6];
  char mqtt_user[20];
  char mqtt_pass[20];
  char base_topic[40];
} Config;

Config cfg;
WiFiClient espClient;
PubSubClient mqtt(espClient);
RTC_DS3231 rtc;
QueueHandle_t q;

typedef struct {
  uint8_t id;
  uint16_t absVal[12], delVal[12];
  uint32_t ts;
} Msg;

void sendTime() {
  uint32_t t = rtc.now().unixtime();
  LoRa.beginPacket();
  LoRa.write(0xFF);
  LoRa.write(t >> 24);
  LoRa.write(t >> 16);
  LoRa.write(t >> 8);
  LoRa.write(t);
  LoRa.endPacket();
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    if (mqtt.connect("srv", cfg.mqtt_user, cfg.mqtt_pass)) {
      // connected
    } else {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void LoRaTask(void* pv) {
  Msg m;
  while (1) {
    int sz = LoRa.parsePacket();
    if (sz >= (1 + 12*2 + 12*2 + 4)) {
      m.id = LoRa.read();
      for (int i = 0; i < 12; i++)
        m.absVal[i] = (LoRa.read() << 8) | LoRa.read();
      for (int i = 0; i < 12; i++)
        m.delVal[i] = (LoRa.read() << 8) | LoRa.read();
      m.ts = ((uint32_t)LoRa.read() << 24) |
             ((uint32_t)LoRa.read() << 16) |
             ((uint32_t)LoRa.read() << 8) |
             LoRa.read();
      xQueueSend(q, &m, 0);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void MqttTask(void* pv) {
  Msg m;
  char tpc[80], pl[64];
  while (1) {
    if (xQueueReceive(q, &m, portMAX_DELAY) == pdPASS) {
      if (!mqtt.connected()) mqttReconnect();
      mqtt.loop();
      for (int i = 0; i < 12; i++) {
        snprintf(tpc, sizeof(tpc), "%s/client%u/sensor%u", cfg.base_topic, m.id, i);
        snprintf(pl, sizeof(pl), "abs:%u,del:%u,ts:%lu", m.absVal[i], m.delVal[i], m.ts);
        mqtt.publish(tpc, pl);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
 

esp_log_level_set("*", ESP_LOG_DEBUG);
  ESP_WiFiManager wm;
  wm.setDebugOutput(true);
  ESP_WMParameter p1("server", "MQTT server", "", 40);
  ESP_WMParameter p2("port",   "MQTT port",   "1883", 6);
  ESP_WMParameter p3("user",   "MQTT user",   "", 20);
  ESP_WMParameter p4("pass",   "MQTT pass",   "", 20);
  ESP_WMParameter p5("topic",  "Base topic",  "sensors", 40);

  wm.addParameter(&p1);
  wm.addParameter(&p2);
  wm.addParameter(&p3);
  wm.addParameter(&p4);
  wm.addParameter(&p5);

  if (!wm.autoConnect("ConfigAP")) {
    delay(1000);
    ESP.restart();
  }

  strncpy(cfg.mqtt_server, p1.getValue(), sizeof(cfg.mqtt_server));
  strncpy(cfg.mqtt_port,   p2.getValue(), sizeof(cfg.mqtt_port));
  strncpy(cfg.mqtt_user,   p3.getValue(), sizeof(cfg.mqtt_user));
  strncpy(cfg.mqtt_pass,   p4.getValue(), sizeof(cfg.mqtt_pass));
  strncpy(cfg.base_topic,  p5.getValue(), sizeof(cfg.base_topic));

  rtc.begin();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
  LoRa.begin(LORA_FREQ);
  LoRa.receive();

  mqtt.setServer(cfg.mqtt_server, atoi(cfg.mqtt_port));
  mqttReconnect();

  q = xQueueCreate(20, sizeof(Msg));
  xTaskCreate(LoRaTask, "LoRaTask", 4096, NULL, 2, NULL);
  xTaskCreate(MqttTask, "MqttTask", 8192, NULL, 1, NULL);
}

void loop() {
  // Tasks handle everything
}
