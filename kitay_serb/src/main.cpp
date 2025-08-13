// main.cpp — DS1302 + OLED + Mongoose (без WiFiManager)
#ifndef LORA_DBG
#define LORA_DBG 1           // 0=off, 1=on

#endif
#ifndef NUM_CH
#define NUM_CH 15  
static constexpr int LORA_PACKET_LEN = 1 + NUM_CH*2 + NUM_CH*2 + 4;  // 65 байт
static_assert(LORA_PACKET_LEN == 65, "Ожидаем 15 каналов (65 байт)");               // теперь везде 15
#endif
#ifndef LORA_LOG_EVERY_OK
#define LORA_LOG_EVERY_OK 5  // печатать каждый N-й успешный приём
#endif

#define LORA_PROMISC 0   // временный режим: печатать любой пакет HEX
#define LORA_MAX 128
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <esp_log.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RtcDS1302.h>         // библиотека DS1302 (CE/RST, IO/DAT, SCLK/CLK)

#include "mongoose.h"
#include <ArduinoJson.h>
#include <string.h>

// ===== LoRa pins & params =====
#define LORA_FREQ 433E6
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_SS    16
#define PIN_RST   14
#define PIN_DIO0  26

// ===== DS1302 pins =====
#define DS1302_CLK 33
#define DS1302_DAT 25
#define DS1302_RST 27

// ===== I2C OLED pins (замени при необходимости) =====
#define I2C_SDA 21
#define I2C_SCL 22
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ===== RTC DS1302 =====
// Было: DS1302 rtc(DS1302_RST, DS1302_DAT, DS1302_CLK);
ThreeWire ds1302_io(DS1302_DAT, DS1302_CLK, DS1302_RST);   // IO, SCLK, CE
RtcDS1302<ThreeWire> rtc(ds1302_io);

// ===== Config in NVS =====
Preferences prefs;
struct AppConfig {
  char wifi_ssid[32]  = "";
  char wifi_pass[64]  = "";
  char mqtt_server[40]= "";
  char mqtt_port[6]   = "1883";
  char mqtt_user[20]  = "";
  char mqtt_pass[20]  = "";
  char base_topic[40] = "sensors";
} cfg;

static inline void sstrlcpy(char* dst, const char* src, size_t n) {
  if (!dst || !n) return; if (!src) { dst[0]=0; return; }
  strncpy(dst, src, n-1); dst[n-1]=0;
}
static void saveConfig() {
  prefs.begin("app", false);
  prefs.putString("wifi_ssid",  cfg.wifi_ssid);
  prefs.putString("wifi_pass",  cfg.wifi_pass);
  prefs.putString("mqtt_srv",   cfg.mqtt_server);
  prefs.putString("mqtt_port",  cfg.mqtt_port);
  prefs.putString("mqtt_user",  cfg.mqtt_user);
  prefs.putString("mqtt_pass",  cfg.mqtt_pass);
  prefs.putString("base_topic", cfg.base_topic);
  prefs.end();
}

// ===== LoRa → MQTT =====
struct Msg {
  uint8_t  id;
  uint16_t absVal[NUM_CH], delVal[NUM_CH];
  uint32_t ts;
};

WiFiClient espClient;
PubSubClient mqtt(espClient);
QueueHandle_t q;

// ===== In-memory state for UI =====
#define MAX_CLIENTS 16
struct ClientState {
  bool     used = false;
  uint8_t  id   = 0;
  uint16_t absVal[NUM_CH]{};
  uint16_t delVal[NUM_CH]{};
  uint32_t ts   = 0;
  int      rssi = 0;
};
ClientState clients[MAX_CLIENTS];

static ClientState* upsertClient(uint8_t id) {
  for (auto &c: clients) if (c.used && c.id==id) return &c;
  for (auto &c: clients) if (!c.used) { c.used=true; c.id=id; return &c; }
  clients[0].used=true; clients[0].id=id; return &clients[0];
}
static int clientCount() { int n=0; for (auto &c: clients) if (c.used) n++; return n; }

// ===== stats for OLED =====
volatile uint32_t g_pktCount = 0;
volatile uint8_t  g_lastId   = 0;
volatile int      g_lastRssi = 0;

// ===== Mongoose =====
static mg_mgr mgr;
static mg_connection* sse_clients[8] = {0};
static QueueHandle_t sseQ;

static const char *HTML_INDEX =
"<!doctype html><html><head><meta charset='utf-8'/>"
"<meta name='viewport' content='width=device-width,initial-scale=1'/>"
"<title>LoRa devices</title>"
"<style>body{font-family:system-ui;background:#111;color:#eee;margin:16px}"
"table{border-collapse:collapse;width:100%}th,td{border:1px solid #333;padding:6px}th{background:#222}"
".small{color:#bbb;font-size:12px}a{color:#8ab4ff;text-decoration:none}</style></head><body>"
"<h2>Внешние устройства (LoRa) — <a href='/settings'>Настройки</a></h2>"
"<div class='small'>IP: <span id='ip'></span> • Live: <span id='live'>подключение…</span></div>"
"<table><thead><tr><th>ID</th><th>TS</th><th>RSSI</th>"
"<th colspan='12'>ABS</th><th colspan='12'>DEL</th></tr></thead><tbody id='rows'></tbody></table>"
"<script>"
"fetch('/api/ip').then(r=>r.json()).then(j=>{document.getElementById('ip').textContent=j.ip||''});"
"const rows={};function row(c){let id=String(c.id),tr=rows[id];"
"if(!tr){tr=document.createElement('tr');rows[id]=tr;document.getElementById('rows').appendChild(tr);}let h='';"
"h+=`<td>${id}</td><td>${c.ts??''}</td><td>${c.rssi??''}</td>`;"
"for(let i=0;i<12;i++)h+=`<td>${c.abs?.[i]??''}</td>`;for(let i=0;i<12;i++)h+=`<td>${c.del?.[i]??''}</td>`;"
"tr.innerHTML=h;}"
"async function snap(){const r=await fetch('/api/clients');const j=await r.json();(j.clients||[]).forEach(row);}snap();"
"const es=new EventSource('/events');const live=document.getElementById('live');"
"es.onopen=()=>live.textContent='активно';es.onerror=()=>live.textContent='ошибка';"
"es.addEventListener('snapshot',e=>{(JSON.parse(e.data).clients||[]).forEach(row)});"
"es.addEventListener('update',e=>row(JSON.parse(e.data)));"
"</script></body></html>";

static const char *HTML_SETTINGS_PREFIX =
"<!doctype html><html><head><meta charset='utf-8'/>"
"<meta name='viewport' content='width=device-width,initial-scale=1'/>"
"<title>Настройки</title>"
"<style>body{font-family:system-ui;background:#111;color:#eee;margin:16px}"
"label{display:block;margin-top:10px}input,select{width:100%;padding:8px;margin-top:4px;background:#222;color:#eee;border:1px solid #333;border-radius:6px}"
"button{margin-top:16px;padding:10px 16px;border:0;border-radius:10px;background:#2d6cdf;color:#fff;cursor:pointer}"
".card{max-width:640px;margin:0 auto;background:#161616;border:1px solid #333;border-radius:14px;padding:16px}"
".row{display:grid;grid-template-columns:1fr 1fr;gap:12px}"
".msg{background:#092;color:#cfc;padding:8px;border-radius:8px;margin-bottom:12px;display:%s}"
"a{color:#8ab4ff;text-decoration:none}</style>"
"</head><body><div class='card'>"
"<h2>Настройки Wi-Fi и MQTT</h2>"
"<p><a href='/'>⟵ Вернуться к данным</a></p>"
"<div class='msg'>Сохранено. Устройство перезагрузится…</div>"
"<form method='POST' action='/settings'>"
"<h3>Wi-Fi</h3>"
"<label>SSID<input name='wifi_ssid' value='";
static const char *HTML_SETTINGS_MID =
"'/></label>"
"<label>Password<input type='password' name='wifi_pass' value='";
static const char *HTML_SETTINGS_MID2 =
"'/></label>"
"<h3>MQTT</h3>"
"<div class='row'>"
"<label>Server<input name='mqtt_server' value='";
static const char *HTML_SETTINGS_MID3 =
"'/></label>"
"<label>Port<input name='mqtt_port' value='";
static const char *HTML_SETTINGS_MID4 =
"'/></label></div>"
"<div class='row'>"
"<label>User<input name='mqtt_user' value='";
static const char *HTML_SETTINGS_MID5 =
"'/></label>"
"<label>Password<input type='password' name='mqtt_pass' value='";
static const char *HTML_SETTINGS_SUFFIX =
"'/></label></div>"
"<label>Base topic<input name='base_topic' value='%s'/></label>"
"<button type='submit'>Сохранить</button>"
"</form></div></body></html>";
struct LoraDbg {
  volatile uint32_t total = 0, ok = 0, bad_len = 0, short_read = 0, flushed = 0;
  volatile int      last_len = 0, last_rssi = 0;
  volatile float    last_snr = 0;
  volatile uint8_t  last_id = 0;
  uint8_t           last_hdr[8] = {0};
} gL;

static inline uint16_t rd16BE(const uint8_t* b, size_t& i){
  uint16_t v = ((uint16_t)b[i] << 8) | b[i+1]; i += 2; return v;
}
static void loadConfig() {
  prefs.begin("app", false);
  String s;
  s = prefs.getString("wifi_ssid",  "");       s.toCharArray(cfg.wifi_ssid,  sizeof(cfg.wifi_ssid));
  s = prefs.getString("wifi_pass",  "");       s.toCharArray(cfg.wifi_pass,  sizeof(cfg.wifi_pass));
  s = prefs.getString("mqtt_srv",   "");       s.toCharArray(cfg.mqtt_server,sizeof(cfg.mqtt_server));
  s = prefs.getString("mqtt_port",  "1883");   s.toCharArray(cfg.mqtt_port,  sizeof(cfg.mqtt_port));
  s = prefs.getString("mqtt_user",  "");       s.toCharArray(cfg.mqtt_user,  sizeof(cfg.mqtt_user));
  s = prefs.getString("mqtt_pass",  "");       s.toCharArray(cfg.mqtt_pass,  sizeof(cfg.mqtt_pass));
  s = prefs.getString("base_topic", "sensors");s.toCharArray(cfg.base_topic, sizeof(cfg.base_topic));
  prefs.end();
}
static bool setupMqttServerFromCfg() {
  if (cfg.mqtt_server[0] == '\0') return false;
  IPAddress ip;
  if (ip.fromString(cfg.mqtt_server)) {
    mqtt.setServer(ip, atoi(cfg.mqtt_port));   // если ввели IP — DNS не нужен
  } else {
    mqtt.setServer(cfg.mqtt_server, atoi(cfg.mqtt_port));
  }
  return true;
}
static void http_settings_reply(mg_connection* c, bool saved) {
  mg_printf(c, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nCache-Control: no-cache\r\n\r\n");
  mg_printf(c, HTML_SETTINGS_PREFIX, saved ? "block" : "none");
  mg_printf(c, "%s", cfg.wifi_ssid);
  mg_printf(c, HTML_SETTINGS_MID);
  mg_printf(c, "%s", cfg.wifi_pass);
  mg_printf(c, HTML_SETTINGS_MID2);
  mg_printf(c, "%s", cfg.mqtt_server);
  mg_printf(c, HTML_SETTINGS_MID3);
  mg_printf(c, "%s", cfg.mqtt_port);
  mg_printf(c, HTML_SETTINGS_MID4);
  mg_printf(c, "%s", cfg.mqtt_user);
  mg_printf(c, HTML_SETTINGS_MID5);
  mg_printf(c, "%s", cfg.mqtt_pass);
  mg_printf(c, HTML_SETTINGS_SUFFIX, cfg.base_topic);
}
static void http_settings_post(mg_connection* c, mg_http_message* hm) {
  char v[128];
  auto get = [&](const char* name, char* dst, size_t dstlen) {
    int n = mg_http_get_var(&hm->body, name, v, sizeof(v));
    if (n > 0) { v[n]=0; sstrlcpy(dst, v, dstlen); }
  };
  get("wifi_ssid",  cfg.wifi_ssid,  sizeof(cfg.wifi_ssid));
  get("wifi_pass",  cfg.wifi_pass,  sizeof(cfg.wifi_pass));
  get("mqtt_server",cfg.mqtt_server,sizeof(cfg.mqtt_server));
  get("mqtt_port",  cfg.mqtt_port,  sizeof(cfg.mqtt_port));
  get("mqtt_user",  cfg.mqtt_user,  sizeof(cfg.mqtt_user));
  get("mqtt_pass",  cfg.mqtt_pass,  sizeof(cfg.mqtt_pass));
  get("base_topic", cfg.base_topic, sizeof(cfg.base_topic));

  saveConfig();
  http_settings_reply(c, true);
  xTaskCreate([](void*){ vTaskDelay(pdMS_TO_TICKS(1000)); ESP.restart(); },
              "rebooter", 2048, NULL, 1, NULL);
}
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println("[WiFi] AP started");
      break;

    case ARDUINO_EVENT_WIFI_AP_STACONNECTED: {
      const uint8_t* m = info.wifi_ap_staconnected.mac;
      Serial.printf("[WiFi] AP: STA connected %02X:%02X:%02X:%02X:%02X:%02X, aid=%d\n",
        m[0], m[1], m[2], m[3], m[4], m[5], info.wifi_ap_staconnected.aid);
      break;
    }

    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: {
      const uint8_t* m = info.wifi_ap_stadisconnected.mac;
      Serial.printf("[WiFi] AP: STA disconnected %02X:%02X:%02X:%02X:%02X:%02X, aid=%d\n",
        m[0], m[1], m[2], m[3], m[4], m[5], info.wifi_ap_stadisconnected.aid);
      break;
    }

    // если ты используешь ещё и STA-режим клиента (подключение к роутеру)
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[WiFi] STA disconnected, reason=%d\n",
                    info.wifi_sta_disconnected.reason);
      break;

    default:
      // Serial.printf("[WiFi] event=%d\n", event);
      break;
  }
}
static void sse_broadcast(const char* json) {
  if (!sseQ) return;
  char buf[512]; size_t n = strnlen(json, sizeof(buf)-1);
  memcpy(buf, json, n); buf[n]=0;
  xQueueSend(sseQ, buf, 0);
}
static inline bool uri_is(struct mg_http_message* hm, const char* s) {
  return mg_strcmp(hm->uri, mg_str(s)) == 0;
}
static inline bool method_is(struct mg_http_message* hm, const char* s) {
  return mg_strcmp(hm->method, mg_str(s)) == 0;
}

static void http_fn(mg_connection* c, int ev, void* ev_data) {
  if (ev == MG_EV_HTTP_MSG) {
    auto *hm = (mg_http_message*) ev_data;

    if (uri_is(hm, "/")) {
      mg_http_reply(c, 200, "Content-Type: text/html\r\nCache-Control: no-cache\r\n", "%s", HTML_INDEX);

    } else if (uri_is(hm, "/api/ip")) {
      StaticJsonDocument<64> d;
      d["ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
      char out[128]; size_t n = serializeJson(d, out, sizeof(out));
      mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%.*s", (int)n, out);

    } else if (uri_is(hm, "/api/clients")) {
      StaticJsonDocument<4096> doc;
      JsonArray arr = doc.createNestedArray("clients");
      for (auto &cs: clients) if (cs.used) {
        JsonObject o = arr.createNestedObject();
        o["id"]=cs.id; o["ts"]=cs.ts; o["rssi"]=cs.rssi;
        JsonArray a=o.createNestedArray("abs"); JsonArray d=o.createNestedArray("del");
        for (int k=0;k<12;k++) a.add(cs.absVal[k]);
        for (int k=0;k<12;k++) d.add(cs.delVal[k]);
      }
      char out[4096]; size_t n = serializeJson(doc, out, sizeof(out));
      mg_http_reply(c, 200, "Content-Type: application/json\r\nCache-Control: no-cache\r\n", "%.*s", (int)n, out);

    } else if (uri_is(hm, "/events")) {
      mg_printf(c,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n\r\n");
      for (auto &slot: sse_clients) if (!slot) { slot=c; break; }
      // моментальный снапшот
      StaticJsonDocument<4096> doc;
      JsonArray arr = doc.createNestedArray("clients");
      for (auto &cs: clients) if (cs.used) {
        JsonObject o = arr.createNestedObject();
        o["id"]=cs.id; o["ts"]=cs.ts; o["rssi"]=cs.rssi;
        JsonArray a=o.createNestedArray("abs"); JsonArray d=o.createNestedArray("del");
        for (int k=0;k<12;k++) a.add(cs.absVal[k]);
        for (int k=0;k<12;k++) d.add(cs.delVal[k]);
      }
      char out[4096]; size_t n = serializeJson(doc, out, sizeof(out));
      mg_printf(c, "event: snapshot\ndata: %.*s\n\n", (int)n, out);
      return;

    }
    else if (uri_is(hm, "/api/lora_dbg")) {
  StaticJsonDocument<256> d;
  d["total"]=gL.total; d["ok"]=gL.ok; d["bad_len"]=gL.bad_len;
  d["short"]=gL.short_read; d["flushed"]=gL.flushed;
  d["last_len"]=gL.last_len; d["last_rssi"]=gL.last_rssi; d["last_snr"]=gL.last_snr;
  char hh[24]; snprintf(hh,sizeof(hh),"%02X %02X %02X %02X %02X %02X %02X %02X",
                        gL.last_hdr[0],gL.last_hdr[1],gL.last_hdr[2],gL.last_hdr[3],
                        gL.last_hdr[4],gL.last_hdr[5],gL.last_hdr[6],gL.last_hdr[7]);
  d["hdr"]=hh;
  char out[256]; size_t n=serializeJson(d,out,sizeof(out));
  mg_http_reply(c,200,"Content-Type: application/json\r\n","%.*s",(int)n,out);
}
    
    else if (uri_is(hm, "/settings")) {
      if (method_is(hm, "POST")) http_settings_post(c, hm);
      else                       http_settings_reply(c, false);
      return;

    } else {
      mg_http_reply(c, 404, "", "Not found");
    }
  } else if (ev == MG_EV_CLOSE) {
    for (auto &slot: sse_clients) if (slot==c) slot=nullptr;
  }
}

// ===== Mongoose RTOS task =====
static void MongooseTask(void*) {
   static char msg[512];       
   for (;;) {
    mg_mgr_poll(&mgr, 10);
    while (xQueueReceive(sseQ, msg, 0) == pdPASS) {
      for (auto c: sse_clients) if (c) mg_printf(c, "event: update\ndata: %s\n\n", msg);
    }
  }
}

// ===== Wi-Fi logic =====
static bool wifiConnectSTA(uint32_t timeout_ms=15000) {
  if (cfg.wifi_ssid[0]==0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
  uint32_t t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0 < timeout_ms) delay(200);
  return WiFi.status()==WL_CONNECTED;
}
static void wifiStartAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("LoRaGW-Setup", "12345678"); // SSID/пароль можешь поменять
}

// ===== MQTT helpers =====
static String mqttClientId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[32]; snprintf(buf, sizeof(buf), "srv-%06X", (uint32_t)(mac & 0xFFFFFF));
  return String(buf);
}
static void mqttEnsure() {
   if (WiFi.status() != WL_CONNECTED) return;       // в AP/оффлайне не дёргать DNS
  if (cfg.mqtt_server[0] == '\0') return;       
  while (!mqtt.connected()) {
    mqtt.connect(mqttClientId().c_str(), cfg.mqtt_user, cfg.mqtt_pass);
    if (!mqtt.connected()) vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

static void sendTime() {
  // если у тебя DS1302 Makuna:
    RtcDateTime now = rtc.GetDateTime();
   uint32_t t = now.Unix32Time();

  // если RTC нет — можно взять системное (для теста):
  //uint32_t t = (uint32_t)(millis()/1000);

  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write(0xFF);
  LoRa.write((t >> 24) & 0xFF);
  LoRa.write((t >> 16) & 0xFF);
  LoRa.write((t >>  8) & 0xFF);
  LoRa.write( t        & 0xFF);
  LoRa.endPacket();
  LoRa.receive();
}
// ===== Tasks =====
/*static void LoRaTask(void*) {
  const int NEED12 = 1 + 12*2 + 12*2 + 4;  // 53
  const int NEED15 = 1 + 15*2 + 15*2 + 4;  // 65
  uint8_t buf[80];

  for (;;) {
    int sz = LoRa.parsePacket();            // >0: принят кадр (CRC прошёл)
    if (sz > 0) {
      gL.total++;
      gL.last_len  = sz;
      gL.last_rssi = LoRa.packetRssi();
      gL.last_snr  = LoRa.packetSnr();

      if (sz != NEED12 && sz != NEED15) {
        size_t n = LoRa.readBytes(buf, (size_t)min(sz, (int)sizeof(buf)));
        memcpy(gL.last_hdr, buf, min(n,(size_t)8));
        while (LoRa.available()) { LoRa.read(); gL.flushed++; }
        gL.bad_len++;
        Serial.printf("[LoRa] bad len=%d rssi=%d snr=%.1f hdr=%02X %02X %02X %02X %02X %02X %02X %02X\n",
          sz, gL.last_rssi, gL.last_snr,
          gL.last_hdr[0],gL.last_hdr[1],gL.last_hdr[2],gL.last_hdr[3],
          gL.last_hdr[4],gL.last_hdr[5],gL.last_hdr[6],gL.last_hdr[7]);
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }

      size_t n = LoRa.readBytes(buf, sz);
      if ((int)n != sz) {
        gL.short_read++;
        while (LoRa.available()) { LoRa.read(); gL.flushed++; }
        Serial.printf("[LoRa] short read need=%d got=%u\n", sz, (unsigned)n);
        vTaskDelay(pdMS_TO_TICKS(5));
        continue;
      }

      Msg m{}; size_t i=0;
      m.id = buf[i++];

      int N = (sz == NEED12) ? 12 : 15;
      for (int k=0;k<N;k++) m.absVal[k] = rd16BE(buf,i);
      for (int k=0;k<N;k++) m.delVal[k] = rd16BE(buf,i);
      m.ts = ((uint32_t)buf[i] << 24) | ((uint32_t)buf[i+1] << 16) |
             ((uint32_t)buf[i+2] << 8) | (uint32_t)buf[i+3];

      gL.ok++; gL.last_id = m.id;

      if (auto* c = upsertClient(m.id)) {
        // сохраняем первые 12 каналов (если пришло 15 — 13..15 игнорим)
        memcpy(c->absVal, m.absVal, 12*sizeof(uint16_t));
        memcpy(c->delVal, m.delVal, 12*sizeof(uint16_t));
        c->ts = m.ts; c->rssi = gL.last_rssi;
      }

      // SSE update
      StaticJsonDocument<512> d;
      d["id"]=m.id; d["ts"]=m.ts; d["rssi"]=gL.last_rssi; d["snr"]=gL.last_snr;
      JsonArray a=d.createNestedArray("abs"); JsonArray dd=d.createNestedArray("del");
      for (int k=0;k<12;k++) a.add(m.absVal[k]);
      for (int k=0;k<12;k++) dd.add(m.delVal[k]);
      char out[512]; size_t jsz = serializeJson(d, out, sizeof(out));
      if (jsz < sizeof(out)) sse_broadcast(out);

      // OLED + MQTT
      g_pktCount++; g_lastId = m.id; g_lastRssi = gL.last_rssi;
      xQueueSend(q, &m, 10 / portTICK_PERIOD_MS);

      // сразу после приёма — маяк времени (клиент держит окно RX ~200 мс)
      sendTime();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
*/
static void LoRaTask(void*) {
  uint8_t buf[LORA_PACKET_LEN + 8];   // с запасом

  for (;;) {
    int sz = LoRa.parsePacket();            // >0 — есть кадр (CRC ок)
    if (sz > 0) {
      if (sz != LORA_PACKET_LEN) {
        // чужой или старый формат — просто прочитаем/прогоним
        size_t n = LoRa.readBytes(buf, (size_t)min(sz, (int)sizeof(buf)));
        Serial.println('че то поймали но не наше');
        // (опционально) логируй длину/первые байты
        continue;
      }

      // читаем ровно 65 байт
      size_t n = LoRa.readBytes(buf, LORA_PACKET_LEN);
      if ((int)n != LORA_PACKET_LEN) continue;

      Msg m{}; size_t i=0;
      m.id = buf[i++];

      for (int k=0;k<NUM_CH;k++) { m.absVal[k] = ((uint16_t)buf[i] << 8) | buf[i+1]; i+=2; }
      for (int k=0;k<NUM_CH;k++) { m.delVal[k] = ((uint16_t)buf[i] << 8) | buf[i+1]; i+=2; }
      m.ts = ((uint32_t)buf[i] << 24) | ((uint32_t)buf[i+1] << 16) |
             ((uint32_t)buf[i+2] << 8) | (uint32_t)buf[i+3];

      if (auto* c = upsertClient(m.id)) {
        memcpy(c->absVal, m.absVal, sizeof(m.absVal));
        memcpy(c->delVal, m.delVal, sizeof(m.delVal));
        c->ts = m.ts; c->rssi = LoRa.packetRssi();
      }

      // SSE — готовим один снимок на 15 каналов
      StaticJsonDocument<768> d;            // 512 может быть впритык — увеличил немного
      d["id"]=m.id; d["ts"]=m.ts; d["rssi"]=LoRa.packetRssi(); d["snr"]=LoRa.packetSnr();
      JsonArray a = d.createNestedArray("abs");
      JsonArray dd= d.createNestedArray("del");
      for (int k=0;k<NUM_CH;k++) a.add(m.absVal[k]);
      for (int k=0;k<NUM_CH;k++) dd.add(m.delVal[k]);
      char out[768]; size_t jsz = serializeJson(d, out, sizeof(out));
      if (jsz < sizeof(out)) sse_broadcast(out);

      // OLED/счётчики/очередь — как было
      g_pktCount++; g_lastId = m.id; g_lastRssi = LoRa.packetRssi();
      xQueueSend(q, &m, 10 / portTICK_PERIOD_MS);

      // (если используешь ACK временем) — оставь свой sendTime();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void MqttTask(void*) {
  Msg m; char tpc[96], pl[64];
  TickType_t lastLoop = xTaskGetTickCount();
  for (;;) {
     if (WiFi.status() != WL_CONNECTED || cfg.mqtt_server[0] == '\0') {
    vTaskDelay(500 / portTICK_PERIOD_MS);  // подожди сеть/настройки
    continue;
  }
    if (xTaskGetTickCount()-lastLoop > pdMS_TO_TICKS(250)) {
      mqttEnsure(); mqtt.loop(); lastLoop = xTaskGetTickCount();
    }
    if (xQueueReceive(q, &m, 50/portTICK_PERIOD_MS) == pdPASS) {
      mqttEnsure(); mqtt.loop();
      for (int i=0;i<12;i++) {
        snprintf(tpc, sizeof(tpc), "%s/client%u/sensor%u", cfg.base_topic, m.id, i);
        snprintf(pl, sizeof(pl), "abs:%u,del:%u,ts:%lu", m.absVal[i], m.delVal[i], (unsigned long)m.ts);
        mqtt.publish(tpc, pl, false);
      }
    }
  }
}

// ===== OLED task (обновление экрана) =====
static void OledTask(void*) {
  for (;;) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
 char line[24];
    // Время/дата из DS1302
RtcDateTime now = rtc.GetDateTime();
snprintf(line, sizeof(line), "%02u:%02u:%02u %02u.%02u.%02u",
         now.Hour(), now.Minute(), now.Second(),
         now.Day(), now.Month(), now.Year() % 100);
    display.println(line);

    // Wi-Fi / IP
    if (WiFi.isConnected()) {
      display.print("WiFi: "); display.println(WiFi.localIP().toString());
    } else {
      display.print("AP: "); display.println(WiFi.softAPIP().toString());
    }

    // MQTT статус
    display.print("MQTT: "); display.println(mqtt.connected() ? "OK" : "NC");

    // Клиенты / Пакеты
    display.print("Clients: "); display.print(clientCount());
    display.print("  Pkts: ");   display.println((unsigned long)g_pktCount);

    // Последний пакет
    display.print("Last ID: "); display.print(g_lastId);
    display.print("  RSSI: ");  display.println(g_lastRssi);

    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ===== setup/loop =====
void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_DEBUG);
WiFi.onEvent(onWiFiEvent);
  loadConfig();
setupMqttServerFromCfg();
  // Очереди/состояние
  memset(clients, 0, sizeof(clients));
  q    = xQueueCreate(32, sizeof(Msg));
  sseQ = xQueueCreate(12, 512);
  // Сеть: STA или AP fallback
  if (!wifiConnectSTA()) {
    wifiStartAP();
    Serial.printf("AP mode. SSID 'LoRaGW-Setup', pass '12345678'. IP: %s\n",
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.printf("STA connected. IP: %s\n", WiFi.localIP().toString().c_str());
  }
  // Mongoose
  mg_mgr_init(&mgr);
  mg_log_set(MG_LL_INFO);
  mg_http_listen(&mgr, "http://0.0.0.0:80", http_fn, NULL);
  xTaskCreate(MongooseTask, "MongooseTask", 8192, NULL, 1, NULL);
  
  // I2C + OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
  } else {
    display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0); display.println("Booting..."); display.display();
  }

  // RTC DS1302
  rtc.Begin();
if (!rtc.GetIsRunning()) rtc.SetIsRunning(true);
if (!rtc.IsDateTimeValid()) {
  rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));  // единоразово
}
  // При необходимости — единоразово выставь время вручную и закомментируй:
  // rtc.time(Time(2025, 8, 11, 12, 0, 0)); // YYYY,MM,DD,hh,mm,ss

  // LoRa
  
  // MQTT
  //mqtt.setServer(cfg.mqtt_server, atoi(cfg.mqtt_port));



  // RTOS задачи

  xTaskCreate(MqttTask,     "MqttTask",     7168, NULL, 1, NULL);

  xTaskCreate(OledTask,     "OledTask",     4096, NULL, 1, NULL);


SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);



  auto loraReadReg = [&](uint8_t reg) {
  digitalWrite(PIN_SS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_SS, HIGH);
  return v;
};

pinMode(PIN_SS, OUTPUT);
digitalWrite(PIN_SS, HIGH);

uint8_t ver = 0;
digitalWrite(PIN_SS, LOW);
SPI.transfer(0x42 & 0x7F);          // RegVersion
ver = SPI.transfer(0x00);
digitalWrite(PIN_SS, HIGH);
Serial.printf("SX127x RegVersion=0x%02X (0x12/0x11 OK)\n", ver);




  if (!LoRa.begin(LORA_FREQ)) { Serial.println("LoRa begin failed"); ESP.restart(); }
  
LoRa.setSyncWord(0xA5);
LoRa.enableCrc();
 LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
LoRa.setSignalBandwidth(125E3);  // BW 125 кГц
LoRa.setSpreadingFactor(7);      // SF7
LoRa.setCodingRate4(5);          // CR 4/5
LoRa.setTimeout(20);             // таймаут Stream.readBytes, мс
LoRa.receive();



auto rd = [&](uint8_t r){ digitalWrite(PIN_SS,LOW); SPI.transfer(r&0x7F);
                          uint8_t v=SPI.transfer(0); digitalWrite(PIN_SS,HIGH); return v; };
uint8_t mc1=rd(0x1D), mc2=rd(0x1E), mc3=rd(0x26), sw=rd(0x39);
uint8_t frfMsb=rd(0x06), frfMid=rd(0x07), frfLsb=rd(0x08);
Serial.printf("LoRa cfg: 1D=0x%02X 1E=0x%02X 26=0x%02X SW=0x%02X FRF=%02X%02X%02X\n",
              mc1, mc2, mc3, sw, frfMsb, frfMid, frfLsb);


  xTaskCreate(LoRaTask,     "LoRaTask",     4096, NULL, 2, NULL);
  Serial.println("Ready.");
}

void loop() {
  // всё в задачах
}
