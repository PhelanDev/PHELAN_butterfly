#pragma once
#include "config.h"
#include "web_html.h"
#include "flight_control.h"
#include "servo_system.h"
#include "display_oled.h"

namespace WebControl {

constexpr char AP_SSID[] = "PHELAN BUTTERFLY";
constexpr char AP_PASS[] = "12345678";
constexpr uint8_t WIFI_CHANNEL = 11;
constexpr unsigned long AP_NO_CLIENT_TIMEOUT_MS = 5 * 60 * 1000;
constexpr unsigned long AP_DISCONNECT_TIMEOUT_MS = 3 * 60 * 1000;

enum class LinkState { GOOD, WEAK, LOST };

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool wifiActive = false;
bool webControlEnabled = false;
bool hadClients = false;
uint8_t lastStationCount = 0;
unsigned long bootTimeMs = 0;
unsigned long disconnectedTimeMs = 0;
volatile unsigned long lastWsPacketTime = 0;
unsigned long lastTelemSendTime = 0;
volatile int8_t wsArmRequest = 0;
volatile LinkState currentLinkState = LinkState::LOST;
constexpr unsigned long WS_FAILSAFE_MS = 500;
constexpr unsigned long TELEM_INTERVAL_MS = 300;

int16_t webChannels[16] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500,
                           1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    LOG_INFO("WebSocket client #%u connected", client->id());
    if (client->client()) {
      client->client()->setNoDelay(true);
    }
  } else if (type == WS_EVT_DISCONNECT) {
    LOG_INFO("WebSocket client #%u disconnected", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == sizeof(FlightPacket) &&
        data[0] == 0xAA) {

      FlightPacket *pkt = (FlightPacket *)data;
      lastWsPacketTime = millis();

      webChannels[Config::CH_THROTTLE] =
          constrain(1000 + (int16_t)pkt->throttle * 10, 1000, 2000);
      webChannels[Config::CH_YAW] =
          constrain(1500 + (int16_t)pkt->yaw * 5, 1000, 2000);
      webChannels[Config::CH_ELEVATOR] =
          constrain(1500 + (int16_t)pkt->pitch * 5, 1000, 2000);
      webChannels[Config::CH_STEERING] =
          constrain(1500 + (int16_t)pkt->roll * 5, 1000, 2000);

      wsArmRequest = (pkt->flags & 0x01) ? 1 : 0;
      webChannels[Config::CH_KILL_SWITCH] = (pkt->flags & 0x02) ? 1900 : 1000;

      webControlEnabled = true;
    } else if (info->opcode == WS_TEXT && len >= 4) {

      char buf[48];
      size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
      memcpy(buf, data, copyLen);
      buf[copyLen] = '\0';
      if (strcmp(buf, "PING") == 0) {
        client->text("PONG");
      } else if (strncmp(buf, "SYNC:", 5) == 0 && len > 5) {
        int y, mon, d, h, m, s;
        if (sscanf(buf + 5, "%d-%d-%d_%d:%d:%d", &y, &mon, &d, &h, &m, &s) ==
            6) {
          SysMonitor::setTime(y, mon, d, h, m, s);
        }
      }
    }
  }
}

void sendTelemetry() {
  if (ws.count() == 0)
    return;

  TelemetryPacket tpkt;
  tpkt.header = 0xBB;

  uint8_t modeId = 0;
  switch (Flight::state.mode) {
  case FlightMode::CONNECTED:
    modeId = 1;
    break;
  case FlightMode::PERCHING:
    modeId = 2;
    break;
  case FlightMode::FAILSAFE:
    modeId = 3;
    break;
  default:
    modeId = 0;
    break;
  }
  tpkt.flags = (Flight::state.isArmed ? 1 : 0) | (modeId << 1);
  tpkt.offset = (int8_t)constrain((int)Flight::state.currentOffset, -127, 127);
  tpkt.servoL = (uint8_t)constrain(ServoSystem::state.lastPosL, 0, 255);
  tpkt.servoR = (uint8_t)constrain(ServoSystem::state.lastPosR, 0, 255);
  tpkt.cpuTemp = (uint8_t)(SysMonitor::getCPUTemp() * 2.0f);
  tpkt.cpuMhz = (uint8_t)SysMonitor::getCPUMhz();
  tpkt.uptimeSec = (uint16_t)(millis() / 1000);

  char timeBuf[32];
  SysMonitor::getFormattedTime(timeBuf, sizeof(timeBuf));
  if (strlen(timeBuf) >= 19) {
    tpkt.timeH = (uint8_t)((timeBuf[11] - '0') * 10 + (timeBuf[12] - '0'));
    tpkt.timeM = (uint8_t)((timeBuf[14] - '0') * 10 + (timeBuf[15] - '0'));
    tpkt.timeS = (uint8_t)((timeBuf[17] - '0') * 10 + (timeBuf[18] - '0'));
  } else {
    tpkt.timeH = 0;
    tpkt.timeM = 0;
    tpkt.timeS = 0;
  }

  ws.binaryAll((uint8_t *)&tpkt, sizeof(tpkt));
}

void stopWiFi() {
  if (wifiActive) {
    ws.closeAll();
    server.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiActive = false;
    webControlEnabled = false;
    Serial.println(
        F("[WIFI] Auto-shutdown executed. WiFi AP turned OFF to save power."));
  }
}

void init() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CHANNEL, 0, 2);
  delay(100);

  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B);

  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);

  esp_wifi_set_max_tx_power(78);

  esp_wifi_set_ps(WIFI_PS_NONE);

  IPAddress ip = WiFi.softAPIP();
  Serial.print(F("[WIFI] AP Started: "));
  Serial.print(AP_SSID);
  Serial.print(F(" | IP: "));
  Serial.print(ip);
  Serial.printf(" | RF: 802.11b/CH%u/HT20/20dBm/NoSleep\n",
                (unsigned int)WIFI_CHANNEL);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGE_HTML);
  });
  server.begin();

  wifiActive = true;
  hadClients = false;
  lastStationCount = 0;
  bootTimeMs = millis();
  disconnectedTimeMs = 0;
}

void handle() {
  if (!wifiActive)
    return;

  unsigned long now = millis();
  unsigned long dtLink = now - lastWsPacketTime;

  if (dtLink < 200) {
    currentLinkState = LinkState::GOOD;
  } else if (dtLink <= WS_FAILSAFE_MS) {
    currentLinkState = LinkState::WEAK;

    if (webChannels[Config::CH_THROTTLE] > 1400) {
      webChannels[Config::CH_THROTTLE] = 1400;
    }
  } else {
    currentLinkState = LinkState::LOST;
  }

  if (webControlEnabled && (currentLinkState == LinkState::LOST)) {
    webControlEnabled = false;
    wsArmRequest = 0;
  }

  if (now - lastTelemSendTime >= TELEM_INTERVAL_MS) {
    lastTelemSendTime = now;
    sendTelemetry();
    ws.cleanupClients();
  }

  uint8_t stationCount = WiFi.softAPgetStationNum();

  if (stationCount > 0) {
    hadClients = true;
    disconnectedTimeMs = 0;
  } else {
    if (!hadClients) {
      if (now - bootTimeMs >= AP_NO_CLIENT_TIMEOUT_MS) {
        Serial.println(
            F("[WIFI] No client connected after 5 mins. Turning OFF WiFi."));
        stopWiFi();
        return;
      }
    } else {
      if (lastStationCount > 0) {
        disconnectedTimeMs = now;
        Serial.println(F("[WIFI] Client disconnected. WiFi will turn off in 3 "
                         "minutes if no reconnect."));
      }

      if (disconnectedTimeMs > 0 &&
          (now - disconnectedTimeMs >= AP_DISCONNECT_TIMEOUT_MS)) {
        Serial.println(
            F("[WIFI] Disconnected timeout (3 mins). Turning OFF WiFi."));
        stopWiFi();
        return;
      }
    }
  }

  lastStationCount = stationCount;
}

}
