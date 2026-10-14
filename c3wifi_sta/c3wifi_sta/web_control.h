#pragma once
#include "config.h"
#include "web_html.h"
#include "flight_control.h"
#include "servo_system.h"
#include "sys_monitor.h"

void sendChannelsUDP(int16_t *channels) {
  if (!udpBridgeEnabled)
    return;

  uint8_t buffer[32];
  for (int i = 0; i < 16; i++) {
    int16_t val = channels[i];
    buffer[i * 2] = val & 0xFF;
    buffer[i * 2 + 1] = (val >> 8) & 0xFF;
  }

  udpBridge.beginPacket(udpTargetIP, udpTargetPort);
  udpBridge.write(buffer, 32);
  udpBridge.endPacket();
}

namespace WebControl {

constexpr char ROUTER_SSID[] = "phongkhach";
constexpr char ROUTER_PASS[] = "123456789";
constexpr char BUTTERFLY_ID[] = "PHELAN-102";
constexpr unsigned long STA_CONNECT_TIMEOUT_MS = 10000;

enum class LinkState { GOOD, WEAK, LOST };

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool wifiActive = false;
bool webControlEnabled = false;
volatile unsigned long lastWsPacketTime = 0;
unsigned long lastTelemSendTime = 0;
volatile int8_t wsArmRequest = 0;
volatile LinkState currentLinkState = LinkState::LOST;
constexpr unsigned long WS_FAILSAFE_MS = 500;
constexpr unsigned long TELEM_INTERVAL_MS = 300;

volatile bool idVerified = true;
char deviceIP[20] = "0.0.0.0";

int16_t webChannels[16] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500,
                           1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    LOG_INFO("WebSocket client #%u connected from %s", client->id(),
             client->remoteIP().toString().c_str());

    udpTargetIP = client->remoteIP();
    LOG_INFO("UDP target IP updated to %s", udpTargetIP.toString().c_str());

    if (client->client()) {
      client->client()->setNoDelay(true);
    }
  } else if (type == WS_EVT_DISCONNECT) {
    LOG_INFO("WebSocket client #%u disconnected", client->id());
    if (ws.count() == 0) {
      idVerified = true;
      webControlEnabled = false;
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == sizeof(FlightPacket) &&
        data[0] == 0xAA) {
      if (!idVerified)
        return;

      FlightPacket *pkt = (FlightPacket *)data;
      lastWsPacketTime = millis();

      webChannels[Config::CH_THROTTLE] =
          constrain(1000 + (int16_t)pkt->throttle * 10, 1000, 2000);
      webChannels[Config::CH_YAW] =
          constrain(1500 + (int16_t)pkt->yaw * 5, 1000, 2000);

      bool webPerchActive = (pkt->flags & 0x04) != 0;
      if (webPerchActive && webChannels[Config::CH_THROTTLE] < Config::THROTTLE_IDLE_MAX) {
        webChannels[Config::CH_ELEVATOR] = Config::CH_MIN;
      } else {
        webChannels[Config::CH_ELEVATOR] =
            constrain(1500 + (int16_t)pkt->pitch * 5, 1000, 2000);
      }

      webChannels[Config::CH_STEERING] =
          constrain(1500 + (int16_t)pkt->roll * 5, 1000, 2000);

      wsArmRequest = (pkt->flags & 0x01) ? 1 : 0;
      webChannels[Config::CH_KILL_SWITCH] = (pkt->flags & 0x02) ? 1900 : 1000;

      webControlEnabled = true;
    } else if (info->opcode == WS_TEXT && len > 3) {
      char buf[48];
      size_t copyLen = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
      memcpy(buf, data, copyLen);
      buf[copyLen] = '\0';

      if (strcmp(buf, "PING") == 0) {
        client->text("PONG");
      } else if (strncmp(buf, "SYNC:", 5) == 0) {
        int y, mon, d, h, m, s;
        if (sscanf(buf + 5, "%d-%d-%d_%d:%d:%d", &y, &mon, &d, &h, &m, &s) ==
            6) {
          SysMonitor::setTime(y, mon, d, h, m, s);
        }
      } else if (strcmp(buf, "UDP:ON") == 0) {
        udpBridgeEnabled = true;
        client->text("UDP:ACK:ON");
        LOG_INFO("UDP Channel Bridge ENABLED (port %u)", udpTargetPort);
      } else if (strcmp(buf, "UDP:OFF") == 0) {
        udpBridgeEnabled = false;
        client->text("UDP:ACK:OFF");
        LOG_INFO("UDP Channel Bridge DISABLED");
      } else if (strncmp(buf, "ID:", 3) == 0) {
        if (strcmp(buf + 3, BUTTERFLY_ID) == 0) {
          idVerified = true;
          client->text("ID:OK");
          LOG_INFO("Butterfly ID verified by client #%u", client->id());
        } else {
          client->text("ID:FAIL");
          LOG_WARN("Invalid Butterfly ID: %s", buf + 3);
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
    MDNS.end();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiActive = false;
    webControlEnabled = false;
    idVerified = false;
    Serial.println(F("[WIFI] WiFi turned OFF."));
  }
}

void init() {
  Serial.printf("[WIFI] Connecting STA -> %s ...\n", ROUTER_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(ROUTER_SSID, ROUTER_PASS);

  unsigned long staStart = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - staStart < 4000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    snprintf(deviceIP, sizeof(deviceIP), "%d.%d.%d.%d", ip[0], ip[1], ip[2],
             ip[3]);
    esp_wifi_set_ps(WIFI_PS_NONE);

    String mdnsName = BUTTERFLY_ID;
    mdnsName.toLowerCase();
    if (MDNS.begin(mdnsName.c_str())) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("[WIFI] mDNS: %s.local\n", mdnsName.c_str());
    }
    Serial.printf("[WIFI] STA OK! SSID:%s IP:%s ID:%s\n", ROUTER_SSID, deviceIP,
                  BUTTERFLY_ID);
  } else {
    Serial.println(
        F("[WIFI] STA not connected yet. Background auto-reconnect active."));
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGE_HTML);
  });
  server.begin();

  wifiActive = true;
}

void handle() {
  if (!wifiActive)
    return;

  unsigned long now = millis();

  static unsigned long lastStaCheck = 0;
  if (now - lastStaCheck >= 3000) {
    lastStaCheck = now;
    if (WiFi.status() == WL_CONNECTED) {
      if (strcmp(deviceIP, "0.0.0.0") == 0) {
        IPAddress ip = WiFi.localIP();
        snprintf(deviceIP, sizeof(deviceIP), "%d.%d.%d.%d", ip[0], ip[1], ip[2],
                 ip[3]);
        esp_wifi_set_ps(WIFI_PS_NONE);
        String mdnsName = BUTTERFLY_ID;
        mdnsName.toLowerCase();
        if (MDNS.begin(mdnsName.c_str())) {
          MDNS.addService("http", "tcp", 80);
        }
        Serial.printf("[WIFI] STA Connected! IP: %s\n", deviceIP);
      }
    } else {
      if (strcmp(deviceIP, "0.0.0.0") != 0) {
        strcpy(deviceIP, "0.0.0.0");
        Serial.println(F("[WIFI] STA lost. Reconnecting..."));
      }
      WiFi.reconnect();
    }
  }

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

  if (udpBridgeEnabled && (currentLinkState == LinkState::LOST)) {
    udpBridgeEnabled = false;
    LOG_WARN("Web lost. UDP Bridge auto-disabled.");
  }

  if (now - lastTelemSendTime >= TELEM_INTERVAL_MS) {
    lastTelemSendTime = now;
    sendTelemetry();
    ws.cleanupClients();
  }
}

}
