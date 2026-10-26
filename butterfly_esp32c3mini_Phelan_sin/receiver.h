#ifndef RECEIVER_H
#define RECEIVER_H

#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace Receiver {

struct State {
  Protocol currentProto = Protocol::DETECT;
  Protocol trialProto = Protocol::CRSF;
  unsigned long lastProtoSwitchTime = 0;
  bool isLocked = false;
  uint8_t validFramesCount = 0;

  uint8_t crsfBuffer[65];
  uint8_t crsfIndex = 0;
  uint8_t crsfLength = 0;
  uint16_t crsfCrcErrors = 0;
  bool crsfFailSafe = false;

  uint8_t sbusBuffer[25];
  uint8_t sbusIndex = 0;
  bool sbusFailSafe = false;
  bool sbusFrameLost = false;
  uint8_t sbusLostFrames = 0;

  int16_t chMicros_cache[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                                1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
  int16_t emaValues[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                           1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

  bool hasFrame = false;
  unsigned long lastTime = 0;
  unsigned long lastByteTimeCrsf = 0;
  unsigned long lastByteTimeSbus = 0;
  HardwareSerial *serialPort = nullptr;
};

extern State state;
extern SemaphoreHandle_t rxMutex;

struct RcFrame {
  int16_t ch[4] = {1500, 1500, 1500, 1500};   // steering, elevator, throttle, yaw
  int16_t kill = 1500;
  bool connected = false;
  Protocol proto = Protocol::DETECT;
};

void init(HardwareSerial *serial, TaskHandle_t taskToNotify = NULL);
void resetParsers();
void syncFilterCache();
bool pollChannels(RcFrame &out);

bool readCrsf();
bool readSbus();

void switchProtocol();
void detectProtocol();
void lockReceiver();
void handleAutoDetect();

}

#endif
