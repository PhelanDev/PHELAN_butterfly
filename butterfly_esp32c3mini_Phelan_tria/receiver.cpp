#include "receiver.h"
#include "driver/uart.h"

namespace Receiver {

State state;
SemaphoreHandle_t rxMutex = NULL;
static TaskHandle_t rxNotifyTaskHandle = NULL;

void onUartData() {
  if (rxNotifyTaskHandle) {
    xTaskNotifyGive(rxNotifyTaskHandle);
  }
}

static const uint8_t crsf_crc8_table[256] PROGMEM = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83,
    0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E,
    0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75,
    0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65,
    0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA,
    0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45,
    0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E,
    0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3,
    0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1,
    0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07,
    0x53, 0x86, 0x2C, 0xF9};

uint8_t crsf_crc8(uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = pgm_read_byte(&crsf_crc8_table[crc ^ ptr[i]]);
  }
  return crc;
}

bool isValidSbusFooter(uint8_t footer) {
  return (footer == 0x00) || ((footer & 0x0F) == 0x04);
}

uint16_t extract11BitChannel(const uint8_t *payload, uint8_t chIndex,
                             uint8_t payloadLen) {
  if (!payload || payloadLen == 0)
    return 0;
  uint16_t bitOffset = chIndex * 11;
  uint8_t byteOffset = bitOffset / 8;
  if (byteOffset >= payloadLen)
    return 0;
  uint8_t bitShift = bitOffset % 8;
  uint32_t val = payload[byteOffset];
  if (byteOffset + 1 < payloadLen) {
    val |= (uint32_t)payload[byteOffset + 1] << 8;
  }
  if (byteOffset + 2 < payloadLen) {
    val |= (uint32_t)payload[byteOffset + 2] << 16;
  }
  return (val >> bitShift) & 0x07FF;
}

void parseCrsfChannels(uint8_t *payload, uint8_t payload_len) {
  if (!payload || payload_len < 22)
    return;
  if (rxMutex) xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  state.crsfFailSafe = false;
  bool firstFrame = !state.hasFrame;
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t bitOffset = i * 11;
    uint8_t byteOffset = bitOffset / 8;
    if (byteOffset + 1 >= payload_len)
      break;
    uint16_t chRaw = extract11BitChannel(payload, i, payload_len);
    int32_t mappedRaw = Config::CH_MIN +
        ((int32_t)(chRaw - Config::CRSF_RAW_MIN) * 1000) /
        (Config::CRSF_RAW_MAX - Config::CRSF_RAW_MIN);
    state.chMicros_cache[i] = (int16_t)constrain(
        mappedRaw, Config::RX_MIN_MICROS, Config::RX_MAX_MICROS);
  }
  if (firstFrame)
    syncFilterCache();
  if (rxMutex) xSemaphoreGiveRecursive(rxMutex);
}

void parseSbusChannels() {
  if (rxMutex) xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  bool firstFrame = !state.hasFrame;
  for (uint8_t i = 0; i < 16; i++) {
    uint16_t chRaw = extract11BitChannel(&state.sbusBuffer[1], i, 22);
    int32_t mappedRaw =
        Config::CH_MIN + ((int32_t)(chRaw - Config::SBUS_RAW_MIN) * 1000) /
                             (Config::SBUS_RAW_MAX - Config::SBUS_RAW_MIN);
    state.chMicros_cache[i] = (int16_t)constrain(
        mappedRaw, Config::RX_MIN_MICROS, Config::RX_MAX_MICROS);
  }
  uint8_t flags = state.sbusBuffer[23];
  state.sbusFrameLost = (flags & 0x04) != 0;
  state.sbusFailSafe = (flags & 0x08) != 0;
  if (state.sbusFrameLost) {
    if (state.sbusLostFrames < Config::MAX_LOST_FRAMES)
      state.sbusLostFrames++;
  } else {
    state.sbusLostFrames = 0;
  }
  if (firstFrame)
    syncFilterCache();
  if (rxMutex) xSemaphoreGiveRecursive(rxMutex);
}

void resetParsers() {
  state.crsfIndex = 0;
  state.crsfLength = 0;
  state.sbusIndex = 0;
  state.lastByteTimeCrsf = 0;
  state.lastByteTimeSbus = 0;
  if (rxMutex) xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  state.hasFrame = false;
  state.validFramesCount = 0;
  state.crsfFailSafe = false;
  state.sbusFailSafe = false;
  state.sbusFrameLost = false;
  state.sbusLostFrames = 0;
  if (rxMutex) xSemaphoreGiveRecursive(rxMutex);
}

void syncFilterCache() {
  for (uint8_t i = 0; i < 16; i++) {
    state.emaValues[i] = state.chMicros_cache[i];
  }
}

bool pollChannels(unsigned long nowMs, RcFrame &out) {
  if (!rxMutex || !xSemaphoreTakeRecursive(rxMutex, pdMS_TO_TICKS(1))) {
    return false;
  }

  for (uint8_t ch = 0; ch < 16; ch++) {
    int16_t diff = state.chMicros_cache[ch] - state.emaValues[ch];
    if (diff != 0) {
      int16_t step = diff / (1 << Config::EMA_ALPHA_SHIFT);
      if (step == 0) {
        step = (diff > 0) ? 1 : -1;
      }
      state.emaValues[ch] += step;
    }
  }

  bool protocolReady = !Config::USE_AUTO_DETECT || state.isLocked;
  bool rawConnected = protocolReady && state.hasFrame &&
                      (nowMs - state.lastTime < Config::RX_TIMEOUT_MS);
  if (state.currentProto == Protocol::SBUS &&
      (state.sbusFailSafe || state.sbusLostFrames >= Config::CONN_LOST_THRESHOLD)) {
    rawConnected = false;
  }
  if (state.currentProto == Protocol::CRSF && state.crsfFailSafe) {
    rawConnected = false;
  }

  out.connected = rawConnected;
  out.ch[0] = state.emaValues[Config::CH_STEERING];
  out.ch[1] = state.emaValues[Config::CH_ELEVATOR];
  out.ch[2] = state.emaValues[Config::CH_THROTTLE];
  out.ch[3] = state.emaValues[Config::CH_YAW];
  out.kill = state.chMicros_cache[Config::CH_KILL_SWITCH];
  out.proto = state.currentProto;

  xSemaphoreGiveRecursive(rxMutex);
  return true;
}

bool readCrsf() {
  if (!state.serialPort)
    return false;
  bool gotFrame = false;
  unsigned long now = millis();
  if (state.crsfIndex > 0 &&
      (now - state.lastByteTimeCrsf > Config::RX_BYTE_TIMEOUT_MS))
    state.crsfIndex = 0;

  bool readAny = false;
  for (int i = 0; i < 256 && state.serialPort->available(); i++) {
    uint8_t c = state.serialPort->read();
    readAny = true;
    if (state.crsfIndex == 0 && c != 0xC8)
      continue;
    state.crsfBuffer[state.crsfIndex++] = c;
    if (state.crsfIndex >= sizeof(state.crsfBuffer)) {
      state.crsfIndex = 0;
      continue;
    }
    if (state.crsfIndex == 2) {
      state.crsfLength = c;
      if (state.crsfLength > Config::CRSF_MAX_PAYLOAD_LEN ||
          state.crsfLength < 2) {
        state.crsfIndex = 0;
        if (c == 0xC8) {
          state.crsfBuffer[0] = 0xC8;
          state.crsfIndex = 1;
        }
      }
    }
    if (state.crsfIndex == 3) {
      uint8_t type = c;
      if (!((type == 0x16 && state.crsfLength == 24) || type == 0x14)) {
        state.crsfIndex = 0;
        if (c == 0xC8) {
          state.crsfBuffer[0] = c;
          state.crsfIndex = 1;
        }
        continue;
      }
    }
    if (state.crsfIndex > 2 && state.crsfIndex == state.crsfLength + 2) {
      uint8_t crc = crsf_crc8(&state.crsfBuffer[2], state.crsfLength - 1);
      if (crc == state.crsfBuffer[state.crsfIndex - 1]) {
        if (state.crsfBuffer[2] == 0x16 && state.crsfLength == 24) {
          parseCrsfChannels(&state.crsfBuffer[3], state.crsfLength - 2);
          gotFrame = true;
        } else if (state.crsfBuffer[2] == 0x14 && state.crsfLength >= 6) {
          uint8_t lq = state.crsfBuffer[5];
          bool failsafe = (lq == 0);
          if (rxMutex) xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
          state.crsfFailSafe = failsafe;
          if (rxMutex) xSemaphoreGiveRecursive(rxMutex);
        }
      } else {
        state.crsfCrcErrors++;
      }
      state.crsfIndex = 0;
    }
  }
  if (readAny)
    state.lastByteTimeCrsf = millis();
  return gotFrame;
}

bool readSbus() {
  if (!state.serialPort)
    return false;
  bool gotFrame = false;
  unsigned long now = millis();
  if (state.sbusIndex > 0 &&
      (now - state.lastByteTimeSbus > Config::RX_BYTE_TIMEOUT_MS))
    state.sbusIndex = 0;

  bool readAny = false;
  for (int i = 0; i < 256 && state.serialPort->available(); i++) {
    uint8_t c = state.serialPort->read();
    readAny = true;
    if (state.sbusIndex == 0 && c != 0x0F)
      continue;
    state.sbusBuffer[state.sbusIndex++] = c;
    if (state.sbusIndex >= 25) {
      state.sbusIndex = 0;
      if (!isValidSbusFooter(state.sbusBuffer[24]))
        continue;
      parseSbusChannels();
      gotFrame = true;
    }
  }
  if (readAny)
    state.lastByteTimeSbus = millis();
  return gotFrame;
}

void switchProtocol() {
  if (!state.serialPort)
    return;
  state.trialProto =
      (state.trialProto == Protocol::CRSF) ? Protocol::SBUS : Protocol::CRSF;

  if (state.trialProto == Protocol::CRSF) {
    uart_set_baudrate(UART_NUM_1, Config::CRSF_BAUD);
    uart_set_word_length(UART_NUM_1, UART_DATA_8_BITS);
    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE);
    uart_set_stop_bits(UART_NUM_1, UART_STOP_BITS_1);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_INV_DISABLE);
  } else {
    uart_set_baudrate(UART_NUM_1, Config::SBUS_BAUD);
    uart_set_word_length(UART_NUM_1, UART_DATA_8_BITS);
    uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);
    uart_set_stop_bits(UART_NUM_1, UART_STOP_BITS_2);
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV);
  }
  uart_flush_input(UART_NUM_1);

  resetParsers();
  state.lastProtoSwitchTime = millis();
}

void detectProtocol() {
  if (!state.serialPort) return;
  bool shouldUnlock = false;
  if (rxMutex) xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  shouldUnlock = state.isLocked && state.hasFrame &&
                 (millis() - state.lastTime > Config::PROTO_LOCK_TIMEOUT_MS);
  if (shouldUnlock) {
    state.isLocked = false;
    state.currentProto = Protocol::DETECT;
    state.hasFrame = false;
  }
  if (rxMutex) xSemaphoreGiveRecursive(rxMutex);

  if (shouldUnlock) {
    resetParsers();
    state.lastProtoSwitchTime = millis();
    LOG_WARN("Protocol unlocked - no frames for 3s, re-detecting...");
  }

  if (!state.isLocked) {
    if (millis() - state.lastProtoSwitchTime >
        Config::PROTO_SWITCH_TIMEOUT_MS) {
      switchProtocol();
    }
  }
}

void lockReceiver() {
  if (rxMutex) xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
  state.validFramesCount++;
  if (state.validFramesCount >= 3) {
    state.isLocked = true;
    state.currentProto = state.trialProto;
    LOG_INFO("Locked protocol: %s",
             state.currentProto == Protocol::CRSF ? "CRSF" : "SBUS");
  }
  if (rxMutex) xSemaphoreGiveRecursive(rxMutex);
}

void handleAutoDetect() {
  if (Config::USE_AUTO_DETECT)
    detectProtocol();

  bool frameReceived = false;
  if (state.trialProto == Protocol::CRSF)
    frameReceived = readCrsf();
  else
    frameReceived = readSbus();

  if (frameReceived) {
    if (rxMutex) xSemaphoreTakeRecursive(rxMutex, portMAX_DELAY);
    state.hasFrame = true;
    state.lastTime = millis();
    if (rxMutex) xSemaphoreGiveRecursive(rxMutex);
    if (Config::USE_AUTO_DETECT && !state.isLocked) {
      lockReceiver();
    }
  }
}

void init(HardwareSerial *serial, TaskHandle_t taskToNotify) {
  if (rxMutex == NULL) {
    rxMutex = xSemaphoreCreateRecursiveMutex();
    configASSERT(rxMutex != NULL);
  }
  state.serialPort = serial;
  rxNotifyTaskHandle = taskToNotify;
  if (!state.serialPort)
    return;
  state.serialPort->setRxBufferSize(Config::RX_BUFFER_SIZE);

  if (Config::USE_AUTO_DETECT) {
    state.serialPort->begin(Config::CRSF_BAUD, SERIAL_8N1, Config::RX_PIN, -1,
                            false);
    state.lastProtoSwitchTime = millis();
  } else {
    state.trialProto = Config::FIXED_PROTOCOL;
    state.currentProto = Config::FIXED_PROTOCOL;
    state.isLocked = true;
    if (state.trialProto == Protocol::CRSF) {
      state.serialPort->begin(Config::CRSF_BAUD, SERIAL_8N1, Config::RX_PIN, -1,
                              false);
    } else {
      state.serialPort->begin(Config::SBUS_BAUD, SERIAL_8E2, Config::RX_PIN, -1,
                              true);
    }
  }
  state.serialPort->onReceive(onUartData);
}

}
