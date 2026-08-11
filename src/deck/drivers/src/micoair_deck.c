/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Antigravity
 * All the changes are made by Hasan
 * micoair_deck.c: Deck driver for MicoAir MTF-01 / MT-06 sensors (Micolink protocol over UART)
 */

#define DEBUG_MODULE "MICOAIR"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "uart1.h"
#include "estimator.h"
#include "usec_time.h"
#include "sensors.h"
#include "cf_math.h"

#define MICOLINK_MSG_HEAD 0xEF
#define MICOLINK_MAX_PAYLOAD_LEN 64
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51

#define NPIX 35.0f
#define THETAPIX 0.71674f
#define FLOW_RESOLUTION 0.10f

typedef enum {
  STATE_HEAD = 0,
  STATE_DEV_ID,
  STATE_SYS_ID,
  STATE_MSG_ID,
  STATE_SEQ,
  STATE_LEN,
  STATE_PAYLOAD,
  STATE_CHECKSUM
} ParserState_t;

typedef struct {
  uint8_t head;
  uint8_t dev_id;
  uint8_t sys_id;
  uint8_t msg_id;
  uint8_t seq;
  uint8_t len;
  uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
  uint8_t checksum;
} __attribute__((packed)) MICOLINK_MSG_t;

typedef struct {
  uint32_t time_ms;
  uint32_t distance;      // distance(mm), 0 Indicates unavailable
  uint8_t  strength;      // signal strength
  uint8_t  precision;     // distance precision
  uint8_t  dis_status;    // distance status
  uint8_t  reserved1;     // reserved
  int16_t  flow_vel_x;    // optical flow velocity in x (cm/s)
  int16_t  flow_vel_y;    // optical flow velocity in y (cm/s)
  uint8_t  flow_quality;  // optical flow quality
  uint8_t  flow_status;   // optical flow status
  uint16_t reserved2;     // reserved
} __attribute__((packed)) MICOLINK_PAYLOAD_RANGE_SENSOR_t;

static bool isInit = false;
static uint32_t micoair_distance = 0;
static float micoair_flow_x = 0.0f;
static float micoair_flow_y = 0.0f;
static uint8_t micoair_quality = 0;

static bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t byte) {
  static ParserState_t state = STATE_HEAD;
  static uint8_t checksum = 0;
  static uint8_t payload_cnt = 0;

  switch (state) {
    case STATE_HEAD:
      if (byte == MICOLINK_MSG_HEAD) {
        msg->head = byte;
        checksum = byte;
        state = STATE_DEV_ID;
      }
      break;
    case STATE_DEV_ID:
      msg->dev_id = byte;
      checksum += byte;
      state = STATE_SYS_ID;
      break;
    case STATE_SYS_ID:
      msg->sys_id = byte;
      checksum += byte;
      state = STATE_MSG_ID;
      break;
    case STATE_MSG_ID:
      msg->msg_id = byte;
      checksum += byte;
      state = STATE_SEQ;
      break;
    case STATE_SEQ:
      msg->seq = byte;
      checksum += byte;
      state = STATE_LEN;
      break;
    case STATE_LEN:
      msg->len = byte;
      checksum += byte;
      if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) {
        state = STATE_HEAD;
      } else if (msg->len == 0) {
        state = STATE_CHECKSUM;
      } else {
        payload_cnt = 0;
        state = STATE_PAYLOAD;
      }
      break;
    case STATE_PAYLOAD:
      msg->payload[payload_cnt++] = byte;
      checksum += byte;
      if (payload_cnt >= msg->len) {
        state = STATE_CHECKSUM;
      }
      break;
    case STATE_CHECKSUM:
      msg->checksum = byte;
      state = STATE_HEAD;
      if (checksum == byte) {
        return true;
      }
      break;
    default:
      state = STATE_HEAD;
      break;
  }
  return false;
}

static void micoairTask(void* arg) {
  systemWaitStart();

  // Initialize UART1 at 115200 baud
  uart1Init(115200);

  MICOLINK_MSG_t rxMsg = {0};
  uint8_t byte;

  uint64_t lastFlowTime = usecTimestamp();

  while (1) {
    while (uart1GetDataWithTimeout(&byte, M2T(5))) {
      if (micolink_parse_char(&rxMsg, byte)) {
        if (rxMsg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR && rxMsg.len == sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t)) {
          MICOLINK_PAYLOAD_RANGE_SENSOR_t* payload = (MICOLINK_PAYLOAD_RANGE_SENSOR_t*)rxMsg.payload;

          micoair_distance = payload->distance;
          micoair_flow_x = (float)payload->flow_vel_x;
          micoair_flow_y = (float)payload->flow_vel_y;
          micoair_quality = payload->flow_quality;

          uint32_t nowMs = xTaskGetTickCount();
          uint64_t nowUs = usecTimestamp();
          float dt = (float)(nowUs - lastFlowTime) / 1000000.0f;
          lastFlowTime = nowUs;

          // Prevent extremely large or small dt
          if (dt < 0.001f) dt = 0.001f;
          if (dt > 0.5f) dt = 0.5f;

          // Enqueue Distance (ToF) if valid
          if (payload->distance > 0 && payload->distance < 8000) {
            tofMeasurement_t tofData;
            tofData.timestamp = nowMs;
            tofData.distance = (float)payload->distance / 1000.0f; // mm to m
            tofData.stdDev = 0.05f; // Standard deviation of 5cm
            estimatorEnqueueTOF(&tofData);
          }

          // Enqueue Flow if optical flow quality is sufficient
          if (payload->flow_quality > 15 && payload->distance > 0) {
            float height = (float)payload->distance / 1000.0f; // mm to m
            if (height < 0.1f) height = 0.1f;

            // Convert velocity (cm/s) to m/s
            float vx = (float)payload->flow_vel_x / 100.0f;
            float vy = (float)payload->flow_vel_y / 100.0f;

            // Get latest gyro data to add rotation back (so EKF can subtract it)
            Axis3f gyro;
            sensorsReadGyro(&gyro);
            float gyro_x_rad = gyro.x * (float)M_PI / 180.0f;
            float gyro_y_rad = gyro.y * (float)M_PI / 180.0f;

            // Compute raw pixel count (NX, NY) and scale by FLOW_RESOLUTION
            float const_factor = NPIX / THETAPIX;
            float rawNX = dt * const_factor * (vx / height - gyro_y_rad);
            float rawNY = dt * const_factor * (vy / height + gyro_x_rad);

            flowMeasurement_t flowData;
            flowData.timestamp = nowMs;
            flowData.dt = dt;
            flowData.dpixelx = rawNX / FLOW_RESOLUTION;
            flowData.dpixely = rawNY / FLOW_RESOLUTION;
            flowData.stdDevX = 2.0f; // Default standard deviation
            flowData.stdDevY = 2.0f;

            estimatorEnqueueFlow(&flowData);
          }
        }
      }
    }
    vTaskDelay(M2T(2));
  }
}

static void micoairInit(DeckInfo* info) {
  if (isInit) return;

  xTaskCreate(micoairTask, "MICOAIR", configMINIMAL_STACK_SIZE + 200, NULL, 3, NULL);

  isInit = true;
  DEBUG_PRINT("MicoAir MTF-01/MT-06 deck driver initialized.\n");
}

static bool micoairTest(void) {
  return isInit;
}

static const DeckDriver micoair_deck = {
  .vid = 0xBC,
  .pid = 0xFE, // Custom PID for MicoAir driver
  .name = "bcMicoAir",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1, // UART1 corresponds to USART3

  .init = micoairInit,
  .test = micoairTest,
};

DECK_DRIVER(micoair_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcMicoAir, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(micoair)
LOG_ADD(LOG_UINT32, distance, &micoair_distance)
LOG_ADD(LOG_FLOAT, flowX, &micoair_flow_x)
LOG_ADD(LOG_FLOAT, flowY, &micoair_flow_y)
LOG_ADD(LOG_UINT8, quality, &micoair_quality)
LOG_GROUP_STOP(micoair)
