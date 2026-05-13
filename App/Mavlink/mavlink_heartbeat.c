#include "mavlink_heartbeat.h"

#include "main.h"
#include "usbd_cdc_if.h"

/*
 * Minimal MAVLink v2 HEARTBEAT sender.
 * This is enough for QGroundControl to detect the board as online.
 */

#define MAVLINK_STX_V2              0xFDU
#define MAVLINK_MSG_ID_HEARTBEAT    0U
#define MAVLINK_MSG_LEN_HEARTBEAT    9U
#define MAVLINK_MSG_CRC_HEARTBEAT   50U

#define MAV_TYPE_GENERIC             0U
#define MAV_AUTOPILOT_GENERIC        0U
#define MAV_COMP_ID_AUTOPILOT1       1U
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 0x01U
#define MAV_STATE_ACTIVE             4U
#define MAVLINK_VERSION_FIELD        3U

#define HEARTBEAT_PERIOD_MS       1000U

typedef struct
{
  uint8_t sysid;
  uint8_t compid;
  uint8_t type;
  uint8_t autopilot;
  uint8_t base_mode;
  uint8_t system_status;
  uint32_t custom_mode;
  uint32_t period_ms;
} MavlinkHeartbeatConfig;

static MavlinkHeartbeatConfig s_cfg = {
  .sysid = 1U,
  .compid = MAV_COMP_ID_AUTOPILOT1,
  .type = MAV_TYPE_GENERIC,
  .autopilot = MAV_AUTOPILOT_GENERIC,
  .base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
  .system_status = MAV_STATE_ACTIVE,
  .custom_mode = 0U,
  .period_ms = HEARTBEAT_PERIOD_MS,
};

static uint8_t s_seq = 0U;
static uint32_t s_last_tx_ms = 0U;
static uint8_t s_tx_buffer[32];

static uint16_t crc_accumulate(uint8_t data, uint16_t crc)
{
  uint8_t tmp;

  tmp = data ^ (uint8_t)(crc & 0xFFU);
  tmp ^= (uint8_t)(tmp << 4);

  return (uint16_t)(((uint16_t)crc >> 8)
                    ^ ((uint16_t)tmp << 8)
                    ^ ((uint16_t)tmp << 3)
                    ^ ((uint16_t)tmp >> 4));
}

static uint16_t crc_calculate(const uint8_t *buffer, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < length; i++)
  {
    crc = crc_accumulate(buffer[i], crc);
  }

  return crc;
}

static uint16_t mavlink_heartbeat_build(uint8_t *out, const MavlinkHeartbeatConfig *cfg)
{
  uint8_t payload[MAVLINK_MSG_LEN_HEARTBEAT];
  uint16_t crc;
  uint16_t index = 0U;

  payload[0] = (uint8_t)(cfg->custom_mode & 0xFFU);
  payload[1] = (uint8_t)((cfg->custom_mode >> 8) & 0xFFU);
  payload[2] = (uint8_t)((cfg->custom_mode >> 16) & 0xFFU);
  payload[3] = (uint8_t)((cfg->custom_mode >> 24) & 0xFFU);
  payload[4] = cfg->type;
  payload[5] = cfg->autopilot;
  payload[6] = cfg->base_mode;
  payload[7] = cfg->system_status;
  payload[8] = MAVLINK_VERSION_FIELD;

  out[index++] = MAVLINK_STX_V2;
  out[index++] = MAVLINK_MSG_LEN_HEARTBEAT;
  out[index++] = 0U;
  out[index++] = 0U;
  out[index++] = s_seq++;
  out[index++] = cfg->sysid;
  out[index++] = cfg->compid;
  out[index++] = (uint8_t)(MAVLINK_MSG_ID_HEARTBEAT & 0xFFU);
  out[index++] = (uint8_t)((MAVLINK_MSG_ID_HEARTBEAT >> 8) & 0xFFU);
  out[index++] = (uint8_t)((MAVLINK_MSG_ID_HEARTBEAT >> 16) & 0xFFU);

  for (uint16_t i = 0U; i < MAVLINK_MSG_LEN_HEARTBEAT; i++)
  {
    out[index++] = payload[i];
  }

  crc = crc_calculate(payload, MAVLINK_MSG_LEN_HEARTBEAT);
  crc = crc_accumulate(MAVLINK_MSG_CRC_HEARTBEAT, crc);

  out[index++] = (uint8_t)(crc & 0xFFU);
  out[index++] = (uint8_t)((crc >> 8) & 0xFFU);

  return index;
}

void MavlinkHeartbeat_Init(void)
{
  s_seq = 0U;
  s_last_tx_ms = HAL_GetTick() - s_cfg.period_ms;
}

void MavlinkHeartbeat_Tick(void)
{
  uint32_t now = HAL_GetTick();
  uint16_t frame_len;

  if ((now - s_last_tx_ms) < s_cfg.period_ms)
  {
    return;
  }

  frame_len = mavlink_heartbeat_build(s_tx_buffer, &s_cfg);
  if (CDC_Transmit_FS(s_tx_buffer, frame_len) == USBD_OK)
  {
    s_last_tx_ms = now;
  }
}
