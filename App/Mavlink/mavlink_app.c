#include "mavlink_app.h"

#include "mpu6050_imu.h"

#include "main.h"
#include "usbd_cdc_if.h"

#define MAVLINK_STX_V2              0xFDU
#define MAVLINK_MSG_ID_HEARTBEAT    0U
#define MAVLINK_MSG_LEN_HEARTBEAT    9U
#define MAVLINK_MSG_CRC_HEARTBEAT   50U
#define MAVLINK_MSG_ID_ATTITUDE     30U
#define MAVLINK_MSG_LEN_ATTITUDE    28U
#define MAVLINK_MSG_CRC_ATTITUDE    39U
#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT 251U
#define MAVLINK_MSG_LEN_NAMED_VALUE_FLOAT 18U
#define MAVLINK_MSG_CRC_NAMED_VALUE_FLOAT 170U
#define MAVLINK_MSG_ID_STATUSTEXT   253U
#define MAVLINK_MSG_LEN_STATUSTEXT  51U
#define MAVLINK_MSG_CRC_STATUSTEXT  83U
#define MAVLINK_V2_HEADER_LEN       10U
#define MAVLINK_V2_CRC_INPUT_LEN    (MAVLINK_V2_HEADER_LEN - 1U)

#define MAV_TYPE_GENERIC             0U
#define MAV_AUTOPILOT_GENERIC        0U
#define MAV_COMP_ID_AUTOPILOT1       1U
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 0x01U
#define MAV_STATE_ACTIVE             4U
#define MAVLINK_VERSION_FIELD        3U

#define HEARTBEAT_PERIOD_MS       1000U
#define ATTITUDE_PERIOD_MS         100U
#define DEBUG_PERIOD_MS           1000U
#define FAKE_ROLL_RAD              0.174533f
#define FAKE_PITCH_RAD            -0.087266f
#define FAKE_YAW_RAD               0.523599f
#define MAV_SEVERITY_INFO          6U

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
} MavlinkConfig;

static MavlinkConfig s_cfg = {
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
static uint32_t s_last_heartbeat_ms = 0U;
static uint32_t s_last_attitude_ms = 0U;
static uint32_t s_last_debug_ms = 0U;
static uint8_t s_tx_buffers[4][64];
static uint8_t s_tx_buffer_index = 0U;
static uint8_t s_imu_ready = 0U;
static uint8_t s_boot_status_sent = 0U;

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

static void put_u32_le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void put_float_le(uint8_t *dst, float value)
{
  union
  {
    float f;
    uint32_t u32;
  } raw;

  raw.f = value;
  put_u32_le(dst, raw.u32);
}

static void put_name_10(uint8_t *dst, const char *name)
{
  uint8_t i = 0U;

  for (; i < 10U; i++)
  {
    if ((name == NULL) || (name[i] == '\0'))
    {
      break;
    }
    dst[i] = (uint8_t)name[i];
  }

  for (; i < 10U; i++)
  {
    dst[i] = 0U;
  }
}

static uint16_t mavlink_frame_build(uint8_t *out,
                                    uint32_t msgid,
                                    const uint8_t *payload,
                                    uint8_t payload_len,
                                    uint8_t crc_extra,
                                    const MavlinkConfig *cfg)
{
  uint16_t crc;
  uint16_t index = 0U;

  out[index++] = MAVLINK_STX_V2;
  out[index++] = payload_len;
  out[index++] = 0U;
  out[index++] = 0U;
  out[index++] = s_seq++;
  out[index++] = cfg->sysid;
  out[index++] = cfg->compid;
  out[index++] = (uint8_t)(msgid & 0xFFU);
  out[index++] = (uint8_t)((msgid >> 8) & 0xFFU);
  out[index++] = (uint8_t)((msgid >> 16) & 0xFFU);

  for (uint16_t i = 0U; i < payload_len; i++)
  {
    out[index++] = payload[i];
  }

  crc = crc_calculate(&out[1], MAVLINK_V2_CRC_INPUT_LEN);
  for (uint16_t i = 0U; i < payload_len; i++)
  {
    crc = crc_accumulate(payload[i], crc);
  }
  crc = crc_accumulate(crc_extra, crc);

  out[index++] = (uint8_t)(crc & 0xFFU);
  out[index++] = (uint8_t)((crc >> 8) & 0xFFU);

  return index;
}

static uint16_t mavlink_heartbeat_build(uint8_t *out, const MavlinkConfig *cfg)
{
  uint8_t payload[MAVLINK_MSG_LEN_HEARTBEAT];

  put_u32_le(&payload[0], cfg->custom_mode);
  payload[4] = cfg->type;
  payload[5] = cfg->autopilot;
  payload[6] = cfg->base_mode;
  payload[7] = cfg->system_status;
  payload[8] = MAVLINK_VERSION_FIELD;

  return mavlink_frame_build(out, MAVLINK_MSG_ID_HEARTBEAT, payload,
                             MAVLINK_MSG_LEN_HEARTBEAT,
                             MAVLINK_MSG_CRC_HEARTBEAT, cfg);
}

static uint16_t mavlink_attitude_build(uint8_t *out, const MavlinkConfig *cfg,
                                       const Mpu6050Attitude *attitude)
{
  uint8_t payload[MAVLINK_MSG_LEN_ATTITUDE];
  uint32_t time_boot_ms = HAL_GetTick();
  float roll_rad = FAKE_ROLL_RAD;
  float pitch_rad = FAKE_PITCH_RAD;
  float yaw_rad = FAKE_YAW_RAD;
  float roll_rate_rad_s = 0.0f;
  float pitch_rate_rad_s = 0.0f;
  float yaw_rate_rad_s = 0.0f;

  if ((attitude != NULL) && (attitude->valid != 0U))
  {
    time_boot_ms = attitude->time_boot_ms;
    roll_rad = attitude->roll_rad;
    pitch_rad = attitude->pitch_rad;
    yaw_rad = attitude->yaw_rad;
    roll_rate_rad_s = attitude->roll_rate_rad_s;
    pitch_rate_rad_s = attitude->pitch_rate_rad_s;
    yaw_rate_rad_s = attitude->yaw_rate_rad_s;
  }

  put_u32_le(&payload[0], time_boot_ms);
  put_float_le(&payload[4], roll_rad);
  put_float_le(&payload[8], pitch_rad);
  put_float_le(&payload[12], yaw_rad);
  put_float_le(&payload[16], roll_rate_rad_s);
  put_float_le(&payload[20], pitch_rate_rad_s);
  put_float_le(&payload[24], yaw_rate_rad_s);

  return mavlink_frame_build(out, MAVLINK_MSG_ID_ATTITUDE, payload,
                             MAVLINK_MSG_LEN_ATTITUDE,
                             MAVLINK_MSG_CRC_ATTITUDE, cfg);
}

static uint16_t mavlink_named_value_float_build(uint8_t *out, const MavlinkConfig *cfg,
                                                const char *name, float value)
{
  uint8_t payload[MAVLINK_MSG_LEN_NAMED_VALUE_FLOAT];

  put_u32_le(&payload[0], HAL_GetTick());
  put_float_le(&payload[4], value);
  put_name_10(&payload[8], name);

  return mavlink_frame_build(out, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, payload,
                             MAVLINK_MSG_LEN_NAMED_VALUE_FLOAT,
                             MAVLINK_MSG_CRC_NAMED_VALUE_FLOAT, cfg);
}

static uint16_t mavlink_statustext_build(uint8_t *out, const MavlinkConfig *cfg,
                                         const char *text)
{
  uint8_t payload[MAVLINK_MSG_LEN_STATUSTEXT] = {0};

  payload[0] = MAV_SEVERITY_INFO;
  for (uint32_t i = 0U; (i < 50U) && (text != NULL) && (text[i] != '\0'); i++)
  {
    payload[1U + i] = (uint8_t)text[i];
  }

  return mavlink_frame_build(out, MAVLINK_MSG_ID_STATUSTEXT, payload,
                             MAVLINK_MSG_LEN_STATUSTEXT,
                             MAVLINK_MSG_CRC_STATUSTEXT, cfg);
}

static uint8_t *mavlink_next_tx_buffer(void)
{
  uint8_t *buffer = s_tx_buffers[s_tx_buffer_index];

  s_tx_buffer_index++;
  if (s_tx_buffer_index >= 4U)
  {
    s_tx_buffer_index = 0U;
  }

  return buffer;
}

void MavlinkApp_Init(I2C_HandleTypeDef *hi2c)
{
  s_seq = 0U;
  s_last_heartbeat_ms = HAL_GetTick() - s_cfg.period_ms;
  s_last_attitude_ms = HAL_GetTick() - ATTITUDE_PERIOD_MS;
  s_last_debug_ms = HAL_GetTick() - DEBUG_PERIOD_MS;
  s_boot_status_sent = 0U;
  s_imu_ready = Mpu6050Imu_Init(hi2c);
}

void MavlinkApp_Tick(void)
{
  uint32_t now = HAL_GetTick();
  Mpu6050Attitude attitude;
  Mpu6050Debug imu_debug;
  Mpu6050Attitude *attitude_ptr = NULL;
  uint8_t *tx_buffer;
  uint16_t frame_len;

  if (s_imu_ready != 0U)
  {
    Mpu6050Imu_Tick();
    if (Mpu6050Imu_GetAttitude(&attitude) != 0U)
    {
      attitude_ptr = &attitude;
    }
  }

  if (CDC_IsTransmitReady_FS() == 0U)
  {
    return;
  }

  Mpu6050Imu_GetDebug(&imu_debug);

  if (s_boot_status_sent == 0U)
  {
    tx_buffer = mavlink_next_tx_buffer();
    frame_len = mavlink_statustext_build(tx_buffer, &s_cfg,
                                         (s_imu_ready != 0U) ? "MPU6050 OK" : "MPU6050 FAIL");
    if (CDC_Transmit_FS(tx_buffer, frame_len) == USBD_OK)
    {
      s_boot_status_sent = 1U;
      return;
    }
  }

  if ((now - s_last_heartbeat_ms) >= s_cfg.period_ms)
  {
    tx_buffer = mavlink_next_tx_buffer();
    frame_len = mavlink_heartbeat_build(tx_buffer, &s_cfg);
    if (CDC_Transmit_FS(tx_buffer, frame_len) == USBD_OK)
    {
      s_last_heartbeat_ms = now;
      return;
    }
  }

  if ((now - s_last_attitude_ms) >= ATTITUDE_PERIOD_MS)
  {
    tx_buffer = mavlink_next_tx_buffer();
    frame_len = mavlink_attitude_build(tx_buffer, &s_cfg, attitude_ptr);
    if (CDC_Transmit_FS(tx_buffer, frame_len) == USBD_OK)
    {
      s_last_attitude_ms = now;
      return;
    }
  }

  if ((now - s_last_debug_ms) >= DEBUG_PERIOD_MS)
  {
    static uint8_t debug_index = 0U;
    const char *name = "MPU_RDY";
    float value = (float)imu_debug.ready;

    switch (debug_index)
    {
      case 0U:
        name = "MPU_RDY";
        value = (float)imu_debug.ready;
        break;
      case 1U:
        name = "MPU_WHO";
        value = (float)imu_debug.who_am_i;
        break;
      case 2U:
        name = "MPU_ADDR";
        value = (float)imu_debug.addr;
        break;
      case 3U:
        name = "MPU_ERR";
        value = (float)imu_debug.last_error;
        break;
      case 4U:
        name = "ACC_X";
        value = (float)imu_debug.accel_raw[0];
        break;
      case 5U:
        name = "ACC_Y";
        value = (float)imu_debug.accel_raw[1];
        break;
      case 6U:
        name = "ACC_Z";
        value = (float)imu_debug.accel_raw[2];
        break;
      case 7U:
        name = "GYR_Z";
        value = (float)imu_debug.gyro_raw[2];
        break;
      default:
        debug_index = 0U;
        return;
    }

    tx_buffer = mavlink_next_tx_buffer();
    frame_len = mavlink_named_value_float_build(tx_buffer, &s_cfg, name, value);
    if (CDC_Transmit_FS(tx_buffer, frame_len) == USBD_OK)
    {
      debug_index++;
      s_last_debug_ms = now;
    }
  }
}
