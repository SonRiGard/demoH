#include "mpu6050_imu.h"

#include <math.h>
#include <string.h>

/* Minimal MPU6050/compatible IMU driver used by the MAVLink app.
   The module owns all sensor state, keeps I2C access in one place, and exposes
   only two outputs: filtered attitude and simple debug counters/raw values. */

/* MPU devices use a 7-bit address on the bus. STM32 HAL expects that address
   shifted left by one bit, so 0x68 becomes 0xD0 and 0x69 becomes 0xD2. */
#define MPU6050_ADDR_AD0_LOW       (0x68U << 1)
#define MPU6050_ADDR_AD0_HIGH      (0x69U << 1)

/* Register map entries used by this driver. They are kept here instead of a
   separate header because only this C file should touch sensor registers. */
#define MPU6050_REG_SMPLRT_DIV     0x19U
#define MPU6050_REG_CONFIG         0x1AU
#define MPU6050_REG_GYRO_CONFIG    0x1BU
#define MPU6050_REG_ACCEL_CONFIG   0x1CU
#define MPU6050_REG_ACCEL_XOUT_H   0x3BU
#define MPU6050_REG_PWR_MGMT_1     0x6BU
#define MPU6050_REG_WHO_AM_I       0x75U

#define MPU6050_I2C_TIMEOUT_MS     20U
#define MPU6050_CALIB_SAMPLES      200U
#define MPU6050_UPDATE_PERIOD_MS   10U

/* Full-scale settings configured below:
   accel +/-2g => 16384 LSB/g
   gyro +/-250 deg/s => 131 LSB/(deg/s) */
#define ACCEL_SCALE_LSB_G          16384.0f
#define GYRO_SCALE_LSB_DPS         131.0f
#define DEG_TO_RAD                 0.017453292519943295f
#define RAD_TO_DEG                 57.29577951308232f

typedef struct
{
  /* 1D Kalman state for one angle axis. The filter estimates angle and gyro
     bias. Accel gives a noisy absolute angle; gyro gives smooth short-term rate. */
  float q_angle;
  float q_bias;
  float r_measure;
  float angle;
  float bias;
  float p[2][2];
} Kalman1D;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t addr;
  /* Biases are stored in raw sensor units so the subtraction happens before
     converting to g or deg/s. */
  int16_t accel_bias_raw[3];
  int16_t gyro_bias_raw[3];
  Kalman1D roll_filter;
  Kalman1D pitch_filter;
  Mpu6050Attitude attitude;
  Mpu6050Debug debug;
  uint32_t last_update_ms;
  uint8_t ready;
} Mpu6050ImuState;

static Mpu6050ImuState s_imu;

/* Read a register block and mirror the HAL status into debug telemetry.
   This makes I2C errors visible in QGC through MPU_ERR. */
static HAL_StatusTypeDef read_reg(uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(s_imu.hi2c, s_imu.addr, reg, I2C_MEMADD_SIZE_8BIT,
                                              data, len, MPU6050_I2C_TIMEOUT_MS);
  s_imu.debug.last_error = (uint8_t)status;
  return status;
}

/* Write one register and keep the last HAL status for field debugging. */
static HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(s_imu.hi2c, s_imu.addr, reg, I2C_MEMADD_SIZE_8BIT,
                                               &data, 1U, MPU6050_I2C_TIMEOUT_MS);
  s_imu.debug.last_error = (uint8_t)status;
  return status;
}

/* MPU6050 outputs high byte first. Convert two bytes from the burst-read buffer
   to the signed 16-bit value used by accel/gyro registers. */
static int16_t read_i16_be(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/* Start the Kalman filter from the current accelerometer angle so the first
   transmitted attitude does not jump from zero to the measured pose. */
static void kalman_init(Kalman1D *filter, float initial_angle_deg)
{
  memset(filter, 0, sizeof(*filter));
  filter->q_angle = 0.001f;
  filter->q_bias = 0.003f;
  filter->r_measure = 0.03f;
  filter->angle = initial_angle_deg;
}

/* Fuse accelerometer angle with gyro rate for one axis.
   Prediction step: integrate gyro rate after subtracting estimated gyro bias.
   Correction step: pull the predicted angle toward the accelerometer angle. */
static float kalman_update(Kalman1D *filter, float angle_deg, float rate_deg_s, float dt_s)
{
  float rate = rate_deg_s - filter->bias;
  filter->angle += dt_s * rate;

  filter->p[0][0] += dt_s * (dt_s * filter->p[1][1] - filter->p[0][1] -
                              filter->p[1][0] + filter->q_angle);
  filter->p[0][1] -= dt_s * filter->p[1][1];
  filter->p[1][0] -= dt_s * filter->p[1][1];
  filter->p[1][1] += filter->q_bias * dt_s;

  float s = filter->p[0][0] + filter->r_measure;
  float k0 = filter->p[0][0] / s;
  float k1 = filter->p[1][0] / s;
  float y = angle_deg - filter->angle;

  filter->angle += k0 * y;
  filter->bias += k1 * y;

  float p00 = filter->p[0][0];
  float p01 = filter->p[0][1];

  filter->p[0][0] -= k0 * p00;
  filter->p[0][1] -= k0 * p01;
  filter->p[1][0] -= k1 * p00;
  filter->p[1][1] -= k1 * p01;

  return filter->angle;
}

/* Probe both possible AD0 strap addresses and accept common compatible WHO_AM_I
   values. The user's board reports 0x70, which is typical for MPU6500-style
   compatible parts, while many MPU6050 modules report 0x68. */
static uint8_t detect_device(void)
{
  uint8_t who_am_i = 0U;
  const uint16_t candidates[] = {MPU6050_ADDR_AD0_LOW, MPU6050_ADDR_AD0_HIGH};

  for (uint32_t i = 0U; i < (sizeof(candidates) / sizeof(candidates[0])); i++)
  {
    s_imu.addr = candidates[i];
    s_imu.debug.addr = s_imu.addr;
    if (HAL_I2C_IsDeviceReady(s_imu.hi2c, s_imu.addr, 2U, MPU6050_I2C_TIMEOUT_MS) != HAL_OK)
    {
      s_imu.debug.last_error = (uint8_t)HAL_ERROR;
      continue;
    }

    if (read_reg(MPU6050_REG_WHO_AM_I, &who_am_i, 1U) == HAL_OK)
    {
      s_imu.debug.who_am_i = who_am_i;
      if ((who_am_i == 0x68U) || (who_am_i == 0x69U) ||
          (who_am_i == 0x70U) || (who_am_i == 0x71U))
      {
        return 1U;
      }
    }
  }

  return 0U;
}

static uint8_t read_raw(int16_t accel[3], int16_t gyro[3])
{
  uint8_t data[14];

  /* ACCEL_XOUT_H starts a contiguous 14-byte block:
     accel XYZ, temperature, then gyro XYZ. Temperature is skipped here. */
  if (read_reg(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data)) != HAL_OK)
  {
    return 0U;
  }

  accel[0] = read_i16_be(&data[0]);
  accel[1] = read_i16_be(&data[2]);
  accel[2] = read_i16_be(&data[4]);
  gyro[0] = read_i16_be(&data[8]);
  gyro[1] = read_i16_be(&data[10]);
  gyro[2] = read_i16_be(&data[12]);
  s_imu.debug.read_ok = 1U;
  for (uint32_t i = 0U; i < 3U; i++)
  {
    s_imu.debug.accel_raw[i] = accel[i];
    s_imu.debug.gyro_raw[i] = gyro[i];
  }

  return 1U;
}

static void calibrate(void)
{
  int16_t accel[3];
  int16_t gyro[3];
  int32_t accel_sum[3] = {0};
  int32_t gyro_sum[3] = {0};
  uint32_t sample_count = 0U;

  for (uint32_t i = 0U; i < MPU6050_CALIB_SAMPLES; i++)
  {
    /* Calibration assumes the board is stationary. Averaging many samples removes
       constant offset, but it cannot compensate if the board is moving at boot. */
    if (read_raw(accel, gyro) != 0U)
    {
      accel_sum[0] += accel[0];
      accel_sum[1] += accel[1];
      accel_sum[2] += accel[2];
      gyro_sum[0] += gyro[0];
      gyro_sum[1] += gyro[1];
      gyro_sum[2] += gyro[2];
      sample_count++;
    }
    HAL_Delay(2U);
  }

  if (sample_count == 0U)
  {
    return;
  }

  /* X/Y accel bias should average around zero when stationary. Z includes gravity,
     so remove 1g from the bias to keep gravity available for roll/pitch angles. */
  s_imu.accel_bias_raw[0] = (int16_t)(accel_sum[0] / (int32_t)sample_count);
  s_imu.accel_bias_raw[1] = (int16_t)(accel_sum[1] / (int32_t)sample_count);
  s_imu.accel_bias_raw[2] = (int16_t)((accel_sum[2] / (int32_t)sample_count) - (int32_t)ACCEL_SCALE_LSB_G);
  s_imu.gyro_bias_raw[0] = (int16_t)(gyro_sum[0] / (int32_t)sample_count);
  s_imu.gyro_bias_raw[1] = (int16_t)(gyro_sum[1] / (int32_t)sample_count);
  s_imu.gyro_bias_raw[2] = (int16_t)(gyro_sum[2] / (int32_t)sample_count);
  for (uint32_t i = 0U; i < 3U; i++)
  {
    s_imu.debug.accel_bias_raw[i] = s_imu.accel_bias_raw[i];
    s_imu.debug.gyro_bias_raw[i] = s_imu.gyro_bias_raw[i];
  }
}

uint8_t Mpu6050Imu_Init(I2C_HandleTypeDef *hi2c)
{
  int16_t accel[3];
  int16_t gyro[3];

  memset(&s_imu, 0, sizeof(s_imu));
  s_imu.hi2c = hi2c;

  if ((hi2c == NULL) || (detect_device() == 0U))
  {
    return 0U;
  }

  /* Wake the sensor and select conservative default full-scale ranges:
     DLPF enabled, sample divider 7, accel +/-2g, gyro +/-250 deg/s. */
  if ((write_reg(MPU6050_REG_PWR_MGMT_1, 0x00U) != HAL_OK) ||
      (write_reg(MPU6050_REG_CONFIG, 0x03U) != HAL_OK) ||
      (write_reg(MPU6050_REG_SMPLRT_DIV, 0x07U) != HAL_OK) ||
      (write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00U) != HAL_OK) ||
      (write_reg(MPU6050_REG_GYRO_CONFIG, 0x00U) != HAL_OK))
  {
    return 0U;
  }

  HAL_Delay(100U);
  calibrate();

  if (read_raw(accel, gyro) == 0U)
  {
    return 0U;
  }

  /* Seed roll/pitch from accelerometer geometry:
     roll = rotation around X from Y/Z gravity projection
     pitch = rotation around Y from X and horizontal gravity magnitude */
  float ax = ((float)accel[0] - (float)s_imu.accel_bias_raw[0]);
  float ay = ((float)accel[1] - (float)s_imu.accel_bias_raw[1]);
  float az = ((float)accel[2] - (float)s_imu.accel_bias_raw[2]);
  float roll_deg = atan2f(ay, az) * RAD_TO_DEG;
  float pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;

  kalman_init(&s_imu.roll_filter, roll_deg);
  kalman_init(&s_imu.pitch_filter, pitch_deg);

  s_imu.attitude.roll_rad = roll_deg * DEG_TO_RAD;
  s_imu.attitude.pitch_rad = pitch_deg * DEG_TO_RAD;
  s_imu.attitude.yaw_rad = 0.0f;
  s_imu.attitude.valid = 1U;
  s_imu.last_update_ms = HAL_GetTick();
  s_imu.ready = 1U;
  s_imu.debug.ready = 1U;

  return 1U;
}

void Mpu6050Imu_Tick(void)
{
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  uint32_t now = HAL_GetTick();

  if ((s_imu.ready == 0U) || ((now - s_imu.last_update_ms) < MPU6050_UPDATE_PERIOD_MS))
  {
    /* Called faster than the configured IMU rate. Return immediately so the main
       loop can keep servicing USB and other tasks. */
    return;
  }

  if (read_raw(accel_raw, gyro_raw) == 0U)
  {
    /* Keep the previous attitude values, but mark them invalid so MAVLink can
       fall back to deterministic fake values instead of sending stale real data. */
    s_imu.attitude.valid = 0U;
    s_imu.debug.read_ok = 0U;
    return;
  }

  float dt_s = (float)(now - s_imu.last_update_ms) / 1000.0f;
  s_imu.last_update_ms = now;

  /* Remove measured biases before converting units. Accel values are left in raw
     LSB here because atan2f only needs ratios, not absolute g units. */
  float ax_raw = (float)accel_raw[0] - (float)s_imu.accel_bias_raw[0];
  float ay_raw = (float)accel_raw[1] - (float)s_imu.accel_bias_raw[1];
  float az_raw = (float)accel_raw[2] - (float)s_imu.accel_bias_raw[2];

  float gx_deg_s = ((float)gyro_raw[0] - (float)s_imu.gyro_bias_raw[0]) / GYRO_SCALE_LSB_DPS;
  float gy_deg_s = ((float)gyro_raw[1] - (float)s_imu.gyro_bias_raw[1]) / GYRO_SCALE_LSB_DPS;
  float gz_deg_s = ((float)gyro_raw[2] - (float)s_imu.gyro_bias_raw[2]) / GYRO_SCALE_LSB_DPS;

  float roll_acc_deg = atan2f(ay_raw, az_raw) * RAD_TO_DEG;
  float pitch_acc_deg = atan2f(-ax_raw, sqrtf((ay_raw * ay_raw) + (az_raw * az_raw))) * RAD_TO_DEG;

  /* Roll and pitch are observable from gravity, so they can be corrected by the
     accelerometer. Yaw cannot be corrected with MPU6050 alone and will drift. */
  float roll_deg = kalman_update(&s_imu.roll_filter, roll_acc_deg, gx_deg_s, dt_s);
  float pitch_deg = kalman_update(&s_imu.pitch_filter, pitch_acc_deg, gy_deg_s, dt_s);

  s_imu.attitude.roll_rad = roll_deg * DEG_TO_RAD;
  s_imu.attitude.pitch_rad = pitch_deg * DEG_TO_RAD;
  s_imu.attitude.yaw_rad += gz_deg_s * DEG_TO_RAD * dt_s;
  s_imu.attitude.roll_rate_rad_s = gx_deg_s * DEG_TO_RAD;
  s_imu.attitude.pitch_rate_rad_s = gy_deg_s * DEG_TO_RAD;
  s_imu.attitude.yaw_rate_rad_s = gz_deg_s * DEG_TO_RAD;
  s_imu.attitude.time_boot_ms = now;
  s_imu.attitude.valid = 1U;
}

uint8_t Mpu6050Imu_GetAttitude(Mpu6050Attitude *attitude)
{
  if (attitude == NULL)
  {
    return 0U;
  }

  /* Return a copy, not a pointer to module state. This keeps callers from
     accidentally modifying filter output while the next Tick is running. */
  *attitude = s_imu.attitude;
  return s_imu.attitude.valid;
}

void Mpu6050Imu_GetDebug(Mpu6050Debug *debug)
{
  if (debug == NULL)
  {
    return;
  }

  /* Debug is also copied by value so MAVLink can format it safely. */
  *debug = s_imu.debug;
}
