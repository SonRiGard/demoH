#ifndef APP_IMU_MPU6050_IMU_H
#define APP_IMU_MPU6050_IMU_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /* MAVLink ATTITUDE uses radians, so this public struct keeps the same unit.
     Internally the Kalman filters run in degrees because the gyro scale from the
     MPU datasheet is easier to reason about in deg/s. */
  float roll_rad;
  float pitch_rad;
  /* Yaw is integrated from gyro Z only. MPU6050 has no magnetometer, so yaw will
     drift over time and should be treated as a relative/debug value. */
  float yaw_rad;
  float roll_rate_rad_s;
  float pitch_rate_rad_s;
  float yaw_rate_rad_s;
  uint32_t time_boot_ms;
  /* 1 means the latest sample came from a successful I2C read and filter update. */
  uint8_t valid;
} Mpu6050Attitude;

typedef struct
{
  /* Debug data is intentionally raw/simple so QGC NAMED_VALUE_FLOAT can show it
     without requiring a custom MAVLink dialect. */
  uint16_t addr;
  uint8_t who_am_i;
  uint8_t ready;
  uint8_t last_error;
  uint8_t read_ok;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  int16_t accel_bias_raw[3];
  int16_t gyro_bias_raw[3];
} Mpu6050Debug;

/* Detect, configure and calibrate the MPU on the supplied I2C bus.
   The board should be kept still during this call because gyro/accel biases are
   measured from the first MPU6050_CALIB_SAMPLES samples. */
uint8_t Mpu6050Imu_Init(I2C_HandleTypeDef *hi2c);

/* Non-blocking periodic update. Call often from the main loop; the function only
   reads the sensor when MPU6050_UPDATE_PERIOD_MS has elapsed. */
void Mpu6050Imu_Tick(void);

/* Copy the latest filtered attitude. Returns 1 when the copied data is valid. */
uint8_t Mpu6050Imu_GetAttitude(Mpu6050Attitude *attitude);

/* Copy raw/debug state for telemetry and field troubleshooting. */
void Mpu6050Imu_GetDebug(Mpu6050Debug *debug);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_MPU6050_IMU_H */
