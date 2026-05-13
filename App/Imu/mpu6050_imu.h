#ifndef APP_IMU_MPU6050_IMU_H
#define APP_IMU_MPU6050_IMU_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  float roll_rate_rad_s;
  float pitch_rate_rad_s;
  float yaw_rate_rad_s;
  uint32_t time_boot_ms;
  uint8_t valid;
} Mpu6050Attitude;

typedef struct
{
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

uint8_t Mpu6050Imu_Init(I2C_HandleTypeDef *hi2c);
void Mpu6050Imu_Tick(void);
uint8_t Mpu6050Imu_GetAttitude(Mpu6050Attitude *attitude);
void Mpu6050Imu_GetDebug(Mpu6050Debug *debug);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_MPU6050_IMU_H */
