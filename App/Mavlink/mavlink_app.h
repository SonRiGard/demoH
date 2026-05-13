#ifndef APP_MAVLINK_APP_H
#define APP_MAVLINK_APP_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the MAVLink-over-USB task and the optional MPU IMU source. */
void MavlinkApp_Init(I2C_HandleTypeDef *hi2c);

/* Main-loop service function. It schedules HEARTBEAT, ATTITUDE and debug MAVLink
   messages without blocking if the USB CDC endpoint is busy. */
void MavlinkApp_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAVLINK_APP_H */
