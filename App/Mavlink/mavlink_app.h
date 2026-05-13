#ifndef APP_MAVLINK_APP_H
#define APP_MAVLINK_APP_H

#include "stm32h7xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MavlinkApp_Init(I2C_HandleTypeDef *hi2c);
void MavlinkApp_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAVLINK_APP_H */
