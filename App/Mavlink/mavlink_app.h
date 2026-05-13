#ifndef APP_MAVLINK_APP_H
#define APP_MAVLINK_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MavlinkApp_Init(void);
void MavlinkApp_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAVLINK_APP_H */
