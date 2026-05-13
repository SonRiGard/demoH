#ifndef APP_MAVLINK_HEARTBEAT_H
#define APP_MAVLINK_HEARTBEAT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MavlinkHeartbeat_Init(void);
void MavlinkHeartbeat_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAVLINK_HEARTBEAT_H */
