#ifndef CAVEBOT_H
#define CAVEBOT_H

#include <stdbool.h>

#include "bsp.h"

typedef enum
{
    CAVEBOT_ERROR_NONE,
    CAVEBOT_ERROR_NULL,
    CAVEBOT_ERROR_BSP,
    CAVEBOT_ERROR_PERIPHERAL,
    CAVEBOT_ERROR_MODE,
    CAVEBOT_ERROR_VALUE,
    CAVEBOT_ERROR_BOT,
    CAVEBOT_ERROR_MOVE
} Cavebot_Error_t;

typedef enum
{
    CAVEBOT_STATE_INITIALIZE,
    CAVEBOT_STATE_READY,
    CAVEBOT_STATE_MANUAL,
    CAVEBOT_STATE_AUTO,
    CAVEBOT_STATE_FAILED,
    CAVEBOT_STATE_MAX
} Cavebot_State_t;

typedef enum
{
    CAVEBOT_BOT_4WS,
    CAVEBOT_BOT_4WD
} Cavebot_Bot_t;

typedef struct
{
    Bsp_Meter_t x;
    Bsp_Meter_t y;
    Bsp_Radian_t heading;
} Cavebot_Pose_t;

typedef struct
{
    Bsp_MetersPerSecond_t linear_velocity;
    Bsp_RadiansPerSecond_t angular_velocity;
} Cavebot_Trajectory_t;

Cavebot_Error_t Cavebot_BspToCavebotError(const Bsp_Error_t bsp_error);
Cavebot_State_t Cavebot_GetState(void);
bool Cavebot_SetState(const Cavebot_State_t state);
Cavebot_Error_t Cavebot_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate);
Cavebot_Pose_t Cavebot_GetPose(void);
Cavebot_Error_t Cavebot_SetPose(const Cavebot_Pose_t *const pose);
Bsp_MetersPerSecond_t Cavebot_GetLinearVelocity(void);
Cavebot_Error_t Cavebot_SetWaypoint(const Cavebot_Pose_t *const waypoint);

#endif /* CAVEBOT_H */