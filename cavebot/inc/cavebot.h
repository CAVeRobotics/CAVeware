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
    CAVEBOT_BOT_4WS,
    CAVEBOT_BOT_4WD
} Cavebot_Bot_t;

typedef enum
{
    CAVEBOT_MODE_DISARMED,
    CAVEBOT_MODE_ARMED_MANUAL,
    CAVEBOT_MODE_ARMED_AUTO
} Cavebot_Mode_t;

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
Cavebot_Error_t Cavebot_Arm(void);
Cavebot_Error_t Cavebot_Disarm(void);
bool Cavebot_IsArmed(void);
Cavebot_Error_t Cavebot_SetAuto(const bool set_auto);
Cavebot_Error_t Cavebot_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate);
Cavebot_Pose_t Cavebot_GetPose(void);
Cavebot_Error_t Cavebot_SetPose(const Cavebot_Pose_t *const pose);
Bsp_MetersPerSecond_t Cavebot_GetLinearVelocity(void);
Cavebot_Error_t Cavebot_SetWaypoint(const Cavebot_Pose_t *const waypoint);
void Cavebot_Task(void);

Cavebot_Error_t Cavebot_RelativeMove(const Bsp_Meter_t position, const Bsp_Radian_t pose);
bool Cavebot_IsRelativeMoving(void);

#endif /* CAVEBOT_H */