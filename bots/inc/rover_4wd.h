#ifndef ROVER_4WD_H
#define ROVER_4WD_H

#include <stdbool.h>

#include "cavebot.h"

bool Rover4wd_Arm(void);
bool Rover4wd_Disarm(void);
void Rover4wd_EnableSpeedControl(void);
void Rover4wd_DisableSpeedControl(void);
void Rover4wd_Run(void);
void Rover4wd_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate);
Cavebot_Pose_t Rover4wd_GetPose(void);
void Rover4wd_SetPose(const Cavebot_Pose_t *const pose);
Bsp_MetersPerSecond_t Rover4wd_GetLinearVelocity(void);
Bsp_RadiansPerSecond_t Rover4wd_GetAngularVelocity(void);

#endif /* ROVER_4WD_H */