#ifndef ROVER_4WD_H
#define ROVER_4WD_H

#include "cavebot.h"

Cavebot_Error_t Rover4wd_Arm(void);
Cavebot_Error_t Rover4wd_Disarm(void);
Cavebot_Error_t Rover4wd_Task(void);
Cavebot_Error_t Rover4wd_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate);

#endif /* ROVER_4WD_H */