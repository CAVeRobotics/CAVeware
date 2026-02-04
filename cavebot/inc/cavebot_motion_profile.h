#ifndef CAVEBOT_MOTION_PROFILE_H
#define CAVEBOT_MOTION_PROFILE_H

#include "bsp.h"

#include "cavebot.h"

Cavebot_Trajectory_t CavebotMotionProfile_Proportional(const Bsp_Meter_t distance, const Bsp_Radian_t angle);
Cavebot_Trajectory_t CavebotMotionProfile_Trapezoidal(const Bsp_Meter_t distance, const Bsp_Radian_t angle, const Bsp_Second_t delta_time);
Cavebot_Trajectory_t CavebotMotionProfile_SCurve(const Bsp_Meter_t distance, const Bsp_Radian_t angle, const Bsp_Second_t delta_time);

#endif /* CAVEBOT_MOTION_PROFILE_H */