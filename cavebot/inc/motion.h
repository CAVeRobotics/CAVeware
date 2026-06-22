#ifndef MOTION_H
#define MOTION_H

#include "bsp.h"

#include "cavebot.h"

Cavebot_Trajectory_t Motion_Proportional(const Bsp_Meter_t distance, const Bsp_Radian_t angle);
Cavebot_Trajectory_t Motion_Trapezoidal(const Bsp_Meter_t start, const Bsp_Meter_t distance, const Bsp_Radian_t angle, const Bsp_Second_t delta_time);
Cavebot_Trajectory_t Motion_SCurve(const Bsp_Meter_t distance, const Bsp_Radian_t angle, const Bsp_Second_t delta_time);

Bsp_MetersPerSecond_t Motion_TrapezoidVelocity(const Bsp_Meter_t current,
                                               const Bsp_Meter_t goal,
                                               const Bsp_MetersPerSecond_t inital_velocity,
                                               const Bsp_MetersPerSecond_t max_velocity,
                                               const Bsp_MetersPerSecondSquared_t acceleration);

#endif /* MOTION_H */