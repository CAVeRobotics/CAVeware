#ifndef CAVEBOT_MOTION_PROFILE_H
#define CAVEBOT_MOTION_PROFILE_H

#include "bsp.h"

Bsp_MetersPerSecond_t CavebotMotionProfile_TrapezoidVelocity(const Bsp_Meter_t current,
                                                             const Bsp_Meter_t goal,
                                                             const Bsp_MetersPerSecond_t inital_velocity,
                                                             const Bsp_MetersPerSecond_t max_velocity,
                                                             const Bsp_MetersPerSecondSquared_t acceleration);

#endif /* CAVEBOT_MOTION_PROFILE_H */