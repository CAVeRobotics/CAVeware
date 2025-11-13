#include "cavebot_motion_profile.h"

#include <math.h>

#include "bsp.h"

/* TODO CVW-72 only works in the positive direction, current and goal must be positive relative values or potentially errors */
Bsp_MetersPerSecond_t CavebotMotionProfile_TrapezoidVelocity(const Bsp_Meter_t current,
                                                             const Bsp_Meter_t goal,
                                                             const Bsp_MetersPerSecond_t inital_velocity,
                                                             const Bsp_MetersPerSecond_t max_velocity,
                                                             const Bsp_MetersPerSecondSquared_t acceleration)
{
    Bsp_Meter_t           acceleration_end = (max_velocity * max_velocity) / (2.0 * acceleration);
    Bsp_Meter_t           half_goal        = goal / 2.0;
    Bsp_MetersPerSecond_t velocity         = 0.0;
    Bsp_Meter_t           deceleration_start;

    if (acceleration_end > half_goal)
    {
        acceleration_end = half_goal;
    }
    deceleration_start = goal - acceleration_end;

    if (current >= goal)
    {
        /* Do nothing, velocity 0 */
    }
    else if (current <= 0)
    {
        velocity = inital_velocity;
    }
    else if (current < acceleration_end)
    {
        velocity = sqrt(2.0 * acceleration * current);
    }
    else if (current >= deceleration_start)
    {
        velocity = sqrt(2.0 * acceleration * (goal - current));
    }
    else
    {
        velocity = max_velocity;
    }

    return velocity;
}