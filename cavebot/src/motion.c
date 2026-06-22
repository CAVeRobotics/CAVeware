#include "motion.h"

#include <math.h>

#include "bsp.h"

#include "cavebot.h"

/* TODO CVW-21 read from config */
static const double                       kMotion_LinearVelocityGain     = 0.5;
static const double                       kMotion_AngularVelocityGain    = 2.0;
static const Bsp_Meter_t                  kMotion_MinimumTurnRadius      = 0.3;
static const Bsp_MetersPerSecond_t        kMotion_MinimumLinearVelocity  = 0.03;
static const Bsp_MetersPerSecond_t        kMotion_MaximumLinearVelocity  = 0.64;
static const Bsp_RadiansPerSecond_t       kMotion_MaximumAngularVelocity = 1.5; /* TODO most likely needs to be less */
static const Bsp_MetersPerSecondSquared_t kMotion_LinearAcceleration     = 0.20;
static const Bsp_Meter_t                  kMotion_DecelerationMargin     = 0.01;

Cavebot_Trajectory_t Motion_Proportional(const Bsp_Meter_t distance, const Bsp_Radian_t angle)
{
    const Bsp_MetersPerSecond_t  linear_velocity          = Bsp_Clip((kMotion_LinearVelocityGain * distance), kMotion_MinimumLinearVelocity, kMotion_MaximumLinearVelocity);
    const Bsp_RadiansPerSecond_t maximum_angular_velocity = fmin(kMotion_MaximumAngularVelocity, (linear_velocity / kMotion_MinimumTurnRadius));
    const Bsp_RadiansPerSecond_t angular_velocity         = Bsp_Clip((kMotion_AngularVelocityGain * angle), -maximum_angular_velocity, maximum_angular_velocity);

    return (Cavebot_Trajectory_t){
               .linear_velocity  = linear_velocity,
               .angular_velocity = angular_velocity
    };
}

Cavebot_Trajectory_t Motion_Trapezoidal(const Bsp_Meter_t start, const Bsp_Meter_t distance, const Bsp_Radian_t angle)
{
    Bsp_Meter_t          deceleration_distance = ((kMotion_MaximumLinearVelocity * kMotion_MaximumLinearVelocity) / (2.0 * kMotion_LinearAcceleration)) + kMotion_DecelerationMargin;
    Cavebot_Trajectory_t trajectory            = {
        .linear_velocity  = 0.0,
        .angular_velocity = 0.0,
    };

    Bsp_Meter_t goal      = start;
    Bsp_Meter_t half_goal = start / 2.0;
    Bsp_Meter_t current   = start - distance;
    Bsp_Meter_t deceleration_start;

    if (deceleration_distance > half_goal)
    {
        deceleration_distance = half_goal;
    }
    deceleration_start = goal - deceleration_distance;

    if (current >= goal)
    {
        /* Do nothing, velocity 0 */
    }
    else if (current <= 0.0)
    {
        trajectory.linear_velocity = kMotion_MinimumLinearVelocity;
    }
    else if (current < deceleration_distance)
    {
        trajectory.linear_velocity = sqrt(2.0 * kMotion_LinearAcceleration * current);
    }
    else if (current >= deceleration_start)
    {
        trajectory.linear_velocity = sqrt(2.0 * kMotion_LinearAcceleration * distance);
    }
    else
    {
        trajectory.linear_velocity = kMotion_MaximumLinearVelocity;
    }

    /* TODO apply linear acceleration limits (actual speed is not guaranteed to match previous desired speed, resulting in possible delta speed greater than max acceleration) */

    const Bsp_RadiansPerSecond_t maximum_angular_velocity = fmin(kMotion_MaximumAngularVelocity, (trajectory.linear_velocity / kMotion_MinimumTurnRadius));
    trajectory.angular_velocity = Bsp_Clip((kMotion_AngularVelocityGain * angle), -maximum_angular_velocity, maximum_angular_velocity);

    /* TODO apply angular acceleration limits */

    return trajectory;
}

Cavebot_Trajectory_t Motion_SCurve(const Bsp_Meter_t distance, const Bsp_Radian_t angle, const Bsp_Second_t delta_time)
{
    BSP_UNUSED(distance);
    BSP_UNUSED(angle);
    BSP_UNUSED(delta_time);

    /* TODO */
    return (Cavebot_Trajectory_t){
               .linear_velocity  = 0.0,
               .angular_velocity = 0.0
    };
}

Bsp_MetersPerSecond_t Motion_TrapezoidVelocity(const Bsp_Meter_t current,
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