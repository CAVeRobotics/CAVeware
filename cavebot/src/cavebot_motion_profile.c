#include "cavebot_motion_profile.h"

#include <math.h>

#include "bsp.h"

#include "cavebot.h"

/* TODO CVW-21 read from config */
static const double                       kCavebotMotionProfile_LinearVelocityGain     = 0.5;
static const double                       kCavebotMotionProfile_AngularVelocityGain    = 2.0;
static const Bsp_Meter_t                  kCavebotMotionProfile_MinimumTurnRadius      = 0.3;
static const Bsp_MetersPerSecond_t        kCavebotMotionProfile_MinimumLinearVelocity  = 0.03;
static const Bsp_MetersPerSecond_t        kCavebotMotionProfile_MaximumLinearVelocity  = 0.64;
static const Bsp_RadiansPerSecond_t       kCavebotMotionProfile_MaximumAngularVelocity = 1.5;    /* TODO most likely needs to be less */
static const Bsp_MetersPerSecondSquared_t kCavebotMotionProfile_LinearAcceleration     = 0.32;
static const Bsp_Meter_t                  kCavebotMotionProfile_DecelerationMargin     = 0.01;

Cavebot_Trajectory_t CavebotMotionProfile_Proportional(const Bsp_Meter_t distance, const Bsp_Radian_t angle)
{
    const Bsp_MetersPerSecond_t  linear_velocity          = Bsp_Clip((kCavebotMotionProfile_LinearVelocityGain * distance), kCavebotMotionProfile_MinimumLinearVelocity, kCavebotMotionProfile_MaximumLinearVelocity);
    const Bsp_RadiansPerSecond_t maximum_angular_velocity = fmin(kCavebotMotionProfile_MaximumAngularVelocity, (linear_velocity / kCavebotMotionProfile_MinimumTurnRadius));
    const Bsp_RadiansPerSecond_t angular_velocity         = Bsp_Clip((kCavebotMotionProfile_AngularVelocityGain * angle), -maximum_angular_velocity, maximum_angular_velocity);

    return (Cavebot_Trajectory_t){
               .linear_velocity  = linear_velocity,
               .angular_velocity = angular_velocity
    };
}

Cavebot_Trajectory_t CavebotMotionProfile_Trapezoidal(const Bsp_Meter_t distance, const Bsp_Radian_t angle, const Bsp_Second_t delta_time)
{
    const Bsp_Meter_t    deceleration_distance = ((kCavebotMotionProfile_MaximumLinearVelocity * kCavebotMotionProfile_MaximumLinearVelocity) / (2.0 * kCavebotMotionProfile_LinearAcceleration)) + kCavebotMotionProfile_DecelerationMargin;
    Cavebot_Trajectory_t trajectory            = {
        .linear_velocity  = 0.0,
        .angular_velocity = 0.0,
    };

    if (distance > deceleration_distance)
    {
        trajectory.linear_velocity = Bsp_Clip((Cavebot_GetLinearVelocity() + (kCavebotMotionProfile_LinearAcceleration * delta_time)), kCavebotMotionProfile_MinimumLinearVelocity, kCavebotMotionProfile_MaximumLinearVelocity);
    }
    else
    {
        trajectory.linear_velocity = sqrt(2.0 * kCavebotMotionProfile_LinearAcceleration * fmax(0, distance));
    }

    /* TODO apply linear acceleration limits (actual speed is not guaranteed to match previous desired speed, resulting in possible delta speed greater than max acceleration) */

    const Bsp_RadiansPerSecond_t maximum_angular_velocity = fmin(kCavebotMotionProfile_MaximumAngularVelocity, (trajectory.linear_velocity / kCavebotMotionProfile_MinimumTurnRadius));
    trajectory.angular_velocity = Bsp_Clip((kCavebotMotionProfile_AngularVelocityGain * angle), -maximum_angular_velocity, maximum_angular_velocity);

    /* TODO apply angular acceleration limits */

    return trajectory;
}

Cavebot_Trajectory_t CavebotMotionProfile_SCurve(const Bsp_Meter_t distance, const Bsp_Radian_t angle, const Bsp_Second_t delta_time)
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