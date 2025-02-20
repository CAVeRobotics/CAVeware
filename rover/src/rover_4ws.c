#include "rover_4ws.h"

#include "math.h"

#include "bsp.h"
#include "bsp_servo.h"

#include "rover.h"
#include "rover_4ws_config.h"

Rover_Error_t Rover4ws_SetSpeed(const Rover_MetersPerSecond_t speed, const Rover_Radian_t steering_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    double radius              = kRover4wsConfig_HalfWheelbase / tan(steering_angle);
    double left_angular_speed  = (speed * (2 - (kRover4wsConfig_Tread / radius))) / kRover4wsConfig_DoubleWheelRadius;
    double right_angular_speed = (speed * (2 + (kRover4wsConfig_Tread / radius))) / kRover4wsConfig_DoubleWheelRadius;

    /* TODO convert angular speed to PWM */
    BSP_UNUSED(left_angular_speed);
    BSP_UNUSED(right_angular_speed);

    return error;
}

Rover_Error_t Rover4ws_SetSteeringAngle(const Rover_Radian_t steering_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    double tan_delta        = tan(steering_angle);
    double scaled_wheelbase = kRover4wsConfig_HalfWheelbase * tan_delta;
    double offset           = kRover4wsConfig_HalfTread * tan_delta;

    double delta_left  = atan(scaled_wheelbase / (kRover4wsConfig_HalfWheelbase - offset));
    double delta_right = atan(scaled_wheelbase / (kRover4wsConfig_HalfWheelbase + offset));

    if ((BSP_ERROR_NONE != BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_0], delta_left)) ||
        (BSP_ERROR_NONE != BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_1], delta_right)) ||
        (BSP_ERROR_NONE != BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_2], -delta_left)) ||
        (BSP_ERROR_NONE != BspServo_SetAngle(&Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_3], -delta_right)))
    {
        error = ROVER_ERROR_BSP;
    }

    return error;
}