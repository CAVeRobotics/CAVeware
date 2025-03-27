#include "rover_camera.h"

#include "bsp.h"
#include "bsp_servo.h"

#include "rover.h"
#include "rover_camera_config.h"

Rover_Error_t RoverCamera_Enable(void)
{
    Rover_Error_t error = Rover_BspToRoverError(BspServo_Start(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_PAN]));

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover_BspToRoverError(BspServo_Start(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_TILT]));
    }

    return error;
}

Rover_Error_t RoverCamera_Disable(void)
{
    Rover_Error_t error = Rover_BspToRoverError(BspServo_Stop(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_PAN]));

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover_BspToRoverError(BspServo_Stop(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_TILT]));
    }

    return error;
}

Rover_Error_t RoverCamera_Pan(const Rover_Radian_t pan_angle)
{
    Rover_Error_t error = ROVER_ERROR_MODE;

    if (Rover_IsArmed())
    {
        error = Rover_BspToRoverError(BspServo_SetAngle(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_PAN], pan_angle));
    }

    return error;
}

Rover_Error_t RoverCamera_Tilt(const Rover_Radian_t tilt_angle)
{
    Rover_Error_t error = ROVER_ERROR_MODE;

    if (Rover_IsArmed())
    {
        error = Rover_BspToRoverError(BspServo_SetAngle(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_TILT], tilt_angle));
    }

    return error;
}