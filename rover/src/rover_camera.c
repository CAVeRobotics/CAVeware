#include "rover_camera.h"

#include "bsp.h"
#include "bsp_servo.h"

#include "rover.h"
#include "rover_camera_config.h"

Rover_Error_t RoverCamera_Pan(const Rover_Radian_t pan_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (BSP_ERROR_NONE != BspServo_SetAngle(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_PAN], pan_angle))
    {
        error = ROVER_ERROR_BSP;
    }

    return error;
}

Rover_Error_t RoverCamera_Tilt(const Rover_Radian_t tilt_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (BSP_ERROR_NONE != BspServo_SetAngle(&RoverCamera_Servos[ROVER_CAMERA_CONFIG_SERVO_TILT], tilt_angle))
    {
        error = ROVER_ERROR_BSP;
    }

    return error;
}