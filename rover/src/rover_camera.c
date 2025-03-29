#include "rover_camera.h"

#include "bsp.h"
#include "bsp_servo.h"

#include "rover.h"
#include "rover_camera_config.h"

Rover_Error_t RoverCamera_ConfigureServo(const RoverCameraConfig_Servo_t servo,
                                         const Bsp_Percent_t minimum_duty_cycle,
                                         const Bsp_Percent_t maximum_duty_cycle,
                                         const Bsp_Radian_t minimum_angle,
                                         const Bsp_Radian_t maximum_angle)
{
    Rover_Error_t error = ROVER_ERROR_NONE;

    if (servo >= ROVER_CAMERA_CONFIG_SERVO_MAX)
    {
        error = ROVER_ERROR_PERIPHERAL;
    }
    else if (Rover_IsArmed())
    {
        error = ROVER_ERROR_MODE;
    }
    else
    {
        RoverCameraConfig_Servos[servo].minimum_duty_cycle = minimum_duty_cycle;
        RoverCameraConfig_Servos[servo].maximum_duty_cycle = maximum_duty_cycle;
        RoverCameraConfig_Servos[servo].minimum_angle      = minimum_angle;
        RoverCameraConfig_Servos[servo].maximum_angle      = maximum_angle;
    }

    return error;
}

Rover_Error_t RoverCamera_Enable(void)
{
    Rover_Error_t error = Rover_BspToRoverError(BspServo_Start(&RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_PAN]));

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover_BspToRoverError(BspServo_Start(&RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_TILT]));
    }

    return error;
}

Rover_Error_t RoverCamera_Disable(void)
{
    Rover_Error_t error = Rover_BspToRoverError(BspServo_Stop(&RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_PAN]));

    if (ROVER_ERROR_NONE == error)
    {
        error = Rover_BspToRoverError(BspServo_Stop(&RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_TILT]));
    }

    return error;
}

Rover_Error_t RoverCamera_Pan(const Rover_Radian_t pan_angle)
{
    Rover_Error_t error = ROVER_ERROR_MODE;

    if (Rover_IsArmed())
    {
        error = Rover_BspToRoverError(BspServo_SetAngle(&RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_PAN], pan_angle));
    }

    return error;
}

Rover_Error_t RoverCamera_Tilt(const Rover_Radian_t tilt_angle)
{
    Rover_Error_t error = ROVER_ERROR_MODE;

    if (Rover_IsArmed())
    {
        error = Rover_BspToRoverError(BspServo_SetAngle(&RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_TILT], tilt_angle));
    }

    return error;
}