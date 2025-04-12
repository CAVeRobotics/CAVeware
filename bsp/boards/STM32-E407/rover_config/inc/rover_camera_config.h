#ifndef ROVER_CAMERA_CONFIG_H
#define ROVER_CAMERA_CONFIG_H

#include "bsp_servo.h"

#include "rover.h"

typedef enum
{
    ROVER_CAMERA_CONFIG_SERVO_PAN,
    ROVER_CAMERA_CONFIG_SERVO_TILT,
    ROVER_CAMERA_CONFIG_SERVO_MAX
} RoverCameraConfig_Servo_t;

extern BspServo_Handle_t RoverCameraConfig_Servos[ROVER_CAMERA_CONFIG_SERVO_MAX];

#endif /* ROVER_CAMERA_CONFIG_H */