#ifndef ROVER_CAMERA_H
#define ROVER_CAMERA_H

#include "rover.h"
#include "rover_camera_config.h"

Rover_Error_t RoverCamera_ConfigureServo(const RoverCameraConfig_Servo_t servo,
                                         const Bsp_Percent_t minimum_duty_cycle,
                                         const Bsp_Percent_t maximum_duty_cycle,
                                         const Bsp_Radian_t minimum_angle,
                                         const Bsp_Radian_t maximum_angle);
Rover_Error_t RoverCamera_Enable(void);
Rover_Error_t RoverCamera_Disable(void);
Rover_Error_t RoverCamera_Pan(const Rover_Radian_t pan_angle);
Rover_Error_t RoverCamera_Tilt(const Rover_Radian_t tilt_angle);

#endif /* ROVER_CAMERA_H */