#ifndef ROVER_4WS_H
#define ROVER_4WS_H

#include "bsp.h"

#include "rover.h"
#include "rover_4ws_config.h"

Rover_Error_t Rover4ws_ConfigureSteering(const Rover4ws_Servo_t servo,
                                         const Bsp_Percent_t minimum_duty_cycle,
                                         const Bsp_Percent_t maximum_duty_cycle,
                                         const Bsp_Radian_t minimum_angle,
                                         const Bsp_Radian_t maximum_angle);
Rover_Error_t Rover4ws_ConfigureMotor(const Rover4ws_Motor_t motor,
                                      const Bsp_Percent_t minimum_duty_cycle,
                                      const Bsp_Percent_t maximum_duty_cycle,
                                      const Bsp_RadiansPerSecond_t minimum_speed,
                                      const Bsp_RadiansPerSecond_t maximum_speed);
Rover_Error_t Rover4ws_EnableSteering(void);
Rover_Error_t Rover4ws_DisableSteering(void);
Rover_Error_t Rover4ws_StartMotors(void);
Rover_Error_t Rover4ws_StopMotors(void);
Rover_Error_t Rover4ws_EnableEncoders(void);
Rover_Error_t Rover4ws_SampleEncoders(void);
Rover_Error_t Rover4ws_Drive(const Rover_MetersPerSecond_t speed, const Rover_RadiansPerSecond_t turn_rate);

#endif /* ROVER_4WS_H */