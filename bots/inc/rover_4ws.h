#ifndef ROVER_4WS_H
#define ROVER_4WS_H

#include "bsp.h"

#include "cavebot.h"
#include "cavebot_user.h"

Cavebot_Error_t Rover4ws_ConfigureSteering(const CavebotUser_Servo_t servo,
                                           const Bsp_Percent_t minimum_duty_cycle,
                                           const Bsp_Percent_t maximum_duty_cycle,
                                           const Bsp_Radian_t minimum_angle,
                                           const Bsp_Radian_t maximum_angle);
Cavebot_Error_t Rover4ws_ConfigureMotor(const CavebotUser_Motor_t motor,
                                        const Bsp_Percent_t minimum_duty_cycle,
                                        const Bsp_Percent_t maximum_duty_cycle,
                                        const Bsp_RadiansPerSecond_t minimum_speed,
                                        const Bsp_RadiansPerSecond_t maximum_speed);
Cavebot_Error_t Rover4ws_ConfigureMotorPid(const CavebotUser_Motor_t motor, const double kp, const double ki, const double kd);
Cavebot_Error_t Rover4ws_ConfigureSteeringPid(const double kp, const double ki, const double kd);
Cavebot_Error_t Rover4ws_ConfigureEncoder(const CavebotUser_Motor_t motor, const double smoothing_factor);
Cavebot_Error_t Rover4ws_EnableSteering(void);
Cavebot_Error_t Rover4ws_DisableSteering(void);
Cavebot_Error_t Rover4ws_StartMotors(void);
Cavebot_Error_t Rover4ws_StopMotors(void);
Cavebot_Error_t Rover4ws_EnableEncoders(void);
Cavebot_Error_t Rover4ws_SampleEncoders(void);
Cavebot_Error_t Rover4ws_EnableSpeedControl(void);
Cavebot_Error_t Rover4ws_DisableSpeedControl(void);
Cavebot_Error_t Rover4ws_EnableSteeringControl(void);
Cavebot_Error_t Rover4ws_DisableSteeringControl(void);
Cavebot_Error_t Rover4ws_Task(void);
Cavebot_Error_t Rover4ws_Drive(const Bsp_MetersPerSecond_t speed, const Bsp_RadiansPerSecond_t turn_rate);
Cavebot_Error_t Rover4ws_ErrorCheck(const Cavebot_Error_t error_0,
                                    const Cavebot_Error_t error_1,
                                    const Cavebot_Error_t error_2,
                                    const Cavebot_Error_t error_3);

#endif /* ROVER_4WS_H */