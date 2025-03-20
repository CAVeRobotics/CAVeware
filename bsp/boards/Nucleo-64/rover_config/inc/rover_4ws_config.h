#ifndef ROVER_4WS_CONFIG_H
#define ROVER_4WS_CONFIG_H

#include "bsp_servo.h"
#include "bsp_motor.h"

#include "rover.h"

typedef enum
{
    ROVER_4WS_SERVO_0,
    ROVER_4WS_SERVO_1,
    ROVER_4WS_SERVO_2,
    ROVER_4WS_SERVO_3,
    ROVER_4WS_SERVO_MAX
} Rover4ws_Servo_t;

typedef enum
{
    ROVER_4WS_MOTOR_0,
    ROVER_4WS_MOTOR_1,
    ROVER_4WS_MOTOR_2,
    ROVER_4WS_MOTOR_3,
    ROVER_4WS_MOTOR_MAX
} Rover4ws_Motor_t;

extern const Rover_Meter_t kRover4wsConfig_Tread;
extern const Rover_Meter_t kRover4wsConfig_Wheelbase;
extern const Rover_Meter_t kRover4wsConfig_WheelRadius;

extern const Rover_Meter_t kRover4wsConfig_HalfTread;
extern const Rover_Meter_t kRover4wsConfig_HalfWheelbase;
extern const Rover_Meter_t kRover4wsConfig_DoubleWheelRadius;

extern BspServo_Handle_t Rover4wsConfig_Servos[ROVER_4WS_SERVO_MAX];
extern BspMotor_Handle_t Rover4wsConfig_Motors[ROVER_4WS_MOTOR_MAX];

#endif /* ROVER_4WS_CONFIG_H */