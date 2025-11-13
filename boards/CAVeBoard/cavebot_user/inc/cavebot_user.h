#ifndef CAVEBOT_USER_H
#define CAVEBOT_USER_H

#include "bsp.h"
#include "bsp_encoder_user.h"
#include "bsp_motor.h"
#include "bsp_servo.h"

#include "accelerometer.h"
#include "gyroscope.h"

#include "cavebot_pid.h"

typedef enum
{
    CAVEBOT_USER_SERVO_0,
    CAVEBOT_USER_SERVO_1,
    CAVEBOT_USER_SERVO_2,
    CAVEBOT_USER_SERVO_3,
    CAVEBOT_USER_SERVO_4,
    CAVEBOT_USER_SERVO_5,
    CAVEBOT_USER_SERVO_MAX
} CavebotUser_Servo_t;

typedef enum
{
    CAVEBOT_USER_MOTOR_0,
    CAVEBOT_USER_MOTOR_1,
    CAVEBOT_USER_MOTOR_2,
    CAVEBOT_USER_MOTOR_3,
    CAVEBOT_USER_MOTOR_MAX
} CavebotUser_Motor_t;

typedef enum
{
    CAVEBOT_USER_ENCODER_0,
    CAVEBOT_USER_ENCODER_1,
    CAVEBOT_USER_ENCODER_2,
    CAVEBOT_USER_ENCODER_3,
    CAVEBOT_USER_ENCODER_MAX,
} CavebotUser_Encoder_t;

extern BspServo_Handle_t      CavebotUser_Servos[CAVEBOT_USER_SERVO_MAX];
extern BspMotor_Handle_t      CavebotUser_Motors[CAVEBOT_USER_MOTOR_MAX];
extern BspEncoderUser_Timer_t CavebotUser_Encoders[CAVEBOT_USER_MOTOR_MAX];
extern Accelerometer_Handle_t CavebotUser_Accelerometer;
extern Gyroscope_Handle_t     CavebotUser_Gyroscope;

Cavebot_Error_t CavebotUser_Initialize(void);
Cavebot_Error_t CavebotUser_SensorTask(void);
Cavebot_Error_t CavebotUser_Task(void);

#endif /* CAVEBOT_USER_H */