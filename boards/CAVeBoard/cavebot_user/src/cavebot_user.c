#include "cavebot_user.h"

#include "bsp_encoder_user.h"
#include "bsp_motor.h"
#include "bsp_servo.h"

/* TODO */
BspServo_Handle_t CavebotUser_Servos[CAVEBOT_USER_SERVO_MAX] = {
    0
};

/* TODO */
BspMotor_Handle_t CavebotUser_Motors[CAVEBOT_USER_MOTOR_MAX] = {
    0
};

/* TODO */
CavebotPid_Handle_t CavebotUser_MotorsPid[CAVEBOT_USER_MOTOR_MAX] = {
    0
};

/* TODO */
BspEncoderUser_Timer_t CavebotUser_Encoders[CAVEBOT_USER_MOTOR_MAX] = {
    0
};