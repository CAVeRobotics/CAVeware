#include "cavebot_user.h"

#include "spi.h"

#include "bsp_encoder_user.h"
#include "bsp_motor.h"
#include "bsp_servo.h"

#include "lsm6dsv16x.h"

const Lsm6dsv16x_Context_t CavebotUser_Lsm6dsv16x = LSM6DSV16X_INSTANTIATE(&hspi2);

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

/* TODO */
Accelerometer_Handle_t CavebotUser_Accelerometer;
Gyroscope_Handle_t     CavebotUser_Gyroscope;