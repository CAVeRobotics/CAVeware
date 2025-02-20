#ifndef ROVER_4WS_CONFIG_H
#define ROVER_4WS_CONFIG_H

typedef enum
{
    ROVER_4WS_CONFIG_SERVO_0,
    ROVER_4WS_CONFIG_SERVO_1,
    ROVER_4WS_CONFIG_SERVO_2,
    ROVER_4WS_CONFIG_SERVO_3,
    ROVER_4WS_CONFIG_SERVO_MAX
} Rover4wsConfig_Servo_t;

extern const Rover_Meter_t kRover4wsConfig_Tread;
extern const Rover_Meter_t kRover4wsConfig_Wheelbase;
extern const Rover_Meter_t kRover4wsConfig_WheelRadius;

extern const Rover_Meter_t kRover4wsConfig_HalfTread;
extern const Rover_Meter_t kRover4wsConfig_HalfWheelbase;
extern const Rover_Meter_t kRover4wsConfig_DoubleWheelRadius;

extern const BspServo_Handle_t Rover4wsConfig_Servos[ROVER_4WS_CONFIG_SERVO_MAX];

#endif /* ROVER_4WS_CONFIG_H */