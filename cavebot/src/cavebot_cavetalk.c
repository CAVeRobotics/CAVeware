#include "cavebot_cavetalk.h"

#include <stdbool.h>
#include <stddef.h>

#include "cave_talk.h"
#include "cave_talk_types.h"
#include "cave_talk_link.h"
#include "config_encoder.pb.h"
#include "log.pb.h"
#include "ooga_booga.pb.h"
#include "odometry.pb.h"

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_encoder_user.h"
#include "bsp_logger.h"
#include "bsp_tick.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

#include "cavebot_dust_sensor.h"
#include "cavebot_gas_sensor.h"
#include "rover.h"
#include "rover_4ws.h"
#include "rover_4ws_config.h"
#include "rover_camera.h"
#include "rover_camera_config.h"

#define CAVEBOT_CAVE_TALK_BUFFER_SIZE        1024U
#define CAVEBOT_CAVE_TALK_HEADER_SIZE        3U
#define CAVEBOT_CAVE_TALK_RETRY_PERIOD       (Bsp_Millisecond_t)1000U
#define CAVEBOT_CAVE_TALK_ODOMETRY_PERIOD    (Bsp_Millisecond_t)20U
#define CAVEBOT_CAVE_TALK_AIR_QUALITY_PERIOD (Bsp_Millisecond_t)1000U

typedef enum
{
    CAVEBOT_CAVE_TALK_RECEIVE_HEADER,
    CAVEBOT_CAVE_TALK_RECEIVE_PAYLOAD
} CavebotCaveTalk_Receive_t;

static uint8_t           CavebotCaveTalk_Buffer[CAVEBOT_CAVE_TALK_BUFFER_SIZE];
static const char *      kCavebotCaveTalk_LogTag            = "CAVE TALK";
static bool              CavebotCaveTalk_Connected          = false;
static bool              CavebotCaveTalk_WasArmed           = false;
static Bsp_Millisecond_t CavebotCaveTalk_PreviousMessage    = 0U;
static Bsp_Millisecond_t CavebotCaveTalk_PreviousOdometry   = 0U;
static Bsp_Millisecond_t CavebotCaveTalk_PreviousAirQuality = 0U;

static CaveTalk_Error_t CavebotCaveTalk_Send(const void *const data, const size_t size);
static CaveTalk_Error_t CavebotCaveTalk_Receive(void *const data, const size_t size, size_t *const bytes_received);
static CaveTalk_Error_t CavebotCaveTalk_ConvertBspError(const Bsp_Error_t bsp_error);
static inline void CavebotCaveTalk_HeardMessage(const char *const log);
static void CavebotCaveTalk_HearOogaBooga(const cave_talk_Say ooga_booga);
static void CavebotCaveTalk_HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate);
static void CavebotCaveTalk_HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt);
static void CavebotCaveTalk_HearLights(const bool headlights);
static void CavebotCaveTalk_HearArm(const bool arm);
static void CavebotCaveTalk_HearConfigServoWheels(const cave_talk_Servo *const servo_wheel_0,
                                                  const cave_talk_Servo *const servo_wheel_1,
                                                  const cave_talk_Servo *const servo_wheel_2,
                                                  const cave_talk_Servo *const servo_wheel_3);
static void CavebotCaveTalk_HearConfigServoCams(const cave_talk_Servo *const servo_cam_pan, const cave_talk_Servo *const servo_cam_tilt);
static void CavebotCaveTalk_HearConfigMotors(const cave_talk_Motor *const motor_wheel_0,
                                             const cave_talk_Motor *const motor_wheel_1,
                                             const cave_talk_Motor *const motor_wheel_2,
                                             const cave_talk_Motor *const motor_wheel_3);
static void CavebotCaveTalk_HearConfigEncoders(const cave_talk_ConfigEncoder *const encoder_wheel_0,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_1,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_2,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_3);
static void CavebotCaveTalk_HearConfigLog(const cave_talk_LogLevel log_level);
static void CavebotCaveTalk_HearConfigWheelSpeedControl(const cave_talk_PID *const wheel_0_params,
                                                        const cave_talk_PID *const wheel_1_params,
                                                        const cave_talk_PID *const wheel_2_params,
                                                        const cave_talk_PID *const wheel_3_params,
                                                        const bool enabled);
static void CavebotCaveTalk_HearConfigSteeringControl(const cave_talk_PID *const turn_rate_params, const bool enabled);
static void CavebotCaveTalk_SendOdometry(void);
static void CavebotCaveTalk_SendAirQuality(void);

static CaveTalk_Handle_t CavebotCaveTalk_Handle = {
    .link_handle = {
        .send    = CavebotCaveTalk_Send,
        .receive = CavebotCaveTalk_Receive,
    },
    .buffer           = CavebotCaveTalk_Buffer,
    .buffer_size      = sizeof(CavebotCaveTalk_Buffer),
    .listen_callbacks = {
        .hear_ooga_booga                 = CavebotCaveTalk_HearOogaBooga,
        .hear_movement                   = CavebotCaveTalk_HearMovement,
        .hear_camera_movement            = CavebotCaveTalk_HearCameraMovement,
        .hear_lights                     = CavebotCaveTalk_HearLights,
        .hear_arm                        = CavebotCaveTalk_HearArm,
        .hear_odometry                   = NULL,
        .hear_log                        = NULL,
        .hear_config_servo_wheels        = CavebotCaveTalk_HearConfigServoWheels,
        .hear_config_servo_cams          = CavebotCaveTalk_HearConfigServoCams,
        .hear_config_motors              = CavebotCaveTalk_HearConfigMotors,
        .hear_config_encoders            = CavebotCaveTalk_HearConfigEncoders,
        .hear_config_log                 = CavebotCaveTalk_HearConfigLog,
        .hear_config_wheel_speed_control = CavebotCaveTalk_HearConfigWheelSpeedControl,
        .hear_config_steering_control    = CavebotCaveTalk_HearConfigSteeringControl,
        .hear_air_quality                = NULL,
    },
};

CaveTalk_Error_t CavebotCaveTalk_Start(void)
{
    return CavebotCaveTalk_ConvertBspError(BspUart_Start(BSP_UART_USER_COMMS));
}

void CavebotCaveTalk_Task(void)
{
    CaveTalk_Error_t error = CaveTalk_Hear(&CavebotCaveTalk_Handle);
    if (CAVE_TALK_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Hear error: %d", (int)error);
    }

    Bsp_Millisecond_t tick = BspTick_GetTick();
    if ((tick - CavebotCaveTalk_PreviousMessage) > CAVEBOT_CAVE_TALK_RETRY_PERIOD)
    {
        if (CavebotCaveTalk_Connected)
        {
            CavebotCaveTalk_Connected = false;
            CavebotCaveTalk_WasArmed  = Rover_IsArmed();
            BspGpio_Write(BSP_GPIO_USER_PIN_COMMS_STATUS, BSP_GPIO_STATE_RESET);
            BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Disconnected");

            if (CavebotCaveTalk_WasArmed)
            {
                (void)Rover_Dearm();
            }
        }

        BSP_LOGGER_LOG_DEBUG(kCavebotCaveTalk_LogTag, "Connecting");
        (void)CaveTalk_SpeakOogaBooga(&CavebotCaveTalk_Handle, cave_talk_Say_SAY_OOGA);
        CavebotCaveTalk_PreviousMessage = tick;
    }

    if (CavebotCaveTalk_Connected)
    {
        if ((tick - CavebotCaveTalk_PreviousOdometry) > CAVEBOT_CAVE_TALK_ODOMETRY_PERIOD)
        {
            CavebotCaveTalk_SendOdometry();

            CavebotCaveTalk_PreviousOdometry = tick;
        }

        if ((tick - CavebotCaveTalk_PreviousAirQuality) > CAVEBOT_CAVE_TALK_AIR_QUALITY_PERIOD)
        {
            CavebotCaveTalk_SendAirQuality();

            CavebotCaveTalk_PreviousAirQuality = tick;
        }
    }
}

static CaveTalk_Error_t CavebotCaveTalk_Send(const void *const data, const size_t size)
{
    return CavebotCaveTalk_ConvertBspError(BspUart_Transmit(BSP_UART_USER_COMMS, data, size));
}

static CaveTalk_Error_t CavebotCaveTalk_Receive(void *const data, const size_t size, size_t *const bytes_received)
{
    return CavebotCaveTalk_ConvertBspError(BspUart_Receive(BSP_UART_USER_COMMS, data, size, bytes_received));
}

static CaveTalk_Error_t CavebotCaveTalk_ConvertBspError(const Bsp_Error_t bsp_error)
{
    CaveTalk_Error_t cavetalk_error = CAVE_TALK_ERROR_NONE;

    /* BSP UART functions should not return BSP_ERROR_VALUE */
    if (BSP_ERROR_NULL == bsp_error)
    {
        cavetalk_error = CAVE_TALK_ERROR_NULL;
    }
    else if ((BSP_ERROR_HAL == bsp_error) || (BSP_ERROR_PERIPHERAL == bsp_error))
    {
        cavetalk_error = CAVE_TALK_ERROR_SOCKET_CLOSED;
    }
    else if ((BSP_ERROR_BUSY == bsp_error) || (BSP_ERROR_TIMEOUT == bsp_error))
    {
        cavetalk_error = CAVE_TALK_ERROR_INCOMPLETE;
    }

    return cavetalk_error;
}

static inline void CavebotCaveTalk_HeardMessage(const char *const log)
{
    BSP_LOGGER_LOG_VERBOSE(kCavebotCaveTalk_LogTag, "Heard %s", log);

    if (CavebotCaveTalk_Connected)
    {
        CavebotCaveTalk_PreviousMessage = BspTick_GetTick();
    }
}

static void CavebotCaveTalk_HearOogaBooga(const cave_talk_Say ooga_booga)
{
    CavebotCaveTalk_HeardMessage("ooga booga");

    switch (ooga_booga)
    {
    case cave_talk_Say_SAY_OOGA:
        (void)CaveTalk_SpeakOogaBooga(&CavebotCaveTalk_Handle, cave_talk_Say_SAY_BOOGA);
        if (!CavebotCaveTalk_Connected)
        {
            (void)CaveTalk_SpeakOogaBooga(&CavebotCaveTalk_Handle, cave_talk_Say_SAY_OOGA);
        }
        break;
    case cave_talk_Say_SAY_BOOGA:
        if (!CavebotCaveTalk_Connected)
        {
            CavebotCaveTalk_Connected = true;
            BspGpio_Write(BSP_GPIO_USER_PIN_COMMS_STATUS, BSP_GPIO_STATE_SET);
            BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Connected");

            if (CavebotCaveTalk_WasArmed)
            {
                (void)Rover_Arm();
            }
        }
        break;
    default:
        break;
    }
}

static void CavebotCaveTalk_HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    CavebotCaveTalk_HeardMessage("movement");

    (void)Rover_Drive(speed, turn_rate);
}

static void CavebotCaveTalk_HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    CavebotCaveTalk_HeardMessage("camera movement");

    Rover_Error_t error = RoverCamera_Pan(pan);

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_WARNING(kCavebotCaveTalk_LogTag, "Failed to set camera pan with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_VERBOSE(kCavebotCaveTalk_LogTag, "Set camera pan %lf rad", pan);
    }

    error = RoverCamera_Tilt(tilt);

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_WARNING(kCavebotCaveTalk_LogTag, "Failed to set camera tilt with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_VERBOSE(kCavebotCaveTalk_LogTag, "Set camera tilt %lf rad", tilt);
    }
}

static void CavebotCaveTalk_HearLights(const bool headlights)
{
    CavebotCaveTalk_HeardMessage("lights");

    if (headlights)
    {
        (void)BspGpio_Write(BSP_GPIO_USER_PIN_HEADLIGHTS_0, BSP_GPIO_STATE_SET);
        (void)BspGpio_Write(BSP_GPIO_USER_PIN_HEADLIGHTS_1, BSP_GPIO_STATE_SET);
        (void)BspGpio_Write(BSP_GPIO_USER_PIN_HEADLIGHTS_2, BSP_GPIO_STATE_SET);
    }
    else
    {
        (void)BspGpio_Write(BSP_GPIO_USER_PIN_HEADLIGHTS_0, BSP_GPIO_STATE_RESET);
        (void)BspGpio_Write(BSP_GPIO_USER_PIN_HEADLIGHTS_1, BSP_GPIO_STATE_RESET);
        (void)BspGpio_Write(BSP_GPIO_USER_PIN_HEADLIGHTS_2, BSP_GPIO_STATE_RESET);
    }
}

static void CavebotCaveTalk_HearArm(const bool arm)
{
    CavebotCaveTalk_HeardMessage("arm");

    if (arm)
    {
        (void)Rover_Arm();
    }
    else
    {
        (void)Rover_Dearm();
    }
}

static void CavebotCaveTalk_HearConfigServoWheels(const cave_talk_Servo *const servo_wheel_0,
                                                  const cave_talk_Servo *const servo_wheel_1,
                                                  const cave_talk_Servo *const servo_wheel_2,
                                                  const cave_talk_Servo *const servo_wheel_3)
{
    CavebotCaveTalk_HeardMessage("config servo wheels");

    Rover_Error_t error = ROVER_ERROR_NULL;

    if ((NULL != servo_wheel_0) && (NULL != servo_wheel_1) && (NULL != servo_wheel_2) && (NULL != servo_wheel_3))
    {
        error = Rover4ws_ErrorCheck(Rover4ws_ConfigureSteering(ROVER_4WS_CONFIG_SERVO_0,
                                                               servo_wheel_0->min_duty_cycle_percentage,
                                                               servo_wheel_0->max_duty_cycle_percentage,
                                                               servo_wheel_0->min_angle_radian,
                                                               servo_wheel_0->max_angle_radian),
                                    Rover4ws_ConfigureSteering(ROVER_4WS_CONFIG_SERVO_1,
                                                               servo_wheel_1->min_duty_cycle_percentage,
                                                               servo_wheel_1->max_duty_cycle_percentage,
                                                               servo_wheel_1->min_angle_radian,
                                                               servo_wheel_1->max_angle_radian),
                                    Rover4ws_ConfigureSteering(ROVER_4WS_CONFIG_SERVO_2,
                                                               servo_wheel_2->min_duty_cycle_percentage,
                                                               servo_wheel_2->max_duty_cycle_percentage,
                                                               servo_wheel_2->min_angle_radian,
                                                               servo_wheel_2->max_angle_radian),
                                    Rover4ws_ConfigureSteering(ROVER_4WS_CONFIG_SERVO_3,
                                                               servo_wheel_3->min_duty_cycle_percentage,
                                                               servo_wheel_3->max_duty_cycle_percentage,
                                                               servo_wheel_3->min_angle_radian,
                                                               servo_wheel_3->max_angle_radian));
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to configure wheels servos with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Wheel servos configured");
    }
}

static void CavebotCaveTalk_HearConfigServoCams(const cave_talk_Servo *const servo_cam_pan, const cave_talk_Servo *const servo_cam_tilt)
{
    CavebotCaveTalk_HeardMessage("config servo cam");

    Rover_Error_t error = ROVER_ERROR_NULL;

    if ((NULL != servo_cam_pan) && (NULL != servo_cam_tilt))
    {
        Rover_Error_t pan_config_error = RoverCamera_ConfigureServo(ROVER_CAMERA_CONFIG_SERVO_PAN,
                                                                    servo_cam_pan->min_duty_cycle_percentage,
                                                                    servo_cam_pan->max_duty_cycle_percentage,
                                                                    servo_cam_pan->min_angle_radian,
                                                                    servo_cam_pan->max_angle_radian);
        Rover_Error_t tilt_config_error = RoverCamera_ConfigureServo(ROVER_CAMERA_CONFIG_SERVO_TILT,
                                                                     servo_cam_tilt->min_duty_cycle_percentage,
                                                                     servo_cam_tilt->max_duty_cycle_percentage,
                                                                     servo_cam_tilt->min_angle_radian,
                                                                     servo_cam_tilt->max_angle_radian);

        if (ROVER_ERROR_NONE != pan_config_error)
        {
            error = pan_config_error;
        }
        else
        {
            error = tilt_config_error;
        }
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to configure camera servos with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Camera servos configured");
    }
}

static void CavebotCaveTalk_HearConfigMotors(const cave_talk_Motor *const motor_wheel_0,
                                             const cave_talk_Motor *const motor_wheel_1,
                                             const cave_talk_Motor *const motor_wheel_2,
                                             const cave_talk_Motor *const motor_wheel_3)
{
    CavebotCaveTalk_HeardMessage("config motors");

    Rover_Error_t error = ROVER_ERROR_NULL;

    if ((NULL != motor_wheel_0) && (NULL != motor_wheel_1) && (NULL != motor_wheel_2) && (NULL != motor_wheel_3))
    {
        error = Rover4ws_ErrorCheck(Rover4ws_ConfigureMotor(ROVER_4WS_CONFIG_MOTOR_0,
                                                            motor_wheel_0->min_duty_cycle_percentage,
                                                            motor_wheel_0->max_duty_cycle_percentage,
                                                            motor_wheel_0->min_speed_loaded_meters_per_second,
                                                            motor_wheel_0->max_speed_loaded_meters_per_second),
                                    Rover4ws_ConfigureMotor(ROVER_4WS_CONFIG_MOTOR_1,
                                                            motor_wheel_1->min_duty_cycle_percentage,
                                                            motor_wheel_1->max_duty_cycle_percentage,
                                                            motor_wheel_1->min_speed_loaded_meters_per_second,
                                                            motor_wheel_1->max_speed_loaded_meters_per_second),
                                    Rover4ws_ConfigureMotor(ROVER_4WS_CONFIG_MOTOR_2,
                                                            motor_wheel_2->min_duty_cycle_percentage,
                                                            motor_wheel_2->max_duty_cycle_percentage,
                                                            motor_wheel_2->min_speed_loaded_meters_per_second,
                                                            motor_wheel_2->max_speed_loaded_meters_per_second),
                                    Rover4ws_ConfigureMotor(ROVER_4WS_CONFIG_MOTOR_3,
                                                            motor_wheel_3->min_duty_cycle_percentage,
                                                            motor_wheel_3->max_duty_cycle_percentage,
                                                            motor_wheel_3->min_speed_loaded_meters_per_second,
                                                            motor_wheel_3->max_speed_loaded_meters_per_second));
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to configure motors with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Motors configured");
    }
}

static void CavebotCaveTalk_HearConfigEncoders(const cave_talk_ConfigEncoder *const encoder_wheel_0,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_1,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_2,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_3)
{
    CavebotCaveTalk_HeardMessage("config encoders");

    Rover_Error_t error = ROVER_ERROR_NULL;

    if ((NULL != encoder_wheel_0) && (NULL != encoder_wheel_1) && (NULL != encoder_wheel_2) && (NULL != encoder_wheel_3))
    {
        error = Rover4ws_ErrorCheck(Rover4ws_ConfigureEncoder(ROVER_4WS_CONFIG_MOTOR_0, encoder_wheel_0->smoothing_factor),
                                    Rover4ws_ConfigureEncoder(ROVER_4WS_CONFIG_MOTOR_1, encoder_wheel_1->smoothing_factor),
                                    Rover4ws_ConfigureEncoder(ROVER_4WS_CONFIG_MOTOR_2, encoder_wheel_2->smoothing_factor),
                                    Rover4ws_ConfigureEncoder(ROVER_4WS_CONFIG_MOTOR_3, encoder_wheel_3->smoothing_factor));
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to configure encoders with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Encoders configured");
    }
}

static void CavebotCaveTalk_HearConfigLog(const cave_talk_LogLevel log_level)
{
    CavebotCaveTalk_HeardMessage("config log");

    BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Setting log level to %d", (int)log_level);
    BspLogger_SetLogLevel((BspLogger_Level_t)log_level);
}

static void CavebotCaveTalk_HearConfigWheelSpeedControl(const cave_talk_PID *const wheel_0_params,
                                                        const cave_talk_PID *const wheel_1_params,
                                                        const cave_talk_PID *const wheel_2_params,
                                                        const cave_talk_PID *const wheel_3_params,
                                                        const bool enabled)
{
    CavebotCaveTalk_HeardMessage("config wheel speed control");

    Rover_Error_t error = ROVER_ERROR_NULL;

    if ((NULL != wheel_0_params) && (NULL != wheel_1_params) && (NULL != wheel_2_params) && (NULL != wheel_3_params))
    {
        error = Rover4ws_ErrorCheck(Rover4ws_ConfigureMotorPid(ROVER_4WS_CONFIG_MOTOR_0, wheel_0_params->Kp, wheel_0_params->Ki, wheel_0_params->Kd),
                                    Rover4ws_ConfigureMotorPid(ROVER_4WS_CONFIG_MOTOR_1, wheel_1_params->Kp, wheel_1_params->Ki, wheel_1_params->Kd),
                                    Rover4ws_ConfigureMotorPid(ROVER_4WS_CONFIG_MOTOR_2, wheel_2_params->Kp, wheel_2_params->Ki, wheel_2_params->Kd),
                                    Rover4ws_ConfigureMotorPid(ROVER_4WS_CONFIG_MOTOR_3, wheel_3_params->Kp, wheel_3_params->Ki, wheel_3_params->Kd));
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to configure wheel speed control with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Wheel speed control configured");
    }

    if (enabled)
    {
        error = Rover4ws_EnableSpeedControl();
    }
    else
    {
        error = Rover4ws_DisableSpeedControl();
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to enable wheel speed control with error %d", (int)error);
    }
    else if (enabled)
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Wheel speed control enabled");
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Wheel speed control disabled");
    }
}

static void CavebotCaveTalk_HearConfigSteeringControl(const cave_talk_PID *const turn_rate_params, const bool enabled)
{
    CavebotCaveTalk_HeardMessage("config steering control");

    Rover_Error_t error = ROVER_ERROR_NULL;

    if (NULL != turn_rate_params)
    {
        error = Rover4ws_ConfigureSteeringPid(turn_rate_params->Kp, turn_rate_params->Ki, turn_rate_params->Kd);
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to configure steering control with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Steering control configured");
    }

    if (enabled)
    {
        error = Rover4ws_EnableSpeedControl();
    }
    else
    {
        error = Rover4ws_DisableSpeedControl();
    }

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Failed to enable steering control with error %d", (int)error);
    }
    else if (enabled)
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Steering control enabled");
    }
    else
    {
        BSP_LOGGER_LOG_INFO(kCavebotCaveTalk_LogTag, "Steering control disabled");
    }
}

static void CavebotCaveTalk_SendOdometry(void)
{
    cave_talk_Imu     imu_message       = cave_talk_Imu_init_zero;
    cave_talk_Encoder encoder_message_0 = cave_talk_Encoder_init_zero;
    cave_talk_Encoder encoder_message_1 = cave_talk_Encoder_init_zero;
    cave_talk_Encoder encoder_message_2 = cave_talk_Encoder_init_zero;
    cave_talk_Encoder encoder_message_3 = cave_talk_Encoder_init_zero;

    Rover_AccelerometerReading_t accelerometer_reading = {
        .x = 0.0,
        .y = 0.0,
        .z = 0.0,
    };
    (void)Rover_ReadAccelerometer(&accelerometer_reading);

    imu_message.accel.x_meters_per_second_squared = accelerometer_reading.x;
    imu_message.accel.y_meters_per_second_squared = accelerometer_reading.y;
    imu_message.accel.z_meters_per_second_squared = accelerometer_reading.z;
    imu_message.has_accel                         = true;

    Rover_GyroscopeReading_t gyro_reading = {
        .x = 0.0,
        .y = 0.0,
        .z = 0.0,
    };
    (void)Rover_ReadGyroscope(&gyro_reading);

    imu_message.gyro.roll_radians_per_second  = gyro_reading.x;
    imu_message.gyro.pitch_radians_per_second = gyro_reading.y;
    imu_message.gyro.yaw_radians_per_second   = gyro_reading.z;
    imu_message.has_gyro                      = true;

    Rover_Quaternion_t quaternion = {
        .w = 0.0,
        .x = 0.0,
        .y = 0.0,
        .z = 0.0
    };
    (void)Rover_ReadQuaternion(&quaternion);

    imu_message.quat.w   = quaternion.w;
    imu_message.quat.x   = quaternion.x;
    imu_message.quat.y   = quaternion.y;
    imu_message.quat.z   = quaternion.z;
    imu_message.has_quat = true;

    encoder_message_0.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses;
    encoder_message_0.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].angular_rate;
    encoder_message_1.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].pulses;
    encoder_message_1.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].angular_rate;
    encoder_message_2.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].pulses;
    encoder_message_2.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].angular_rate;
    encoder_message_3.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].pulses;
    encoder_message_3.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].angular_rate;

    CaveTalk_Error_t error = CaveTalk_SpeakOdometry(&CavebotCaveTalk_Handle, &imu_message, &encoder_message_0, &encoder_message_1, &encoder_message_2, &encoder_message_3);
    if (CAVE_TALK_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Speak odometry error: %d", (int)error);
    }
}

static void CavebotCaveTalk_SendAirQuality(void)
{
    /* TODO SD-349 update gas and temperature with correct values, temporarily using temperature to report raw voltage */
    CaveTalk_Error_t error = CaveTalk_SpeakAirQuality(&CavebotCaveTalk_Handle, (uint32_t)CavebotDustSensor_Read(), 0U, CavebotGasSensor_ReadRaw());
    if (CAVE_TALK_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavebotCaveTalk_LogTag, "Speak air quality error: %d", (int)error);
    }
}