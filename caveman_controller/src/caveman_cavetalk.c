#include "caveman_cavetalk.h"

#include <stdbool.h>
#include <stddef.h>

#include "cave_talk.h"
#include "cave_talk_types.h"
#include "cave_talk_link.h"
#include "config_encoder.pb.h"
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

#include "rover.h"
#include "rover_4ws.h"
#include "rover_4ws_config.h"
#include "rover_camera.h"

#define CAVEMAN_CAVE_TALK_BUFFER_SIZE 1024U
#define CAVEMAN_CAVE_TALK_HEADER_SIZE 3U

typedef enum
{
    CAVEMAN_CAVE_TALK_RECEIVE_HEADER,
    CAVEMAN_CAVE_TALK_RECEIVE_PAYLOAD
} CavemanCaveTalk_Receive_t;

static uint8_t     CavemanCaveTalk_Buffer[CAVEMAN_CAVE_TALK_BUFFER_SIZE];
static const char *kCavemanCaveTalk_LogTag = "CAVE TALK";

static CaveTalk_Error_t CavemanCaveTalk_Send(const void *const data, const size_t size);
static CaveTalk_Error_t CavemanCaveTalk_Receive(void *const data, const size_t size, size_t *const bytes_received);
static CaveTalk_Error_t CavemanCaveTalk_ConvertBspError(const Bsp_Error_t bsp_error);
static void CavemanCaveTalk_HearOogaBooga(const cave_talk_Say ooga_booga);
static void CavemanCaveTalk_HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate);
static void CavemanCaveTalk_HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt);
static void CavemanCaveTalk_HearLights(const bool headlights);
static void CavemanCaveTalk_HearArm(const bool arm);
static void CavemanCaveTalk_HearConfigEncoders(const cave_talk_ConfigEncoder *const encoder_wheel_0,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_1,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_2,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_3);
static void CavemanCaveTalk_SendOdometry(void);

static CaveTalk_Handle_t CavemanCaveTalk_Handle = {
    .link_handle = {
        .send    = CavemanCaveTalk_Send,
        .receive = CavemanCaveTalk_Receive,
    },
    .buffer           = CavemanCaveTalk_Buffer,
    .buffer_size      = sizeof(CavemanCaveTalk_Buffer),
    .listen_callbacks = {
        .hear_ooga_booga          = CavemanCaveTalk_HearOogaBooga,
        .hear_movement            = CavemanCaveTalk_HearMovement,
        .hear_camera_movement     = CavemanCaveTalk_HearCameraMovement,
        .hear_lights              = CavemanCaveTalk_HearLights,
        .hear_arm                 = CavemanCaveTalk_HearArm,
        .hear_odometry            = NULL,
        .hear_log                 = NULL,
        .hear_config_servo_wheels = NULL,
        .hear_config_servo_cams   = NULL,
        .hear_config_motors       = NULL,
        .hear_config_encoders     = CavemanCaveTalk_HearConfigEncoders,
    },
};

CaveTalk_Error_t CavemanCaveTalk_Start(void)
{
    return CavemanCaveTalk_ConvertBspError(BspUart_Start(BSP_UART_USER_COMMS));
}

void CavemanCaveTalk_Task(void)
{
    CaveTalk_Error_t error = CaveTalk_Hear(&CavemanCaveTalk_Handle);
    if (CAVE_TALK_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavemanCaveTalk_LogTag, "Hear error: %d", (int)error);
    }
}

static CaveTalk_Error_t CavemanCaveTalk_Send(const void *const data, const size_t size)
{
    return CavemanCaveTalk_ConvertBspError(BspUart_Transmit(BSP_UART_USER_COMMS, data, size));
}

static CaveTalk_Error_t CavemanCaveTalk_Receive(void *const data, const size_t size, size_t *const bytes_received)
{
    return CavemanCaveTalk_ConvertBspError(BspUart_Receive(BSP_UART_USER_COMMS, data, size, bytes_received));
}

static CaveTalk_Error_t CavemanCaveTalk_ConvertBspError(const Bsp_Error_t bsp_error)
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

static void CavemanCaveTalk_HearOogaBooga(const cave_talk_Say ooga_booga)
{
    switch (ooga_booga)
    {
    case cave_talk_Say_SAY_OOGA:
        (void)CaveTalk_SpeakOogaBooga(&CavemanCaveTalk_Handle, cave_talk_Say_SAY_BOOGA);
        BSP_LOGGER_LOG_INFO(kCavemanCaveTalk_LogTag, "Comms established");
        break;
    case cave_talk_Say_SAY_BOOGA:
        (void)CaveTalk_SpeakOogaBooga(&CavemanCaveTalk_Handle, cave_talk_Say_SAY_OOGA);
        break;
    default:
        break;
    }
}

static void CavemanCaveTalk_HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    (void)Rover_Drive(speed, turn_rate);
    CavemanCaveTalk_SendOdometry();
    BSP_LOGGER_LOG_VERBOSE(kCavemanCaveTalk_LogTag, "Heard movement message");
}

static void CavemanCaveTalk_HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    Rover_Error_t error = RoverCamera_Pan(pan);

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavemanCaveTalk_LogTag, "Failed to set camera pan with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_VERBOSE(kCavemanCaveTalk_LogTag, "Set camera pan %lf rad", pan);
    }

    error = RoverCamera_Tilt(tilt);

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavemanCaveTalk_LogTag, "Failed to set camera tilt with error %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_VERBOSE(kCavemanCaveTalk_LogTag, "Set camera tilt %lf rad", tilt);
    }
}

static void CavemanCaveTalk_HearLights(const bool headlights)
{
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

static void CavemanCaveTalk_HearArm(const bool arm)
{
    if (arm)
    {
        (void)Rover_Arm();
    }
    else
    {
        (void)Rover_Dearm();
    }
}

static void CavemanCaveTalk_HearConfigEncoders(const cave_talk_ConfigEncoder *const encoder_wheel_0,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_1,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_2,
                                               const cave_talk_ConfigEncoder *const encoder_wheel_3)
{
    if ((NULL != encoder_wheel_0) && (NULL != encoder_wheel_1) && (NULL != encoder_wheel_2) && (NULL != encoder_wheel_3))
    {
        Rover_Error_t error = Rover4ws_ErrorCheck(Rover4ws_ConfigureEncoder(ROVER_4WS_MOTOR_0, encoder_wheel_0->smoothing_factor),
                                                  Rover4ws_ConfigureEncoder(ROVER_4WS_MOTOR_1, encoder_wheel_1->smoothing_factor),
                                                  Rover4ws_ConfigureEncoder(ROVER_4WS_MOTOR_2, encoder_wheel_2->smoothing_factor),
                                                  Rover4ws_ConfigureEncoder(ROVER_4WS_MOTOR_3, encoder_wheel_3->smoothing_factor));

        if (ROVER_ERROR_NONE != error)
        {
            BSP_LOGGER_LOG_ERROR(kCavemanCaveTalk_LogTag, "Failed to configure encoders with error %d", (int)error);
        }
        else
        {
            BSP_LOGGER_LOG_INFO(kCavemanCaveTalk_LogTag, "Encoders configured");
        }
    }
}

static void CavemanCaveTalk_SendOdometry(void)
{
    cave_talk_Imu     imu_message       = cave_talk_Imu_init_zero;
    cave_talk_Encoder encoder_message_0 = cave_talk_Encoder_init_zero;
    cave_talk_Encoder encoder_message_1 = cave_talk_Encoder_init_zero;
    cave_talk_Encoder encoder_message_2 = cave_talk_Encoder_init_zero;
    cave_talk_Encoder encoder_message_3 = cave_talk_Encoder_init_zero;

    encoder_message_0.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].pulses;
    encoder_message_0.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_0].angular_rate;
    encoder_message_1.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].pulses;
    encoder_message_1.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_1].angular_rate;
    encoder_message_2.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].pulses;
    encoder_message_2.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_2].angular_rate;
    encoder_message_3.total_pulses            = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].pulses;
    encoder_message_3.rate_radians_per_second = BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_3].angular_rate;

    CaveTalk_Error_t error = CaveTalk_SpeakOdometry(&CavemanCaveTalk_Handle, &imu_message, &encoder_message_0, &encoder_message_1, &encoder_message_2, &encoder_message_3);
    if (CAVE_TALK_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavemanCaveTalk_LogTag, "Speak odometry error: %d", (int)error);
    }
    else
    {
        BSP_LOGGER_LOG_VERBOSE(kCavemanCaveTalk_LogTag, "Spoke odometry");
    }
}