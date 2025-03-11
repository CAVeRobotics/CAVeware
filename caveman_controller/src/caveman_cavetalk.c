#include "caveman_cavetalk.h"

#include <stddef.h>

#include "cave_talk.h"
#include "cave_talk_types.h"
#include "cave_talk_link.h"
#include "ooga_booga.pb.h"

#include "bsp.h"
#include "bsp_logger.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

#include "rover_4ws.h"

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

static CaveTalk_Handle_t CavemanCaveTalk_Handle = {
    .link_handle = {
        .send    = CavemanCaveTalk_Send,
        .receive = CavemanCaveTalk_Receive,
    },
    .buffer           = CavemanCaveTalk_Buffer,
    .buffer_size      = sizeof(CavemanCaveTalk_Buffer),
    .listen_callbacks = {
        .hear_ooga_booga      = CavemanCaveTalk_HearOogaBooga,
        .hear_movement        = CavemanCaveTalk_HearMovement,
        .hear_camera_movement = NULL,
        .hear_lights          = NULL,
        .hear_mode            = NULL,
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

    /* TODO send odometry data */
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
    BSP_LOGGER_LOG_INFO(kCavemanCaveTalk_LogTag, "Comms success");

    switch (ooga_booga)
    {
    case cave_talk_Say_SAY_OOGA:
        (void)CaveTalk_SpeakOogaBooga(&CavemanCaveTalk_Handle, cave_talk_Say_SAY_BOOGA);
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
    Rover_Error_t error = Rover4ws_Drive(speed, turn_rate);

    if (ROVER_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kCavemanCaveTalk_LogTag, "Failed to set speed and turn rate with error %d", (int)error);
    }
}