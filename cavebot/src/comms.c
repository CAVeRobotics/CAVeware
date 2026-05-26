#include "comms.h"

#include <stdbool.h>
#include <stddef.h>

#include "aether.h"
#include "cavetalk.h"

#include "bsp_logger.h"
#include "bsp_tick.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

#include "cavebot.h"
#include "fault_handler.h"

#define COMMS_CAVETALK_ID 0x00000001U
#define COMMS_UART        BSP_UART_USER_1

static const char *const kComms_LogTag = "COMMS";
static a_Socket_t        Comms_Socket;
static uint8_t           Comms_SendBuffer[AETHER_TRANSPORT_MTU];
static uint8_t           Comms_ReceiveBuffer[AETHER_TRANSPORT_MTU];
static uint8_t           Comms_MessageBuffer[AETHER_TRANSPORT_MTU];
static uint8_t           Comms_HandleBuffer[AETHER_TRANSPORT_MTU];

static a_Err_t Comms_Start(void *arg);
static a_Err_t Comms_Stop(void *arg);
static size_t Comms_Send(const uint8_t *const data, const size_t size, void *arg);
static size_t Comms_Receive(uint8_t *const data, const size_t size, void *arg);
static void Comms_Hear(const char *const key, const uint8_t *const data, const size_t size, void *arg);
static void Comms_HearArm(const cavetalk_Mode mode);
static void Comms_HearDrive(const cavetalk_Drive *const drive);

static CaveTalk_Handle_t    Comms_Handle;
static CaveTalk_Callbacks_t Comms_Callbacks = {
    .hear_log          = NULL,
    .hear_arm          = Comms_HearArm,
    .hear_drive        = Comms_HearDrive,
    .hear_acceleration = NULL,
    .hear_gyroscope    = NULL,
    .hear_encoders     = NULL,
};

a_Tick_Ms_t a_TickUser_GetTick(void)
{
    return (a_Tick_Ms_t)BspTick_GetTick();
}

bool Comms_Initialize(void)
{
    const a_Socket_Functions_t functions = {
        .start   = Comms_Start,
        .stop    = Comms_Stop,
        .send    = Comms_Send,
        .receive = Comms_Receive,
        .arg     = NULL,
    };

    a_EnableRouting(false);
    a_Err_t error = a_Initialize(A_TRANSPORT_PEER_ID_MAX);

    if (A_ERR_NONE == error)
    {
        error = a_Socket_Initialize(&Comms_Socket,
                                    A_SOCKET_TYPE_SERIAL,
                                    functions,
                                    Comms_SendBuffer,
                                    sizeof(Comms_SendBuffer),
                                    Comms_ReceiveBuffer,
                                    sizeof(Comms_ReceiveBuffer));
    }

    if (A_ERR_NONE == error)
    {
        error = a_AddSocket(&Comms_Socket, Comms_MessageBuffer, sizeof(Comms_MessageBuffer), true);
    }

    if (A_ERR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kComms_LogTag, "Failed to initialize with error %s", a_Err_ToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_COMMS, error);
    }
    else
    {
        (void)CaveTalk_Initialize(&Comms_Handle,
                                  &Comms_Callbacks,
                                  COMMS_CAVETALK_ID,
                                  Comms_HandleBuffer,
                                  sizeof(Comms_HandleBuffer));

        error = a_Subscribe(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_ARM), Comms_Hear, NULL);

        if (A_ERR_NONE == error)
        {
            error = a_Subscribe(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_DRIVE), Comms_Hear, NULL);
        }

        /* Add other subscriptions here */

        if (A_ERR_NONE == error)
        {
            error = a_Declare(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_ACCELERATION));
        }

        if (A_ERR_NONE == error)
        {
            error = a_Declare(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_ENCODERS));
        }

        if (A_ERR_NONE == error)
        {
            error = a_Declare(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_GYROSCOPE));
        }

        if (A_ERR_NONE == error)
        {
            error = a_Declare(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_LOG));
        }

        /* Add other declarations here */

        if (A_ERR_NONE != error)
        {
            BSP_LOGGER_LOG_ERROR(kComms_LogTag, "Failed to setup keys with error %s", a_Err_ToString(error));
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_COMMS, error);
        }
    }

    return A_ERR_NONE == error;
}

void Comms_Task(void)
{
    a_Task();
}

static a_Err_t Comms_Start(void *arg)
{
    BSP_UNUSED(arg);

    a_Err_t     aether_error = A_ERR_NONE;
    Bsp_Error_t error        = BspUart_Start(COMMS_UART);

    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kComms_LogTag, "Failed to start with error %s", Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_COMMS, error);
        aether_error = A_ERR_SOCKET;
    }

    return aether_error;
}

static a_Err_t Comms_Stop(void *arg)
{
    BSP_UNUSED(arg);

    a_Err_t     aether_error = A_ERR_NONE;
    Bsp_Error_t error        = BspUart_Stop(COMMS_UART);

    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_ERROR(kComms_LogTag, "Failed to stop with error %s", Bsp_ErrorToString(error));
        FaultHandler_SetFault(FAULT_HANDLER_FAULT_COMMS, error);
        aether_error = A_ERR_SOCKET;
    }

    return aether_error;
}

static size_t Comms_Send(const uint8_t *const data, const size_t size, void *arg)
{
    BSP_UNUSED(arg);

    size_t      sent  = size;
    Bsp_Error_t error = BspUart_Transmit(COMMS_UART, data, size);

    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_DEBUG(kComms_LogTag, "Failed to send with error %s", Bsp_ErrorToString(error));
        sent = SIZE_MAX;
    }

    return sent;
}

static size_t Comms_Receive(uint8_t *const data, const size_t size, void *arg)
{
    BSP_UNUSED(arg);

    size_t      received = 0U;
    Bsp_Error_t error    = BspUart_Receive(COMMS_UART, data, size, &received);

    if (BSP_ERROR_NONE != error)
    {
        BSP_LOGGER_LOG_DEBUG(kComms_LogTag, "Failed to receive with error %s", Bsp_ErrorToString(error));
        received = SIZE_MAX;
    }

    return received;
}

static void Comms_Hear(const char *const key, const uint8_t *const data, const size_t size, void *arg)
{
    BSP_UNUSED(arg);

    CaveTalk_Hear(&Comms_Handle, (CaveTalk_Message_t){
        .key  = key,
        .data = data,
        .size = size,
    });
}

static void Comms_HearArm(const cavetalk_Mode mode)
{
    bool set = false;

    switch (mode)
    {
    case cavetalk_Mode_MODE_DISARMED:
        set = Cavebot_SetState(CAVEBOT_STATE_READY);
        break;
    case cavetalk_Mode_MODE_ARMED_MANUAL:
        set = Cavebot_SetState(CAVEBOT_STATE_MANUAL);
        break;
    case cavetalk_Mode_MODE_ARMED_AUTO:
        set = Cavebot_SetState(CAVEBOT_STATE_AUTO);
        break;
    default:
        break;
    }

    if (!set)
    {
        BSP_LOGGER_LOG_WARNING(kComms_LogTag, "Failed to set state %d", mode);
    }
}

static void Comms_HearDrive(const cavetalk_Drive *const drive)
{
    Cavebot_Drive(drive->speed_meters_per_second, drive->turn_rate_radians_per_second);
}