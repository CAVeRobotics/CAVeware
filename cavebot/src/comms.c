#include "comms.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aether.h"
#include "cavetalk.h"

#include "bsp_logger.h"
#include "bsp_tick.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

#include "cavebot.h"
#include "fault_handler.h"

#define COMMS_UART BSP_UART_USER_1

static const char *const kComms_LogTag   = "COMMS";
static bool              Comms_Connected = false;
static a_Socket_t        Comms_Socket;
static a_Session_t       Comms_Session;
static uint8_t           Comms_SendBuffer[AETHER_TRANSPORT_MTU];
static uint8_t           Comms_ReceiveBuffer[AETHER_TRANSPORT_MTU];
static uint8_t           Comms_MessageBuffer[AETHER_TRANSPORT_MTU];
static uint8_t           Comms_HandleBuffer[AETHER_TRANSPORT_MTU];

static a_Err_t Comms_Start(void *arg);
static a_Err_t Comms_Stop(void *arg);
static size_t Comms_Send(const uint8_t *const data, const size_t size, void *arg);
static size_t Comms_Receive(uint8_t *const data, const size_t size, void *arg);
static void Comms_EventHandler(const a_Event_t event, const a_Session_t *const session, const a_Err_t *const error, void *arg);
static void Comms_Speak(const char *const key, const uint8_t *const data, const size_t size);
static void Comms_Hear(const char *const key, const uint8_t *const data, const size_t size, void *arg);
static void Comms_HearSetMode(const cavetalk_Mode mode);
static void Comms_HearDrive(const cavetalk_Drive *const drive);

static CaveTalk_Handle_t    Comms_Handle;
static CaveTalk_Callbacks_t Comms_Callbacks = {
    .hear_log          = NULL,
    .hear_set_mode     = Comms_HearSetMode,
    .hear_get_mode     = NULL,
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

    a_Err_t error = a_Initialize(A_TRANSPORT_PEER_ID_MAX);

    if (A_ERR_NONE == error)
    {
        a_EnableRouting(false);
        a_RegisterEventHandler(Comms_EventHandler, NULL);

        error = a_InitializeSocket(&Comms_Socket,
                                   A_SOCKET_TYPE_SERIAL,
                                   functions,
                                   Comms_SendBuffer,
                                   sizeof(Comms_SendBuffer),
                                   Comms_ReceiveBuffer,
                                   sizeof(Comms_ReceiveBuffer));
    }

    if (A_ERR_NONE == error)
    {
        error = a_AddSession(&Comms_Session, &Comms_Socket, Comms_MessageBuffer, sizeof(Comms_MessageBuffer), true);
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

        error = a_Subscribe(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_SET_MODE), Comms_Hear, NULL);

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

        if (A_ERR_NONE == error)
        {
            error = a_Declare(CaveTalk_GetKey(&Comms_Handle, COMMS_CAVETALK_ID, cavetalk_Id_ID_GET_MODE));
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

void Comms_SpeakLog(char *const log)
{
    if (Comms_Connected && !FaultHandler_HasFault(FAULT_HANDLER_FAULT_COMMS))
    {
        CaveTalk_Message_t *message = CaveTalk_SpeakLog(&Comms_Handle, log);

        if (NULL != message)
        {
            Comms_Speak(message->key, message->data, message->size);
        }
    }
}

void Comms_SpeakGetMode(const cavetalk_Mode mode)
{
    if (Comms_Connected && !FaultHandler_HasFault(FAULT_HANDLER_FAULT_COMMS))
    {
        CaveTalk_Message_t *message = CaveTalk_SpeakGetMode(&Comms_Handle, mode);

        if (NULL != message)
        {
            Comms_Speak(message->key, message->data, message->size);
        }
    }
}

void Comms_SpeakAcceleration(const cavetalk_Acceleration *const acceleration)
{
    if (Comms_Connected && !FaultHandler_HasFault(FAULT_HANDLER_FAULT_COMMS))
    {
        CaveTalk_Message_t *message = CaveTalk_SpeakAcceleration(&Comms_Handle, acceleration);

        if (NULL != message)
        {
            Comms_Speak(message->key, message->data, message->size);
        }
    }
}

void Comms_SpeakGyroscope(const cavetalk_Gyroscope *const gyroscope)
{
    if (Comms_Connected && !FaultHandler_HasFault(FAULT_HANDLER_FAULT_COMMS))
    {
        CaveTalk_Message_t *message = CaveTalk_SpeakGyroscope(&Comms_Handle, gyroscope);

        if (NULL != message)
        {
            Comms_Speak(message->key, message->data, message->size);
        }
    }
}

void Comms_SpeakEncoders(cavetalk_Encoder *const encoders, const size_t count)
{
    if (Comms_Connected && !FaultHandler_HasFault(FAULT_HANDLER_FAULT_COMMS))
    {
        CaveTalk_Message_t *message = CaveTalk_SpeakEncoders(&Comms_Handle, encoders, count);

        if (NULL != message)
        {
            Comms_Speak(message->key, message->data, message->size);
        }
    }
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
        BSP_LOGGER_LOG_WARNING(kComms_LogTag, "Failed to send with error %s", Bsp_ErrorToString(error));
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
        BSP_LOGGER_LOG_WARNING(kComms_LogTag, "Failed to receive with error %s", Bsp_ErrorToString(error));
        received = SIZE_MAX;
    }

    return received;
}

static void Comms_EventHandler(const a_Event_t event, const a_Session_t *const session, const a_Err_t *const error, void *arg)
{
    BSP_UNUSED(session);
    BSP_UNUSED(arg);

    switch (event)
    {
    case A_EVENT_OPEN:
        Comms_Connected = true;
        BSP_LOGGER_LOG_DEBUG(kComms_LogTag, "Connected");
        break;
    case A_EVENT_CLOSE:
        Comms_Connected = false;
        BSP_LOGGER_LOG_DEBUG(kComms_LogTag, "Disconnected");
        break;
    case A_EVENT_ERROR:
        if ((NULL != error) && (A_ERR_MEMORY == *error))
        {
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_MEMORY, *error);
            FaultHandler_SetFault(FAULT_HANDLER_FAULT_COMMS, *error);
        }
        break;
    default:
        break;
    }
}

static void Comms_Speak(const char *const key, const uint8_t *const data, const size_t size)
{
    FaultHandler_SetFault(FAULT_HANDLER_FAULT_COMMS, a_Publish(key, data, size));
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

static void Comms_HearSetMode(const cavetalk_Mode mode)
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