#include "comms.h"

#include <stdbool.h>
#include <stddef.h>

#include "acceleration.pb.h"
#include "aether.h"
#include "arm.pb.h"
#include "cavetalk.h"
#include "drive.pb.h"
#include "encoders.pb.h"
#include "gyroscope.pb.h"
#include "log.pb.h"

#include "bsp.h"
#include "bsp_gpio.h"
#include "bsp_gpio_user.h"
#include "bsp_encoder_user.h"
#include "bsp_logger.h"
#include "bsp_tick.h"
#include "bsp_uart.h"
#include "bsp_uart_user.h"

#include "accelerometer.h"
#include "gyroscope.h"

#include "cavebot.h"
#include "cavebot_user.h"
#ifdef BOARD_CAVEBOARD
#include "rover_4ws.h"
#endif
#ifdef BOARD_CAVEBOARD_MINI
#include "rover_4wd.h"
#endif

#define CAVEBOT_CAVETALK_ID 0x00000001U

static a_Socket_t Comms_Socket;
static uint8_t    Comms_SendBuffer[AETHER_TRANSPORT_MTU];
static uint8_t    Comms_ReceiveBuffer[AETHER_TRANSPORT_MTU];
static uint8_t    Comms_MessageBuffer[AETHER_TRANSPORT_MTU];
static uint8_t    Comms_HandleBuffer[AETHER_TRANSPORT_MTU];

static Cavebot_Error_t Comms_AetherToCavebotError(const a_Err_t error);
static a_Err_t Comms_Start(void *arg);
static a_Err_t Comms_Stop(void *arg);
static size_t Comms_Send(const uint8_t *const data, const size_t size, void *arg);
static size_t Comms_Receive(uint8_t *const data, const size_t size, void *arg);

static CaveTalk_Handle_t    Comms_Handle;
static CaveTalk_Callbacks_t Comms_Callbacks = CAVETALK_CALLBACKS_NULL; /* TODO CVW-22 */

a_Tick_Ms_t a_TickUser_GetTick(void)
{
    return (a_Tick_Ms_t)BspTick_GetTick();
}

Cavebot_Error_t Comms_Initialize(void)
{
    Cavebot_Error_t error = Comms_AetherToCavebotError(a_Initialize(A_TRANSPORT_PEER_ID_MAX));

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Comms_AetherToCavebotError(
            a_Socket_Initialize(
                &Comms_Socket,
                A_SOCKET_TYPE_SERIAL,
                (a_Socket_Functions_t){
            .start   = Comms_Start,
            .stop    = Comms_Stop,
            .send    = Comms_Send,
            .receive = Comms_Receive,
            .arg     = NULL,
        },
                Comms_SendBuffer,
                sizeof(Comms_SendBuffer),
                Comms_ReceiveBuffer,
                sizeof(Comms_ReceiveBuffer)));
    }

    if (CAVEBOT_ERROR_NONE == error)
    {
        error = Comms_AetherToCavebotError(
            a_AddSocket(&Comms_Socket, Comms_MessageBuffer, sizeof(Comms_MessageBuffer), true));
    }

    if (CAVEBOT_ERROR_NONE == error)
    {
        (void)CaveTalk_Initialize(&Comms_Handle,
                                  &Comms_Callbacks,
                                  CAVEBOT_CAVETALK_ID,
                                  Comms_HandleBuffer,
                                  sizeof(Comms_HandleBuffer));
    }

    return error;
}

void Comms_Task(void)
{
    a_Task();
}

static Cavebot_Error_t Comms_AetherToCavebotError(const a_Err_t error)
{

    Cavebot_Error_t cavebot_error = CAVEBOT_ERROR_NONE;

    switch (error)
    {
    case A_ERR_NULL:
        cavebot_error = CAVEBOT_ERROR_NULL;
        break;
    case A_ERR_SIZE:
    case A_ERR_MEMORY:
    case A_ERR_SERIALIZATION:
    case A_ERR_SOCKET:
    case A_ERR_SEQUENCE:
    case A_ERR_DUPLICATE:
    case A_ERR_TIMEOUT:
    case A_ERR_MAX:
    case A_ERR_NONE:
    default:
        break;
    }

    return cavebot_error;
}

static a_Err_t Comms_Start(void *arg)
{
    BSP_UNUSED(arg);

    /* TODO CVW-22 */

    return A_ERR_MAX;
}

static a_Err_t Comms_Stop(void *arg)
{
    BSP_UNUSED(arg);

    /* TODO CVW-22 */

    return A_ERR_MAX;
}

static size_t Comms_Send(const uint8_t *const data, const size_t size, void *arg)
{
    BSP_UNUSED(data);
    BSP_UNUSED(size);
    BSP_UNUSED(arg);

    /* TODO CVW-22 */

    return SIZE_MAX;
}

static size_t Comms_Receive(uint8_t *const data, const size_t size, void *arg)
{
    BSP_UNUSED(data);
    BSP_UNUSED(size);
    BSP_UNUSED(arg);

    /* TODO CVW-22 */

    return SIZE_MAX;
}