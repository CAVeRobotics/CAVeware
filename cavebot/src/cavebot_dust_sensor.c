#include "cavebot_dust_sensor.h"

#include <stdint.h>

#include "bsp_uart.h"
#include "bsp_uart_user.h"

#define CAVEBOT_DUST_SENSOR_CHARACTERISTIC_BYTE 0xA5U
#define CAVEBOT_DUST_SENSOR_MASK                0x7FU
#define CAVEBOT_DUST_SENSOR_DATA_HIGH_SHIFT     7U

typedef enum
{
    CAVEBOT_DUST_SENSOR_BYTE_CHARACTERISTIC,
    CAVEBOT_DUST_SENSOR_BYTE_DATA_HIGH,
    CAVEBOT_DUST_SENSOR_BYTE_DATA_LOW,
    CAVEBOT_DUST_SENSOR_BYTE_CHECKSUM,
    CAVEBOT_DUST_SENSOR_BYTE_MAX
} CavebotDustSensor_Byte_t;

typedef struct
{
    uint8_t buffer[CAVEBOT_DUST_SENSOR_BYTE_MAX];
    CavebotDustSensor_Byte_t byte;
    CavebotDustSensor_MicrogramsPerMeterCubed_t reading;
} CavebotDustSensor_Handle_t;

static CavebotDustSensor_Handle_t CavebotDustSensor_Handle = {
    .buffer = {
        0U
    },
    .byte    = CAVEBOT_DUST_SENSOR_BYTE_CHARACTERISTIC,
    .reading = 0U,
};

Bsp_Error_t CavebotDustSensor_Initialize(void)
{
    return BspUart_Start(BSP_UART_USER_DUST_SENSOR);
}

CavebotDustSensor_MicrogramsPerMeterCubed_t CavebotDustSensor_Read(void)
{
    size_t  bytes_received = 0u;
    uint8_t sum            = 0U;

    (void)BspUart_Receive(BSP_UART_USER_DUST_SENSOR,
                          (uint8_t *)((uint32_t)CavebotDustSensor_Handle.buffer + (uint32_t)CavebotDustSensor_Handle.byte),
                          (sizeof(CavebotDustSensor_Handle.buffer) - (size_t)CavebotDustSensor_Handle.byte),
                          &bytes_received);

    for (size_t i = 0; i < bytes_received; i++)
    {
        switch (CavebotDustSensor_Handle.byte)
        {
        case CAVEBOT_DUST_SENSOR_BYTE_CHARACTERISTIC:
            if (CAVEBOT_DUST_SENSOR_CHARACTERISTIC_BYTE == CavebotDustSensor_Handle.buffer[CAVEBOT_DUST_SENSOR_BYTE_CHARACTERISTIC])
            {
                CavebotDustSensor_Handle.byte = CAVEBOT_DUST_SENSOR_BYTE_DATA_HIGH;
            }
            break;
        case CAVEBOT_DUST_SENSOR_BYTE_DATA_HIGH:
            CavebotDustSensor_Handle.byte = CAVEBOT_DUST_SENSOR_BYTE_DATA_LOW;
            break;
        case CAVEBOT_DUST_SENSOR_BYTE_DATA_LOW:
            CavebotDustSensor_Handle.byte = CAVEBOT_DUST_SENSOR_BYTE_CHECKSUM;
            break;
        case CAVEBOT_DUST_SENSOR_BYTE_CHECKSUM:
            sum = CavebotDustSensor_Handle.buffer[CAVEBOT_DUST_SENSOR_BYTE_CHARACTERISTIC] +
                  CavebotDustSensor_Handle.buffer[CAVEBOT_DUST_SENSOR_BYTE_DATA_HIGH] +
                  CavebotDustSensor_Handle.buffer[CAVEBOT_DUST_SENSOR_BYTE_DATA_LOW];

            if ((sum & CAVEBOT_DUST_SENSOR_MASK) == (CavebotDustSensor_Handle.buffer[CAVEBOT_DUST_SENSOR_BYTE_CHECKSUM] & CAVEBOT_DUST_SENSOR_MASK))
            {
                CavebotDustSensor_Handle.reading =
                    (uint16_t)((CavebotDustSensor_Handle.buffer[CAVEBOT_DUST_SENSOR_BYTE_DATA_HIGH] & CAVEBOT_DUST_SENSOR_MASK) << CAVEBOT_DUST_SENSOR_DATA_HIGH_SHIFT) +
                    (uint16_t)(CavebotDustSensor_Handle.buffer[CAVEBOT_DUST_SENSOR_BYTE_DATA_LOW] & CAVEBOT_DUST_SENSOR_MASK);
            }

            CavebotDustSensor_Handle.byte = CAVEBOT_DUST_SENSOR_BYTE_CHARACTERISTIC;
            break;
        case CAVEBOT_DUST_SENSOR_BYTE_MAX:
        default:
            break;
        }
    }

    return CavebotDustSensor_Handle.reading;
}