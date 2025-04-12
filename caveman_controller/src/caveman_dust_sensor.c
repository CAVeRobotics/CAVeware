#include "caveman_dust_sensor.h"

#include <stdint.h>

#include "bsp_uart.h"
#include "bsp_uart_user.h"

#define CAVEMAN_DUST_SENSOR_CHARACTERISTIC_BYTE 0xA5U
#define CAVEMAN_DUST_SENSOR_MASK                0x7FU
#define CAVEMAN_DUST_SENSOR_DATA_HIGH_SHIFT     7U

typedef enum
{
    CAVEMAN_DUST_SENSOR_BYTE_CHARACTERISTIC,
    CAVEMAN_DUST_SENSOR_BYTE_DATA_HIGH,
    CAVEMAN_DUST_SENSOR_BYTE_DATA_LOW,
    CAVEMAN_DUST_SENSOR_BYTE_CHECKSUM,
    CAVEMAN_DUST_SENSOR_BYTE_MAX
} CavemanDustSensor_Byte_t;

typedef struct
{
    uint8_t buffer[CAVEMAN_DUST_SENSOR_BYTE_MAX];
    CavemanDustSensor_Byte_t byte;
    CavemanDustSensor_MicrogramsPerMeterCubed_t reading;
} CavemanDustSensor_Handle_t;

static CavemanDustSensor_Handle_t CavemanDustSensor_Handle = {
    .buffer = {
        0U
    },
    .byte    = CAVEMAN_DUST_SENSOR_BYTE_CHARACTERISTIC,
    .reading = 0U,
};

Bsp_Error_t CavemanDustSensor_Initialize(void)
{
    return BspUart_Start(BSP_UART_USER_DUST_SENSOR);
}

CavemanDustSensor_MicrogramsPerMeterCubed_t CavemanDustSensor_Read(void)
{
    size_t  bytes_received = 0u;
    uint8_t sum            = 0U;

    (void)BspUart_Receive(BSP_UART_USER_DUST_SENSOR,
                          (uint8_t *)((uint32_t)CavemanDustSensor_Handle.buffer + (uint32_t)CavemanDustSensor_Handle.byte),
                          (sizeof(CavemanDustSensor_Handle.buffer) - (size_t)CavemanDustSensor_Handle.byte),
                          &bytes_received);

    for (size_t i = 0; i < bytes_received; i++)
    {
        switch (CavemanDustSensor_Handle.byte)
        {
        case CAVEMAN_DUST_SENSOR_BYTE_CHARACTERISTIC:
            if (CAVEMAN_DUST_SENSOR_CHARACTERISTIC_BYTE == CavemanDustSensor_Handle.buffer[CAVEMAN_DUST_SENSOR_BYTE_CHARACTERISTIC])
            {
                CavemanDustSensor_Handle.byte = CAVEMAN_DUST_SENSOR_BYTE_DATA_HIGH;
            }
            break;
        case CAVEMAN_DUST_SENSOR_BYTE_DATA_HIGH:
            CavemanDustSensor_Handle.byte = CAVEMAN_DUST_SENSOR_BYTE_DATA_LOW;
            break;
        case CAVEMAN_DUST_SENSOR_BYTE_DATA_LOW:
            CavemanDustSensor_Handle.byte = CAVEMAN_DUST_SENSOR_BYTE_CHECKSUM;
            break;
        case CAVEMAN_DUST_SENSOR_BYTE_CHECKSUM:
            sum = CavemanDustSensor_Handle.buffer[CAVEMAN_DUST_SENSOR_BYTE_CHARACTERISTIC] +
                  CavemanDustSensor_Handle.buffer[CAVEMAN_DUST_SENSOR_BYTE_DATA_HIGH] +
                  CavemanDustSensor_Handle.buffer[CAVEMAN_DUST_SENSOR_BYTE_DATA_LOW];

            if ((sum & CAVEMAN_DUST_SENSOR_MASK) == (CavemanDustSensor_Handle.buffer[CAVEMAN_DUST_SENSOR_BYTE_CHECKSUM] & CAVEMAN_DUST_SENSOR_MASK))
            {
                CavemanDustSensor_Handle.reading =
                    (uint16_t)((CavemanDustSensor_Handle.buffer[CAVEMAN_DUST_SENSOR_BYTE_DATA_HIGH] & CAVEMAN_DUST_SENSOR_MASK) << CAVEMAN_DUST_SENSOR_DATA_HIGH_SHIFT) +
                    (uint16_t)(CavemanDustSensor_Handle.buffer[CAVEMAN_DUST_SENSOR_BYTE_DATA_LOW] & CAVEMAN_DUST_SENSOR_MASK);
            }

            CavemanDustSensor_Handle.byte = CAVEMAN_DUST_SENSOR_BYTE_CHARACTERISTIC;
            break;
        case CAVEMAN_DUST_SENSOR_BYTE_MAX:
        default:
            break;
        }
    }

    return CavemanDustSensor_Handle.reading;
}