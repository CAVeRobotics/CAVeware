#ifndef BSP_H
#define BSP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define BSP_UNUSED(x) (void)(x)
#define BSP_UART_DOUBLE_BUFFER_COUNT 2U

typedef double   Bsp_Percent_t;
typedef double   Bsp_Radian_t;
typedef double   Bsp_RadiansPerSecond_t;
typedef uint32_t Bsp_Millisecond_t;
typedef uint64_t Bsp_Microsecond_t;
typedef int64_t  Bsp_EncoderPulse_t;
typedef int32_t  Bsp_EncoderPeriod_t; /* Must not exceed 4 bytes for atomic read/write */

typedef TIM_HandleTypeDef  Bsp_TimerHandle_t;
typedef UART_HandleTypeDef Bsp_UartHandle_t;

typedef struct Bsp_Encoder          Bsp_Encoder_t;
typedef struct Bsp_PwmConfig        Bsp_PwmConfig_t;
typedef struct Bsp_UartDoubleBuffer Bsp_UartDoubleBuffer_t;
typedef struct Bsp_Uart             Bsp_Uart_t;

typedef enum
{
    BSP_ERROR_NONE = HAL_OK,
    BSP_ERROR_HAL = HAL_ERROR,
    BSP_ERROR_BUSY = HAL_BUSY,
    BSP_ERROR_TIMEOUT = HAL_TIMEOUT,
    BSP_ERROR_PERIPHERAL = 0x04U,
    BSP_ERROR_VALUE = 0x05U,
    BSP_ERROR_NULL = 0x06U,
} Bsp_Error_t;

typedef enum
{
    BSP_TIMER_CHANNEL_1 = TIM_CHANNEL_1,
    BSP_TIMER_CHANNEL_2 = TIM_CHANNEL_2,
    BSP_TIMER_CHANNEL_3 = TIM_CHANNEL_3,
    BSP_TIMER_CHANNEL_4 = TIM_CHANNEL_4,
    BSP_TIMER_CHANNEL_ALL = TIM_CHANNEL_ALL
} Bsp_TimerChannel_t;

typedef enum
{
    BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON,
    BSP_ENCODER_USER_MODE_RADIANS_PER_PULSE
} Bsp_EncoderMode_t;

struct Bsp_Encoder
{
    Bsp_TimerHandle_t *timer_handle;
    Bsp_EncoderPulse_t pulses_per_period;
    double smoothing_factor; /* Exponential moving average smoothing factor, 0 < smoothing_factor < 1 */
    Bsp_EncoderMode_t mode;
    union
    {
        double pulses_per_rotation;
        Bsp_Radian_t radians_per_pulse;
    };
    volatile bool sampling;                                /* True when sampling timer CNT register and periods elapsed */
    volatile uint16_t pulse_offset;                        /* Timer CNT register sample, can be updated by interrupt when sampling is true */
    volatile Bsp_EncoderPeriod_t previous_periods_elapsed; /* Periods elapsed sample, can be updated by interrupt when sampling is true */
    volatile Bsp_EncoderPeriod_t periods_elapsed;          /* Updated by interrupt */
    Bsp_Microsecond_t time;                                /* Time when pulses were measured */
    Bsp_EncoderPulse_t pulses;                             /* Pulse from most recent sample */
    Bsp_RadiansPerSecond_t raw_angular_rate;               /* Angular rate calculated from most recent sample */
    Bsp_RadiansPerSecond_t angular_rate;                   /* Filtered angular rate */
};

struct Bsp_PwmConfig
{
    Bsp_TimerHandle_t *timer_handle;
    Bsp_TimerChannel_t max_channel;
};

struct Bsp_UartDoubleBuffer
{
    uint8_t *buffer;
    uint32_t half_buffer_size;
    volatile bool writing;
    volatile bool reading;
    volatile uint8_t unlocked; /* 0 - write or 1 - read, numeric type because used as index */
    volatile uint32_t write_count[BSP_UART_DOUBLE_BUFFER_COUNT];
    volatile uint32_t read_count[BSP_UART_DOUBLE_BUFFER_COUNT];
};

struct Bsp_Uart
{
    Bsp_UartHandle_t *uart_handle;
    Bsp_UartDoubleBuffer_t tx_buffer;
    uint8_t *rx_buffer;
    uint32_t rx_buffer_size;
    uint32_t read_pointer;
};

void Bsp_Initialize(void);
void Bsp_Delay(const Bsp_Millisecond_t delay);
double Bsp_Map(const double value, const double in_min, const double in_max, const double out_min, const double out_max);

#endif /* BSP_H */