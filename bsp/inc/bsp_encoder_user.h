#ifndef BSP_ENCODER_USER_H
#define BSP_ENCODER_USER_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp.h"

typedef int64_t BspEncoderUser_Pulse_t;
typedef int32_t BspEncoderUser_Period_t; /* Must not exceed 4 bytes for atomic read/write */

typedef enum
{
    BSP_ENCODER_USER_TIMER_0,
    BSP_ENCODER_USER_TIMER_1,
    BSP_ENCODER_USER_TIMER_MAX
} BspEncoderUser_Timer_t;

typedef enum
{
    BSP_ENCODER_USER_MODE_PULSES_PER_ROTATON,
    BSP_ENCODER_USER_MODE_RADIANS_PER_PULSE
} BspEncoderUser_Mode_t;

typedef struct
{
    Bsp_TimerHandle_t *timer_handle;
    BspEncoderUser_Pulse_t pulses_per_period;
    double smoothing_factor; /* Exponential moving average smoothing factor, 0 < smoothing_factor < 1 */
    BspEncoderUser_Mode_t mode;
    union
    {
        double pulses_per_rotation;
        Bsp_Radian_t radians_per_pulse;
    };
    volatile bool sampling;                                    /* True when sampling timer CNT register and periods elapsed */
    volatile uint16_t pulse_offset;                            /* Timer CNT register sample, can be updated by interrupt when sampling is true */
    volatile BspEncoderUser_Period_t previous_periods_elapsed; /* Periods elapsed sample, can be updated by interrupt when sampling is true */
    volatile BspEncoderUser_Period_t periods_elapsed;          /* Updated by interrupt */
    Bsp_Microsecond_t time;                                    /* Time when pulses were measured */
    BspEncoderUser_Pulse_t pulses;                             /* Pulse from most recent sample */
    Bsp_RadiansPerSecond_t raw_angular_rate;                   /* Angular rate calculated from most recent sample */
    Bsp_RadiansPerSecond_t angular_rate;                       /* Filtered angular rate */
} BspEncoderUser_Handle_t;

extern BspEncoderUser_Handle_t BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_MAX];

BspEncoderUser_Handle_t *BspEncoderUser_GetEncoderHandle(const Bsp_TimerHandle_t *const timer_handle);

#endif /* BSP_ENCODER_USER_H */