#ifndef BSP_ENCODER_USER_H
#define BSP_ENCODER_USER_H

#include "bsp.h"

typedef enum
{
    BSP_ENCODER_USER_TIMER_0,
    BSP_ENCODER_USER_TIMER_1,
    BSP_ENCODER_USER_TIMER_2,
    BSP_ENCODER_USER_TIMER_3,
    BSP_ENCODER_USER_TIMER_MAX
} BspEncoderUser_Timer_t;

extern Bsp_Encoder_t BspEncoderUser_HandleTable[BSP_ENCODER_USER_TIMER_MAX];

Bsp_Encoder_t *BspEncoderUser_GetEncoderHandle(const Bsp_TimerHandle_t *const timer_handle);

#endif /* BSP_ENCODER_USER_H */