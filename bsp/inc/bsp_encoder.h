#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "bsp.h"
#include "bsp_encoder_user.h"

Bsp_Error_t BspEncoder_Start(const BspEncoderUser_Timer_t timer);
Bsp_Error_t BspEncoder_Sample(const BspEncoderUser_Timer_t timer);

#endif /* BSP_ENCODER_H */