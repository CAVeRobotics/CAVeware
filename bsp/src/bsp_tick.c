#include "bsp_tick.h"

#include "stm32f4xx_hal.h"

#include "bsp_tick_user.h"

extern __IO uint32_t uwTick;
extern void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);

static volatile uint64_t BspTick_MicrosecondsElapsed = 0U;

static void BspTick_TimerCallback(Bsp_TimerHandle_t *handle);

Bsp_Error_t BspTick_Start(void)
{
    BspTickUser_TimerHandle->PeriodElapsedCallback = BspTick_TimerCallback;

    HAL_TIM_Base_MspInit(BspTickUser_TimerHandle);

    return (Bsp_Error_t)HAL_TIM_Base_Start_IT(BspTickUser_TimerHandle);
}

Bsp_Millisecond_t BspTick_GetTick(void)
{
    return (Bsp_Millisecond_t)uwTick;
}

Bsp_Microsecond_t BspTick_GetMicroseconds(void)
{
    return BspTick_MicrosecondsElapsed + (Bsp_Microsecond_t)BspTickUser_TimerHandle->Instance->CNT;
}

static void BspTick_TimerCallback(Bsp_TimerHandle_t *handle)
{
    BSP_UNUSED(handle);

    BspTick_MicrosecondsElapsed += (Bsp_Microsecond_t)BspTickUser_TimerHandle->Instance->ARR + 1;
}