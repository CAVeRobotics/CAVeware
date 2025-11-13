#include "bsp_timer.h"

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_timer_user.h"

extern void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *tim_baseHandle);

static void BspTimer_PeriodElapsedCallback(Bsp_TimerHandle_t *timer_handle);
static Bsp_Timer_t *BspTimer_GetTimer(const Bsp_TimerHandle_t *const timer_handle);

Bsp_Error_t BspTimer_Start(const BspTimerUser_Timer_t timer)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (timer < BSP_TIMER_USER_TIMER_MAX)
    {
        BspTimerUser_HandleTable[timer].timer_handle->PeriodElapsedCallback = BspTimer_PeriodElapsedCallback;

        HAL_TIM_Base_MspInit(BspTimerUser_HandleTable[timer].timer_handle);

        error = (Bsp_Error_t)HAL_TIM_Base_Start_IT(BspTimerUser_HandleTable[timer].timer_handle);
    }

    return error;
}

Bsp_Error_t BspTimer_Stop(const BspTimerUser_Timer_t timer)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (timer < BSP_TIMER_USER_TIMER_MAX)
    {
        error = (Bsp_Error_t)HAL_TIM_Base_Stop_IT(BspTimerUser_HandleTable[timer].timer_handle);

        HAL_TIM_Base_MspDeInit(BspTimerUser_HandleTable[timer].timer_handle);
    }

    return error;
}

Bsp_Error_t BspTimer_Reset(const BspTimerUser_Timer_t timer)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (timer < BSP_TIMER_USER_TIMER_MAX)
    {
        BspTimerUser_HandleTable[timer].counts_elapsed        = 0U;
        BspTimerUser_HandleTable[timer].counts_elapsed_shadow = 0U;
        BspTimerUser_HandleTable[timer].counts_offset         = 0U;
        BspTimerUser_HandleTable[timer].counts                = 0U;

        error = BSP_ERROR_NONE;
    }

    return error;
}

Bsp_Error_t BspTimer_Sample(const BspTimerUser_Timer_t timer)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (timer < BSP_TIMER_USER_TIMER_MAX)
    {
        BspTimerUser_HandleTable[timer].sampling              = true;
        BspTimerUser_HandleTable[timer].counts_elapsed_shadow = BspTimerUser_HandleTable[timer].counts_elapsed;
        BspTimerUser_HandleTable[timer].counts_offset         = BspTimerUser_HandleTable[timer].timer_handle->Instance->CNT;
        BspTimerUser_HandleTable[timer].sampling              = false;

        BspTimerUser_HandleTable[timer].counts = BspTimerUser_HandleTable[timer].counts_elapsed_shadow + BspTimerUser_HandleTable[timer].counts_offset;

        error = BSP_ERROR_NONE;
    }

    return error;
}

Bsp_Error_t BspTimer_RegisterPeriodElapsedCallback(const BspTimerUser_Timer_t timer, const Bsp_Callback_t *const callback)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (timer >= BSP_TIMER_USER_TIMER_MAX)
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else if (NULL == callback)
    {
        error = BSP_ERROR_NULL;
    }
    else
    {
        BspTimerUser_HandleTable[timer].period_elapsed = *callback;
    }

    return error;
}

static void BspTimer_PeriodElapsedCallback(Bsp_TimerHandle_t *timer_handle)
{
    Bsp_Timer_t *timer = BspTimer_GetTimer(timer_handle);

    if (NULL != timer)
    {
        timer->counts_elapsed += (uint64_t)timer->timer_handle->Instance->ARR + 1;

        if (timer->sampling)
        {
            timer->counts_elapsed_shadow = timer->counts_elapsed;
            timer->counts_offset         = timer->timer_handle->Instance->CNT;
        }

        if (NULL != timer->period_elapsed.function)
        {
            timer->period_elapsed.function(timer->period_elapsed.arg);
        }
    }
}

static Bsp_Timer_t *BspTimer_GetTimer(const Bsp_TimerHandle_t *const timer_handle)
{
    Bsp_Timer_t *timer = NULL;

    for (BspTimerUser_Timer_t user_timer = BSP_TIMER_USER_TIMER_0; user_timer < BSP_TIMER_USER_TIMER_MAX; user_timer++)
    {
        if (timer_handle == BspTimerUser_HandleTable[user_timer].timer_handle)
        {
            timer = &BspTimerUser_HandleTable[user_timer];
            break;
        }
    }

    return timer;
}