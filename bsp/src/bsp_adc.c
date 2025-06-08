#include "bsp_adc.h"

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_adc_user.h"

#define BSP_ADC_REFERENCE_VOLTAGE (double)3.3
#define BSP_ADC_RESOLUTION_BITS   12U
#define BSP_ADC_MAX               (uint16_t)(1U << BSP_ADC_RESOLUTION_BITS)

static void BspAdc_ConversionCompleteCallback(Bsp_AdcHandle_t *adc_handle);

Bsp_Error_t BspAdc_Start(const BspAdcUser_Adc_t adc)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (adc < BSP_ADC_USER_ADC_MAX)
    {
        BspAdcUser_HandleTable[adc].adc_handle->ConvCpltCallback = BspAdc_ConversionCompleteCallback;

        /* DMA data width is set to half word (uint16_t), but cast buffer to uint32_t to match prototype */
        /* TODO SD-349 set large buffer to reduce interrupt frequency */
        error = (Bsp_Error_t)HAL_ADC_Start_DMA(BspAdcUser_HandleTable[adc].adc_handle, (uint32_t*)BspAdcUser_HandleTable[adc].buffer, BspAdcUser_HandleTable[adc].channels * 4096U);
    }

    return error;
}

Bsp_Error_t BspAdc_ReadRaw(const BspAdcUser_Adc_t adc, const BspAdcUser_ChannelRank_t channel_rank, Bsp_AdcReading_t *const adc_reading)
{
    Bsp_Error_t error = BSP_ERROR_NONE;

    if (NULL == adc_reading)
    {
        error = BSP_ERROR_NULL;
    }
    else if ((adc >= BSP_ADC_USER_ADC_MAX) || (channel_rank >= BspAdcUser_HandleTable[adc].channels))
    {
        error = BSP_ERROR_PERIPHERAL;
    }
    else
    {
        *adc_reading = *(BspAdcUser_HandleTable[adc].shadow_buffer + channel_rank);
    }

    return error;
}

Bsp_Error_t BspAdc_Read(const BspAdcUser_Adc_t adc, const BspAdcUser_ChannelRank_t channel_rank, Bsp_Volt_t *const volts)
{
    Bsp_AdcReading_t adc_reading = 0;
    Bsp_Error_t      error       = BspAdc_ReadRaw(adc, channel_rank, &adc_reading);

    if (BSP_ERROR_NONE != error)
    {
    }
    else if (NULL == volts)
    {
        error = BSP_ERROR_NULL;
    }
    else
    {
        *volts = Bsp_Map(adc_reading, 0.0, (double)BSP_ADC_MAX, 0.0, BSP_ADC_REFERENCE_VOLTAGE);
    }

    return error;
}

static void BspAdc_ConversionCompleteCallback(Bsp_AdcHandle_t *adc_handle)
{
    Bsp_Adc_t *adc = BspAdcUser_GetAdc(adc_handle);

    if (NULL != adc)
    {
        for (uint8_t i = 0; i < adc->channels; i++)
        {
            *(adc->shadow_buffer + i) = *(adc->buffer + i); /* TODO SD-349 memcpy and average? */
        }
    }
}