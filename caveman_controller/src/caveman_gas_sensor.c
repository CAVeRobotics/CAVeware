#include "caveman_gas_sensor.h"

#include "bsp_adc.h"
#include "bsp_adc_user.h"

Bsp_Volt_t CavemanGasSensor_ReadRaw(void)
{
    Bsp_Volt_t volts = 0U;

    (void)BspAdc_Read(BSP_ADC_USER_ADC_1, BSP_ADC_USER_CHANNEL_RANK_1, &volts);

    return volts;
}