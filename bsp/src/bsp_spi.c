#include "bsp_spi.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "bsp.h"
#include "bsp_spi_user.h"

extern void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle);

static void BspSpi_Callback(Bsp_SpiHandle_t *spi_handle);

Bsp_Error_t BspSpi_Start(const BspSpiUser_Spi_t spi)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (spi < BSP_SPI_USER_MAX)
    {
        BspSpiUser_HandleTable[spi].spi_handle->TxCpltCallback = BspSpi_Callback;
        BspSpiUser_HandleTable[spi].spi_handle->RxCpltCallback = BspSpi_Callback;
        BspSpiUser_HandleTable[spi].busy                       = false;
        BspSpiUser_HandleTable[spi].callback.function          = NULL;
        BspSpiUser_HandleTable[spi].callback.arg               = NULL;

        HAL_SPI_MspInit(BspSpiUser_HandleTable[spi].spi_handle);
    }

    return error;
}

Bsp_Error_t BspSpi_Transmit(const BspSpiUser_Spi_t spi, const uint8_t *const data, const size_t size, const Bsp_Callback_t *const callback)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (spi >= BSP_SPI_USER_MAX)
    {
    }
    else if (BspSpiUser_HandleTable[spi].busy)
    {
        error = BSP_ERROR_BUSY;
    }
    else if (size > (size_t)UINT16_MAX)
    {
        error = BSP_ERROR_SIZE;
    }
    else
    {
        if (NULL != callback)
        {
            BspSpiUser_HandleTable[spi].callback = *callback;
        }
        else
        {
            BspSpiUser_HandleTable[spi].callback.function = NULL;
        }

        BspSpiUser_HandleTable[spi].busy = true;
        error                            = (Bsp_Error_t)HAL_SPI_Transmit_DMA(BspSpiUser_HandleTable[spi].spi_handle, data, (uint16_t)size);
    }

    return error;
}

Bsp_Error_t BspSpi_Receive(const BspSpiUser_Spi_t spi, uint8_t *const data, const size_t size, const Bsp_Callback_t *const callback)
{
    Bsp_Error_t error = BSP_ERROR_PERIPHERAL;

    if (spi >= BSP_SPI_USER_MAX)
    {
    }
    else if (BspSpiUser_HandleTable[spi].busy)
    {
        error = BSP_ERROR_BUSY;
    }
    else if (size > (size_t)UINT16_MAX)
    {
        error = BSP_ERROR_SIZE;
    }
    else
    {
        if (NULL != callback)
        {
            BspSpiUser_HandleTable[spi].callback = *callback;
        }
        else
        {
            BspSpiUser_HandleTable[spi].callback.function = NULL;
        }

        BspSpiUser_HandleTable[spi].busy = true;
        error                            = (Bsp_Error_t)HAL_SPI_Receive_DMA(BspSpiUser_HandleTable[spi].spi_handle, data, (uint16_t)size);
    }

    return error;
}

static void BspSpi_Callback(Bsp_SpiHandle_t *spi_handle)
{
    Bsp_Spi_t *spi = BspSpiUser_GetSpi(spi_handle);

    if (NULL != spi)
    {
        spi->busy = false;

        if (NULL != spi->callback.function)
        {
            spi->callback.function(spi->callback.arg);
        }
    }
}