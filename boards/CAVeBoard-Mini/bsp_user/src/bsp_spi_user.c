#include "bsp_spi_user.h"

#include <stdbool.h>

#include "spi.h"

#include "bsp.h"

Bsp_Spi_t BspSpiUser_HandleTable[BSP_SPI_USER_MAX] = {
    [BSP_SPI_USER_0] = {
        .spi_handle = &hspi2,
        .busy       = false,
        .callback   = {
            .function = NULL,
            .arg      = NULL,
        },
    },
};

Bsp_Spi_t *BspSpiUser_GetSpi(const Bsp_SpiHandle_t *const spi_handle)
{
    Bsp_Spi_t *spi = NULL;

    /* TODO CVW-76 duplicate this pattern to the rest of the BSP User "get" methods */
    for (BspSpiUser_Spi_t user_spi = BSP_SPI_USER_0; user_spi < BSP_SPI_USER_MAX; user_spi++)
    {
        if (spi_handle == BspSpiUser_HandleTable[user_spi].spi_handle)
        {
            spi = &BspSpiUser_HandleTable[user_spi];
        }
    }

    return spi;
}