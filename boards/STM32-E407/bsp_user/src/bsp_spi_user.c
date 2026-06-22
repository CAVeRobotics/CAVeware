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