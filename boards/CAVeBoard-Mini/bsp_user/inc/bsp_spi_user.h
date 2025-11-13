#ifndef BSP_SPI_USER_H
#define BSP_SPI_USER_H

#include "bsp.h"

typedef enum
{
    BSP_SPI_USER_0,
    BSP_SPI_USER_MAX
} BspSpiUser_Spi_t;

extern Bsp_Spi_t BspSpiUser_HandleTable[BSP_SPI_USER_MAX];

#endif /* BSP_SPI_USER_H */