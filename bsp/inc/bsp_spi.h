#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stddef.h>
#include <stdint.h>

#include "bsp.h"
#include "bsp_spi_user.h"

Bsp_Error_t BspSpi_Start(const BspSpiUser_Spi_t spi);
Bsp_Error_t BspSpi_Transmit(const BspSpiUser_Spi_t spi, const uint8_t *const data, const size_t size, const Bsp_Callback_t *const callback);
Bsp_Error_t BspSpi_Receive(const BspSpiUser_Spi_t spi, uint8_t *const data, const size_t size, const Bsp_Callback_t *const callback);

#endif /* BSP_SPI_H */