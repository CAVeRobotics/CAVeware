#ifndef CAVEMAN_DUST_SENSOR_H
#define CAVEMAN_DUST_SENSOR_H

#include <stdint.h>

#include "bsp.h"

typedef uint16_t CavemanDustSensor_MicrogramsPerMeterCubed_t;

Bsp_Error_t CavemanDustSensor_Initialize(void);
CavemanDustSensor_MicrogramsPerMeterCubed_t CavemanDustSensor_Read(void);

#endif /* CAVEMAN_DUST_SENSOR_H */