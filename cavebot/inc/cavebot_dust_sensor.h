#ifndef CAVEBOT_DUST_SENSOR_H
#define CAVEBOT_DUST_SENSOR_H

#include <stdint.h>

#include "bsp.h"

typedef uint16_t CavebotDustSensor_MicrogramsPerMeterCubed_t;

Bsp_Error_t CavebotDustSensor_Initialize(void);
CavebotDustSensor_MicrogramsPerMeterCubed_t CavebotDustSensor_Read(void);

#endif /* CAVEBOT_DUST_SENSOR_H */