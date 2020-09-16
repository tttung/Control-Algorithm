#ifndef __MS5611_H
#define __MS5611_H

#include "iic.h"
#include "r_cg_userdefine.h"
#include <math.h>

void MS5611_Init(void);
void Baro_update(void);

extern int32_t baroHigh;
extern int32_t baroTemperature;

#endif
