#ifndef __MPU6050_H
#define	__MPU6050_H

#include "iic.h"
#include "imu.h"
#include "r_cg_serial.h"

#define ACC_1G 4096

uint8_t Read_MPU6050(void);

uint8_t MPU6050_Init(void);

#endif 