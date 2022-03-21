/*
 * MPU6050.h
 *
 *  Created on: 31-01-2015
 *      Author: CKDTCNTT
 */

#ifndef MPU6050_H_
#define MPU6050_H_


/*  Prototype  */
#include "include.h"

//*****************************************************************************
//
// The factors used to convert the acceleration readings from the MPU6050 into
// floating point values in meters per second squared.
//
// Values are obtained by taking the g conversion factors from the data sheet
// and multiplying by 9.81 (1 g = 9.81 m/s^2).
//
//*****************************************************************************
static const float g_fMPU6050AccelFactors[] =
{
    0.00059875,                              // Range = +/- 2 g (16384 lsb/g)
    0.00119751,                              // Range = +/- 4 g (8192 lsb/g)
    0.00239502,                              // Range = +/- 8 g (4096 lsb/g)
    0.00479004                               // Range = +/- 16 g (2048 lsb/g)
};

//*****************************************************************************
//
// The factors used to convert the acceleration readings from the MPU6050 into
// floating point values in radians per second.
//
// Values are obtained by taking the degree per second conversion factors
// from the data sheet and then converting to radians per sec (1 degree =
// 0.0174532925 radians).
//
//*****************************************************************************
static const float g_fMPU6050GyroFactors[] =
{
    1.3323124e-4f,   // Range = +/- 250 dps  (131.0 LSBs/DPS)
    2.6646248e-4f,   // Range = +/- 500 dps  (65.5 LSBs/DPS)
    5.3211258e-4f,   // Range = +/- 1000 dps (32.8 LSBs/DPS)
    0.0010642252f    // Range = +/- 2000 dps (16.4 LSBs/DPS)
};


void initMPU6050();
void mpu6050_Config();
void mpu6050_Read_All(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);
void mpu6050_Read_GyroXYZ(int16_t *x, int16_t *y, int16_t *z);
void mpu6050_Read_AccelXYZ(int16_t *x, int16_t *y, int16_t *z);
void mpu6050_Read_Temp(int16_t *temp);


#endif /* MPU6050_H_ */
