/*
 * mpu9265_fun.h
 *
 *  Created on: 23 giu 2023
 *      Author: rosso
 */

#ifndef INC_MPU9265_FUN_H_
#define INC_MPU9265_FUN_H_

#include <mpu9265.h>
#include "i2c.h"

void MPU9265_CheckStatus(uint8_t *id, HAL_StatusTypeDef *status);
void MPU9265_Init(void);
void MPU9265_CalibrateGyro(float *gyroOffset);
void MPU9265_ReadAccel(float* accelData);
void MPU9265_ReadGyro(float* gyroData);
void MPU9265_ReadAccelGyro(float* accelData, float* gyroData);
void MPU9265_ComputeRollPitch(float *accelData, float *roll, float *pitch);
void MPU9265_ComputeRollPitchYaw(float *gyroData, float dt, float *roll, float *pitch, float *yaw);

#endif /* INC_MPU9265_FUN_H_ */
