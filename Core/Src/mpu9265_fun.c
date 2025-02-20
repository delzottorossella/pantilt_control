/*
 * mpu9265_fun.c
 *
 * Contains functions to read data from IMU MPU92/65 and compute angles
 *
 *  Created on: 23 giu 2023
 *      Author: rosso
 */
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <mpu9265.h>
#include <mpu9265_fun.h>

float OffsetGyro[3] = {-1.96690583f, 0.816289783f, -1.07923603f}; // offset for drift error of gyroscope

void MPU9265_CheckStatus(uint8_t *id, HAL_StatusTypeDef *status){
	 // reading ID device
	*status = HAL_I2C_Mem_Read(&hi2c1,MPU9265_I2C_ADDR<<1,MPU9265_WHO_AM_I,1,id,1,1000);

	if (*status != HAL_OK){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // turn on the led
	} else if(*status == HAL_OK){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // turn off the led
	}
}

void MPU9265_Init(void)
{
	  // Turn on the IMU
	  HAL_GPIO_WritePin(OUT_VDD_GPIO_Port, OUT_VDD_Pin, GPIO_PIN_SET);
	  HAL_Delay(100);

	  // Reset IMU
	  uint8_t reset = 0x00;
	  HAL_I2C_Mem_Write(&hi2c1, MPU9265_I2C_ADDR << 1, MPU9265_PWR_MGMT_1, 1, &reset, 1, 100);
	  HAL_Delay(100);

	  // Exit the sleep mode and select internal clock
	  uint8_t pwr_mgmt_1 = 0x00; // CLKSEL = 000, SLEEP = 0
	  HAL_I2C_Mem_Write(&hi2c1, MPU9265_I2C_ADDR << 1, MPU9265_PWR_MGMT_1, 1, &pwr_mgmt_1, 1, 100);
	  HAL_Delay(100);

	  // Enable Gyroscope and Accelerometer
	  uint8_t pwr_mgmt_2 = 0x00; // Enable all sensors
	  HAL_I2C_Mem_Write(&hi2c1, MPU9265_I2C_ADDR << 1, MPU9265_PWR_MGMT_2, 1, &pwr_mgmt_2, 1, 100);
	  HAL_Delay(100);

	  // Set Full Scale Range of Gyroscope (500dps)
	  uint8_t gyro_config = 0x08; // FS_SEL = 01
	  HAL_I2C_Mem_Write(&hi2c1, MPU9265_I2C_ADDR << 1, MPU9265_GYRO_CONFIG, 1, &gyro_config, 1, 100);
	  HAL_Delay(100);

	  // Set Full Scale Range of Accelerometer (2g)
	  uint8_t accel_config = 0x00; // AFS_SEL = 00
	  HAL_I2C_Mem_Write(&hi2c1, MPU9265_I2C_ADDR << 1, MPU9265_ACCEL_CONFIG, 1, &accel_config, 1, 100);
	  HAL_Delay(100);

	  // Set LPF for Gyroscope and Accelerometer (44 Hz)
	  uint8_t config = 0x03; // DLPF_CFG = 011
	  HAL_I2C_Mem_Write(&hi2c1, MPU9265_I2C_ADDR << 1, MPU9265_CONFIG, 1, &config, 1, 100);
	  HAL_Delay(100);

	  // Set Sample Read Register (to read data at 200Hz)
	  uint8_t smplrt_div = 0x04; //0x02 (400Hz)
	  HAL_I2C_Mem_Write(&hi2c1, MPU9265_I2C_ADDR << 1, MPU9265_SMPLRT_DIV, 1, &smplrt_div, 1, 100);
	  HAL_Delay(100);
}

void MPU9265_CalibrateGyro(float *gyroOffset) {
    int numSamples = 1000;  // Number of samples to read
    float temp[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < numSamples; ++i) {
        float gyroData[3];
        MPU9265_ReadGyro(gyroData); // read raw data
        temp[0] += gyroData[0];
        temp[1] += gyroData[1];
        temp[2] += gyroData[2];

        HAL_Delay(5); // wait 5 ms between samples
    }

    // mean of the read values and compute offset
    gyroOffset[0] = temp[0] / numSamples;
    gyroOffset[1] = temp[1] / numSamples;
    gyroOffset[2] = temp[2] / numSamples;
}

void MPU9265_ReadAccel(float* accelData)
{
  uint8_t rawData[6];
  int16_t accelRaw[3];

  HAL_I2C_Mem_Read(&hi2c1, MPU9265_I2C_ADDR<<1, MPU9265_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);

  accelRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
  accelRaw[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  accelRaw[2] = (int16_t)((rawData[4] << 8) | rawData[5]);

  // Conversion from raw to acceleration values
  accelData[0] = (float)accelRaw[0] / 16384.0f; // Sensitivity scale factor: ±2g range
  accelData[1] = (float)accelRaw[1] / 16384.0f;
  accelData[2] = (float)accelRaw[2] / 16384.0f;
}

void MPU9265_ReadGyro(float* gyroData)
{
  uint8_t rawData[6];
  int16_t gyroRaw[3];

  HAL_I2C_Mem_Read(&hi2c1, MPU9265_I2C_ADDR<<1, MPU9265_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);

  gyroRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
  gyroRaw[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  gyroRaw[2] = (int16_t)((rawData[4] << 8) | rawData[5]);

  // Conversion from raw to acceleration values
  gyroData[0] = (float)gyroRaw[0] / 65.5f; // Sensitivity scale factor: ±500dps range
  gyroData[1] = (float)gyroRaw[1] / 65.5f;
  gyroData[2] = (float)gyroRaw[2] / 65.5f;
}

void MPU9265_ReadAccelGyro(float* accelData, float* gyroData)
{
    uint8_t rawData[14];
    int16_t accelRaw[3], gyroRaw[3];

    HAL_I2C_Mem_Read(&hi2c1, MPU9265_I2C_ADDR<<1, MPU9265_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 14, HAL_MAX_DELAY);

    accelRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
    accelRaw[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
    accelRaw[2] = (int16_t)((rawData[4] << 8) | rawData[5]);

    gyroRaw[0] = (int16_t)((rawData[8] << 8) | rawData[9]);
    gyroRaw[1] = (int16_t)((rawData[10] << 8) | rawData[11]);
    gyroRaw[2] = (int16_t)((rawData[12] << 8) | rawData[13]);

    // Accel Data
    accelData[0] = (float)accelRaw[0] / 16384.0f; // Sensitivity scale factor: ±2g range
    accelData[1] = (float)accelRaw[1] / 16384.0f;
    accelData[2] = (float)accelRaw[2] / 16384.0f;

    // Gyro Data
    gyroData[0] = (float)gyroRaw[0] / 65.5f - OffsetGyro[0]; // Sensitivity scale factor: ±500dps range
    gyroData[1] = (float)gyroRaw[1] / 65.5f - OffsetGyro[1];
    gyroData[2] = (float)gyroRaw[2] / 65.5f - OffsetGyro[2];
}

/* Function to compute roll and pitch based on accelerometer data */
void MPU9265_ComputeRollPitch(float *accelData, float *roll, float *pitch){
	  uint8_t rawData[6];
	  int16_t accelRaw[3];

	  HAL_I2C_Mem_Read(&hi2c1, MPU9265_I2C_ADDR<<1, MPU9265_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);

	  accelRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
	  accelRaw[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
	  accelRaw[2] = (int16_t)((rawData[4] << 8) | rawData[5]);

	  // Conversion from raw to acceleration values
	  accelData[0] = (float)accelRaw[0] / 16384.0f; // Sensitivity scale factor: ±2g range
	  accelData[1] = (float)accelRaw[1] / 16384.0f;
	  accelData[2] = (float)accelRaw[2] / 16384.0f;

	*roll = atan2(accelData[1], sqrt(pow(accelData[0], 2) + pow(accelData[2], 2))) * 180.0f / M_PI; // roll in degrees
	*pitch = atan2(-accelData[0], sqrt(pow(accelData[1], 2) + pow(accelData[2], 2))) * 180.0f / M_PI; // pitch in degrees
}


/* Compute Roll, Pitch and Yaw with gyroscope */
void MPU9265_ComputeRollPitchYaw(float *gyroData, float dt, float *roll, float *pitch, float *yaw){
	  uint8_t rawData[6];
	  int16_t gyroRaw[3];

	  HAL_I2C_Mem_Read(&hi2c1, MPU9265_I2C_ADDR<<1, MPU9265_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, rawData, 6, HAL_MAX_DELAY);

	  gyroRaw[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
	  gyroRaw[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
	  gyroRaw[2] = (int16_t)((rawData[4] << 8) | rawData[5]);

	  // Conversion from raw to acceleration values
	  gyroData[0] = (float)gyroRaw[0] / 65.5f; // Sensitivity scale factor: ±500dps range
	  gyroData[1] = (float)gyroRaw[1] / 65.5f;
	  gyroData[2] = (float)gyroRaw[2] / 65.5f;

	// Updates roll, pitch and yaw
	*roll += gyroData[0]  * dt;
	*pitch +=  gyroData[1] * dt;
	*yaw += gyroData[2] * dt;

    // Keep roll angle in range [-180, 180]
    if (*roll > 180.0f) {
        *roll -= 360.0f;
    } else if (*roll < -180.0f) {
        *roll += 360.0f;
    }
    // Keep pitch angle in range [-180, 180]
    if (*pitch > 180.0f) {
        *pitch -= 360.0f;
    } else if (*pitch < -180.0f) {
        *pitch += 360.0f;
    }
    // Keep yaw angle in range [-180, 180]
    if (*yaw > 180.0f) {
        *yaw -= 360.0f;
    } else if (*yaw < -180.0f) {
        *yaw += 360.0f;
    }
}
