/*
 * mpu6050.h
 *
 *  Created on: Nov 29, 2025
 *      Author: LENOVO
 */

#ifndef SRC_MPU6050_H_
#include <stdint.h>
#include "math.h"
#include "stm32f4xx_hal.h"
#define SRC_MPU6050_H_

#define MPU6050_ADDR 		0xD0U
#define PWR_MGMT_1 			0x6BU
#define SMPRT_DIV			0x19U
#define ACCEL_CONFIG_REG	0x1CU
#define ACCEL_XOUT_H 		0x3BU
#define GYRO_CONFIG_REG		0x1BU
#define GYRO_XOUT_H			0x43U
#define WHO_AM_I_REG		0x75U

#define RAD_TO_DEG 			57.2957795

typedef struct
{
	// comment not use variable
//	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	float GYRO_X;
//	int16_t GYRO_Y;
//	int16_t GYRO_Z;
    int16_t Accel_Y_Offset;
    int16_t Accel_Z_Offset;
    float GYRO_X_Offset;
}MPUData;


void MPU6050_Init(I2C_HandleTypeDef *handle);
void MPU6050_Read_Accel(I2C_HandleTypeDef *handle, volatile MPUData *d);
void MPU6050_Read_Data(I2C_HandleTypeDef *handle, volatile MPUData *d);
float MPU6050_Calib(I2C_HandleTypeDef *handle, MPUData *o);
//void MPU6050
#endif /* SRC_MPU6050_H_ */
