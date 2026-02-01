
#include "mpu6050.h"

void MPU6050_Init(I2C_HandleTypeDef *handle){
	uint8_t check;
	uint8_t data;
	HAL_I2C_Mem_Read (handle, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	if (check == 0x68){
		data = 0;
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);
		//HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pdata, uint16_t Size, uint32_t Timeout)
		HAL_I2C_Mem_Write(handle, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 1000);
		data = 0x07;
		HAL_I2C_Mem_Write(handle, MPU6050_ADDR, SMPRT_DIV, 1, &data, 1, 1000);
		data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> <strong>±</strong> 2g
		HAL_I2C_Mem_Write(handle, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
		// Set Gyroscopic configuration in GYRO_CONFIG Register
		data = 0x00;  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> <strong>±</strong> 250 ̐/s
		HAL_I2C_Mem_Write(handle, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}
	else
	{
		HAL_Delay(100);
		HAL_I2C_Mem_Read (handle, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
		if (check == 0x68)
			MPU6050_Init(handle);
	}
}
void MPU6050_Read_Accel(I2C_HandleTypeDef *handle, volatile MPUData *d)
{
	uint8_t Rec_data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (handle, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_data, 6, 1000);
//	d->Accel_X_RAW = (int16_t)(Rec_data[0] << 8 | Rec_data [1]);
	d->Accel_Y_RAW = (int16_t)(Rec_data[2] << 8 | Rec_data [3]);
	d->Accel_Z_RAW = (int16_t)(Rec_data[4] << 8 | Rec_data [5]);
}
void MPU6050_Read_Gyro(I2C_HandleTypeDef *handle, volatile MPUData *d)
{
	uint8_t Rec_data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (handle, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_data, 6, 1000);
	d->GYRO_X = (int16_t)(Rec_data[0] << 8 | Rec_data [1]);
//	d->GYRO_Y = (int16_t)(Rec_data[2] << 8 | Rec_data [3]);
//	d->GYRO_Z = (int16_t)(Rec_data[4] << 8 | Rec_data [5]);
	d->GYRO_X = ((float_t)d->GYRO_X) / 131.0;
//	d->GYRO_Y = (float_t)d->GYRO_Y / 131.0;
//	d->GYRO_Z = (float_t)d->GYRO_Z / 131.0;
}

void MPU6050_Read_Data(I2C_HandleTypeDef *handle, volatile MPUData *d)
{
	MPU6050_Read_Gyro(handle, d);
	MPU6050_Read_Accel(handle, d);
}

float MPU6050_Calib(I2C_HandleTypeDef *handle, MPUData *o){

    // Simple calibration: assumes robot is held upright at start
	long sumY = 0, sumZ = 0;
    int32_t sum = 0;
    const int N = 200;

    for (int i = 0; i < N; i++)
    {
        MPU6050_Read_Data(handle, o);
        sum += o->GYRO_X;
        sumY += o->Accel_Y_RAW;
        sumZ += o->Accel_Z_RAW;
        HAL_Delay(2);   // ~1–2kHz là ổn
    }

    o->GYRO_X_Offset = (float)sum / N;
    o->Accel_Y_Offset = (float)sumY / N;
    o->Accel_Z_Offset = (float)sumZ / N;
    return atan2(
         o->Accel_Y_Offset,
        o->Accel_Z_Offset
    ) * RAD_TO_DEG;
}
