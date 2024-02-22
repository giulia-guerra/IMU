/*
 * IMU_MPU6050.c
 *
 *  Created on: Feb 20, 2024
 *      Author: giuby
 */

#include "IMU_MPU6050.h"
#include "string.h"
#include "stdio.h"

float g = 9.80665;
float Acc_LSB_Sensitivity = 8192.0;
float Gyro_LSB_Sensitivity = 65.5;

uint8_t IMU_MPU6050_Init(void) {
    uint8_t Data;
    HAL_StatusTypeDef ret;

    // Controlla la disponibilità del dispositivo
    ret = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 2, 1000);
    if (ret != HAL_OK) {
        return -1;
    }

    // Verifica che il dispositivo sia quello atteso
    uint8_t check;
    ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
    if ((ret != HAL_OK) || (check != 0x68)) {
        return -1;
    }

	Data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

	Data = 0x08;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	Data = 0x08;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

	Data = 0x31;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

	Data = 0x40;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

	Data = 0x78;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000);

	Data = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1, 1000);

	Data = 0x01;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1, 1000);

	return 0;
}

void IMU_MPU6050_Read_Acc_Gyro(MPU6050_Data *y){
	uint8_t Data[12];
	uint16_t counts= Read_FIFO_Count();
	printf("count %d\r\n", counts);
	int16_t app = 0;
	if(counts>=12){
		HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_R_W_REG, 1, Data, 12, 1000);
		if (ret == HAL_OK) {
		    int16_t app;

		    app = (int16_t)((Data[0] << 8) | Data[1]);
		    y->Ax_raw = (app / Acc_LSB_Sensitivity) * g;

		    app = (int16_t)((Data[2] << 8) | Data[3]);
		    y->Ay_raw = (app / Acc_LSB_Sensitivity) * g;

		    app = (int16_t)((Data[4] << 8) | Data[5]);
		    y->Az_raw = (app / Acc_LSB_Sensitivity) * g;

		    app = (int16_t)((Data[6] << 8) | Data[7]);
		    y->Wx_raw = (app / Gyro_LSB_Sensitivity) * g;

		    app = (int16_t)((Data[8] << 8) | Data[9]);
		    y->Wy_raw = (app / Gyro_LSB_Sensitivity) * g;

		    app = (int16_t)((Data[10] << 8) | Data[11]);
		    y->Wz_raw = (app / Gyro_LSB_Sensitivity) * g;
		} else {
		    printf("Error I2C\r\n");
		    switch (ret) {
		        case HAL_ERROR:
		            printf("HAL_ERROR \r\n");
		            break;
		        case HAL_BUSY:
		            printf("HAL_BUSY \r\n");
		            break;
		        case HAL_TIMEOUT:
		            printf("HAL_TIMEOUT \r\n");
		            break;
		        default:
		            // Gestisci il caso HAL_OK
		            break;
		    }
		}
		if(counts>12){
			printf("Delay \r\n");
			printf("Reset FIFO buffer \r\n");
			Reset_Reable_FIFO();
		}
	}
}

uint16_t Read_FIFO_Count(){
	HAL_StatusTypeDef ret;
	uint16_t fifo_count = 0;
	uint8_t Rec_Data[2];
	ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNT_H_REG, 1, Rec_Data, 1, 1000);
	ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, FIFO_COUNT_L_REG, 1, Rec_Data+1, 1, 1000);
	fifo_count = (uint16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	return fifo_count;
}

void Reset_Reable_FIFO(){
	HAL_StatusTypeDef ret;
	uint8_t Data = 0x04;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

	Data = 0x40;
	ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);
}

