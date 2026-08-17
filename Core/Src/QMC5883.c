#include "main.h"
#include "QMC5883.h"
extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c1;

void DelayHMC(__IO uint32_t num)
{
	__IO uint32_t index = 0;

	/* default system clock is 72MHz */
	for(index = (72*num); index != 0; index--)
	{
	}
}

void QMC5883_Init(void)
{
   uint8_t RegCon1 = 0x19;  // 100HZ - 8G
   uint8_t Reg1BH = 0x01;
  
    while(HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR, 0x0B, 1, &Reg1BH , 1, 100)!=HAL_OK) HAL_Delay(10);
    while(HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR, 0x09, 1, &RegCon1 , 1, 100)!=HAL_OK) HAL_Delay(10);
}


void QMC5883_Get_Values(signed short int *x_mag, signed short int *y_mag, signed short int *z_mag)
{
        uint8_t Buffer_Tx1[10];
        uint8_t Buffer_Rx1[10];
	Buffer_Tx1[0] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, QMC5883L_ADDR, Buffer_Tx1, 1, 50);//while(HAL_I2C_Master_Transmit(&hi2c1, HMC5883_ADDR, Buffer_Tx1, 1, 1000)!= HAL_OK)if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)  Error_Handler();//I2C_Master_BufferWrite(I2C1, Buffer_Tx1,1,Polling, HMC5883_ADDR);
	DelayHMC(5);
	HAL_I2C_Master_Receive(&hi2c1, QMC5883L_ADDR, Buffer_Rx1, 6, 50);//I2C_Master_BufferRead(I2C1,Buffer_Rx1,6,Polling, HMC5883_ADDR);  
	*x_mag = Buffer_Rx1[0] | (Buffer_Rx1[1] << 8);
	*y_mag = Buffer_Rx1[2] | (Buffer_Rx1[3] << 8);
	*z_mag = Buffer_Rx1[4] | (Buffer_Rx1[5] << 8);
}