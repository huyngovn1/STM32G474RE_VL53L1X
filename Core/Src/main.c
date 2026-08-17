/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ADDR_OVERFLASH  (0x08080000) //512k
#define IMU_HEADING (0)
#define AVG_TIME (4)
#define GY_NOISE (8)
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "bmp3.h"
#include "QMC5883.h"
#include "ICM-42670.h"
#include "BSP.h"
//#include <VL53L0X.h>
#include <I2CDev.h>
#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO int countTest,IMU_Status,countImu;
int16_t Gyro[3], Acell[3], TempRaw,TempAvg , GyroOffset[3]={0,0,0};
int32_t GyroAvg[3], AcellAvg[3],TempSum;
int32_t AcellSum[3], GyroSum[3], AcellSum2[3], GyroSum2[3];
__IO float GyroLP[3],  AcellLP[3],ImuTempLP=0;
uint8_t ImuRegs[16], ImuData[16];
uint8_t RxBuffer1[256];
uint8_t Calib=0;
uint8_t Rx1Buffer[256];
uint8_t Rx2Buffer[256];
uint8_t Rx3Buffer[256];
uint8_t Rx4Buffer[256];
uint8_t Rx5Buffer[256];

uint8_t Tx1Buffer[256];
uint8_t Tx2Buffer[256];
uint8_t Tx3Buffer[256];
uint8_t Tx4Buffer[256];
uint8_t Tx5Buffer[256];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct bmp3_dev dev;
struct bmp3_data bmp3_comp_data;
int32_t AltiMmSum=0;
#define TOF_COUNT              5
#define VL53_DEFAULT_ADDR      0x52

#define TOF_READ_SLOT_MS       10

#define TOF_ADDR_1             0x54
#define TOF_ADDR_2             0x56
#define TOF_ADDR_3             0x58
#define TOF_ADDR_4             0x5A
#define TOF_ADDR_5             0x5C
//VL53L0X TofU, TofF;
//I2C_HandleTypeDef I2C;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
typedef struct
{
    I2C_HandleTypeDef *i2c;

    GPIO_TypeDef *xshut_port;
    uint16_t xshut_pin;

    uint16_t addr;

    volatile uint16_t distance_mm;
    volatile uint8_t range_status;
    volatile uint8_t read_status;
    volatile uint8_t online;
} TOF_Sensor_t;

TOF_Sensor_t tof[TOF_COUNT] =
{
    {
        &hi2c1,
        XSHUT1_GPIO_Port,
        XSHUT1_Pin,
        TOF_ADDR_1,
        0, 0, 0, 0
    },

    {
        &hi2c2,
        XSHUT2_GPIO_Port,
        XSHUT2_Pin,
        TOF_ADDR_2,
        0, 0, 0, 0
    },

    {
        &hi2c2,
        XSHUT2B_GPIO_Port,
        XSHUT2B_Pin,
        TOF_ADDR_3,
        0, 0, 0, 0
    },

    {
        &hi2c3,
        XSHUT3_GPIO_Port,
        XSHUT3_Pin,
        TOF_ADDR_4,
        0, 0, 0, 0
    },

    {
        &hi2c3,
        XSHUT3B_GPIO_Port,
        XSHUT3B_Pin,
        TOF_ADDR_5,
        0, 0, 0, 0
    }
};

volatile uint8_t tof_init_error = 0;
volatile uint8_t tof_online_count = 0;
volatile uint8_t tof_current_index = 0;

uint32_t tof_read_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
__IO uint32_t CountTest;
int16_t x_mag,y_mag,z_mag;
uint16_t TestVal16;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct bmp3_dev dev;

int8_t rslt = BMP3_OK;

/* Sensor_0 interface over SPI with native chip select line */
uint8_t dev_addr = 0;


void user_delay_us(uint32_t period, void *intf_ptr)
{    uint32_t delay=period*16;
     for (uint32_t i=0;i<delay;i++);
}

uint8_t spi2Transfer(SPI_TypeDef *SPIx, uint8_t data)
{
   // 1. Chờ cho đến khi bộ đệm truyền trống (TXE = 1)
   while( !(SPIx->SR & SPI_SR_TXE) );

   // 2. Ép kiểu ghi 8-bit vào DR để kích hoạt bộ phát (Tránh kích hoạt nhầm 16-bit FIFO)
   *(__IO uint8_t *)&SPIx->DR = data;

   // 3. Chờ cho đến khi nhận đủ 1 byte dữ liệu về (RXNE = 1)
   // Phải đợi RXNE trước khi đợi BSY!
   while( !(SPIx->SR & SPI_SR_RXNE) );

   // 4. Đọc dữ liệu ra từ DR (Thao tác này tự động xóa cờ RXNE)
   return (*(__IO uint8_t *)&SPIx->DR);
}

int8_t  user_spi_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    delay(15);
	BMP_CS0;
    delay(15);

    spi2Transfer(SPI2, reg_addr | 0x80);
	for( uint16_t i=0;i<len;i++) read_data[i] = spi2Transfer(SPI2, 0x00);
	BMP_CS1;
    return 0;
}


int8_t  user_spi_write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len,
                                                void *intf_ptr)
{

	delay(15);
		BMP_CS0;
                delay(15);
                spi2Transfer(SPI2, reg_addr);          // Device Reset
		for( uint16_t i=0;i<len;i++) spi2Transfer(SPI2, read_data[i]);
		BMP_CS1;
    return 0;
}

//float getAltitude(double press, double temp) {
//  return ((pow((1013.25 / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
//}

struct bmp3_data bmp3_comp_data;

uint32_t PageErrorFlash = 0;
void WriteFLASH_all_Parameters(void){
    FLASH_EraseInitTypeDef EraseInitStruct;
        // ----------------------------------------------------------------------
        // PHẦN 1: NGỪNG TOÀN BỘ HỆ THỐNG
        // ----------------------------------------------------------------------
    __disable_irq();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Unlock();
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page      = 127;
    EraseInitStruct.NbPages   = 1;
    EraseInitStruct.Banks     = FLASH_BANK_1 ;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageErrorFlash) != HAL_OK) {
        //CountError = HAL_FLASH_GetError();
    	while(1);
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,ADDR_OVERFLASH-8 ,0x01) != HAL_OK) {
        	PageErrorFlash = HAL_FLASH_GetError();
    }

    HAL_FLASH_Lock();
    __enable_irq();
}
static void TOF_All_XSHUT_Off(void)
{
    for (uint8_t i = 0; i < TOF_COUNT; i++)
    {
        HAL_GPIO_WritePin(
            tof[i].xshut_port,
            tof[i].xshut_pin,
            GPIO_PIN_RESET
        );
    }

    HAL_Delay(50);
}

static uint8_t TOF_Init_Fail(uint8_t i, uint8_t error)
{
    tof[i].online = 0;
    tof[i].distance_mm = 0;
    tof[i].read_status = error;

    HAL_GPIO_WritePin(
        tof[i].xshut_port,
        tof[i].xshut_pin,
        GPIO_PIN_RESET
    );

    return error;
}

static uint8_t TOF_Init_One(uint8_t i)
{
    uint8_t status;
    uint8_t boot_state = 0;
    uint8_t model_id = 0;
    uint32_t timeout_tick;

    /* Chọn I2C của cảm biến */
    VL53L1_SelectI2C(tof[i].i2c);

    /* Bật riêng cảm biến */
    HAL_GPIO_WritePin(
        tof[i].xshut_port,
        tof[i].xshut_pin,
        GPIO_PIN_SET
    );

    HAL_Delay(10);

    timeout_tick = HAL_GetTick();

    /* Chờ cảm biến khởi động */
    while (boot_state == 0)
    {
        status = VL53L1X_BootState(
            VL53_DEFAULT_ADDR,
            &boot_state
        );

        if (status != 0)
        {
            return TOF_Init_Fail(i, 10 + i);
        }

        if ((HAL_GetTick() - timeout_tick) > 1000)
        {
            return TOF_Init_Fail(i, 20 + i);
        }

        HAL_Delay(2);
    }

    /* Kiểm tra Model ID */
    status = VL53L1_RdByte(
        VL53_DEFAULT_ADDR,
        VL53L1__MODELID_INDEX,
        &model_id
    );

    if ((status != 0) ||
        (model_id != VL53L1__MODELID_VALUE))
    {
        return TOF_Init_Fail(i, 30 + i);
    }

    status = VL53L1X_SensorInit(VL53_DEFAULT_ADDR);

    if (status != 0)
    {
        return TOF_Init_Fail(i, 40 + i);
    }

    status = VL53L1X_SetDistanceMode(
        VL53_DEFAULT_ADDR,
        2
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 50 + i);
    }

    status = VL53L1X_SetTimingBudgetInMs(
        VL53_DEFAULT_ADDR,
        50
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 60 + i);
    }

    status = VL53L1X_SetInterMeasurementInMs(
        VL53_DEFAULT_ADDR,
        60
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 70 + i);
    }

    /* Đổi sang địa chỉ riêng */
    status = VL53L1X_SetI2CAddress(
        VL53_DEFAULT_ADDR,
        tof[i].addr
    );

    if (status != 0)
    {
        return TOF_Init_Fail(i, 80 + i);
    }

    HAL_Delay(5);

    status = VL53L1X_StartRanging(tof[i].addr);

    if (status != 0)
    {
        return TOF_Init_Fail(i, 90 + i);
    }

    tof[i].online = 1;
    tof[i].read_status = 0;
    tof[i].distance_mm = 0;

    return 0;
}

static uint8_t TOF_Init_All(void)
{
    uint8_t first_error = 0;

    tof_online_count = 0;
    tof_current_index = 0;

    TOF_All_XSHUT_Off();

    for (uint8_t i = 0; i < TOF_COUNT; i++)
    {
        tof[i].online = 0;
        tof[i].distance_mm = 0;
        tof[i].range_status = 0;
        tof[i].read_status = 0;

        uint8_t error = TOF_Init_One(i);

        if (error == 0)
        {
            tof_online_count++;
        }
        else if (first_error == 0)
        {
            first_error = error;
        }
    }

    return first_error;
}

static void TOF_Read_One_By_One(void)
{
    uint8_t ready = 0;
    uint8_t range_status = 0;
    uint16_t distance = 0;

    uint8_t i = tof_current_index;

    if ((HAL_GetTick() - tof_read_tick) < TOF_READ_SLOT_MS)
    {
        return;
    }

    tof_read_tick = HAL_GetTick();

    if (tof[i].online == 1)
    {
        VL53L1_SelectI2C(tof[i].i2c);

        tof[i].read_status =
            VL53L1X_CheckForDataReady(
                tof[i].addr,
                &ready
            );

        if ((tof[i].read_status == 0) &&
            (ready != 0))
        {
            tof[i].read_status =
                VL53L1X_GetRangeStatus(
                    tof[i].addr,
                    &range_status
                );

            tof[i].range_status = range_status;

            if ((tof[i].read_status == 0) &&
                (range_status <= 2))
            {
                tof[i].read_status =
                    VL53L1X_GetDistance(
                        tof[i].addr,
                        &distance
                    );

                if (tof[i].read_status == 0)
                {
                    tof[i].distance_mm = distance;
                }
            }

            VL53L1X_ClearInterrupt(tof[i].addr);
        }
    }

    tof_current_index++;

    if (tof_current_index >= TOF_COUNT)
    {
        tof_current_index = 0;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USB_Device_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

  IMU_Status++;
  WriteFLASH_all_Parameters();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)Rx1Buffer, 256);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Rx2Buffer, 256);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)Rx3Buffer, 256);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, (uint8_t *)Rx4Buffer, 256);
  __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t *)Rx5Buffer, 256);
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);


  // 1. Bật Channel 1 của TIM1
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);



  HAL_Delay(200);
  tof_init_error = TOF_Init_All();
  IMU_Status=2;
  M8N_GPS_Config();
  IMU_Status=3;
  QMC5883_Init();
//  SPI1->CR1 |= SPI_CR1_SPE;
   IMU_Status=4;

   HAL_Delay(100);
   //HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin,GPIO_PIN_SET);
 //  HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin,GPIO_PIN_SET);
 //  HAL_Delay(24);

//   I2C=hi2c2;
//   setup_VL53L0X(&TofU); //Set up the VL53L0X struct with default address and timeout status.
////   //changeAddresses(); //Reset the sensor and initialise it. Then change its address to something unique from other sensors.
////
//   init(&TofU,true); //attempt to initialise it with the necessary settings for normal operation. Returns 0 if fail, 1 if success.
////
//   setTimeout(&TofU,500);
////
//   setVcselPulsePeriod(&TofU,VcselPeriodPreRange, 14);    //  VcselPeriodPreRange:  12 to 18 (initialized default: 14)
//   setVcselPulsePeriod(&TofU,VcselPeriodFinalRange, 10);  //  VcselPeriodFinalRange: 8 to 14 (initialized default: 10)
////
//   startContinuous(&TofU,0); //start the sensor in continuous back-to-back reading mode.

 //  #if defined LONG_RANGE
 //  // lower the return signal rate limit (default is 0.25 MCPS)
 //   setSignalRateLimit(&TofU,0.25);
 //  #endif
 //
 //  #if defined HIGH_SPEED
 //   // reduce timing budget to 20 ms (default is about 33 ms)
 //   setMeasurementTimingBudget(&TofU,20000);
 //  #elif defined HIGH_ACCURACY
 //          // increase timing budget to 200 ms
 //   setMeasurementTimingBudget(&TofU,200000);
 //  #endif


   IMU_Status=5;
   LL_SPI_Enable(SPI2);

	dev.intf_ptr = &dev_addr;
	dev.intf = BMP3_SPI_INTF;
	dev.read = &user_spi_read;
	dev.write = &user_spi_write;
	dev.delay_us = &user_delay_us;
	dev.dummy_byte = 1;
	//
	//  dev.chip_id = cs_pin;
	//  dev.intf_ptr = spi_dev;


	rslt=bmp3_init(&dev);


	if (rslt != BMP3_OK)
	{
	//sendUsart1("BMP_Error\n");
	} //else sendUsart1("BMP_initOK\n" );
	//sprintf(temUsart, "BMP_OK: %d\n", dev.chip_id); sendUsart1(temUsart);

	// Configure settings for drone use case as per datasheet
	dev.settings.press_en = BMP3_ENABLE;
	dev.settings.temp_en = BMP3_ENABLE;
	dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
	dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_1;
	dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
	bmp3_set_sensor_settings(BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER | BMP3_SEL_ODR, &dev);

	// Set the power mode to normal mode
	dev.settings.op_mode = BMP3_MODE_NORMAL;
	bmp3_set_op_mode(&dev);


	IMU_Status=6;
  ICM42670_Init();
  IMU_Status=7;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  bmp3_get_sensor_data(BMP3_ALL, &bmp3_comp_data, &dev); //10Hz
	    TOF_Read_One_By_One();
	    HAL_Delay(20);
	 				      AltiMmSum= (int32_t)(getAltitude(bmp3_comp_data.pressure/ 100.,
	 				    		  bmp3_comp_data.temperature/ 100.)*1000.0f) ;

//	  if( SPI_ByteRead(DATA_RDY_INT)){ //1600Hz
//			  SPI_BytesRead(0x20,16,ImuRegs);

		//	  SPI_BytesRead(0x09,16,ImuData);

		//	  TempRaw= (ImuData[0]<<8)|ImuData[1];
		//	  ImuTempLP+=(((float)TempRaw/128)+25-ImuTempLP)*0.01f;

		//	  int16_t AcellRaw[3],GyroRaw[3];
//
		//	  AcellRaw[0]= (ImuData[2]<<8)|ImuData[3];
		//	  AcellRaw[1]= (ImuData[4]<<8)|ImuData[5];
		//	  AcellRaw[2]= (ImuData[6]<<8)|ImuData[7];
//
	//		  GyroRaw[0]= (ImuData[8]<<8)|ImuData[9];
		//	  GyroRaw[1]= (ImuData[10]<<8)|ImuData[11];
			//  GyroRaw[2]= (ImuData[12]<<8)|ImuData[13];
//
	//		  Acell[0]= AcellRaw[1];
		//	  Acell[1]= AcellRaw[0];
			//  Acell[2]= -AcellRaw[2];
//
	//		  Gyro[0]= -GyroRaw[1];
		//	  Gyro[1]= -GyroRaw[0];
		//  Gyro[2]= -GyroRaw[2];
//
	//		   if(++countImu==AVG_TIME){ //AVG_TIME 400Hz
		//		   TempAvg=(TempSum/AVG_TIME);
			//	   AcellAvg[0]=AcellSum[0]/AVG_TIME;
				//   AcellAvg[1]=AcellSum[1]/AVG_TIME;
				  // AcellAvg[2]=AcellSum[2]/AVG_TIME;
//
	//			   GyroAvg[0]=GyroSum[0]/AVG_TIME;
		//		   GyroAvg[1]=GyroSum[1]/AVG_TIME;
			//	   GyroAvg[2]=GyroSum[2]/AVG_TIME;
//
//
	//			   TempSum=0;
		//		   AcellSum[0]=0;  AcellSum[1]=0;  AcellSum[2]=0;
			//	   GyroSum[0]=0;  GyroSum[1]=0;  GyroSum[2]=0;
				//   countImu=0;
			  // }
			   //if(++IMU_Status%40==0){
//
	//			      QMC5883_Get_Values(&x_mag,&y_mag,&z_mag); //50Hz
		//			  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//					  CountTest++;
			//   }
	  //}
//	  HAL_Delay(50);
//      bmp3_get_sensor_data(BMP3_ALL, &bmp3_comp_data, &dev); //10Hz
//      AltiMmSum= (int32_t)(getAltitude(bmp3_comp_data.pressure/ 100.0, bmp3_comp_data.temperature/ 100.)*1000.0f) ;
//				      QMC5883_Get_Values(&x_mag,&y_mag,&z_mag); //50Hz
//					  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//	  CountTest++;

	  //if(CountTest>=44 && CountTest<=60) {
		//  LL_TIM_OC_SetCompareCH1(TIM1, IMU_Status*100);
		  //LL_TIM_OC_SetCompareCH2(TIM1, IMU_Status*100);
		  //LL_TIM_OC_SetCompareCH3(TIM1, IMU_Status*100);
		  //LL_TIM_OC_SetCompareCH4(TIM1, IMU_Status*100);
	  //}
//
//	  TestVal16=readRangeContinuousMillimeters(&TofU);
//
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x60505F8C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x60505F8C;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x60505F8C;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x60505F8C;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI2);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 41;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 4400;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_ENABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**TIM1 GPIO Configuration
  PC0   ------> TIM1_CH1
  PC1   ------> TIM1_CH2
  PC2   ------> TIM1_CH3
  PC3   ------> TIM1_CH4
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 41;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 12011;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB0   ------> TIM3_CH3
  PB4   ------> TIM3_CH1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 250000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 420000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|LED2_Pin|XSHUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, XSHUT2_Pin|XSHUT2B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, XSHUT3_Pin|XSHUT3B_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_BMP_GPIO_Port, CS_BMP_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_Pin LED2_Pin XSHUT1_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED2_Pin|XSHUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT2_Pin XSHUT2B_Pin */
  GPIO_InitStruct.Pin = XSHUT2_Pin|XSHUT2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT3_Pin XSHUT3B_Pin PB8 */
  GPIO_InitStruct.Pin = XSHUT3_Pin|XSHUT3B_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_BMP_Pin */
  GPIO_InitStruct.Pin = CS_BMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_BMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1){
//        HAL_HalfDuplex_EnableReceiver(huart);
//        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)Rx4Buffer, 256);
//    	__HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
//        CountTest++;
    }


}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	 if(huart->Instance == USART1){
		 if(Rx1Buffer[0]==0 &&Size==18){

		 }
	 }

	 if(huart->Instance == USART2){ // ELRS PORT
		 if(Rx2Buffer[0]==0 &&Size==18){

		 }

	 }

	 if(huart->Instance == USART3){ // GPS
		 if(Rx3Buffer[0]==0 &&Size==18){

		 }

	 }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 1. Lưu mã lỗi và Xóa cờ lỗi phần cứng ngay lập tức
    __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);

    // Đọc RDR để xóa hoàn toàn lỗi Overrun (đặc biệt quan trọng với dòng F4, F7, H7)
    volatile uint32_t temp = huart->Instance->RDR;
    (void)temp;

    // 2. Chỉ dừng bộ nhận (RX), giữ nguyên bộ truyền (TX)
    HAL_UART_AbortReceive_IT(huart);

    // 3. Reset các biến liên quan đến lỗi RX
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxXferCount = 0;

    // 4. Khởi động lại RX cho từng cổng
    if (huart->Instance == USART1) { // Full-Duplex

        // Không gọi HAL_HalfDuplex_EnableReceiver ở đây
        HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)Rx1Buffer, 256);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }

//    CountError++;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
