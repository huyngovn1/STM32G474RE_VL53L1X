///////////////////////////////////////////////////////////////////////////////
// MPU6000 Defines and Variables
///////////////////////////////////////////////////////////////////////////////
#include "BMP388.h"
extern float TempGyroDrift[];
uint32_t CountSensorError;
void delay(__IO uint32_t num)
{
	__IO uint32_t index = 0;

	/* default system clock is 72MHz */
	for(index = (8*num); index != 0; index--)
	{
	}
}

//uint8_t spiTransfer(SPI_TypeDef *SPIx, uint8_t data)
//{
//    while ((SPI1->SR & SPI_SR_TXE) == 0){};
//    SPI1->DR = data;
//    /* Wait until it's 1, so we can read out ==> Receiver buffer not empty*/
//    while ((SPI1->SR & SPI_SR_RXNE) == 0){};
//    return SPI1->DR;
//}

uint8_t spiTransfer(SPI_TypeDef *SPIx, uint8_t data)
{   
   while( !(SPI1->SR & SPI_FLAG_TXE) ); // wait until transmit complete
  *(uint8_t *)&(SPI1->DR) = data;//SPI1->DR = data; // write data to be transmitted to the SPI data register
   while( SPI1->SR & SPI_FLAG_BSY ); // wait until SPI is not busy anymore
   while( !(SPI1->SR & SPI_FLAG_RXNE) ); // wait until receive complete
   return ((uint8_t)SPI1->DR); // return received data from SPI data register
}

void ICM20689_Init(void)
{	
		unsigned char data;

		delay(150000); 
             
		SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_PWR_MGMT_1, 0x80);		// Reset MPU6000
                delay(150000);   
                
                SPI_ByteWrite(MPU6000_SPI, MPU6000_USER_CTRL, BIT_I2C_IF_DIS);  // Dis I2C
                
                delay(1000); 
                SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_PWR_MGMT_1, 0x01);		 // Config clock
		delay(1000); 

		SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_PWR_MGMT_2, 0x00);	
				
		delay(1000); 
		SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_CONFIG, 0x07);  //LP	 -8Khz Sample Rate
                
               do{
                delay(1000);
		SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_GYRO_CONFIG, BITS_FS_2000DPS);
                delay(1000);
		data = SPI_ByteRead(MPU6000_RA_GYRO_CONFIG);
               }   while (data != BITS_FS_2000DPS); 
               
                do{
               	  delay(1000);
                  SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_ACCEL_CONFIG, BITS_FS_8G);
		  data = SPI_ByteRead(MPU6000_RA_ACCEL_CONFIG);
		}while (data != BITS_FS_8G);
                
               
                do{
                delay(1000);		   
		 SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_ACCEL_CONFIG2, 0xC6);  //  FIFO4K  0xC4:1Khz Acell Rate, 0xC8:4Khz
                 data = SPI_ByteRead(MPU6000_RA_ACCEL_CONFIG2);
               }   while (data != 0xC6); 

		delay(1000);		   
		SPI_ByteWrite(MPU6000_SPI, MPU6000_RA_SMPLRT_DIV, 0x00); while (SPI_ByteRead(MPU6000_RA_SMPLRT_DIV)!=0x00);  /// Sample Rate=8Khz ---- Output =8Khz
                
                delay(1000);
		SPI_ByteWrite(MPU6000_SPI, MPU6000_FIFO_EN, 0xF8);	while (SPI_ByteRead(MPU6000_FIFO_EN)!=0xF8);  //  FIFO END Accel, Temp,GYRO  0xF8

		delay(1000);
	        SPI_ByteWrite(MPU6000_SPI,0x37, 0x30);	 while (SPI_ByteRead(0x37)!=0x30) ;// Hold INT pin to FIFO
	        delay(1000); 
		SPI_ByteWrite(MPU6000_SPI, 0x38, 0x10);	  while (SPI_ByteRead(0x38)!=0x10) ;// IntPin FIFO Over	En
 
	        delay(3000); 
		SPI_ByteWrite(MPU6000_SPI, MPU6000_USER_CTRL, BIT_I2C_IF_DIS |0x45);  // Dis I2C_En FIFO_ RST FIFO
		delay(3000); 
}

void SPI_ByteWrite(SPI_TypeDef *SPIx, uint8_t Reg, uint8_t data)
{		delay(15);
		BMP_CS0;
                delay(15);
		spiTransfer(MPU6000_SPI, Reg);          // Device Reset
		spiTransfer(MPU6000_SPI, data);
		BMP_CS1;
		
}
	

unsigned char SPI_ByteRead(uint8_t Reg)
{
		unsigned char data;
		delay(15);
		BMP_CS0;
                delay(15);
		spiTransfer(MPU6000_SPI, Reg | MPU6000_SPI_READ);
		data = spiTransfer(MPU6000_SPI, 0x00);
		BMP_CS1;
		return data;	
}



void SPI_BytesRead(uint8_t Reg,uint16_t len, uint8_t* data)
{   uint16_t i;
        delay(15);
	BMP_CS0;
        delay(15);
	spiTransfer(MPU6000_SPI, Reg | MPU6000_SPI_READ);
	for( i=0;i<len;i++) data[i] = spiTransfer(MPU6000_SPI, 0x00);
	BMP_CS1;

}


uint16_t  MPUgetFIFOCount(void){  
uint8_t FIFOCount[2];
	 SPI_BytesRead(0x72,2,FIFOCount);								
 	 return ((uint16_t)((((uint16_t)FIFOCount[0]&0x001F)<<8)  |FIFOCount[1]));
}

uint16_t  MPUresetFIFO(void){
   SPI_ByteWrite(MPU6000_SPI, MPU6000_FIFO_EN, 0); 
//   SPI_ByteWrite(MPU6000_SPI, MPU6000_USER_CTRL, BIT_I2C_IF_DIS |0x01);  // Dis I2C--DIS FIFO _RST SIG_COND 
//   SPI_ByteWrite(MPU6000_SPI, MPU6000_USER_CTRL, BIT_I2C_IF_DIS |0x04 |0x40);  // Dis I2C--EN _RST FIFO 
   SPI_ByteWrite(MPU6000_SPI, MPU6000_USER_CTRL, BIT_I2C_IF_DIS|0x05);  // Reset FIFO--SIG_COND_RST
   SPI_ByteWrite(MPU6000_SPI, MPU6000_USER_CTRL, BIT_I2C_IF_DIS |0x40);  // Dis I2C--EN FIFO 
   SPI_ByteWrite(MPU6000_SPI, MPU6000_FIFO_EN, 0xF8);
   return 0;
}

uint8_t MPU6000_Get_FIFO_Packets (int16_t *AccelAvg,int16_t *GyroAvg,float *temp)
{ 	
         uint8_t FIFO_Buf[224];
	 int32_t SumG=0;
	 uint8_t i;
         uint16_t mpuIntStatus =MPUgetIntStatus();
	 uint16_t fifoCount = MPUgetFIFOCount();
	 if((mpuIntStatus & 0x10) ||(fifoCount>=4096)){
	    MPUresetFIFO();
            CountSensorError++;
	    //USART1->TDR = 'F';  
	    return 2; 
	 }
	 else if(fifoCount>=224){ 
		  //while (fifoCount < 168 ){   fifoCount = MPUgetFIFOCount(); }
		  SPI_BytesRead(0x74,224,FIFO_Buf);
                  
                  if((FIFO_Buf[3]!=FIFO_Buf[17]) ||(FIFO_Buf[1]!=FIFO_Buf[15]) || (FIFO_Buf[5]!=FIFO_Buf[19])){  
                      //USART1->DR = 'S'; 
                    // USART1->TDR = 'S';
                     CountSensorError++;
                      MPUresetFIFO(); 
                      return 3; 
                  }
                  
                  
                  SumG=0;
                  for(i=0;i<8;i++)SumG+= ((int16_t)FIFO_Buf[(i*28)+6]<<8) |FIFO_Buf[(i*28)+7]; //4Khz //for(i=0;i<6;i++)SumG+= (int16_t)(((int16_t)FIFO_Buf[(i*28)+6]<<8)  |FIFO_Buf[(i*28)+7]); 
                  *temp=(float)(SumG/8)/326.8f  + 25.0f;	 
//                  if(((*temp)<(-50.0f))||((*temp)>80.0f)){ // Detect Error FIFO
//                     //if(TEST_MODE)USART1->DR = 'F';
//                    // MPUresetFIFO(); 
//                     return 4; 
//                  } 

		  SumG=0; for(i=0;i<2;i++)SumG+= (int16_t)(((int16_t)FIFO_Buf[(i*112)+0]<<8)  |FIFO_Buf[(i*112)+1]); 
		  AccelAvg[0]=SumG/2;
                  
                 // if((FIFO_Buf[0]!=FIFO_Buf[14])||(FIFO_Buf[1]!=FIFO_Buf[15])||(FIFO_Buf[4]!=FIFO_Buf[18])||(FIFO_Buf[5]!=FIFO_Buf[19]))USART1->TDR = 'F';

		  SumG=0; for(i=0;i<2;i++)SumG+= (int16_t)(((int16_t)FIFO_Buf[(i*112)+2]<<8)  |FIFO_Buf[(i*112)+3]); 
		  AccelAvg[1]=SumG/2;

		  SumG=0; for(i=0;i<2;i++)SumG+= (int16_t)(((int16_t)FIFO_Buf[(i*112)+4]<<8)  |FIFO_Buf[(i*112)+5]); 

		  AccelAvg[2]=SumG/2;
      
		  SumG=0; for(i=0;i<16;i++)  SumG+= (int16_t)(((int16_t)FIFO_Buf[(i*14)+8]<<8)  |FIFO_Buf[(i*14)+9]); 
		  GyroAvg[0]=SumG/16; 

		  SumG=0; for(i=0;i<16;i++)SumG+= (int16_t)(((int16_t)FIFO_Buf[(i*14)+10]<<8)  |FIFO_Buf[(i*14)+11]);  
		  GyroAvg[1]=SumG/16;    

		  SumG=0; for(i=0;i<16;i++)SumG+= (int16_t)(((int16_t)FIFO_Buf[(i*14)+12]<<8)  |FIFO_Buf[(i*14)+13]); 
                  
		  GyroAvg[2]=SumG/16;

//                  GyroAvg[0]-=(int16_t)((*temp)*TempGyroDrift[0]);
//	          GyroAvg[1]-=(int16_t)((*temp)*TempGyroDrift[1]);
//                  GyroAvg[2]-=(int16_t)((*temp)*TempGyroDrift[2]);
		  //Test0=AccelAvg[0]; Test1=AccelAvg[1];Test2=AccelAvg[2];
	       return 1; 
     } else  return 0; 
}








uint8_t MPU6000_Get_INT(void)
{
  //Nhiet do tang offset X giam, Y tang
 uint8_t tmp8;
 delay(15);
 BMP_CS0;
 delay(15);
 spiTransfer(MPU6000_SPI, 0x3a | MPU6000_SPI_READ);
 tmp8= spiTransfer(MPU6000_SPI, 0x00);
 BMP_CS1;
 return tmp8&0x01;
}

uint8_t MPUgetIntStatus() {
  return SPI_ByteRead(0x3A);
}

