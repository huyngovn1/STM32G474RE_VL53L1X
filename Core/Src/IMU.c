#include <math.h>
#include "main.h"
#include "IMU.h"
#include "Kalman.h"
#include "ICM-42670.h"
int16_t accel_offsetX=0,accel_offsetY=0,accel_offsetZ=0;
uint16_t accel_TravelX=4100,accel_TravelY=4100,accel_TravelZ=4100;
extern float gyro[],accel[];
extern uint32_t ClkThisRound;
extern float Z_GyroRate;// Radian 
extern float TempMPU;
extern uint8_t MountSensorType ,Rx_type,Flag10Hz;   

uint8_t Usart1Interface(void);
uint8_t RX_Query (char rx_type);
void LogWrite(uint32_t Round);

Kalman_f KalmanGyroX = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};
Kalman_f KalmanGyroY = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanGyroZ = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};


Kalman_f KalmanAccelX = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};
Kalman_f KalmanAccelY = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};

Kalman_f KalmanAccelZ = {
        .Q = 0.001f,
        .Q_bias = 0.003f,
        .R_measure = 0.06f
};
void HeadSensorConvert (uint8_t Heading,float *Gyro,float *Accel){
float GyroTemp[3],AccelTemp[3];

GyroTemp[0]=Gyro[0];
GyroTemp[1]=Gyro[1];
GyroTemp[2]=Gyro[2];

AccelTemp[0]=Accel[0];
AccelTemp[1]=Accel[1];
AccelTemp[2]=Accel[2];

 if(Heading==2){
   Gyro[0]=0-Gyro[0];
   Gyro[1]=0-Gyro[1];
   
   Accel[0]=0-AccelTemp[0];
   Accel[1]=0-AccelTemp[1];
  }

else if(Heading==3){
   Gyro[1]=0-Gyro[1];
   Gyro[2]=0-Gyro[2];
   
   
   Accel[1]=0-AccelTemp[1];
   Accel[2]=0-AccelTemp[2];

  }
  else if(Heading==4){
   Gyro[0]=0-GyroTemp[0];
   Gyro[2]=0-GyroTemp[2];
   
   Accel[0]=0-AccelTemp[0];
   Accel[2]=0-AccelTemp[2];
  }
 else if(Heading==5){
   Gyro[1]=0-GyroTemp[2];
   Gyro[2]=GyroTemp[1];
   
   Accel[1]=0-AccelTemp[2];
   Accel[2]=AccelTemp[1];
  }
  else if(Heading==6){
   Gyro[0]=0-GyroTemp[0];
   Gyro[1]=0-GyroTemp[2];
   Gyro[2]=0-GyroTemp[1];
   
   Accel[0]=0-AccelTemp[0];
   Accel[1]=0-AccelTemp[2];
   Accel[2]=0-AccelTemp[1];
   
  }
 else if(Heading==7){
   Gyro[1]=GyroTemp[2];
   Gyro[2]=0-GyroTemp[1];
   
   Accel[1]=AccelTemp[2];
   Accel[2]=0-AccelTemp[1];

  }
  else if(Heading==8){
   Gyro[0]=0-GyroTemp[0];
   Gyro[1]=GyroTemp[2];
   Gyro[2]=GyroTemp[1];
   
   Accel[0]=0-AccelTemp[0];
   Accel[1]=AccelTemp[2];
   Accel[2]=AccelTemp[1];
  }
}

void LowpassAccel(float* accel,float* accelOut)
{   
//    accelOut[0]+=((float)(accel[0]-accel_offsetX)-accelOut[0])*0.1415f; //15 Hz
//    accelOut[1]+=((float)(accel[1]-accel_offsetY)-accelOut[1])*0.1415f;
//    accelOut[2]+=((float)(accel[2]-accel_offsetZ)-accelOut[2])*0.1415f;   
      
    accelOut[0]=  KalmanFiller(&KalmanAccelX,(accel[0]), 50.0f/(float)Frq_Update); //10Hz
    accelOut[1]=  KalmanFiller(&KalmanAccelY,(accel[1]), 50.0f/(float)Frq_Update); 
    accelOut[2]=  KalmanFiller(&KalmanAccelZ,(accel[2]-accel_offsetZ), 50.0f/(float)Frq_Update); 
}  

void LowpassGyro(float* SpeedGyro,float* GyroLP)
{   
                        Z_GyroRate=SpeedGyro[2]/((float)Frq_Update*deg_rad);  // Xoay Cyclic theo Rudder
                        
                        GyroLP[0]=  KalmanFiller(&KalmanGyroX,SpeedGyro[0], 150.0f/(float)Frq_Update);//30Hz   KALMAN_Q 0.001f    KALMAN_BIAS 0.003f  KALMAN_R 0.06f
                        GyroLP[1]=  KalmanFiller(&KalmanGyroY,SpeedGyro[1], 125.0f/(float)Frq_Update);//25Hz   KALMAN_Q 0.001f    KALMAN_BIAS 0.003f  KALMAN_R 0.06f
                        GyroLP[2]=  KalmanFiller(&KalmanGyroZ,SpeedGyro[2], 200.0f/(float)Frq_Update);//40Hz   Tested Lost 0.3   KALMAN_Q 0.001f    KALMAN_BIAS 0.003f  KALMAN_R 0.06f
}  

void FIFOStabling(float *GyroRef) {	// 2s Stable
      uint16_t CountStable=0;
      float	Sum[3];
      float SumAccel=0;
      float Max[3],Min[3];
        
      while(CountStable<2200) { //3s	
	while(MPU6000_Get_FIFO_Packets (accel,gyro,&TempMPU)!=1);	//1.5ms
        HeadSensorConvert(MountSensorType,gyro,accel);
        //LowpassAccel(accel,accelLP); 
        SumAccel+=sqrtf((accel[0]*accel[0])+(accel[1]*accel[1])+(accel[2]*accel[2]));
        
	if(CountStable==0) {
		   Max[0]=Min[0]=gyro[0];
		   Max[1]=Min[1]=gyro[1];
		   Max[2]=Min[2]=gyro[2];
	} 
	else if(CountStable==200){Sum[0]=0;Sum[1]=0;Sum[2]=0;  SumAccel=0; }

        if(gyro[0]<Min[0])Min[0]=gyro[0];
        else if(gyro[0]>Max[0])Max[0]=gyro[0];

	if(gyro[1]<Min[1])Min[1]=gyro[1];
        else if(gyro[1]>Max[1])Max[1]=gyro[1];

	if(gyro[2]<Min[2])Min[2]=gyro[2];
        else if(gyro[2]>Max[2])Max[2]=gyro[2];
    
        if(( Max[0]>(Min[0]+150))||( Max[1]>(Min[1]+150))||( Max[2]>(Min[2]+150)))CountStable=0; 
        else CountStable++; 
        if(accel[2]<0)CountStable=0;  //TempMPU<-45 ||TempMPU>65||
        Sum[0]+=gyro[0];
        Sum[1]+=gyro[1];
        Sum[2]+=gyro[2];
        
        RX_Query (Rx_type); 
	Usart1Interface();  
        #ifdef LOG_EN
        if(Flag10Hz){LogWrite(ClkThisRound);Flag10Hz=0;}
        #endif
     }
     GyroRef[0]=	Sum[0]/2000.0f;
     GyroRef[1]=	Sum[1]/2000.0f;
     GyroRef[2]=	Sum[2]/2000.0f;
     accel_offsetZ=(int16_t)(SumAccel/2000.0f)-accel_TravelZ;
     
     #ifdef TEST_MODE
       sprintf(temUsart, "Of:%d_%d_%d-%d\n",GyroRef[0],GyroRef[1],GyroRef[2],accel_offsetZ); sendUsart1(temUsart); 
       sprintf(temUsart, "\nMiMax:%d_%d_%d\n",Max[0]-Min[0],Max[1]-Min[1],Max[2]-Min[2]); sendUsart1(temUsart); 
     #endif
}
void GyroOffsetLoop(float *SpeedGyro ,float *GyroRef) {
  static uint32_t CountStable=0;
  static	float	Sum[3],SumAccel[3];
  static	float Max[3],Min[3];
        
	if(CountStable==0) {
		   Max[0]=Min[0]=SpeedGyro[0];
		   Max[1]=Min[1]=SpeedGyro[1];
		   Max[2]=Min[2]=SpeedGyro[2];
                   Sum[0]=0;Sum[1]=0;Sum[2]=0;
                 
	} else if(CountStable==500) {
           SumAccel[0]=0; SumAccel[1]=0;SumAccel[2]=0;
        }
        
        Sum[0]+=SpeedGyro[0];
        Sum[1]+=SpeedGyro[1];
        Sum[2]+=SpeedGyro[2];
          
          SumAccel[0]+=accel[0];
          SumAccel[1]+=accel[1];
          SumAccel[2]+=accel[2];

        if(SpeedGyro[0]<Min[0])Min[0]=SpeedGyro[0];
        else if(SpeedGyro[0]>Max[0])Max[0]=SpeedGyro[0];

	if(SpeedGyro[1]<Min[1])Min[1]=SpeedGyro[1];
        else if(SpeedGyro[1]>Max[1])Max[1]=SpeedGyro[1];

	if(SpeedGyro[2]<Min[2])Min[2]=SpeedGyro[2];
        else if(SpeedGyro[2]>Max[2])Max[2]=SpeedGyro[2];
    
        if(( Max[0]>(Min[0]+80))||( Max[1]>(Min[1]+80))||( Max[2]>(Min[2]+80)))CountStable=0; 
        else if(++CountStable>=(3330+500)){ //5s
             GyroRef[0]+=	(Sum[0])/3330.f;
             GyroRef[1]+=	(Sum[1])/3330.f;
             GyroRef[2]+=	(Sum[2])/3330.f;
             
             float AccelAVG[3];
             AccelAVG[0]=(SumAccel[0])/3330.f;
             AccelAVG[1]=(SumAccel[1])/3330.f;
             AccelAVG[2]=(SumAccel[2])/3330.f;
             
             CountStable=0;
             if((AccelAVG[0]>-600.0f)&&(AccelAVG[0]<600.0f)&&(AccelAVG[1]>-600.0f)&&(AccelAVG[1]<600.0f)){ //8Deg
              float Z_accel_landing=sqrtf((AccelAVG[0]*AccelAVG[0])+(AccelAVG[1]*AccelAVG[1])+(AccelAVG[2]*AccelAVG[2]));
              accel_offsetZ=(int16_t)Z_accel_landing-accel_TravelZ;
              #ifdef TEST_MODE
                 sprintf(temUsart, "O:%d_%d_%d__%d\n",GyroRef[0],GyroRef[1],GyroRef[2],accel_offsetZ); CDC_Transmit_FS((uint8_t*)temUsart, strlen(temUsart));//sendUsart1(temUsart); 
              #endif 
            }
             
        }
}

 //uint8_t temUsartTx[64];  

uint8_t GyroOffsetLoopRaw(float *GyroRaw ,float *GyroOffsetRaw) {
  static uint32_t CountStable=0;
  static	float Sum[3],SumAccel[3];
  static	float Max[3],Min[3];
        
	if(CountStable==0) {
		   Max[0]=Min[0]=GyroRaw[0];
		   Max[1]=Min[1]=GyroRaw[1];
		   Max[2]=Min[2]=GyroRaw[2];
                   Sum[0]=0;Sum[1]=0;Sum[2]=0;
                   SumAccel[0]=0; SumAccel[1]=0;SumAccel[2]=0;
	} 
        else if(CountStable>=500) {
          Sum[0]+=GyroRaw[0];
          Sum[1]+=GyroRaw[1];
          Sum[2]+=GyroRaw[2];
          SumAccel[0]+=accel[0];SumAccel[1]+=accel[1];SumAccel[2]+=accel[2];
        }
        CountStable++;

        if(GyroRaw[0]<Min[0])Min[0]=GyroRaw[0];
        else if(GyroRaw[0]>Max[0])Max[0]=GyroRaw[0];

	if(GyroRaw[1]<Min[1])Min[1]=GyroRaw[1];
        else if(GyroRaw[1]>Max[1])Max[1]=GyroRaw[1];

	if(GyroRaw[2]<Min[2])Min[2]=GyroRaw[2];
        else if(GyroRaw[2]>Max[2])Max[2]=GyroRaw[2];
    
        if(( Max[0]>(Min[0]+90))||( Max[1]>(Min[1]+90))||( Max[2]>(Min[2]+90)))CountStable=0; 
        else if(CountStable>=(2500+500)){ //5s
             GyroOffsetRaw[0]=	(Sum[0])/2500.0f;
             GyroOffsetRaw[1]=	(Sum[1])/2500.0f;
             GyroOffsetRaw[2]=	(Sum[2])/2500.0f;
             
             float  AccelAVG[3];
             AccelAVG[0]=(SumAccel[0])/2500.0f;
             AccelAVG[1]=(SumAccel[1])/2500.0f;
             AccelAVG[2]=(SumAccel[2])/2500.0f;
             
              CountStable=0;
             if((AccelAVG[0]>-800.0f)&&(AccelAVG[0]<800.0f)&&(AccelAVG[1]>-800.0f)&&(AccelAVG[1]<800.0f)){ //10Deg
              float Z_accel_landing=sqrtf((AccelAVG[0]*AccelAVG[0])+(AccelAVG[1]*AccelAVG[1])+(AccelAVG[2]*AccelAVG[2]));
              accel_offsetZ=(int16_t)Z_accel_landing-accel_TravelZ;

              //sprintf(temUsartTx, "O:%f_%f_%f_%f_%d\n",AccelAVG[0],AccelAVG[1],AccelAVG[2],Z_accel_landing,accel_offsetZ); CDC_Transmit_FS((uint8_t*)temUsartTx, strlen(temUsartTx));//sendUsart1(temUsart); 
              return 1;
            }
             
        }
        return 0;
}

uint8_t RingLimit(float *x, float *y,float Limit,float *RingOut){
                    float tmpring;
                    tmpring=((*x)*(*x)) + ((*y)*(*y));
                    tmpring=sqrtf(tmpring);
                    *RingOut=tmpring;
                    if(tmpring>Limit){
                            *x=(*x/tmpring)*Limit;
                            *y=(*y/tmpring)*Limit;
                            *RingOut=Limit;
                            return 1;
                    }
                    return 0;
}
