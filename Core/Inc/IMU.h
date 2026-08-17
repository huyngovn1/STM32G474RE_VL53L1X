#include <stdint.h>
#define deg_rad 57.29578f
void HeadSensorConvert (uint8_t Heading,float *Gyro,float *Accel);
void LowpassAccel(float* accel,float* accelOut);
void LowpassGyro(float* SpeedGyro,float* GyroLP);
void GyroOffsetLoop(float *SpeedGyro ,float *GyroRef) ;
void FIFOStabling(float *GyroRef);
uint8_t GyroOffsetLoopRaw(float *GyroRaw ,float *GyroOffsetRaw);
