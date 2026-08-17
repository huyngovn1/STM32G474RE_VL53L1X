################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/Core/BSP.c \
../Application/User/Core/Global.c \
../Application/User/Core/ICM-42670.c \
../Application/User/Core/IMU.c \
../Application/User/Core/Kalman.c \
../Application/User/Core/QMC5883.c \
../Application/User/Core/VL53L0X.c \
../Application/User/Core/VL53L1X_api.c \
../Application/User/Core/bmp3.c \
C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/main.c \
C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/stm32g4xx_hal_msp.c \
C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/stm32g4xx_hal_timebase_tim.c \
C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/stm32g4xx_it.c \
../Application/User/Core/syscalls.c \
../Application/User/Core/sysmem.c \
../Application/User/Core/vl53l1_platform.c 

OBJS += \
./Application/User/Core/BSP.o \
./Application/User/Core/Global.o \
./Application/User/Core/ICM-42670.o \
./Application/User/Core/IMU.o \
./Application/User/Core/Kalman.o \
./Application/User/Core/QMC5883.o \
./Application/User/Core/VL53L0X.o \
./Application/User/Core/VL53L1X_api.o \
./Application/User/Core/bmp3.o \
./Application/User/Core/main.o \
./Application/User/Core/stm32g4xx_hal_msp.o \
./Application/User/Core/stm32g4xx_hal_timebase_tim.o \
./Application/User/Core/stm32g4xx_it.o \
./Application/User/Core/syscalls.o \
./Application/User/Core/sysmem.o \
./Application/User/Core/vl53l1_platform.o 

C_DEPS += \
./Application/User/Core/BSP.d \
./Application/User/Core/Global.d \
./Application/User/Core/ICM-42670.d \
./Application/User/Core/IMU.d \
./Application/User/Core/Kalman.d \
./Application/User/Core/QMC5883.d \
./Application/User/Core/VL53L0X.d \
./Application/User/Core/VL53L1X_api.d \
./Application/User/Core/bmp3.d \
./Application/User/Core/main.d \
./Application/User/Core/stm32g4xx_hal_msp.d \
./Application/User/Core/stm32g4xx_hal_timebase_tim.d \
./Application/User/Core/stm32g4xx_it.d \
./Application/User/Core/syscalls.d \
./Application/User/Core/sysmem.d \
./Application/User/Core/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Core/%.o Application/User/Core/%.su Application/User/Core/%.cyclo: ../Application/User/Core/%.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../../USB_Device/App -I../../USB_Device/Target -I../../Core/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/main.o: C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/main.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../../USB_Device/App -I../../USB_Device/Target -I../../Core/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32g4xx_hal_msp.o: C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/stm32g4xx_hal_msp.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../../USB_Device/App -I../../USB_Device/Target -I../../Core/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32g4xx_hal_timebase_tim.o: C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/stm32g4xx_hal_timebase_tim.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../../USB_Device/App -I../../USB_Device/Target -I../../Core/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32g4xx_it.o: C:/Users/huy/Downloads/FirmwareSTM32G474RE_V011/FirmwareSTM32G474RE_V01/Core/Src/stm32g4xx_it.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../../USB_Device/App -I../../USB_Device/Target -I../../Core/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc -I../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-Core

clean-Application-2f-User-2f-Core:
	-$(RM) ./Application/User/Core/BSP.cyclo ./Application/User/Core/BSP.d ./Application/User/Core/BSP.o ./Application/User/Core/BSP.su ./Application/User/Core/Global.cyclo ./Application/User/Core/Global.d ./Application/User/Core/Global.o ./Application/User/Core/Global.su ./Application/User/Core/ICM-42670.cyclo ./Application/User/Core/ICM-42670.d ./Application/User/Core/ICM-42670.o ./Application/User/Core/ICM-42670.su ./Application/User/Core/IMU.cyclo ./Application/User/Core/IMU.d ./Application/User/Core/IMU.o ./Application/User/Core/IMU.su ./Application/User/Core/Kalman.cyclo ./Application/User/Core/Kalman.d ./Application/User/Core/Kalman.o ./Application/User/Core/Kalman.su ./Application/User/Core/QMC5883.cyclo ./Application/User/Core/QMC5883.d ./Application/User/Core/QMC5883.o ./Application/User/Core/QMC5883.su ./Application/User/Core/VL53L0X.cyclo ./Application/User/Core/VL53L0X.d ./Application/User/Core/VL53L0X.o ./Application/User/Core/VL53L0X.su ./Application/User/Core/VL53L1X_api.cyclo ./Application/User/Core/VL53L1X_api.d ./Application/User/Core/VL53L1X_api.o ./Application/User/Core/VL53L1X_api.su ./Application/User/Core/bmp3.cyclo ./Application/User/Core/bmp3.d ./Application/User/Core/bmp3.o ./Application/User/Core/bmp3.su ./Application/User/Core/main.cyclo ./Application/User/Core/main.d ./Application/User/Core/main.o ./Application/User/Core/main.su ./Application/User/Core/stm32g4xx_hal_msp.cyclo ./Application/User/Core/stm32g4xx_hal_msp.d ./Application/User/Core/stm32g4xx_hal_msp.o ./Application/User/Core/stm32g4xx_hal_msp.su ./Application/User/Core/stm32g4xx_hal_timebase_tim.cyclo ./Application/User/Core/stm32g4xx_hal_timebase_tim.d ./Application/User/Core/stm32g4xx_hal_timebase_tim.o ./Application/User/Core/stm32g4xx_hal_timebase_tim.su ./Application/User/Core/stm32g4xx_it.cyclo ./Application/User/Core/stm32g4xx_it.d ./Application/User/Core/stm32g4xx_it.o ./Application/User/Core/stm32g4xx_it.su ./Application/User/Core/syscalls.cyclo ./Application/User/Core/syscalls.d ./Application/User/Core/syscalls.o ./Application/User/Core/syscalls.su ./Application/User/Core/sysmem.cyclo ./Application/User/Core/sysmem.d ./Application/User/Core/sysmem.o ./Application/User/Core/sysmem.su ./Application/User/Core/vl53l1_platform.cyclo ./Application/User/Core/vl53l1_platform.d ./Application/User/Core/vl53l1_platform.o ./Application/User/Core/vl53l1_platform.su

.PHONY: clean-Application-2f-User-2f-Core

