################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_initialize_hardware.c \
../src/_write.c \
../src/bsp_driver_sd.c \
../src/fatfs.c \
../src/main.c \
../src/stm32f4xx_hal_msp.c \
../src/stm32f4xx_it.c 

OBJS += \
./src/_initialize_hardware.o \
./src/_write.o \
./src/bsp_driver_sd.o \
./src/fatfs.o \
./src/main.o \
./src/stm32f4xx_hal_msp.o \
./src/stm32f4xx_it.o 

C_DEPS += \
./src/_initialize_hardware.d \
./src/_write.d \
./src/bsp_driver_sd.d \
./src/fatfs.d \
./src/main.d \
./src/stm32f4xx_hal_msp.d \
./src/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -DHSE_VALUE=16000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\include\Middlewares\Third_Party\FatFs\src" -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\include\Middlewares\Third_Party\FatFs\src\drivers" -I/Engine_Controller_V1/include/Middlewares/Third_Party/FatFs/src/option -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\Inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/stm32f4xx_hal_msp.o: ../src/stm32f4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -DHSE_VALUE=16000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\include\Middlewares\Third_Party\FatFs\src" -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\include\Middlewares\Third_Party\FatFs\src\drivers" -I/Engine_Controller_V1/include/Middlewares/Third_Party/FatFs/src/option -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\Inc" -std=gnu11 -Wno-missing-prototypes -Wno-missing-declarations -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/stm32f4xx_hal_msp.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


