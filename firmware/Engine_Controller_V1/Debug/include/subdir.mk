################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../include/diskio.c \
../include/ff.c \
../include/ff_gen_drv.c \
../include/sd_diskio.c \
../include/syscall.c \
../include/unicode.c 

OBJS += \
./include/diskio.o \
./include/ff.o \
./include/ff_gen_drv.o \
./include/sd_diskio.o \
./include/syscall.o \
./include/unicode.o 

C_DEPS += \
./include/diskio.d \
./include/ff.d \
./include/ff_gen_drv.d \
./include/sd_diskio.d \
./include/syscall.d \
./include/unicode.d 


# Each subdirectory must supply rules for building sources it contributes
include/%.o: ../include/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F446xx -DUSE_HAL_DRIVER -DHSE_VALUE=16000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\include\Middlewares\Third_Party\FatFs\src" -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\include\Middlewares\Third_Party\FatFs\src\drivers" -I/Engine_Controller_V1/include/Middlewares/Third_Party/FatFs/src/option -I"C:\Users\nicks\Desktop\engine_controller\firmware\Engine_Controller_V1\Inc" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


