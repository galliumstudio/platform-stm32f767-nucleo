################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/BSP/Components/wm8994/wm8994.c 

OBJS += \
./system/BSP/Components/wm8994/wm8994.o 

C_DEPS += \
./system/BSP/Components/wm8994/wm8994.d 


# Each subdirectory must supply rules for building sources it contributes
system/BSP/Components/wm8994/%.o: ../system/BSP/Components/wm8994/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f7xx" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


