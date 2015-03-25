################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/stm32l1xx_it.c 

OBJS += \
./User/stm32l1xx_it.o 

C_DEPS += \
./User/stm32l1xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -std=c99 -DUSE_STDPERIPH_DRIVER -DSTM32L1XX_MD -I"/Users/nathan/Development/stm32/STM32L1xx_StdPeriph_Template" -I"/Users/nathan/Development/stm32/STM32L1xx_StdPeriph_Template/Board" -I"/Users/nathan/Development/stm32/STM32L1xx_StdPeriph_Template/User" -I"/Users/nathan/Development/stm32/STM32L1xx_StdPeriph_Template/STM32L1xx_StdPeriph_Driver/inc" -I"/Users/nathan/Development/stm32/STM32L1xx_StdPeriph_Template/CMSIS/ST/STM32L1xx/Include" -I"/Users/nathan/Development/stm32/STM32L1xx_StdPeriph_Template/CMSIS/Include" -O0 -g3 -DWARF2 -Wall -c -fmessage-length=0 -mthumb -mcpu=cortex-m3 -O0 -ffunction-sections -fdata-sections -g -Wall -w -mlong-calls -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


