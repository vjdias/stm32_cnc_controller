################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/app.c \
../App/Src/board_config.c 

OBJS += \
./App/Src/app.o \
./App/Src/board_config.o 

C_DEPS += \
./App/Src/app.d \
./App/Src/board_config.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/%.o App/Src/%.su App/Src/%.cyclo: ../App/Src/%.c App/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../App/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src

clean-App-2f-Src:
	-$(RM) ./App/Src/app.cyclo ./App/Src/app.d ./App/Src/app.o ./App/Src/app.su \
	       ./App/Src/board_config.cyclo ./App/Src/board_config.d ./App/Src/board_config.o ./App/Src/board_config.su

.PHONY: clean-App-2f-Src

