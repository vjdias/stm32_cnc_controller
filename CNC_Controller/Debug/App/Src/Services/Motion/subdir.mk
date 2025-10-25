################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/Services/Motion/motion_hw.c \
../App/Src/Services/Motion/motion_service.c 

OBJS += \
./App/Src/Services/Motion/motion_hw.o \
./App/Src/Services/Motion/motion_service.o 

C_DEPS += \
./App/Src/Services/Motion/motion_hw.d \
./App/Src/Services/Motion/motion_service.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/Services/Motion/%.o App/Src/Services/Motion/%.su App/Src/Services/Motion/%.cyclo: ../App/Src/Services/Motion/%.c App/Src/Services/Motion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../App/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src-2f-Services-2f-Motion

clean-App-2f-Src-2f-Services-2f-Motion:
	-$(RM) ./App/Src/Services/Motion/motion_hw.cyclo ./App/Src/Services/Motion/motion_hw.d ./App/Src/Services/Motion/motion_hw.o ./App/Src/Services/Motion/motion_hw.su ./App/Src/Services/Motion/motion_service.cyclo ./App/Src/Services/Motion/motion_service.d ./App/Src/Services/Motion/motion_service.o ./App/Src/Services/Motion/motion_service.su

.PHONY: clean-App-2f-Src-2f-Services-2f-Motion

