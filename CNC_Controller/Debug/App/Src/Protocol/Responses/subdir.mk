################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/Protocol/Responses/encoder_status_response.c \
../App/Src/Protocol/Responses/home_status_response.c \
../App/Src/Protocol/Responses/led_control_response.c \
../App/Src/Protocol/Responses/move_end_response.c \
../App/Src/Protocol/Responses/move_home_response.c \
../App/Src/Protocol/Responses/move_probe_level_response.c \
../App/Src/Protocol/Responses/move_queue_add_ack_response.c \
../App/Src/Protocol/Responses/move_queue_status_response.c \
../App/Src/Protocol/Responses/motion_auto_friction_response.c \
../App/Src/Protocol/Responses/set_origin_response.c \
../App/Src/Protocol/Responses/start_move_response.c 

OBJS += \
./App/Src/Protocol/Responses/encoder_status_response.o \
./App/Src/Protocol/Responses/home_status_response.o \
./App/Src/Protocol/Responses/led_control_response.o \
./App/Src/Protocol/Responses/move_end_response.o \
./App/Src/Protocol/Responses/move_home_response.o \
./App/Src/Protocol/Responses/move_probe_level_response.o \
./App/Src/Protocol/Responses/move_queue_add_ack_response.o \
./App/Src/Protocol/Responses/move_queue_status_response.o \
./App/Src/Protocol/Responses/motion_auto_friction_response.o \
./App/Src/Protocol/Responses/set_origin_response.o \
./App/Src/Protocol/Responses/start_move_response.o 

C_DEPS += \
./App/Src/Protocol/Responses/encoder_status_response.d \
./App/Src/Protocol/Responses/home_status_response.d \
./App/Src/Protocol/Responses/led_control_response.d \
./App/Src/Protocol/Responses/move_end_response.d \
./App/Src/Protocol/Responses/move_home_response.d \
./App/Src/Protocol/Responses/move_probe_level_response.d \
./App/Src/Protocol/Responses/move_queue_add_ack_response.d \
./App/Src/Protocol/Responses/move_queue_status_response.d \
./App/Src/Protocol/Responses/motion_auto_friction_response.d \
./App/Src/Protocol/Responses/set_origin_response.d \
./App/Src/Protocol/Responses/start_move_response.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/Protocol/Responses/%.o App/Src/Protocol/Responses/%.su App/Src/Protocol/Responses/%.cyclo: ../App/Src/Protocol/Responses/%.c App/Src/Protocol/Responses/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../App/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src-2f-Protocol-2f-Responses

clean-App-2f-Src-2f-Protocol-2f-Responses:
	-$(RM) ./App/Src/Protocol/Responses/encoder_status_response.cyclo ./App/Src/Protocol/Responses/encoder_status_response.d ./App/Src/Protocol/Responses/encoder_status_response.o ./App/Src/Protocol/Responses/encoder_status_response.su ./App/Src/Protocol/Responses/home_status_response.cyclo ./App/Src/Protocol/Responses/home_status_response.d ./App/Src/Protocol/Responses/home_status_response.o ./App/Src/Protocol/Responses/home_status_response.su ./App/Src/Protocol/Responses/led_control_response.cyclo ./App/Src/Protocol/Responses/led_control_response.d ./App/Src/Protocol/Responses/led_control_response.o ./App/Src/Protocol/Responses/led_control_response.su ./App/Src/Protocol/Responses/move_end_response.cyclo ./App/Src/Protocol/Responses/move_end_response.d ./App/Src/Protocol/Responses/move_end_response.o ./App/Src/Protocol/Responses/move_end_response.su ./App/Src/Protocol/Responses/move_home_response.cyclo ./App/Src/Protocol/Responses/move_home_response.d ./App/Src/Protocol/Responses/move_home_response.o ./App/Src/Protocol/Responses/move_home_response.su ./App/Src/Protocol/Responses/move_probe_level_response.cyclo ./App/Src/Protocol/Responses/move_probe_level_response.d ./App/Src/Protocol/Responses/move_probe_level_response.o ./App/Src/Protocol/Responses/move_probe_level_response.su ./App/Src/Protocol/Responses/move_queue_add_ack_response.cyclo ./App/Src/Protocol/Responses/move_queue_add_ack_response.d ./App/Src/Protocol/Responses/move_queue_add_ack_response.o ./App/Src/Protocol/Responses/move_queue_add_ack_response.su ./App/Src/Protocol/Responses/move_queue_status_response.cyclo ./App/Src/Protocol/Responses/move_queue_status_response.d ./App/Src/Protocol/Responses/move_queue_status_response.o ./App/Src/Protocol/Responses/move_queue_status_response.su ./App/Src/Protocol/Responses/set_origin_response.cyclo ./App/Src/Protocol/Responses/set_origin_response.d ./App/Src/Protocol/Responses/set_origin_response.o ./App/Src/Protocol/Responses/set_origin_response.su ./App/Src/Protocol/Responses/start_move_response.cyclo ./App/Src/Protocol/Responses/start_move_response.d ./App/Src/Protocol/Responses/start_move_response.o ./App/Src/Protocol/Responses/start_move_response.su

.PHONY: clean-App-2f-Src-2f-Protocol-2f-Responses

