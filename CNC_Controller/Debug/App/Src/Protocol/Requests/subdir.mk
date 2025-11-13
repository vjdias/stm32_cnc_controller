################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/Src/Protocol/Requests/encoder_status_request.c \
../App/Src/Protocol/Requests/fpga_status_request.c \
../App/Src/Protocol/Requests/led_control_request.c \
../App/Src/Protocol/Requests/move_end_request.c \
../App/Src/Protocol/Requests/move_home_request.c \
../App/Src/Protocol/Requests/move_probe_level_request.c \
../App/Src/Protocol/Requests/move_queue_add_request.c \
../App/Src/Protocol/Requests/move_queue_status_request.c \
../App/Src/Protocol/Requests/set_microsteps_axes_request.c \
../App/Src/Protocol/Requests/set_microsteps_request.c \
../App/Src/Protocol/Requests/set_origin_request.c \
../App/Src/Protocol/Requests/start_move_request.c 

OBJS += \
./App/Src/Protocol/Requests/encoder_status_request.o \
./App/Src/Protocol/Requests/fpga_status_request.o \
./App/Src/Protocol/Requests/led_control_request.o \
./App/Src/Protocol/Requests/move_end_request.o \
./App/Src/Protocol/Requests/move_home_request.o \
./App/Src/Protocol/Requests/move_probe_level_request.o \
./App/Src/Protocol/Requests/move_queue_add_request.o \
./App/Src/Protocol/Requests/move_queue_status_request.o \
./App/Src/Protocol/Requests/set_microsteps_axes_request.o \
./App/Src/Protocol/Requests/set_microsteps_request.o \
./App/Src/Protocol/Requests/set_origin_request.o \
./App/Src/Protocol/Requests/start_move_request.o 

C_DEPS += \
./App/Src/Protocol/Requests/encoder_status_request.d \
./App/Src/Protocol/Requests/fpga_status_request.d \
./App/Src/Protocol/Requests/led_control_request.d \
./App/Src/Protocol/Requests/move_end_request.d \
./App/Src/Protocol/Requests/move_home_request.d \
./App/Src/Protocol/Requests/move_probe_level_request.d \
./App/Src/Protocol/Requests/move_queue_add_request.d \
./App/Src/Protocol/Requests/move_queue_status_request.d \
./App/Src/Protocol/Requests/set_microsteps_axes_request.d \
./App/Src/Protocol/Requests/set_microsteps_request.d \
./App/Src/Protocol/Requests/set_origin_request.d \
./App/Src/Protocol/Requests/start_move_request.d 


# Each subdirectory must supply rules for building sources it contributes
App/Src/Protocol/Requests/%.o App/Src/Protocol/Requests/%.su App/Src/Protocol/Requests/%.cyclo: ../App/Src/Protocol/Requests/%.c App/Src/Protocol/Requests/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../App/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-App-2f-Src-2f-Protocol-2f-Requests

clean-App-2f-Src-2f-Protocol-2f-Requests:
	-$(RM) ./App/Src/Protocol/Requests/encoder_status_request.cyclo ./App/Src/Protocol/Requests/encoder_status_request.d ./App/Src/Protocol/Requests/encoder_status_request.o ./App/Src/Protocol/Requests/encoder_status_request.su ./App/Src/Protocol/Requests/fpga_status_request.cyclo ./App/Src/Protocol/Requests/fpga_status_request.d ./App/Src/Protocol/Requests/fpga_status_request.o ./App/Src/Protocol/Requests/fpga_status_request.su ./App/Src/Protocol/Requests/led_control_request.cyclo ./App/Src/Protocol/Requests/led_control_request.d ./App/Src/Protocol/Requests/led_control_request.o ./App/Src/Protocol/Requests/led_control_request.su ./App/Src/Protocol/Requests/move_end_request.cyclo ./App/Src/Protocol/Requests/move_end_request.d ./App/Src/Protocol/Requests/move_end_request.o ./App/Src/Protocol/Requests/move_end_request.su ./App/Src/Protocol/Requests/move_home_request.cyclo ./App/Src/Protocol/Requests/move_home_request.d ./App/Src/Protocol/Requests/move_home_request.o ./App/Src/Protocol/Requests/move_home_request.su ./App/Src/Protocol/Requests/move_probe_level_request.cyclo ./App/Src/Protocol/Requests/move_probe_level_request.d ./App/Src/Protocol/Requests/move_probe_level_request.o ./App/Src/Protocol/Requests/move_probe_level_request.su ./App/Src/Protocol/Requests/move_queue_add_request.cyclo ./App/Src/Protocol/Requests/move_queue_add_request.d ./App/Src/Protocol/Requests/move_queue_add_request.o ./App/Src/Protocol/Requests/move_queue_add_request.su ./App/Src/Protocol/Requests/move_queue_status_request.cyclo ./App/Src/Protocol/Requests/move_queue_status_request.d ./App/Src/Protocol/Requests/move_queue_status_request.o ./App/Src/Protocol/Requests/move_queue_status_request.su ./App/Src/Protocol/Requests/set_microsteps_axes_request.cyclo ./App/Src/Protocol/Requests/set_microsteps_axes_request.d ./App/Src/Protocol/Requests/set_microsteps_axes_request.o ./App/Src/Protocol/Requests/set_microsteps_axes_request.su ./App/Src/Protocol/Requests/set_microsteps_request.cyclo ./App/Src/Protocol/Requests/set_microsteps_request.d ./App/Src/Protocol/Requests/set_microsteps_request.o ./App/Src/Protocol/Requests/set_microsteps_request.su ./App/Src/Protocol/Requests/set_origin_request.cyclo ./App/Src/Protocol/Requests/set_origin_request.d ./App/Src/Protocol/Requests/set_origin_request.o ./App/Src/Protocol/Requests/set_origin_request.su ./App/Src/Protocol/Requests/start_move_request.cyclo ./App/Src/Protocol/Requests/start_move_request.d ./App/Src/Protocol/Requests/start_move_request.o ./App/Src/Protocol/Requests/start_move_request.su

.PHONY: clean-App-2f-Src-2f-Protocol-2f-Requests

