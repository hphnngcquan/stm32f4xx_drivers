################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/002led_button.c \
../Src/003led_button_ext.c \
../Src/005button_interrupt.c \
../Src/gpio_test.c \
../Src/gpio_test_func.c \
../Src/spi_cmd_handling.c \
../Src/spi_tx_testing.c \
../Src/spi_txonly_arduino.c 

OBJS += \
./Src/002led_button.o \
./Src/003led_button_ext.o \
./Src/005button_interrupt.o \
./Src/gpio_test.o \
./Src/gpio_test_func.o \
./Src/spi_cmd_handling.o \
./Src/spi_tx_testing.o \
./Src/spi_txonly_arduino.o 

C_DEPS += \
./Src/002led_button.d \
./Src/003led_button_ext.d \
./Src/005button_interrupt.d \
./Src/gpio_test.d \
./Src/gpio_test_func.d \
./Src/spi_cmd_handling.d \
./Src/spi_tx_testing.d \
./Src/spi_txonly_arduino.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/002led_button.d ./Src/002led_button.o ./Src/002led_button.su ./Src/003led_button_ext.d ./Src/003led_button_ext.o ./Src/003led_button_ext.su ./Src/005button_interrupt.d ./Src/005button_interrupt.o ./Src/005button_interrupt.su ./Src/gpio_test.d ./Src/gpio_test.o ./Src/gpio_test.su ./Src/gpio_test_func.d ./Src/gpio_test_func.o ./Src/gpio_test_func.su ./Src/spi_cmd_handling.d ./Src/spi_cmd_handling.o ./Src/spi_cmd_handling.su ./Src/spi_tx_testing.d ./Src/spi_tx_testing.o ./Src/spi_tx_testing.su ./Src/spi_txonly_arduino.d ./Src/spi_txonly_arduino.o ./Src/spi_txonly_arduino.su

.PHONY: clean-Src

