################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src(tests)/spi_cmd_handling.c 

OBJS += \
./Src(tests)/spi_cmd_handling.o 

C_DEPS += \
./Src(tests)/spi_cmd_handling.d 


# Each subdirectory must supply rules for building sources it contributes
Src(tests)/spi_cmd_handling.o: ../Src(tests)/spi_cmd_handling.c Src(tests)/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"Q:/FASTBit Embedded/MCU1-Course/HelloWorld/stm32f4xx_drivers/drivers/Inc" -I"Q:/FASTBit Embedded/MCU1-Course/HelloWorld/stm32f4xx_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src(tests)/spi_cmd_handling.d" -MT"$@" --specs=nano.specs -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-28-tests-29-

clean-Src-28-tests-29-:
	-$(RM) ./Src(tests)/spi_cmd_handling.d ./Src(tests)/spi_cmd_handling.o ./Src(tests)/spi_cmd_handling.su

.PHONY: clean-Src-28-tests-29-

