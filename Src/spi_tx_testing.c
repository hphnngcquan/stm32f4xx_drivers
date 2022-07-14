/*
 * spi_tx_testing.c
 *
 *  Created on: Jun 10, 2022
 *      Author: NONAMENEEDED
 */
#include "stm32f407xx.h"
#include <string.h>

//PB15 -- SPI2_MOSI
//PB14 -- SPI2_MISO
//PB13 -- SPI2_SCK
//PB12 -- SPI2_NSS
//Alternate Function mode :5

void SPI2_GPIOInits(void);
void SPI2_Inits(void);
int main()
{
	char user_data[] = "Hello World";


	//Function used to initialize the GPIO pins the behave as SPI2

	SPI2_GPIOInits();
	//initialize SPI2 peripheral parameter
	SPI2_Inits();

	//Makes NSS internally high and avoids MODF errors
	SPI_SSIConfig(SPI2,ENABLE);

	//enable SPI2 peripheral
	SPI_PeriControl(SPI2, ENABLE);

	//send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));


	//wait until transmission is done
	while (SPI_GetFlagStatus(SPI2, (1<<SPI_SR_BSY)));

	//disable SPI2 peripheral
	SPI_PeriControl(SPI2, DISABLE);

	while(1);
	return 0;
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPI2Pins;
	//MOSI
	SPI2Pins.pGPIOx = GPIOB;
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	SPI2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPI2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_MODE_AF15;

	//MISO
//	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPI2Pins);

	//SCK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2Pins);

	//NSS
//	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPI2Pins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPD_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

	SPI_Init (&SPI2Handle);

}
