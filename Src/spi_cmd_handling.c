/*
 * spi_cmd_handling.c
 *
 *  Created on: Jun 15, 2022
 *      Author: NONAMENEEDED
 */
#include "stm32f407xx.h"
#include <string.h>



#define BTN_PRESSED ENABLE

//command codes
#define CMD_LED_CTRL 		0x50
#define CMD_SENSOR_READ		0x51
#define CMD_LED_READ 		0x52
#define CMD_PRINT 			0x53
#define CMD_ID_READ 		0x54

#define LED_ON 1
#define LED_OFF 0

//	arduino analog pins
#define ANALOG_PIN0 0
#define ANALOG_PIN1 1
#define ANALOG_PIN2 2
#define ANALOG_PIN3 3
#define ANALOG_PIN4 4

//Arduino led pin,  hardware connected
#define LED_PIN 9

//PB15 -- SPI2_MOSI
//PB14 -- SPI2_MISO
//PB13 -- SPI2_SCK
//PB12 -- SPI2_NSS
//Alternate Function mode :5

void SPI2_GPIOInits(void);
void SPI2_Inits(void);
void GPIO_ButtonInit(void);
void delay(void);
uint8_t SPI_VeriResponse(uint8_t ackbyte);



int main() {
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//initialize GPIO button
	GPIO_ButtonInit();

	//Function used to initialize the GPIO pins the behave as SPI2
	SPI2_GPIOInits();

	//initialize SPI2 peripheral parameter
	SPI2_Inits();

	/*
	 * Making SSOE 1 does NSS output enabled
	 * The NSS pin is automatically managed by the hardware
	 * i.e.  when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE =0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {

		//wait until btn is pressed
		while (!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));
		delay();

		//enable SPI2 peripheral
		SPI_PeriControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin no(1)> <value(1)>

		uint8_t cmdcode = CMD_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//Send command
		SPI_SendData(SPI2, &cmdcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bit (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive the ackbyte and read
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if (SPI_VeriResponse(ackbyte)) //ACK received
				{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			SPI_SendData(SPI2, args, 2);

			//receive dummy
			SPI_ReceiveData(SPI2, args, 2);

		} //END CMD_LED_CNTRL

		//2. CMD_SENSOR_READ <analog pin number(1)>

		//wait until btn is pressed
		while (!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));
		delay();

		cmdcode = CMD_SENSOR_READ;

		//send command
		SPI_SendData(SPI2, &cmdcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bit (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive the ackbyte and read
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		if (SPI_VeriResponse(ackbyte)) //ACK received
				{
			//send arguments
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2, args, 1);

			//do dummy read to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//delay so that slave is ready for next data
			delay();

			//send some dummy bit (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}

		//3. CMD_LED_READ

		//wait till button is pressed
		while (!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));
		delay();

		cmdcode = CMD_LED_READ;

		//send command
		SPI_SendData(SPI2, &cmdcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bit (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive the ackbyte and read
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VeriResponse(ackbyte)) //ACK received
				{
			//send arguments
			args[0] = LED_PIN;

			//send arg
			SPI_SendData(SPI2, args, 1);

			//do dummy read
			SPI_ReceiveData(SPI2, &dummy_read, 2);

			delay();

			//send dummy
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
		}

		//4.CMD_PRINTF  <len(2)> <message(len)>
		//wait till button is pressed
		while (!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));
		delay();

		cmdcode = CMD_PRINT;

		//send command
		SPI_SendData(SPI2, &cmdcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bit (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive the ackbyte and read
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		char message[] = "Hello World! Embedded Engineer hear";
		if (SPI_VeriResponse(ackbyte)) //ACK received
				{
			//send arguments
			args[0] = strlen((char*) message);

			//send arg
			SPI_SendData(SPI2, args, 1);

			//do dummy read
			SPI_ReceiveData(SPI2, &dummy_read, 2);

			delay();

			//send message
			for (int i = 0; i < args[0]; i++) {
				SPI_SendData(SPI2, &message[i], 1); //NOTE
				//receive dummy
				SPI_ReceiveData(SPI2, &dummy_read,1);
			}
		}
		//5. CMD_ID_READ
		//wait till button is pressed
		while (!(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0)));
		delay();

		cmdcode = CMD_ID_READ;

		//send command
		SPI_SendData(SPI2, &cmdcode, 1);

		//do dummy read to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//send some dummy bit (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		//receive the ackbyte and read
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[11];
		uint32_t i = 0;
		if (SPI_VeriResponse(ackbyte)) //ACK received
				{
			//read 10 bytes id from slave
			for (i=0; i < 10; i++)
			{
				//send dummy
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[10] = '\0';
			//printf("CMD_ID %s\n",id);
		}

		//wait until transmission is done
		while (SPI_GetFlagStatus(SPI2, (1 << SPI_SR_BSY)))
			;

		//disable SPI2 peripheral
		SPI_PeriControl(SPI2, DISABLE);

	}

	return 0;
}

void SPI2_GPIOInits(void) {
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
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPI2Pins);

	//SCK
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPI2Pins);

	//NSS
	SPI2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPI2Pins);

}

void SPI2_Inits(void) {
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPD_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //HARDWARE slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GpioBtn;
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

void delay(void) {
	for (uint32_t i = 0; i > 500000 / 2; i++)
		;
}

uint8_t SPI_VeriResponse(uint8_t ackbyte) {
	if (ackbyte == 0xF5) {
		//ACK
		return 1;
	}
	return 0; //NACK

}


