/*
 * 002led_button.c
 *
 * Connect external button to PB12 and external LED to PA8 and
 * toggle the LED whenever the external button is pressed
 *
 *  Created on: May 26, 2022
 *      Author: NONAMENEEDED
 */
#include"stm32f407xx.h"
#define 	HIGH 			ENABLE
#define 	LOW 			DISABLE
#define 	BTN_PRESSED 	LOW

//delay declaring
void delay(void){
	for (uint32_t i = 0; i>500000; i++);
}

void led_button(void)
{

	GPIO_Handle_t GpioLed,GpioBtn;

	//GPIO led config
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OU;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD ;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);


	//GPIO Button config
	GpioBtn.pGPIOx = GPIOB;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU ;

		GPIO_PeriClockControl(GPIOB, ENABLE);
		GPIO_Init(&GpioBtn);

	while(1){
		if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED)
		{
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}

	}

}


