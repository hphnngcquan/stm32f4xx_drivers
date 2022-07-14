/*
 * 002led_button.c
 *
 *  Created on: May 26, 2022
 *      Author: NONAMENEEDED
 */
#include"stm32f407xx.h"
#define HIGH ENABLE
#define BTN_PRESSED HIGH

//delay declaring
void delay(void){
	for (uint32_t i = 0; i>500000; i++);
}

void led_button_ext(void)
{

	GPIO_Handle_t GpioLed,GpioBtn;

	//GPIO led config
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OU;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD ;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);


	//GPIO Button config
	GpioBtn.pGPIOx = GPIOA;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_0;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD ;

		GPIO_PeriClockControl(GPIOA, ENABLE);
		GPIO_Init(&GpioBtn);

	while(1){
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED)
		{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}

	}

}


