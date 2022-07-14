/*
 * 005button_interrupt.c
 *
 *  Created on: May 28, 2022
 *      Author: NONAMENEEDED
 */
#include"stm32f407xx.h"
#include<string.h>
#define 	HIGH 			ENABLE
#define 	LOW 			DISABLE
#define 	BTN_PRESSED 	LOW

//delay declaring
void delay_200ms(void){
	//200ms
	for (uint32_t i = 0; i>500000/2; i++);
}

void button_interrupt(void)
{

	GPIO_Handle_t GpioLed,GpioBtn;
	memset (&GpioLed,0,sizeof(GpioLed)); //copies the character '0'(an unsigned char) to the first sizeof(GpioLed) characters of the string pointed &GpioLed.
	memset (&GpioBtn,0,sizeof(GpioBtn));


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
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IT_FT; // we use GPIOD Port 5 to trigger the interrupt, so PinMode is IT_FT
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_PIN_PU ;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioBtn);

	//IRQConfig
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);

	while(1);
}


void EXTI9_5_IRQHandler (void){
	delay_200ms();//wait for 200ms
	GPIO_IRQHandling(GPIO_PIN_NO_5);//clear the pending event from EXTI line, means set PR (pending) register
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}







