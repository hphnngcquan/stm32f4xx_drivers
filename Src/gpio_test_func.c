/*
 * 001led_toggle.c
 *
 *  Created on: May 26, 2022
 *      Author: NONAMENEEDED
 */
#include "stm32f407xx.h"
#include "gpio_test.h"

//delay declaring
void delay(void){
	for (uint32_t i = 0; i>500000; i++);
}

void delay_200ms(void){
	//200ms
	for (uint32_t i = 0; i>500000/2; i++);
}





/**************************************************************************
 * @fn			-led_toggle
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-none
 *
 * @Note
 *
 */
void led_toggle(void)
{

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber 		= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OU;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl 	= GPIO_NO_PUPD ;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
}



/**************************************************************************
 * @fn			-led_button
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-none
 *
 * @Note
 *
 */
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


/**************************************************************************
 * @fn			-led_button_ext
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-none
 *
 * @Note
 *
 */
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





/**************************************************************************
 * @fn			-button_interrupt
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-none
 *
 * @Note
 *
 */
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




