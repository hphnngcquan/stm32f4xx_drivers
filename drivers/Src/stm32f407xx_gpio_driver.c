/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 23, 2022
 *      Author: NONAMENEEDED
 */
#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */
/**************************************************************************
 * @fn			-GPIO_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given GPIO Port
 *
 * @param[in]	-base addr of the gpio peripheral
 * @param[in]	-ENABLE or DISABLE macros
 * @param[in]
 *
 * @Return		-none
 *
 * @Note
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) //enable or disable the peripheral clock
{
	if (EnorDi == ENABLE){

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();

		}else if (pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}else if (pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}else if (pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}else if (pGPIOx==GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else{

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();

		}else if (pGPIOx==GPIOB){
			GPIOB_PCLK_DI();
		}else if (pGPIOx==GPIOC){
			GPIOC_PCLK_DI();
		}else if (pGPIOx==GPIOD){
			GPIOD_PCLK_DI();
		}else if (pGPIOx==GPIOE){
			GPIOE_PCLK_DI();
		}else if (pGPIOx==GPIOF){
			GPIOF_PCLK_DI();
		}else if (pGPIOx==GPIOG){
			GPIOG_PCLK_DI();
		}else if (pGPIOx==GPIOH){
			GPIOH_PCLK_DI();
		}else if (pGPIOx==GPIOI){
			GPIOI_PCLK_DI();
		}

	}

}






/*
 * Initialize and De-Initialize
 */






/**************************************************************************
 * @fn			-GPIO_Init
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */
void GPIO_Init	(GPIO_Handle_t *pGPIOHandle) //take pointer to the handle structure
{
	uint32_t temp = 0;
	//Enable Peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG){
		//non interrupt mode

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{
		if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)==GPIO_MODE_IT_FT)
		{
			//1. Config the falling trigger selection register(FTSR)

			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)==GPIO_MODE_IT_RT)
		{
			//1. Config the rising tiggr selection register(RTSR)
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if ((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)==GPIO_MODE_IT_RFT)
		{
			//1. Cofig both RTSR AND FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;
		uint32_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);


		//3. enable the exit interrupt delivery using IMR (Interrupt mask register)
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp=0;
	//2. Configure Speed
	temp =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//3. Configure the pupd settings
	temp =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//4. Configure optype
	temp =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//5. Configure the alt functionality
	/* in GPIO_PinConfig_t structure, the alternate functionality mode (variable GPIO_PinAltFunMode) only applicable when the mode is set to alternate function mode
	 * (which means pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALFN), so that we have the condition below */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//	configure alt function registers
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}

}






/**************************************************************************
 * @fn			-GPIO_DeInit
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) //sending that register back to its reset state
{


			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}else if (pGPIOx==GPIOB){
				GPIOB_REG_RESET();

			}else if (pGPIOx==GPIOC){
				GPIOC_REG_RESET();

			}else if (pGPIOx==GPIOD){
				GPIOD_REG_RESET();

			}else if (pGPIOx==GPIOE){
				GPIOE_REG_RESET();

			}else if (pGPIOx==GPIOF){
				GPIOF_REG_RESET();

			}else if (pGPIOx==GPIOG){
				GPIOG_REG_RESET();

			}else if (pGPIOx==GPIOH){
				GPIOH_REG_RESET();

			}else if (pGPIOx==GPIOI){
				GPIOI_REG_RESET();
			}
}





/*
 * Data Read Write
 */
/**************************************************************************
 * @fn			-GPIO_ReadFromInputPin
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-0 or 1
 *
 * @Note		-
 *
 */
uint8_t GPIO_ReadFromInputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR>> PinNumber) & 0x00000001);
	return value;

}




/**************************************************************************
 * @fn			-GPIO_ReadFromInputPort
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		- port number 0xXXXXXXXX (X=0 or 1)
 *
 * @Note		-
 *
 */
uint16_t GPIO_ReadFromInputPort	(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint8_t)pGPIOx->IDR;
	return value;

}




/**************************************************************************
 * @fn			-GPIO_WriteToOutputPin
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */
void GPIO_WriteToOutputPin		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if (Value == GPIO_PIN_SET)
	{
		//write 1 to the output data at the bit field corresponding pin number
		pGPIOx->ODR |= (1<<PinNumber);
	}else{
		pGPIOx->ODR &= ~(1<<PinNumber);
		//write 0
	}

}




/**************************************************************************
 * @fn			-GPIO_WriteToOutputPort
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */
void GPIO_WriteToOutputPort		(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;

}




/**************************************************************************
 * @fn			-GPIO_ToggleOutputPin
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */
void GPIO_ToggleOutputPin 		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^=(1<<PinNumber);

}

/*
 *IRQ Configuration and ISR Handling
 */




/**************************************************************************
 * @fn			-GPIO_IRQInterruptConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */
void GPIO_IRQInterruptConfig 	(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber<=31)
		{

			//program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);

		}else if (IRQNumber>31 && IRQNumber<61)
		{

			//Program ISER1 register
			*NVIC_ISER1 |= (1<<IRQNumber % 32);


		}else if (IRQNumber >=64 && IRQNumber <96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1<<IRQNumber % 64);

		}
	}else
	{
		if(IRQNumber<=31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);

		}else if (IRQNumber>31 && IRQNumber<61)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1<<IRQNumber % 32);

		}else if (IRQNumber >=64 && IRQNumber <96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1<<IRQNumber % 64);

		}
	}


}





/**************************************************************************
 * @fn			-GPIO_IRQPriorityConfig
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */

void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{

	//1. find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority<<shift_amount);


}



/**************************************************************************
 * @fn			-GPIO_IRQHandling
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-
 *
 */
void GPIO_IRQHandling 	(uint8_t PinNumber)
{
	// clear the EXTI pr register corresponding to the pin number
	if(EXTI->PR & (1<<PinNumber))
	{
		//clear the pending register bit
		EXTI-> PR |=(1<<PinNumber);
	}

}


