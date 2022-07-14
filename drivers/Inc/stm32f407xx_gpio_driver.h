/*
 * stm32f407xx_gpio_driver.h
 * this iss DRIVER SPECIFIC HEAR FILE
 *  Created on: May 23, 2022
 *      Author: NONAMENEEDED
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct {
	uint8_t GPIO_PinNumber;			/*!<possible values from @GPIO_PIN_NUMBERS>*/
	uint8_t GPIO_PinMode; 			/*!<possible values from @GPIO_PIN_MODES>*/
	uint8_t GPIO_PinSpeed;			/*!<possible values from @GPIO_PIN_SPEED>*/
	uint8_t GPIO_PinPuPdControl;	/*!<possible values from @GPIO_PIN_PUPD>*/
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*
 * This is Handle STructrue for a GPIO pin
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx;		 /*This holds the base addr of the GPIO Port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin possible modes
 */
#define GPIO_PIN_NO_0	 0
#define GPIO_PIN_NO_1	 1
#define GPIO_PIN_NO_2	 2
#define GPIO_PIN_NO_3	 3
#define GPIO_PIN_NO_4	 4
#define GPIO_PIN_NO_5	 5
#define GPIO_PIN_NO_6	 6
#define GPIO_PIN_NO_7	 7
#define GPIO_PIN_NO_8	 8
#define GPIO_PIN_NO_9	 9
#define GPIO_PIN_NO_10	 10
#define GPIO_PIN_NO_11	 11
#define GPIO_PIN_NO_12	 12
#define GPIO_PIN_NO_13	 13
#define GPIO_PIN_NO_14	 14
#define GPIO_PIN_NO_15   15





/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define  GPIO_MODE_IN 		0
#define  GPIO_MODE_OU 		1
#define  GPIO_MODE_ALTFN 	2
#define  GPIO_MODE_ANALOG 	3
#define  GPIO_MODE_IT_FT 	4
#define  GPIO_MODE_IT_RT	5
#define  GPIO_MODE_IT_RFT	6

/*
 * GPIO pin possible OUTPUT modes
 */
#define GPIO_OP_TYPE_PP		0 //push-pull
#define GPIO_OP_TYPE_OD		1 //open drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible OUTPUT speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD 	0
#define GPIO_PIN_PU 	1
#define GPIO_PIN_PD		2

/*
 * @GPIO_PIN_ALTFN_MODES
 */
#define GPIO_MODE_AF1 1
#define GPIO_MODE_AF2 2
#define GPIO_MODE_AF3 3
#define GPIO_MODE_AF4 4
#define GPIO_MODE_AF5 5
#define GPIO_MODE_AF6 6
#define GPIO_MODE_AF7 7
#define GPIO_MODE_AF8 8
#define GPIO_MODE_AF9 9
#define GPIO_MODE_AF10 10
#define GPIO_MODE_AF11 11
#define GPIO_MODE_AF12 12
#define GPIO_MODE_AF13 13
#define GPIO_MODE_AF14 14
#define GPIO_MODE_AF15 15







/*********************************************************************************************************************
 * 											APIssupported by this driver
 * 						For more information about APIs check the function definitions
 * *******************************************************************************************************************/


/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi); //enable or disable the peripheral clock


/*
 * Initialize and De-Initialize
 */
void GPIO_Init	(GPIO_Handle_t *pGPIOHandle); //take pointer to the handle structure
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); //sending that register back to its reset state

/*
 * Data Read Write
 */
uint8_t GPIO_ReadFromInputPin	(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort	(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort		(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin 		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 *IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig 	(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling 	(uint8_t PinNumber);




#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
