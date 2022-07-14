/*
 * stm32f407_i2c_driver.h
 *
 *  Created on: Jun 16, 2022
 *      Author: NONAMENEEDED
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_
#include "stm32f407xx.h"


/*
 * Configuration structure for I2C peripheral
 */
typedef struct {
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle; //Fast mode duty cycle

}I2C_Config_t;

/*
 * Handle structure for I2C peripheral
 */
typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2CConfig;

}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM2K 	200000
#define I2C_SCL_SPEED_FM4K 	400000


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_DI 0
#define I2C_ACK_EN 1


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET


/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9
/*********************************************************************************************************************
 * 											APIssupported by this driver
 * 						For more information about APIs check the function definitions
 * *******************************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi); //enable or disable the peripheral clock

/*
 * Init and deinit
 */

void I2C_Init(I2C_Handle_t *pI2CHandle); //take pointer to the handle structure
void I2C_DeInit(I2C_RegDef_t *pI2Cx); //sending that register back to its reset state


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);



/*
 * IRQ configuration and ISR handling
 */

void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Other peripheral control API
 */
void I2C_PeriControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname);



/*
 * Application callback
 */
void I2C_ApplicationEventCallback (I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */