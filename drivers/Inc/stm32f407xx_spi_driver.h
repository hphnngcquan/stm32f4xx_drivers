/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 8, 2022
 *      Author: NONAMENEEDED
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


/*
 * Configuration structure for SPI peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 * Handle Structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; /* !<To store the app. Tx buffer address>*/
	uint8_t 		*pRxBuffer; /* !<To store the app. Rx buffer address>*/
	uint8_t 		TxLen; 		/* !<To store Tx Length >*/
	uint8_t 		RxLen; 		/* !<To store Rx Length>*/
	uint8_t 		TxState;	/* !<To store Tx State>*/
	uint8_t 		RxState;	/* !<To store Rx state>*/

} SPI_Handle_t;


/*
 * Possible SPI Application states
 */
#define SPI_READY 		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX 	2


/*
 * Possible SPI application event
 */
#define SPI_EVENT_TX_CMPLT 	1
#define SPI_EVENT_RX_CMPLT 	2
#define SPI_EVENT_OVR_ERR 	3






/*
 * @SPI_DeviceModes
 */
#define SPI_DEVICE_MODE_MASTER 	1
#define SPI_DEVICE_MODE_SLAVE 	0 //default

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPD_DIV2 0
#define SPI_SCLK_SPD_DIV4 1
#define SPI_SCLK_SPD_DIV8 2
#define SPI_SCLK_SPD_DIV16 3
#define SPI_SCLK_SPD_DIV32 4
#define SPI_SCLK_SPD_DIV64 5
#define SPI_SCLK_SPD_DIV128 6
#define SPI_SCLK_SPD_DIV256 7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0


/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW 0
#define SPI_CPHA_HIGH 1

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN 1
#define SPI_SSM_DI 0 //DEFAULT





/*********************************************************************************************************************
 * 											APIssupported by this driver
 * 						For more information about APIs check the function definitions
 * *******************************************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi); //enable or disable the peripheral clock

/*
 * Init and deinit
 */

void SPI_Init(SPI_Handle_t *pSPIHandle); //take pointer to the handle structure
void SPI_DeInit(SPI_RegDef_t *pSPIx); //sending that register back to its reset state


/*
 * Data Send and Receive
 */
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);

/*
 * Data send and receive with interrupt
 */
uint8_t SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len);




/*
 * IRQ configuration and ISR handling
 */

void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling (SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral control API
 */
void SPI_PeriControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagname);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);




/*
 * Application callback
 */
__weak void SPI_ApplicationEventCallback (SPI_Handle_t *pSPIHandle,uint8_t AppEv);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
