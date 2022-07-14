/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 8, 2022
 *      Author: NONAMENEEDED
 */

#include "stm32f407xx.h"
/*
 *Private helper function
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/**************************************************************************
 * @fn			-SPI_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]	-base addr of the spi peripheral
 * @param[in]	-ENABLE or DISABLE macros
 * @param[in]
 *
 * @Return		-none
 *
 * @Note
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if (EnorDi == DISABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx ==  SPI3)
		{
			SPI3_PCLK_DI();
		}


	}else

	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx ==  SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
}



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
void SPI_Init	(SPI_Handle_t *pSPIHandle)
{
	//Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//First configure the SPI_CR1 register
	uint32_t temp = 0;
	//1. configure device mode (master or slave)
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	//2. configure bus configuration
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI must be cleared
		temp &= ~(1<<SPI_CR1_BIDIMODE);

	} else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI must be set
		temp |= (1<<SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI must be cleared
		temp &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY must be set
		temp |= (1<<SPI_CR1_RXONLY);
	}
	//3. Configure the SPI peripheral clock speed
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;
	//4. Configure DFF
	temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//5. Configure CPOL
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	//6. Configure CPHA
	temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = temp;

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
void SPI_DeInit(SPI_RegDef_t *pSPIx)//sending that register back to its reset state
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagname)
{
	if (pSPIx->SR & Flagname){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/**************************************************************************
 * @fn			-SPI_SendData
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		- This is blocking call
 *
 */
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)

{
	while (Len>0)
	{
		//1. wait until TXE is set
	while(!(pSPIx->SR & (1<<SPI_SR_TXE)));//wait if while(1)
		//2. check the DFF bit in CR1
	if (pSPIx->CR1 & (1<< SPI_CR1_DFF))
	{
		//16 bit data frame format
		//1. Load the data into DR
		pSPIx->DR = *((uint16_t*)pTxBuffer);
		//2.Decrease the Length
		Len--;
		Len--;
		(uint16_t*)pTxBuffer ++;

	}else
	{
		//8 bit data frame format
		//1.Load the data into DR
		pSPIx->DR = *pTxBuffer;
		//2. Decrease the length
		Len--;
		pTxBuffer++;
	}
	}
}




/**************************************************************************
 * @fn			-SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len >0)
	{
		//1.Wait until the RXNE buffer os non empty
		while (!(pSPIx->SR &(1<<SPI_SR_RXNE)));
		if (pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16Bit
			//Read data 2bytes
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			//Increment the RX buffer addr
			(uint16_t*)pRxBuffer++;
			Len--;
			Len--;

		}else
		{
			//8Bit
			//Read data 1byte
			*(pRxBuffer) = pSPIx->DR;
			//Increment the RX buffer address
			pRxBuffer++;
			Len--;
		}

	}
}







/**************************************************************************
 * @fn			-SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		*NVIC_ISER1 |= (1<<IRQNumber % 32);
	}else
	{
		*NVIC_ICER1 |= (1<<IRQNumber % 32);
	}
}



/**************************************************************************
 * @fn			-SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;

	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+(iprx)) |= (IRQPriority<<shift_amount);
}








/**************************************************************************
 * @fn			-SPI_IRQHandling
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

void SPI_IRQHandling (SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//1. Check for TXE

	//test the status of TXE position
	temp1 = (pSPIHandle->pSPIx->SR & (1<<SPI_SR_TXE));

	//see if TXEIE is set or not
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));

	if (temp1 && temp2 )
	{
		//handle TXE,
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//2.Check for RXNE


	//test the status of RXNE position
	temp1 = (pSPIHandle->pSPIx->SR & (1<<SPI_SR_RXNE));

	//see if RXNEIE is set or not
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));

	if (temp1 && temp2 )
	{
		//handle RXNE,
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//3. check for OVERRUN ERROR
	//check if overrun error is occured or not
	temp1 = (pSPIHandle->pSPIx->SR & (1<<SPI_SR_OVR));

	//see if ERRIE(Error interrupt enable)  is set or not
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));
	if (temp1 && temp2 )
	{
		//handle RXNE,
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}


}





/**************************************************************************
 * @fn			-SPI_SendDataIT
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
uint8_t SPI_SendDataIT (SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle ->TxState;
	if ( state != SPI_BUSY_IN_TX)
	{
	//1. Save the Tx buffer address andLen information in some global variables
	pSPIHandle ->pTxBuffer = pTxBuffer;
	pSPIHandle ->TxLen = Len;

	//2. Mark the SPI state as busy in transmission so that
	//   no other code can take over the same SPI peripheral until transmission is over
	pSPIHandle ->TxState = SPI_BUSY_IN_TX;

	//3. Enable TXEIE (SPI_CR2) control bit to get interrupt whenever TXE flag is set in SR

		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}

	//4. Data transmission will be handled by the ISR (Interrupt service routine) code (will implement later)

	return state;
}


/**************************************************************************
 * @fn			-SPI_ReceiveDataIT
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
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle ->RxState;
	if ( state != SPI_BUSY_IN_RX)
	{
	//1. Save the Rx buffer address andLen information in some global variables
	pSPIHandle ->pRxBuffer = pRxBuffer;
	pSPIHandle ->RxLen = Len;

	//2. Mark the SPI state as busy in transmission so that
	//   no other code can take over the same SPI peripheral until transmission is over
	pSPIHandle ->RxState = SPI_BUSY_IN_RX;

	//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
	pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}


	return state;
}








/**************************************************************************
 * @fn			-SPI_PeriControl
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
void SPI_PeriControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);

	}
}


/**************************************************************************
 * @fn			-SPI_SSIConfig
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
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi ==0)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}



/**************************************************************************
 * @fn			-SPI_SSOEConfig
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi ==0)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}





/*
 * Helper function implementation
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. check the DFF bit in CR1
		if (pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF))
		{
			//16 bit data frame format
			//1. Load the data into DR
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			//2.Decrease the Length
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer ++;

		}else
		{
			//8 bit data frame format
			//1.Load the data into DR
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
			//2. Decrease the length
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}
		if(! (pSPIHandle->TxLen))
		{
			//TxLen is zero, close the SPI communication
			//inform the application that TX is over

			//prevent interupt from setting up of TXE flag
			SPI_CloseTransmission(pSPIHandle);

			SPI_ApplicationEventCallback (pSPIHandle,SPI_EVENT_TX_CMPLT);


		}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. check the DFF bit in CR1
		if (pSPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF))
		{
			//16 bit data frame format
			//1. Load the data into DR
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pRxBuffer);
			//2.Decrease the Length
			pSPIHandle->RxLen--;
			pSPIHandle->RxLen--;
			(uint16_t*)pSPIHandle->pRxBuffer ++;

		}else
		{
			//8 bit data frame format
			//1.Load the data into DR
			pSPIHandle->pSPIx->DR = *pSPIHandle->pRxBuffer;
			//2. Decrease the length
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}
		if(! (pSPIHandle->RxLen))
		{
			//RxLen is zero, close the SPI communication
			//inform the application that RX is over

			//prevent interupt from setting up of RXNEIE flag
			SPI_CloseReception(pSPIHandle);

			SPI_ApplicationEventCallback (pSPIHandle,SPI_EVENT_RX_CMPLT);


		}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. Infrom the Application
	SPI_ApplicationEventCallback (pSPIHandle,SPI_EVENT_OVR_ERR);
}






/**************************************************************************
 * @fn			-SPI_ClearOVRFlag
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
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	//1. Clear the OVR flag
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp;

}




/**************************************************************************
 * @fn			-SPI_CloseTransmission
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
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}




/**************************************************************************
 * @fn			-SPI_CloseReception
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
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}







/**************************************************************************
 * @fn			-SPI_ApplicationEventCallback
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
__weak void SPI_ApplicationEventCallback (SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	//this is a weak implementation, the application may override is function
}
