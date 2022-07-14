/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: Jun 16, 2022
 *      Author: NONAMENEEDED
 */
#include "stm32f407xx.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};


//helper function
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t  SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);

/**************************************************************************
 * @fn			-I2C_GenerateStartCondition
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-Private
 *
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}


/**************************************************************************
 * @fn			-I2C_GenerateStopCondition
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-Private
 *
 */
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}


/**************************************************************************
 * @fn			-I2C_ExecuteAddressPhase
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-Private
 *
 */
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t  SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; //makes space for r/w bit
	SlaveAddr &= ~(1<<0); //slave addr + r/w bit
	pI2Cx->DR = SlaveAddr;
}




/**************************************************************************
 * @fn			-I2C_ClearADDRFlag
 *
 * @brief		-
 *
 * @param[in]	-
 * @param[in]	-
 * @param[in]
 *
 * @Return		-
 *
 * @Note		-Private
 *
 */
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;

}




/**************************************************************************
 * @fn			-I2C_PeriControl
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
void I2C_PeriControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);

	}
}






/**************************************************************************
 * @fn			-I2C_PeriClockControl
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
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}

}




uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);
	if (clksrc == 0)
	{
		SystemClk = 16000000;
	}else if (clksrc == 0)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		//SystemClk = RCC_GetPLLOutputClock();
	}

	//for AHB
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if (temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//For APB1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if (temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}
	pclk1 = (SystemClk / ahbp) / apb1p;


	return pclk1;
}

/**************************************************************************
 * @fn			-I2C_Init
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
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

	//16/6 i did myself

	uint32_t tempreg = 0;

	//Enable the clock for the i2c peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ACK control bit
	tempreg |= pI2CHandle->I2CConfig.I2C_ACKControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//COnfigure FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Program the device own address
	tempreg |= pI2CHandle->I2CConfig.I2C_DeviceAddress << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//Configure the clock control register  (CCR Calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode standard mode
		ccr_value =  (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2CConfig.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << 14);

		if (pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value =  (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2CConfig.I2C_SCLSpeed));
		}else
		{
			ccr_value =  (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2CConfig.I2C_SCLSpeed));

		}

		tempreg |= (ccr_value & 0xFFF);



	}
	pI2CHandle->pI2Cx->CCR = tempreg;







}






/**************************************************************************
 * @fn			-I2C_DeInit
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
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
 {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}







/**************************************************************************
 * @fn			-I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	//2. Confirm that generation is completed by checking the SB flag in the SR1
	//   Note: Until the SB is cleared SCL will be stretched  (Pulled to LOW)
	while  ( !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));


	//3. Send the address of the slave with read/nwrite bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);


	//4. Confirm that address phase is completed by checking the ADDR flag in SSR1
	while  ( !	I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//5. Clear the ADDR flag according to its software sequence
	//	 Note: Until ADDR is cleared SCL will be stretched
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);


	//6. Send data until len become 0

	while (Len>0)
	{
		 while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //wait until TXE flag is set
		 pI2CHandle->pI2Cx->DR = *pTxBuffer;
		 pTxBuffer++;
		 Len--;
	}

	//7. When Len becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	//	 Note: TXE = 1, BTF = 1,  means that both SR and DR are empty and next transmissiOn should begin
	// 	 When BTF = 1 SCL will be stretched
	 while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //wait until TXE flag is set
	 while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); //wait until BTF flag is set


	//8. Generate STOP condition and master need not to wait for the completion of stop condition
	//	 NOTE: Generating STOP, automatically clears the BTF
	 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}



/**************************************************************************
 * @fn			-I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn			-I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;

	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+(iprx)) |= (IRQPriority<<shift_amount);
}





/**************************************************************************
 * @fn			-I2C_GetFlagStatus
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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname)
{
	if ((pI2Cx->SR1 & Flagname)||(pI2Cx->SR2 & Flagname)){
		return FLAG_SET;
	}
	return FLAG_RESET;
}




/**************************************************************************
 * @fn			-I2C_ApplicationEventCallback
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
__weak void I2C_ApplicationEventCallback (I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	//this is a weak implementation, the application may override is function
}


