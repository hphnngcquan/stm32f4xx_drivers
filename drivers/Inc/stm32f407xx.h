/*
 * stm32f407xx.h
 *
 *  Created on: May 22, 2022
 *      Author: NONAMENEEDED
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
#include<stddef.h>


#define __vo volatile
#define __weak __attribute__((weak))

/*************************PROCESSOR SPECIFIC DETAILS*************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses,
 * only 81 IRQNumbers are implemented in the MicroController, so use up to ISER2 (95 IRQNumbers)
 */
#define NVIC_ISER0 			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 			((__vo uint32_t*)0xE000E10C)
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses,
 * only 81 IRQNumbers are implemented in the MicroController, so use up to ICER2 (95 IRQNumbers)
 */
#define NVIC_ICER0 			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1 			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2 			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3 			((__vo uint32_t*)0xE000E18C)

/*
 * Arm Cortex Mx Processor Priority Register Addr Calculation
 */
#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0xE000E400)


//In NVIC Number of priority which is implemented is 4 in ST case, in other vendor different

#define NO_PR_BITS_IMPLEMENTED 4



/*
 * base addresses of flash and SRAM memories
 */                                                                  /*macros explanation*/
#define FLASH_BASEADDR                            0x08000000U      //rm0090 Embedded Flash Memory Section
#define SRAM1_BASEADDR                            0x20000000U      //stm32f407vg memmap
#define SRAM2_BASEADDR                            0x2001C000U      //stm32f407vg memmap
#define ROM                                       0x1FFF0000U      //rm0090 Embedded Flash Memory Section, ROM = System Mem
#define SRAM                                      SRAM1_BASEADDR



/*
 * base ADDRESESS of bus domain AHBx APBx
 */
#define PERIPH_BASE                               0x40000000U
#define APB1PERIPH_BASE                           PERIPH_BASE      //
#define APB2PERIPH_BASE                           0x40010000U      // RM0090
#define AHB1PERIPH_BASE                           0x40020000U      // MEMMAP
#define AHB2PERIPH_BASE                           0x50000000U      //
#define AHB3PERIPH_BASE                           0xA0000000U      //


/*
 * Base addresses  of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR                           (AHB1PERIPH_BASE + 0x0000U) //RM0090 MEMMAP
#define GPIOB_BASEADDR                           (AHB1PERIPH_BASE + 0x0400U) //RM0090 MEMMAP
#define GPIOC_BASEADDR                           (AHB1PERIPH_BASE + 0x0800U) //RM0090 MEMMAP
#define GPIOC_BASEADDR                           (AHB1PERIPH_BASE + 0x0800U) //RM0090 MEMMAP
#define GPIOD_BASEADDR                           (AHB1PERIPH_BASE + 0x0C00U) //RM0090 MEMMAP
#define GPIOE_BASEADDR                           (AHB1PERIPH_BASE + 0x1000U) //RM0090 MEMMAP
#define GPIOF_BASEADDR                           (AHB1PERIPH_BASE + 0x1400U) //RM0090 MEMMAP
#define GPIOG_BASEADDR                           (AHB1PERIPH_BASE + 0x1800U) //RM0090 MEMMAP
#define GPIOH_BASEADDR                           (AHB1PERIPH_BASE + 0x1C00U) //RM0090 MEMMAP
#define GPIOI_BASEADDR                           (AHB1PERIPH_BASE + 0x2000U) //RM0090 MEMMAP
#define GPIOJ_BASEADDR                           (AHB1PERIPH_BASE + 0x2400U) //RM0090 MEMMAP
#define GPIOK_BASEADDR                           (AHB1PERIPH_BASE + 0x2800U) //RM0090 MEMMAP
#define RCC_BASEADDR 							 (AHB1PERIPH_BASE + 0x3800U) //RM0090 MEMMAP


/*
 * Base addresses  of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR                            (APB1PERIPH_BASE + 0x5400U) //RM0090 MEMMAP
#define I2C2_BASEADDR                            (APB1PERIPH_BASE + 0x5800U) //RM0090 MEMMAP
#define I2C3_BASEADDR                            (APB1PERIPH_BASE + 0x5C00U) //RM0090 MEMMAP


#define SPI2_BASEADDR                            (APB1PERIPH_BASE + 0x3800U) //RM0090 MEMMAP
#define SPI3_BASEADDR                            (APB1PERIPH_BASE + 0x3C00U) //RM0090 MEMMAP



#define USART2_BASEADDR                          (APB1PERIPH_BASE + 0x4800U) //RM0090 MEMMAP
#define USART3_BASEADDR                          (APB1PERIPH_BASE + 0x4400U) //RM0090 MEMMAP

#define UART4_BASEADDR                           (APB1PERIPH_BASE + 0x4C00U) //RM0090 MEMMAP
#define UART5_BASEADDR                           (APB1PERIPH_BASE + 0x5000U) //RM0090 MEMMAP


/*
 * Base addresses  of peripherals which are hanging on APB2 bus
 */
#define EXTI_BASEADDR                            (APB2PERIPH_BASE + 0x3C00U)//RM0090 MEMMAP

#define SPI1_BASEADDR                            (APB2PERIPH_BASE + 0x3000U)//RM0090 MEMMAP
#define SPI4_BASEADDR                            (APB2PERIPH_BASE + 0x3400U)//RM0090 MEMMAP
#define SPI5_BASEADDR                            (APB2PERIPH_BASE + 0x5000U)//RM0090 MEMMAP
#define SPI6_BASEADDR                            (APB2PERIPH_BASE + 0x5400U)//RM0090 MEMMAP

#define SYSCFG_BASEADDR                          (APB2PERIPH_BASE + 0x3800U)//RM0090 MEMMAP

#define USART1_BASEADDR                          (APB2PERIPH_BASE + 0x1000U)//RM0090 MEMMAP
#define USART6_BASEADDR                          (APB2PERIPH_BASE + 0x1400U)//RM0090 MEMMAP

/************peripheral register definition structure************/
/*
 * Note: Registers of a peripheral are specific to MCU
 * check device Reference Manual
 */

//GPIO Registers Definitions
typedef struct{

	__vo uint32_t MODER;    //GPIO port mode register,							offset 0x00
	__vo uint32_t OTYPER;	//GPIO port output type register,					offset 0x04
	__vo uint32_t OSPEEDR;	//GPIO port output speed register,					offset 0x08
	__vo uint32_t PUPDR;	//GPIO port pull-up/pull-down register,				offset 0x0C
	__vo uint32_t IDR;		//GPIO port input data register,					offset 0x10
	__vo uint32_t ODR;		//GPIO port output data register,					offset 0x14
	__vo uint32_t BSRR;		//GPIO port bit set/reset register,					offset 0x18
	__vo uint32_t LCKR;		//GPIO port configuration lock register,			offset 0x1C
	__vo uint32_t AFR[2]; 	//including offsets for AFRL(AFR[0],offset 0x20, altfun low) and AFRH (AFR[1],offset 0x24, alt fun high)

}GPIO_RegDef_t;


/*
 * SPI Registers Definition
 */

typedef struct{

	__vo uint32_t CR1;		//0x00
	__vo uint32_t CR2;		//0x04
	__vo uint32_t SR;		//0x08
	__vo uint32_t DR;		//0x0C
	__vo uint32_t CRCPR;	//0x10
	__vo uint32_t RXCRCR;	//0x14
	__vo uint32_t TXCRCR;	//0x18
	__vo uint32_t I2SCFGR;	//0x1C
	__vo uint32_t I2SPR;	//0x20


}SPI_RegDef_t;



/*
 *I2C Register Definition
 */
typedef struct {

	__vo uint32_t CR1;		//0x00
	__vo uint32_t CR2;		//0x04
	__vo uint32_t OAR1;		//0x08
	__vo uint32_t OAR2;		//0x0C
	__vo uint32_t DR;		//0x10
	__vo uint32_t SR1;		//0x14
	__vo uint32_t SR2;		//0x18
	__vo uint32_t CCR;		//0x1C
	__vo uint32_t TRISE;	//0x20
	__vo uint32_t FLTR;		//0x24

} I2C_RegDef_t;





/*
 * Peripheral register definition structure for EXTI
 */
typedef struct{
	__vo uint32_t IMR; 		//Interrupt mask register , offset 0x00
	__vo uint32_t EMR; 		//Event mask register , offset 0x04
	__vo uint32_t RTSR; 	//Rising trigger selection register , offset 0x08
	__vo uint32_t FTSR; 	//Falling trigger selection register, offset 0x0C
	__vo uint32_t SWIER; 	// Software interrupt event register, offset 0x10
	__vo uint32_t PR; 		// 	Pending Register, offset 0x14

}EXTI_RegDef_t;




//SYSCFG Registers Definitions
typedef struct{
	__vo uint32_t MEMRMP;		//SYSCFG memory remap register , offset 0x00
	__vo uint32_t PMC;			//SYSCFG peripheral mode configuration register, offset 0x04
	__vo uint32_t EXTICR[4];	//SYSCFG external interrupt configuration register 1: EXTICR[0], 2: EXTICR[1], 3: EXTICR[2], 4: EXTICR[3], offset 0x08-0x14
	uint32_t	  RESERVED[2];
	__vo uint32_t CMPCR;		//Compensation cell control register, offset 0x20

}SYSCFG_RegDef_t;



//RCC Registers Definitions
typedef struct{

	__vo uint32_t CR; 			//RCC clock control register, 			0x00
	__vo uint32_t PLLCFGR;		//RCC PLL configuration register , 		0x04
	__vo uint32_t CFGR;			//RCC clock configuration register, 	0x08
	__vo uint32_t CIR;			//RCC clock interrupt register, 		0x0c

	__vo uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register, 	0x10
	__vo uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register, 	0x14
	__vo uint32_t AHB3RSTR;		//RCC AHB3 peripheral reset register, 	0x18
	uint32_t RESERVED0;			//0x1C

	__vo uint32_t APB1RSTR;		//RCC APB1 peripheral reset register,	0x20
	__vo uint32_t APB2RSTR;		//RCC APB2 peripheral reset register,	0x24
	uint32_t RESERVED1;			//0x28
	uint32_t RESERVED2;			//0x2C

	__vo uint32_t AHB1ENR;		//RCC AHB1 peripheral clock register, 	0x30
	__vo uint32_t AHB2ENR;		//RCC AHB2 peripheral clock register, 	0x34
	__vo uint32_t AHB3ENR;		//RCC AHB3 peripheral clock register, 	0x38
	uint32_t RESERVED3;			//0x3C

	__vo uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register, 0x40
	__vo uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register, 0x44
	uint32_t RESERVED4;			//0x48
	uint32_t RESERVED5;			//0X4C

	__vo uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register, 0x50
	__vo uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register, 0x54
	__vo uint32_t AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register, 0x58
	uint32_t RESERVED6;			//0x5C

	__vo uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register, 0x60
	__vo uint32_t APB2LPENR;	//RCC APB2 peripheral clock enable in low power mode register, 0x64
	uint32_t RESERVED7;			//0x68
	uint32_t RESERVED8;			//0x6C

	__vo uint32_t BDCR;			//RCC Backup domain control register, 	0x70
	__vo uint32_t CSR;			//RCC clock control & status register, 	0x74
	uint32_t RESERVED9;			//0x78
	uint32_t RESERVED10;			//0x7C

	__vo uint32_t SSCGR;		//RCC spread spectrum clock generation register, 	0x80
	__vo uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register, 				0x84
	__vo uint32_t PLLSAICFGR;	//RCC PLL configuration register, 					0x88
	__vo uint32_t DCKCFGR;		//RCC Dedicated Clock Configuration Register, 		0x8C
}RCC_RegDef_t;









/*
 * peripheral definitions (Peripheral base addresses type casted to xxx_RedDef_t)
 */
#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)


#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)


#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)



/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define	GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1<<0)) //GPIOx Peripheral Clock Enable
#define	GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1<<1)) //GPIOx Peripheral Clock Enable
#define	GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1<<2)) //GPIOx Peripheral Clock Enable
#define	GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1<<3)) //GPIOx Peripheral Clock Enable
#define	GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1<<4)) //GPIOx Peripheral Clock Enable
#define	GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1<<5)) //GPIOx Peripheral Clock Enable
#define	GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1<<6)) //GPIOx Peripheral Clock Enable
#define	GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1<<7)) //GPIOx Peripheral Clock Enable
#define	GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1<<8)) //GPIOx Peripheral Clock Enable


/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define	I2C1_PCLK_EN()		(RCC->APB1ENR |= (1<<21))
#define	I2C2_PCLK_EN()		(RCC->APB1ENR |= (1<<22))
#define	I2C3_PCLK_EN()		(RCC->APB1ENR |= (1<<23))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define	SPI1_PCLK_EN()		(RCC->APB2ENR |= (1<<12))
#define	SPI2_PCLK_EN()		(RCC->APB1ENR |= (1<<14))
#define	SPI3_PCLK_EN()		(RCC->APB1ENR |= (1<<15))




/*
 * Clock Enable Macros for USARTx peripherals
 */


#define	USART2_PCLK_EN()		(RCC->APB1ENR |= (1<<17))
#define	USART3_PCLK_EN()		(RCC->APB1ENR |= (1<<18))
#define	USART1_PCLK_EN()		(RCC->APB2ENR |= (1<<4))
#define	USART6_PCLK_EN()		(RCC->APB2ENR |= (1<<5))


/*
 * Clock Enable Macros for SYSCFGx peripherals
 */

#define	SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1<<14))




/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define	GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<0)) //GPIOx Peripheral Clock Disable
#define	GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<1)) //GPIOx Peripheral Clock Disable
#define	GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<2)) //GPIOx Peripheral Clock Disable
#define	GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<3)) //GPIOx Peripheral Clock Disable
#define	GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<4)) //GPIOx Peripheral Clock Disable
#define	GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<5)) //GPIOx Peripheral Clock Disable
#define	GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<6)) //GPIOx Peripheral Clock Disable
#define	GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<7)) //GPIOx Peripheral Clock Disable
#define	GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<8)) //GPIOx Peripheral Clock Disable



/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define	I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1<<21))
#define	I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<22))
#define	I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define	SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<12))
#define	SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<14))
#define	SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<15))

/*
 * Clock Disable Macros for USARTx peripherals
 */


#define	USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define	USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define	USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define	USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1<<5))


/*
 * Clock Disable Macros for SYSCFGx peripherals
 */

#define	SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1<<14))


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));} while (0) //do 2 statements in 1 macro , while(0) so the statements are executed once
#define GPIOB_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));} while (0)
#define GPIOC_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));} while (0)
#define GPIOD_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));} while (0)
#define GPIOE_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));} while (0)
#define GPIOF_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));} while (0)
#define GPIOG_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));} while (0)
#define GPIOH_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));} while (0)
#define GPIOI_REG_RESET()		do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));} while (0)


/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));} while (0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));} while (0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));} while (0)

/*
 *Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()		do{(RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21));} while (0)
#define I2C2_REG_RESET()		do{(RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<22));} while (0)
#define I2C3_REG_RESET()		do{(RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<23));} while (0)











#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0:\
									(x == GPIOB) ? 1:\
									(x == GPIOC) ? 2:\
									(x == GPIOD) ? 3:\
									(x == GPIOE) ? 4:\
									(x == GPIOF) ? 5:\
									(x == GPIOG) ? 6:\
									(x == GPIOH) ? 7:0)

//syntax condition?true_value:false_value, "\" means check the next value

/*
 * IRQ exti numbers
 */
#define IRQ_NO_PVD			1
#define IRQ_NO_TAMP_STAMP	2
#define IRQ_NO_RTC_WKUP		3

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_RTC_Alarm	41
#define IRQ_NO_OTG_FS_WKUP	42
#define IRQ_NO_ETH_WKUP		62
#define IRQ_NO_OTG_HS_WKUP	76




/*
 * SPI interrupt IRQ numbers
 */
#define IRQ_NO_SPI1 35
#define IRQ_NO_SPI2 36
#define IRQ_NO_SPI3 51





/*
 * IRQ Priority levels
 */
#define NVIC_IRQ_PRIO0 0
#define NVIC_IRQ_PRIO1 1
#define NVIC_IRQ_PRIO2 2
#define NVIC_IRQ_PRIO3 3
#define NVIC_IRQ_PRIO4 4
#define NVIC_IRQ_PRIO5 5
#define NVIC_IRQ_PRIO6 6
#define NVIC_IRQ_PRIO7 7
#define NVIC_IRQ_PRIO8 8
#define NVIC_IRQ_PRIO9 9
#define NVIC_IRQ_PRIO10 10
#define NVIC_IRQ_PRIO11 11
#define NVIC_IRQ_PRIO12 12
#define NVIC_IRQ_PRIO13 13
#define NVIC_IRQ_PRIO14 14
#define NVIC_IRQ_PRIO15 15







//Some generic macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET		SET
#define FLAG_RESET		RESET

/**************************************************
 * Bit Position Definition of SPI peripheral
 ***************************************************/
/*
 * Bit position definition for CR1
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL 		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR 			3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST 	7
#define SPI_CR1_SSI 		8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY 		10
#define SPI_CR1_DFF 		11
#define SPI_CR1_CRCNEXT 	12
#define SPI_CR1_CRCEN 		13
#define SPI_CR1_BIDIOE 		14
#define SPI_CR1_BIDIMODE 	15

/*
 * Bit position definition for CR2
 */
#define SPI_CR2_RXDMAEN 	0
#define SPI_CR2_TXDMAEN 	1
#define SPI_CR2_SSOE 		2
#define SPI_CR2_Reserved 	3
#define SPI_CR2_FRF 		4
#define SPI_CR2_ERRIE 		5
#define SPI_CR2_RXNEIE 		6
#define SPI_CR2_TXEIE 		7

/*
 * Bit position definition for SR
 */
#define SPI_SR_RXNE 		0
#define SPI_SR_TXE 			1
#define SPI_SR_CHSIDE 		2
#define SPI_SR_UDR 			3
#define SPI_SR_CRCERR 		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR 			6
#define SPI_SR_BSY 			7
#define SPI_SR_FRE 			8


/*
 * Bit position definition for DR
 */
#define SPI_DR				0

//#define SPI_I2SCFGR_CHLEN	0
//#define SPI_I2SCFGR_DATLEN	1
//#define SPI_I2SCFGR_CKPOL	3
//#define SPI_I2SCFGR_I2SSTD	4
//#define SPI_I2SCFGR_PCMSYNC	7
//#define SPI_I2SCFGR_I2SCFG	8
//#define SPI_I2SCFGR_I2SE	10
//#define SPI_I2SCFGR_I2SMOD	11
//
//#define SPI_I2SPR_I2SDIV	0
//#define SPI_I2SPR_ODD		8
//#define SPI_I2SPR_MCKOE		9



/**************************************************
 * Bit Position Definition of I2C peripheral
 ***************************************************/

/*
 * Bit position definition for CR1
 */

#define I2C_CR1_PE 			0
#define I2C_CR1_SMBUS 		1
#define I2C_CR1_SMBTYPE 	3
#define I2C_CR1_ENARP 		4
#define I2C_CR1_ENPEC 		5
#define I2C_CR1_ENGC 		6
#define I2C_CR1_NOSTRETCH 	7
#define I2C_CR1_START 		8
#define I2C_CR1_STOP 		9
#define I2C_CR1_ACK 		10
#define I2C_CR1_POS 		11
#define I2C_CR1_PEC 		12
#define I2C_CR1_ALERT 		13
#define I2C_CR1_SWRST 		15

/*
 * Bit position definition for CR2
 */
#define I2C_CR2_FREQ 		0
#define I2C_CR2_ITERREN 	8
#define I2C_CR2_ITEVTEN 	9
#define I2C_CR2_ITBUFEN 	10
#define I2C_CR2_DMAEN 		11
#define I2C_CR2_LAST 		12


/*
 * Bit position definition for OAR1
 */
#define I2C_OAR1_ADD0 		0
#define I2C_OAR1_ADD_1_7 	1
#define I2C_OAR1_ADD0_8_9 	8
#define I2C_OAR1_KEPT_AT1 	14
#define I2C_OAR1_ADDMODE 	15


/*
 * Bit position definition for OAR2
 */
#define I2C_OAR2_ENDUAL 	0
#define I2C_OAR1_ADD2_1_7 	1


/*
 * Bit position definition for DR
 */
#define I2C_DR_DR 			0


/*
 * Bit position definition for SR1
 */
#define I2C_SR1_SB 			0
#define I2C_SR1_ADDR 		1
#define I2C_SR1_BTF 		2
#define I2C_SR1_ADD10 		3
#define I2C_SR1_STOPF 		4
#define I2C_SR1_RXNE 		6
#define I2C_SR1_TXE 		7
#define I2C_SR1_BERR 		8
#define I2C_SR1_ARLO 		9
#define I2C_SR1_AF 			10
#define I2C_SR1_OVR 		11
#define I2C_SR1_PECERR 		12
#define I2C_SR1_TIMEOUT 	14
#define I2C_SR1_SMBALERT 	15



/*
 * Bit position definition for SR2
 */
#define I2C_SR2_MSL 		0
#define I2C_SR2_BUSY 		1
#define I2C_SR2_TRA 		2
#define I2C_SR2_GENCALL 	4
#define I2C_SR2_SMBDEFAULT 	5
#define I2C_SR2_SMBHOST 	6
#define I2C_SR2_DUALF 		7
#define I2C_SR2_PEC 		8



/*
 * Bit position definition for CCR
 */
#define I2C_CCR_CCR 		0
#define I2C_CCR_DUTY 		14
#define I2C_CCR_FS 			15



/*
 * Bit position definition for TRISE
 */
#define I2C_TRISE_TRISE 	0



/*
 * Bit position definition for FLTR
 */
#define I2C_FLTR_DNF 		0
#define I2C_FLTR_ANOFF 		4





#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_spi_driver.h"
#include"stm32f407xx_i2c_driver.h"




#endif /* INC_STM32F407XX_H_ */
