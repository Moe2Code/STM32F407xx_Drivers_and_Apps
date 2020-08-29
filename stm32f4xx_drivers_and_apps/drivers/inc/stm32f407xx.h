/**
  ******************************************************************************
  * @file    stm32f407xx.h
  * @author  Moe2Code
  * @brief   STM32F407xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for peripherals
  *           - Peripherals registers declarations and bits definition
  *           - Macros to access peripherals registers hardware
*/

/* Define to prevent recursive inclusion */
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/* Includes */
#include<stddef.h>
#include<stdint.h>

/* Defines */
#define __vo volatile
#define __weak __attribute__((weak))


/*****************************************START:Processor Specific Details********************************************
 *
 * ARM Cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0							( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1							( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2							( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3							( (__vo uint32_t*)0xE000E10C )


/*
 * ARM Cortex Mx Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0							( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1							( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2							( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3							( (__vo uint32_t*)0XE000E18C )


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASEADDR					( (__vo uint32_t*)0xE000E400 )


/*
 * ARM Cortex Mx Processor number of priority bits used in the priority register
 */

#define NO_PR_BITS_IMPLEMENTED				4

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U			/* Base address for flash memory */
#define SRAM1_BASEADDR						0x20000000U			/* Base address for SRAM1 */
#define SRAM2_BASEADDR						0x2001C000U			/* Base address for SRAM2 */
#define ROM									0x1FFF0000U			/* Base address for ROM (system memory) */
#define SRAM								SRAM1_BASEADDR


/*
 * AHBx and APBx bus peripheral base addresses
 */

#define PERIPH_BASEADDR						0X40000000U			/* Peripherals base address */
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR		/* APB1 bus peripherals base address */
#define	APB2PERIPH_BASEADDR					0X40010000U			/* APB2 bus peripherals base address */
#define AHB1PERIPH_BASEADDR					0X40020000U			/* AHB1 bus peripherals base address */
#define AHB2PERIPH_BASEADDR					0X50000000U			/* AHB2 bus peripherals base address */


/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR+ 0X0000) 		/* GPIOA base address */
#define GPIOB_BASEADDR 						(AHB1PERIPH_BASEADDR+ 0X0400) 		/* GPIOB base address */
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR+ 0X0800) 		/* GPIOC base address */
#define GPIOD_BASEADDR  					(AHB1PERIPH_BASEADDR+ 0X0C00) 		/* GPIOD base address */
#define GPIOE_BASEADDR  					(AHB1PERIPH_BASEADDR+ 0X1000) 		/* GPIOE base address */
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR+ 0X1400) 		/* GPIOF base address */
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR+ 0X1800) 		/* GPIOG base address */
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR+ 0X1C00) 		/* GPIOH base address */
#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR+ 0X2000) 		/* GPIOI base address */

#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR+ 0X3800)


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR+ 0X5400) 		/* IC2C1 base address */
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR+ 0X5800) 		/* IC2C2 base address */
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR+ 0X5C00) 		/* IC2C3 base address */

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR+ 0X3800) 		/* SPI2 base address */
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR+ 0X3C00) 		/* SPI3 base address */

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR+ 0X4400) 		/*USART2 base address */
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR+ 0X4800) 		/*USART3 base address */
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR+ 0X4C00) 		/*UART4 base address */
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR+ 0X5000) 		/*UART5 base address */


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR+ 0X3C00)		/* EXTI base address */

#define SPI1_BASEADDR 						(APB2PERIPH_BASEADDR+ 0X3000)		/* SPI1 base address */

#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR+ 0X3800)		/* SYSCFG base address */

#define USART1_BASEADDR 					(APB2PERIPH_BASEADDR+ 0X1000)		/* USART1 base address */
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR+ 0X1400)		/* USART6 base address */


/*********************************Peripheral register definition structures***************************************/


/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;			/*GPIO port mode register							, address offset = 0x00 */
	__vo uint32_t OTYPER;		    /*GPIO port output type register					, address offset = 0x04 */
	__vo uint32_t OSPEEDR;			/*GPIO port output speed register					, address offset = 0x08 */
	__vo uint32_t PUPDR;			/*GPIO port pull-up/pull-down register				, address offset = 0x0C */
	__vo uint32_t IDR;				/*GPIO port input data register						, address offset = 0x10 */
	__vo uint32_t ODR;				/*GPIO port output data register					, address offset = 0x14 */
	__vo uint32_t BSRR;				/*GPIO port bit set/reset register					, address offset = 0x18 */
	__vo uint32_t LCKR;				/*GPIO port configuration lock register				, address offset = 0x1C */
	__vo uint32_t AFR[2];			/*AFR[0] GPIO alternate function low register		, address offset = 0x20 */
									/*AFR[1] GPIO alternate function high register		, address offset = 0x24 */
}GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */

typedef struct{

    __vo uint32_t CR;   		    /*RCC clock control register						   									, address offset = 0x00 */
	__vo uint32_t PLLCFGR;          /*RCC PLL configuration register														, address offset = 0x04 */
	__vo uint32_t CFGR;             /*RCC clock configuration register														, address offset = 0x08 */
	__vo uint32_t CIR;   			/*RCC clock interrupt register															, address offset = 0x0C */
	__vo uint32_t AHB1RSTR;  	    /*RCC AHB1 peripheral reset register													, address offset = 0x10 */
	__vo uint32_t AHB2RSTR;         /*RCC AHB2 peripheral reset register													, address offset = 0x14 */
	__vo uint32_t AHB3RSTR;   		/*RCC AHB3 peripheral reset register													, address offset = 0x18 */
	uint32_t 	  RESERVED0;  		/*Reserved																				, address offset = 0x1C */
	__vo uint32_t APB1RSTR;   		/*RCC APB1 peripheral reset register													, address offset = 0x20 */
	__vo uint32_t APB2RSTR;   		/*RCC APB2 peripheral reset register													, address offset = 0x24 */
	uint32_t      RESERVED1[2];  	/*Reserved																				, address offset = 0x28-2C */
	__vo uint32_t AHB1ENR;   		/*RCC AHB1 peripheral clock enable register												, address offset = 0x30 */
	__vo uint32_t AHB2ENR;  		/*RCC AHB2 peripheral clock enable register												, address offset = 0x34 */
	__vo uint32_t AHB3ENR;   		/*RCC AHB3 peripheral clock enable register												, address offset = 0x38 */
	uint32_t      RESERVED2;		/*Reserved                                                                              , address offset = 0x3C */
	__vo uint32_t APB1ENR;   		/*RCC APB1 peripheral clock enable register							                    , address offset = 0x40 */
	__vo uint32_t APB2ENR;  		/*RCC APB2 peripheral clock enable register							                    , address offset = 0x44 */
	uint32_t      RESERVED3[2];  	/*Reserved							                                                    , address offset = 0x48-4C */
	__vo uint32_t AHB1LPENR;   		/*RCC AHB1 peripheral clock enable in low power mode register							, address offset = 0x50 */
	__vo uint32_t AHB2LPENR;   		/*RCC AHB2 peripheral clock enable in low power mode register							, address offset = 0x54 */
	__vo uint32_t AHB3LPENR;   		/*RCC AHB3 peripheral clock enable in low power mode register							, address offset = 0x58 */
	uint32_t      RESERVED4;   		/*Reserved						                                                    	, address offset = 0x5C */
	__vo uint32_t APB1LPENR;   		/*RCC APB1 peripheral clock enable in low power mode register						 	, address offset = 0x60 */
	__vo uint32_t APB2LPENR;   		/*RCC APB2 peripheral clock enable in low power mode register							, address offset = 0x64 */
	uint32_t      RESERVED5[2]; 	/*Reserved																				, address offset = 0x68-6C */
	__vo uint32_t BDCR;   			/*RCC back up domain control register													, address offset = 0x70 */
	__vo uint32_t CSR;  			/*RCC clock control and status register													, address offset = 0x74 */
	uint32_t      RESERVED6[2];  	/*Reserved																				, address offset = 0x78-0x7C */
	__vo uint32_t SSCGR;   			/*RCC spread spectrum clock generation register											, address offset = 0x80 */
	__vo uint32_t PLLI2SCFGR;   	/*RCC PLLI2S configuration register														, address offset = 0x84 */

}RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;			/*EXTI line interrupt mask register							, address offset = 0x00 */
	__vo uint32_t EMR;		    /*EXTI line event mask register								, address offset = 0x04 */
	__vo uint32_t RTSR;			/*EXTI rising trigger selection register					, address offset = 0x08 */
	__vo uint32_t FTSR;			/*EXTI falling trigger selection register					, address offset = 0x0C */
	__vo uint32_t SWIER;		/*EXTI software interrupt event register					, address offset = 0x10 */
	__vo uint32_t PR;			/*EXTI pending register										, address offset = 0x14 */
}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;			/*SYSCFG memory re-map register							, address offset = 0x00 */
	__vo uint32_t PMC;		    	/*SYSCFG peripheral mode configuration register			, address offset = 0x04 */
	__vo uint32_t EXTICR[4];		/*SYSCFG external interrupt configuration register		, address offset = 0x08 - 0x14 */
	uint32_t 	  RESERVED0[2];		/*Reserved												, address offset = 0x18 - 0x1C */
	__vo uint32_t CMPCR;			/*Compensation cell control register					, address offset = 0x20 */
}SYSCFG_RegDef_t;


/*
 * Peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t CR1;			/*SPI control register 1													, address offset = 0x00 */
	__vo uint32_t CR2;		    /*SPI control register 2													, address offset = 0x04 */
	__vo uint32_t SR;			/*SPI status register														, address offset = 0x08 */
	__vo uint32_t DR;			/*SPI data register															, address offset = 0x0C */
	__vo uint32_t CRCPR;		/*SPI CRC polynomial register (not used in I2S mode)						, address offset = 0x10 */
	__vo uint32_t RXCRCR;		/*SPI RX CRC register (not used in I2S mode)								, address offset = 0x14 */
	__vo uint32_t TXCRCR;		/*SPI TX CRC register (not used in I2S mode)								, address offset = 0x18 */
	__vo uint32_t I2SCFGR;		/*SPI I2S configuration register											, address offset = 0x1C */
	__vo uint32_t I2SPR;		/*SPI I2S prescalar register												, address offset = 0x20 */
}SPI_RegDef_t;


/*
 * Peripheral register definition structure for I2C
 */

typedef struct
{
	__vo uint32_t CR1;			/*I2C control register 1													, address offset = 0x00 */
	__vo uint32_t CR2;		    /*I2C control register 2													, address offset = 0x04 */
	__vo uint32_t OAR1;			/*I2C own address register 1												, address offset = 0x08 */
	__vo uint32_t OAR2;			/*I2C own address register 2												, address offset = 0x0C */
	__vo uint32_t DR;			/*I2C data register 														, address offset = 0x10 */
	__vo uint32_t SR1;			/*I2C status register 1														, address offset = 0x14 */
	__vo uint32_t SR2;			/*I2C status register 2														, address offset = 0x18 */
	__vo uint32_t CCR;			/*I2C clock control register												, address offset = 0x1C */
	__vo uint32_t TRISE;		/*I2C TRISE register														, address offset = 0x20 */
	__vo uint32_t FLTR;			/*I2C FLTR register															, address offset = 0x24 */
}I2C_RegDef_t;


/*
 * Peripheral register definition structure for USART
 */

typedef struct
{
	__vo uint32_t SR;			/*USART status register													, address offset = 0x00 */
	__vo uint32_t DR;		    /*USART data register													, address offset = 0x04 */
	__vo uint32_t BRR;			/*USART baud rate register												, address offset = 0x08 */
	__vo uint32_t CR1;			/*USART control register 1												, address offset = 0x0C */
	__vo uint32_t CR2;			/*USART control register 2 												, address offset = 0x10 */
	__vo uint32_t CR3;			/*USART control register 3												, address offset = 0x14 */
	__vo uint32_t GTPR;			/*USART guard time and prescaler register								, address offset = 0x18 */
}USART_RegDef_t;


/*
 * Peripheral pointer definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1				((USART_RegDef_t*)USART1_BASEADDR)
#define USART2				((USART_RegDef_t*)USART2_BASEADDR)
#define USART3				((USART_RegDef_t*)USART3_BASEADDR)
#define UART4				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6				((USART_RegDef_t*)USART6_BASEADDR)



/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |=(1<<0))      /* Enabling GPIOA by turning on the appropriate bit in AHB1ENR */
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |=(1<<1))      /* Enabling GPIOB by turning on the appropriate bit in AHB1ENR */
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |=(1<<2))      /* Enabling GPIOC by turning on the appropriate bit in AHB1ENR */
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |=(1<<3))      /* Enabling GPIOD by turning on the appropriate bit in AHB1ENR */
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |=(1<<4))      /* Enabling GPIOE by turning on the appropriate bit in AHB1ENR */
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |=(1<<5))      /* Enabling GPIOF by turning on the appropriate bit in AHB1ENR */
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |=(1<<6))      /* Enabling GPIOG by turning on the appropriate bit in AHB1ENR */
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |=(1<<7))      /* Enabling GPIOH by turning on the appropriate bit in AHB1ENR */
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |=(1<<8))      /* Enabling GPIOI by turning on the appropriate bit in AHB1ENR */


/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()			(RCC->APB1ENR |=(1<<21))  	  /* Enabling I2C1 by turning on the appropriate bit in APB1ENR */
#define I2C2_PCLK_EN()			(RCC->APB1ENR |=(1<<22))  	  /* Enabling I2C2 by turning on the appropriate bit in APB1ENR */
#define I2C3_PCLK_EN()			(RCC->APB1ENR |=(1<<23))  	  /* Enabling I2C3 by turning on the appropriate bit in APB1ENR */


/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()			(RCC->APB2ENR |=(1<<12))  	  /* Enabling SPI1 by turning on the appropriate bit in APB2ENR */
#define SPI2_PCLK_EN()			(RCC->APB1ENR |=(1<<14))  	  /* Enabling SPI2 by turning on the appropriate bit in APB1ENR */
#define SPI3_PCLK_EN()			(RCC->APB1ENR |=(1<<15))  	  /* Enabling SPI3 by turning on the appropriate bit in APB1ENR */


/*
 * Clock enable macros for USARTx peripherals
 */

#define USART2_PCLK_EN()		(RCC->APB1ENR |=(1<<17))  	  /* Enabling USART2 by turning on the appropriate bit in APB1ENR */
#define USART3_PCLK_EN()		(RCC->APB1ENR |=(1<<18))  	  /* Enabling USART3 by turning on the appropriate bit in APB1ENR */
#define UART4_PCLK_EN()			(RCC->APB1ENR |=(1<<19))  	  /* Enabling UART4 by turning on the appropriate bit in APB1ENR */
#define UART5_PCLK_EN()			(RCC->APB1ENR |=(1<<20))  	  /* Enabling UART5 by turning on the appropriate bit in APB1ENR */
#define USART1_PCLK_EN()		(RCC->APB2ENR |=(1<<4))  	  /* Enabling USART1 by turning on the appropriate bit in APB2ENR */
#define USART6_PCLK_EN()		(RCC->APB2ENR |=(1<<5))  	  /* Enabling USART6 by turning on the appropriate bit in APB2ENR */


/*
 * Clock enable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |=(1<<14))  	  /* Enabling SYSCFG by turning on the appropriate bit in APB2ENR */

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &=~(1<<0))      /* Disabling GPIOA by turning off the appropriate bit in AHB1ENR */
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &=~(1<<1))      /* Disabling GPIOB by turning off the appropriate bit in AHB1ENR */
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &=~(1<<2))      /* Disabling GPIOC by turning off the appropriate bit in AHB1ENR */
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &=~(1<<3))      /* Disabling GPIOD by turning off the appropriate bit in AHB1ENR */
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &=~(1<<4))      /* Disabling GPIOE by turning off the appropriate bit in AHB1ENR */
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &=~(1<<5))      /* Disabling GPIOF by turning off the appropriate bit in AHB1ENR */
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &=~(1<<6))      /* Disabling GPIOG by turning off the appropriate bit in AHB1ENR */
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &=~(1<<7))      /* Disabling GPIOH by turning off the appropriate bit in AHB1ENR */
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &=~(1<<8))      /* Disabling GPIOI by turning off the appropriate bit in AHB1ENR */


/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()			(RCC->APB1ENR &=~(1<<21))  	  /* Disabling I2C1 by turning off the appropriate bit in APB1ENR */
#define I2C2_PCLK_DI()			(RCC->APB1ENR &=~(1<<22))  	  /* Disabling I2C2 by turning off the appropriate bit in APB1ENR */
#define I2C3_PCLK_DI()			(RCC->APB1ENR &=~(1<<23))  	  /* Disabling I2C3 by turning off the appropriate bit in APB1ENR */


/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()			(RCC->APB2ENR &=~(1<<12))  	  /* Disabling SPI1 by turning off the appropriate bit in APB2ENR */
#define SPI2_PCLK_DI()			(RCC->APB1ENR &=~(1<<14))  	  /* Disabling SPI2 by turning off the appropriate bit in APB1ENR */
#define SPI3_PCLK_DI()			(RCC->APB1ENR &=~(1<<15))  	  /* Disabling SPI3 by turning off the appropriate bit in APB1ENR */


/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		(RCC->APB2ENR &=~(1<<4))  	  /* Disabling USART1 by turning off the appropriate bit in APB2ENR */
#define USART2_PCLK_DI()		(RCC->APB1ENR &=~(1<<17))  	  /* Disabling USART2 by turning off the appropriate bit in APB1ENR */
#define USART3_PCLK_DI()		(RCC->APB1ENR &=~(1<<18))  	  /* Disabling USART3 by turning off the appropriate bit in APB1ENR */
#define UART4_PCLK_DI()			(RCC->APB1ENR &=~(1<<19))  	  /* Disabling UART4 by turning off the appropriate bit in APB1ENR */
#define UART5_PCLK_DI()			(RCC->APB1ENR &=~(1<<20))  	  /* Disabling UART5 by turning off the appropriate bit in APB1ENR */
#define USART6_PCLK_DI()		(RCC->APB2ENR &=~(1<<5))  	  /* Disabling USART6 by turning off the appropriate bit in APB2ENR */


/*
 * Clock disable macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &=~(1<<14))  	  /* Disabling SYSCFG by turning off the appropriate bit in APB2ENR */


/*
 * GPIO port reset macros
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &=~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &=~(1<<1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &=~(1<<2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &=~(1<<3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &=~(1<<4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &=~(1<<5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &=~(1<<6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &=~(1<<7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |=(1<<8)); (RCC->AHB1RSTR &=~(1<<8)); }while(0)


/*
 * SPI peripheral reset macros
 */

#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |=(1<<12)); (RCC->APB2RSTR &=~(1<<12)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<14)); (RCC->APB1RSTR &=~(1<<14)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<15)); (RCC->APB1RSTR &=~(1<<15)); }while(0)


/*
 * I2C peripheral reset macros
 */

#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<21)); (RCC->APB1RSTR &=~(1<<21)); }while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<22)); (RCC->APB1RSTR &=~(1<<22)); }while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<23)); (RCC->APB1RSTR &=~(1<<23)); }while(0)


/*
 * USART peripheral reset macros
 */

#define USART1_REG_RESET()		do{ (RCC->APB2RSTR |=(1<<4));  (RCC->APB2RSTR &=~(1<<4)); }while(0)
#define USART2_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<17)); (RCC->APB1RSTR &=~(1<<17)); }while(0)
#define USART3_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<18)); (RCC->APB1RSTR &=~(1<<18)); }while(0)
#define UART4_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<19)); (RCC->APB1RSTR &=~(1<<19)); }while(0)
#define UART5_REG_RESET()		do{ (RCC->APB1RSTR |=(1<<20)); (RCC->APB1RSTR &=~(1<<20)); }while(0)
#define USART6_REG_RESET()		do{ (RCC->APB2RSTR |=(1<<5));  (RCC->APB2RSTR &=~(1<<5)); }while(0)


/*
 * Macros to return port code for a given GPIOx base address
 */

#define GPIO_BASEADDR_TO_PORTCODE(x)	  ( (x==GPIOA)? 0:\
											(x==GPIOB)?	1:\
											(x==GPIOC)? 2:\
											(x==GPIOD)? 3:\
											(x==GPIOE)? 4:\
											(x==GPIOF)? 5:\
											(x==GPIOG)? 6:\
											(x==GPIOH)? 7:\
											(x==GPIOI)? 8:0 )


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73
#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53
#define IRQ_NO_USART6			71



/*
 * Macros for NVIC IRQ Priorities
 */

#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15



// Some generic macros

#define ENABLE				 1
#define DISABLE 			 0
#define SET 				 ENABLE
#define RESET 				 DISABLE
#define GPIO_PIN_SET		 SET
#define GPIO_PIN_RESET	     RESET
#define FLAG_SET			 SET
#define FLAG_RESET			 RESET


/*********************************Bit position definitions of SPI peripheral***************************************/

/*
 * Bit position for SPI control register 1 (SPI_CR1)
 */

#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFIRST				7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRCNEXT					12
#define SPI_CR1_CRCEN					13
#define SPI_CR1_BIDIOE					14
#define SPI_CR1_BIDIMODE				15

/*
 * Bit position for SPI control register 2 (SPI_CR2)
 */

#define SPI_CR2_RXDMAEN					0
#define SPI_CR2_TXDMAEN					1
#define SPI_CR2_SSOE					2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE					6
#define SPI_CR2_TXEIE					7

/*
 * Bit position for SPI status register (SPI_SR)
 */

#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRCERR					4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7
#define SPI_SR_FRE						8


/*********************************Bit position definitions of I2C peripheral***************************************/

/*
 * Bit position for I2C control register 1 (I2C_CR1)
 */

#define I2C_CR1_PE						0
#define I2C_CR1_SMBus					1
#define I2C_CR1_SMBType					3
#define I2C_CR1_ENARP					4
#define I2C_CR1_ENPEC					5
#define I2C_CR1_ENGC					6
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START					8
#define I2C_CR1_STOP					9
#define I2C_CR1_ACK						10
#define I2C_CR1_POS						11
#define I2C_CR1_PEC						12
#define I2C_CR1_ALERT					13
#define I2C_CR1_SWRST					15

/*
 * Bit position for I2C control register 2 (I2C_CR2)
 */

#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12

/*
 * Bit position for I2C status register 1 (I2C_SR1)
 */

#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RXNE					6
#define I2C_SR1_TXE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15

/*
 * Bit position for I2C status register 2 (I2C_SR2)
 */

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT				5
#define I2C_SR2_SMBHOST					6
#define I2C_SR2_DUALF					7
#define I2C_SR2_PEC						8

/*
 * Bit position for I2C clock control register (I2C_CCR)
 */

#define I2C_CCR_CCR						0
#define I2C_CCR_DUTY					14
#define I2C_CCR_FS						15

/*
 * Bit position for I2C own address register 1 (I2C_OAR1)
 */

#define I2C_OAR1_ADDMODE				15


/*********************************Bit position definitions of USART peripheral***************************************/

/*
 * Bit position for USART clock control register (USART_SR)
 */

#define USART_SR_PE						0
#define USART_SR_FE						1
#define USART_SR_NF						2
#define USART_SR_ORE					3
#define USART_SR_IDLE					4
#define USART_SR_RXNE					5
#define USART_SR_TC						6
#define USART_SR_TXE					7
#define USART_SR_LBD					8
#define USART_SR_CTS					9


/*
 * Bit position for control register 1 (USART_CR1)
 */

#define USART_CR1_SBK					0
#define USART_CR1_RWU					1
#define USART_CR1_RE					2
#define USART_CR1_TE					3
#define USART_CR1_IDLEIE				4
#define USART_CR1_RXNEIE				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE					8
#define USART_CR1_PS					9
#define USART_CR1_PCE					10
#define USART_CR1_WAKE					11
#define USART_CR1_M						12
#define USART_CR1_UE					13
#define USART_CR1_OVER8					15


/*
 * Bit position for control register 2 (USART_CR2)
 */

#define USART_CR2_ADD					0
#define USART_CR2_LBDL					5
#define USART_CR2_LBDIE					6
#define USART_CR2_LBCL					8
#define USART_CR2_CPHA					9
#define USART_CR2_CPOL					10
#define USART_CR2_CLKEN					11
#define USART_CR2_STOP					12
#define USART_CR2_LINEN					14


/*
 * Bit position for control register 3 (USART_CR3)
 */

#define USART_CR3_EIE					0
#define USART_CR3_IREN					1
#define USART_CR3_IRLP					2
#define USART_CR3_HDSEL					3
#define USART_CR3_NACK					4
#define USART_CR3_SCEN					5
#define USART_CR3_DMAR					6
#define USART_CR3_DMAT					7
#define USART_CR3_RTSE					8
#define USART_CR3_CTSE					9
#define USART_CR3_CTSIE					10
#define USART_CR3_ONEBIT				11


/*
 * Bit position for guard time and prescaler register (USART_GTPR)
 */

#define USART_GTPR_PSC					0
#define USART_GTPR_GT					8


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
