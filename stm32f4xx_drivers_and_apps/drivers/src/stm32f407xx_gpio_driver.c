/**
  ******************************************************************************
  * @file    stm32f407xx_gpio_driver.c
  * @author  Moe2Code
  * @brief   GPIO module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the General Purpose Input/Output (GPIO) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  */

/* Includes */
#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral Clock Setup
 */

/*******************************************************
*@fn					- GPIO_PeriClockControl
*
*@brief					- This function enables or disables peripheral clock for a given GPIO port
*
*@param[in]				- Base address of the GPIO peripheral
*@param[in]				- ENABLE OR DISABLE macros
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE){
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Initialization and De-initialization
 */

/*******************************************************
*@fn					- GPIO_Init
*
*@brief					- This function initializes a GPIO port
*
*@param[in]				- GPIO handle structure
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// 0. Enable peripheral clock. This code was added while conducting SPI exercise (L161))

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configuring mode of the GPIO port

	uint32_t temp = 0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <=GPIO_MODE_ANALOG)
	{
		// Non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); 			// resetting the appropriate pin in register
		pGPIOHandle->pGPIOx->MODER |=temp;																// setting the appropriate pin in register

	}else{
		// Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the FTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure both the FTSR and RTSR
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_PORTCODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] &= ~( 0xF << ( temp2 * 4));
		SYSCFG->EXTICR[temp1] |= portcode << ( temp2 * 4);


		//3. Enable the EXTI interrupt delivery using IMR (from EXTI to NVIC)
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// Configuring speed of the GPIO port

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;

	temp = 0;

	// Configuring the pull up and pull down mode of the pin on the GPIO port

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |=temp;

	temp = 0;

	// Configuring the output type of the pin on the GPIO port
	// It is a good idea to configure only if mode is output type. Change code and add if condition.

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &=~(0x1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  //clearing
	pGPIOHandle->pGPIOx->OTYPER |=temp;

	temp = 0;

	// Configuring the alternate functionality of the pin on the GPIO port

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;	// to determine if pin is on low or high AF register
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;	// to help determine pin location in register
		pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF<<(4*temp2));    // resetting the appropriate pin in register
		pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));  // setting the appropriate pin in register
	}
}

/*******************************************************
*@fn					- GPIO_DeInit
*
*@brief					- This function de-initializes a GPIO port (resets its registers)
*
*@param[in]				- Base address of the GPIO peripheral
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}

/*
 * Data Read and Write
 */

/*******************************************************
*@fn					- GPIO_ReadFromInputPin
*
*@brief					- This function reads from a specified pin on a GPIO port
*
*@param[in]				- Base address of the GPIO peripheral
*@param[in]				- Pin number to read from
*@param[in]				-
*
*@return				- 1 or 0
*
*@note					- None
*
*/


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 );

	return value;
}

/*******************************************************
*@fn					- GPIO_ReadFromInputPort
*
*@brief					- This function reads from all pins of a given GPIO port
*
*@param[in]				- Base address of the GPIO peripheral
*@param[in]				-
*@param[in]				-
*
*@return				- Unsigned 16-bit integer of data
*
*@note					- None
*
*/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*******************************************************
*@fn					- GPIO_WriteToOutputPin
*
*@brief					- This function writes to a specified pin on a GPIO port
*
*@param[in]				- Base address of the GPIO peripheral
*@param[in]				- Pin number to write to
*@param[in]				- Value to write to pin (1 or 0)
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |=(1<<PinNumber);  //write 1;
	}else
	{
		pGPIOx->ODR &=~(1<<PinNumber);  //write 0
	}
}

/*******************************************************
*@fn					- GPIO_WriteToOutputPort
*
*@brief					- This function writes to all pins of a given GPIO port
*
*@param[in]				- Base address of the GPIO peripheral
*@param[in]				- Value to write to all pins
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;

}

/*******************************************************
*@fn					- GPIO_ToggleOutputPin
*
*@brief					- This function toggles the output to a specified pin on a GPIO port
*
*@param[in]				- Base address of the GPIO peripheral
*@param[in]				- Pin number
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber );   //XOR operator, "^", used to toggle the appropriate pin
}

/*
 * IRQ Configuration and ISR Handling
 */


/*******************************************************
*@fn					- GPIO_IRQConfig
*
*@brief					- This function enables the IRQ number the EXTI interrupt is arriving at
*
*@param[in]				- IRQ number to configure
*@param[in]				- Enable and disable macros for the IRQ number
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber <= 63 )
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1<<IRQNumber%32);

		}else if(IRQNumber >= 64 && IRQNumber <= 95)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1<<IRQNumber%64);

		}
	}else{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1<<IRQNumber);

		}else if(IRQNumber >= 32 && IRQNumber <= 63 )
		{
			*NVIC_ICER1 |= (1<<IRQNumber%32);

		}else if(IRQNumber >= 64 && IRQNumber <= 95)
		{
			*NVIC_ICER2 |= (1<<IRQNumber%64);
		}

	}

}

/*******************************************************
*@fn					- GPIO_IRQPriorityConfig
*
*@brief					- This function configures the priority of the IRQ Number the GPIO interrupt is arriving at
*
*@param[in]				- IRQ Number
*@param[in]				- IRQ Priority
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4; 			// to determine in what register the priority of IRQNumber resides (IPR1 to IPR4)
	uint8_t iprx_section = IRQNumber %4;	// to determine where in IPRx the priority of IRQNumber resides
	uint8_t shift_amount = (iprx_section*8) +(8- NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx)&= ~( 0xF<< shift_amount ); // clearing before writing the priority. 0xF is chosen since NO_PR_BITS_IMPLEMENTED = 4
	*(NVIC_PR_BASEADDR + iprx)|= ( IRQPriority<< shift_amount );  // every increment of an uint32_t pointer type is 4 bytes
}



/*******************************************************
*@fn					- GPIO_IRQHandling
*
*@brief					- This function handles the interrupt on a specified pin
*
*@param[in]				- Pin number where the interrupt is configured
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the EXTI PR register bit corresponding to the EXTI line activated
	if(EXTI->PR&(1<<PinNumber))
	{
		// clear EXTI PR register bit
		EXTI->PR |=(1<<PinNumber);
	}
}
