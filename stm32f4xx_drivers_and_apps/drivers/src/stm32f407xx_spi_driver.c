/**
  ******************************************************************************
  * @file    stm32f407xx_spi_driver.c
  * @author  Moe2Code
  * @brief   SPI module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Serial Peripheral Interface (SPI) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  */

/* Includes */
#include "stm32f407xx_spi_driver.h"


/* Private function prototypes */
// Interrupt implementation helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Peripheral Clock Setup
 */

/*******************************************************
*@fn					- SPI_PeriClockControl
*
*@brief					- This function enables or disables peripheral clock for a SPI peripheral
*
*@param[in]				- Base address of the SPI peripheral
*@param[in]				- ENABLE OR DISABLE macros
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


/*
 * Initialization and De-initialization
 */

/*******************************************************
*@fn					- SPI_Init
*
*@brief					- This function initializes a SPI peripheral
*
*@param[in]				- SPI handle structure
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	// 0. Enable peripheral clock

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure the SPI_CR1 register

	uint32_t tempreg = 0;

	// 1. configure the device mode

	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	// 2. configure the bus config

	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_SMPLX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. configure the bus DFF

	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// 4. configure the bus clock CPHA

	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	// 5. configure the bus clock CPOL

	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// 6. configure the SPI serial clock speed (baud rate)

	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	// 7. configure the bus SSM

	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << 9);

	pSPIHandle->pSPIx->CR1 = tempreg;
}


/*******************************************************
*@fn					- SPI_DeInit
*
*@brief					- This function de-initializes a SPI peripheral
*
*@param[in]				- Base address of the SPI peripheral
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}


/*******************************************************
*@fn					- SPI_GetFlagStatus
*
*@brief					- This function checks if a certain flag is set or not
*
*@param[in]				- SPI peripheral pointer
*@param[in]				- Flag to check its condition
*@param[in]				-
*
*@return				- Flag set or reset
*
*@note					- None
*
*/

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


/*
 * Data Send and Receive
 */

/*******************************************************
*@fn					- SPI_SendData
*
*@brief					- Use this function to transmit data via SPI peripheral
*
*@param[in]				- Base address of the SPI peripheral
*@param[in]				- Address of transmit buffer
*@param[in]				- Length of data to send
*
*@return				- None
*
*@note					- This is a blocking call (also called polling call)
*
*/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len>0)  // while there are still bytes to send
	{
			// 1. wait until the TXE is empty

		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET ); // polling for TXE flag to be set

			//2. check the DFF bit in CR1

		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			// 16 bit DFF
			// 3. load the data into DR

			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;

		}else{

			// 8 bit DFF
			// 3. load the data into DR

			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}


/*******************************************************
*@fn					- SPI_SendDataIT
*
*@brief					- This function utilizes interrupt to transmit data via SPI peripheral
*
*@param[in]				- Pointer to SPIHandle structure
*@param[in]				- Address of transmit buffer
*@param[in]				- Length of data to send
*
*@return				- State, busy or not in transmission
*
*@note					- This is a non blocking call
*
*/

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len info in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code
		// can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable TXEIE bit. Used to generate an interrupt request when the TXE flag is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}


/*******************************************************
*@fn					- SPI_ReceiveData
*
*@brief					- Use this function to receive data via SPI peripheral
*
*@param[in]				- Base address of the SPI peripheral
*@param[in]				- Address of receive buffer
*@param[in]				- Length of data to receive
*
*@return				- None
*
*@note					-  This is a blocking call (also called polling call)
*
*/

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)  // while there are still bytes to receive
	{
		// 1. wait until there are data to read

		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET ); // polling for RXNE flag to be set

		//2. check the DFF bit in CR1

		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			// 16 bit DFF
			// 3. Read the data from DR to Rxbuffer address

			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;   // pointing to the next empty memory location
		}else{
			// 8 bit DFF
			// 3. load the data into DR

			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;   // pointing to the next empty memory location
		}
	}
}

/*******************************************************
*@fn					- SPI_ReceiveDataIT
*
*@brief					- This function utilizes interrupt to receive data via SPI peripheral
*
*@param[in]				- Pointer to SPIHandle structure
*@param[in]				- Address of Receive Buffer
*@param[in]				- Length of data to receive
*
*@return				- State, busy or not in reception
*
*@note					-  This is a non blocking call
*
*/

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Rx buffer address and Len info in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in reception so that no other code
		// can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable RXNEIE bit. Used to generate an interrupt request when the RXNE flag is set.
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;

}


/*
 * IRQ Configuration and ISR Handling
 */

/*******************************************************
*@fn					- SPI_IRQInterruptConfig
*
*@brief					- This function enables the IRQ number the SPI interrupt is arriving at
*
*@param[in]				- IRQ number to configure
*@param[in]				- Enable and disable macros for the IRQ number
*
*@return				- None
*
*@note					- None
*
*/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI)
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
*@fn					- SPI_IRQPriorityConfig
*
*@brief					- This function configures the priority of the IRQ Number the SPI interrupt is arriving at
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;   			// to determine in what register the priority of IRQNumber resides (IPR1 to IPR4)
	uint8_t iprx_section = IRQNumber%4;		// to determine where in IPRx the priority of IRQNumber resides
	uint8_t shift_amount = (iprx_section*8) +(8- NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx)&= ~( 0xF<< shift_amount ); // clearing before writing the priority. 0xF is chosen since NO_PR_BITS_IMPLEMENTED = 4
	*(NVIC_PR_BASEADDR + iprx)|= ( IRQPriority<< shift_amount );  // every increment of an uint32_t pointer type is 4 bytes
}


/*******************************************************
*@fn					- SPI_IRQHandling
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

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	// first let's check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		// handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		// handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


/*
 * Other peripheral control APIs
 */

/*******************************************************
*@fn					- SPI_PeripheralControl
*
*@brief					- This function enables/disables the SPI peripheral.
*@brief					  Use after assigning the SPI parameters using SPI_Init().
*
*@param[in]				- Base address to the SPI peripheral
*@param[in]				- ENABLE or DISABLE macro
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*******************************************************
*@fn					- SPI_SSIConfig
*
*@brief					- The SSI bit has an effect only when the SSM bit is set. The value of the SSI bit
*@brief					  is forced onto the NSS pin and the IO value of the NSS pin is ignored.
*
*@param[in]				- Base address to the SPI peripheral
*@param[in]				- ENABLE or DISABLE macro
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void SPI_SSIConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}


/*******************************************************
*@fn					- SPI_SSOEConfig
*
*@brief					- SSOE bit set to 0: SS output is disabled in master mode and the cell can work in multimaster configuration
*@brief					  SSOE bit set to 1: SS output is enabled in master mode and when the cell is enabled. The cell cannot work in a multimaster environment
*
*@param[in]				- Base address to the SPI peripheral
*@param[in]				- ENABLE or DISABLE macro
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void SPI_SSOEConfig (SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

// Helper functions implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
	{
		// 16 bit DFF
		// Load the data into DR

		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		// 8 bit DFF
		// Load the data into DR

		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		// TxLen is zero. Close SPI transmission.
		// and inform the app that Tx is over

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
	{
		// 16 bit DFF
		// Load the data into DR

		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
	    (uint16_t*)pSPIHandle->pRxBuffer++;
	}else
	{
		// 8 bit DFF
		// Load the data into DR

		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
	    pSPIHandle->pRxBuffer++;
	}

		if(!pSPIHandle->RxLen)
		{
			// RxLen is zero. Close SPI reception.
			// and inform the app that Rx is over

			SPI_CloseReception(pSPIHandle);

			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	// 1. clear the ovr flag by conducting a read on DR and SR registers
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp; // to avoid unused variable error

	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	// This prevents interrupt from TXE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// This prevents interrupt from RXNE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	// clear the ovr flag by conducting a read on DR and SR registers
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp; // to avoid unused variable error
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	// This is a weak implementation. The application may override this function
}
