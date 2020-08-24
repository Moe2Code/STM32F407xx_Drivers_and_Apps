/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Oct 8, 2019
 *      Author: Mohammed
 */

#include "stm32f407xx_usart_driver.h"


/*
 * Peripheral Clock Setup
 */

/*******************************************************
*@fn					- USART_PeriClockControl
*
*@brief					- This function enables or disables peripheral clock for a USART peripheral
*
*@param[in]				- Base address of the USART peripheral
*@param[in]				- ENABLE OR DISABLE macros
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}else{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}


/*******************************************************
*@fn					- USART_PeripheralControl
*
*@brief					- This function enables/disables the USARTx peripheral
*
*@param[in]				- Base address to the USART peripheral
*@param[in]				- ENABLE or DISABLE macro
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}


/*******************************************************
*@fn					- USART_Init
*
*@brief					- This function initializes a USARTx peripheral
*
*@param[in]				- Base address of the USARTx peripheral
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Temporary register
	uint32_t tempreg = 0;

/*********************************************Configuration of CR1 (Control Register 1)*****************************************************/

	// Implement the code to enable the clock for a given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//  Enable both the transmitter and receiver bit fields
		tempreg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));

	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		// Enable the transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		// Enable the receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	}

	// Configure the word length
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	// Configure parity bit control
	if(pUSARTHandle->USART_Config.USART_ParityControl != USART_PARITY_DISABLE)
	{
		// enable parity bit control
		tempreg |= (1 << USART_CR1_PCE);

		// enable even or odd parity based on configuration
		tempreg |= pUSARTHandle->USART_Config.USART_ParityControl << USART_CR1_PS;
	}

	// Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/*********************************************Configuration of CR2 (Control Register 2)*****************************************************/

	// Configure number of stop bits
	tempreg = 0;
	tempreg = pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/*********************************************Configuration of CR3 (Control Register 3)*****************************************************/

	// Configure hardware flow control for USART peripheral
	tempreg = 0;
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
	}
	pUSARTHandle->pUSARTx->CR3 = tempreg;

/*********************************************Configuration of BRR (Baudrate Register)*****************************************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}


/*******************************************************
*@fn					- USART_DeInit
*
*@brief					- This function de-initializes a USARTx peripheral
*
*@param[in]				- Base address of the USARTx peripheral
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}


/*******************************************************
*@fn					- USART_GetFlagStatus
*
*@brief					- This function checks if a certain flag is set or not
*
*@param[in]				- USARTx peripheral pointer
*@param[in]				- Flag to check its condition
*@param[in]				-
*
*@return				- Flag set or reset
*
*@note					- None
*
*/

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	if(pUSARTx->SR & StatusFlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*******************************************************
*@fn					- USART_ClearFlag
*
*@brief					- This function resets the flag passed
*
*@param[in]				- USARTx peripheral pointer
*@param[in]				- Flag to reset
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~StatusFlagName;
}


/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - This function is used to be transmit data through USARTx peripheral using blocking call method
 *
 * @param[in]         - USARTx handle pointer
 * @param[in]         - Pointer to data to transmit
 * @param[in]         - Length of data to transmit
 *
 * @return            - None
 *
 * @Note              - This is a blocking/polling call

 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;

   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. So, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer. So, 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));

	// Disable the USART peripheral (if needed) only after the TC flag is set
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - This function is used to be receive data through USARTx peripheral using blocking call method
 *
 * @param[in]         - USARTx handle pointer
 * @param[in]         - Pointer to data buffer. Data received will be stored in this buffer
 * @param[in]         - Length of data to receive
 *
 * @return            - None
 *
 * @Note              - This is a blocking/ polling call

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//Check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. So, all 9bits will be of user data
				//Read only first 9 bits. So, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				// The 9th bit will be replaced by parity bit by the hardware
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//Check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//Read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//Read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0X7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             - This function is used to be transmit data through USARTx peripheral using interrupts method
 *
 * @param[in]         - USARTx handle pointer
 * @param[in]         - Pointer to data to transmit
 * @param[in]         - Length of data to transmit
 *
 * @return            - None
 *
 * @Note              - This is a non blocking call

 */

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - This function is used to be receive data through USARTx peripheral using interrupts method
 *
 * @param[in]         - USARTx handle pointer
 * @param[in]         - Pointer to data buffer. Data received will be stored in this buffer
 * @param[in]         - Length of data to receive
 *
 * @return            - None
 *
 * @Note              - This is a none blocking call

 */

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}


/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));

  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . Refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //Copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}


/*
 * IRQ Configuration and ISR Handling
 */

/*******************************************************
*@fn					- USART_IRQInterruptConfig
*
*@brief					- This function enables the IRQ number the USART interrupt is arriving at
*
*@param[in]				- IRQ number to configure
*@param[in]				- Enable and disable macros for the IRQ number
*
*@return				- None
*
*@note					- None
*
*/

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI)
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
*@fn					- USART_IRQPriorityConfig
*
*@brief					- This function configures the priority of the IRQ Number the USART interrupt is arriving at
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;   			// to determine in what register the priority of IRQNumber resides (IPR1 to IPR4)
	uint8_t iprx_section = IRQNumber%4;		// to determine where in IPRx the priority of IRQNumber resides
	uint8_t shift_amount = (iprx_section*8) +(8- NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx)&= ~( 0xF<< shift_amount ); // clearing before writing the priority. 0xF is chosen since NO_PR_BITS_IMPLEMENTED = 4
	*(NVIC_PR_BASEADDR + iprx)|= ( IRQPriority<< shift_amount );  // every increment of an uint32_t pointer type is 4 bytes
}


/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1 , temp2, temp3;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(pUSARTHandle->TxLen == 0 )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until TxLen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				uint16_t *pdata;

				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen--;
						pUSARTHandle->TxLen--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxLen--;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of RXNE

		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//Receive data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame
					//Now, check if we are using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used, so, all 9bits will be of user data

						//Read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen--;
						pUSARTHandle->RxLen--;
					}
					else
					{
						//Parity is used, so, 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check if we are using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used, so all 8bits will be of user data

						//Read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//Read only 7 bits , hence, mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now, increment the pRxBuffer
					 pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					 pUSARTHandle->RxLen--;
				}
			}

			if(! pUSARTHandle->RxLen)
			{
				//Disable the RXNE
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}

/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3. This bit is not available for UART4 & UART5.
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1 && temp2 && temp3 )
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//This interrupt is because of CTS
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->DR;

		//This interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an API for the application to clear the ORE flag.
		//This interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
	}

/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error are in multibuffer communication
//We don't discuss multibuffer communication in this course. Please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (a read to the USART_SR register
				followed by a read to the USART_DR register).
			*/

			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (a read to the USART_SR register followed by a read to the
				USART_DR register).
			*/

			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			/*
			This bit is set by hardware when the word currently being received in the shift register is
			ready to be transferred into the RDR register while RXNE=1. It is cleared by a software sequence
			(a read to the USART_SR register followed by a read to the USART_DR register).
			*/

			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}
}


/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}
