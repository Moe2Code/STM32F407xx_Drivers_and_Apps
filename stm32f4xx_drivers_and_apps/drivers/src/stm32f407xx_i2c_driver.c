/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Sep 27, 2019
 *      Author: Mohammed
 */

#include "stm32f407xx_i2c_driver.h"


/*
 * Operations for address phase
 */

#define AddrWrite			0
#define AddrRead			1


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t Operation);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);


// Interrupt Helper Functions

static void I2C_ClearADDRFlagIT(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);


/*******************************************************
*@fn					- I2C_GenerateStartCondition
*
*@brief					- This function generates a start condition on the I2Cx bus
*
*@param[in]				- Base address of the I2Cx peripheral
*@param[in]				-
*
*@return				- None
*
*@note					- This function is private (not runnable by app)
*
*/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


/*******************************************************
*@fn					- I2C_ExecuteAddressPhase
*
*@brief					- This function writes the slave address to the data register of an I2Cx peripheral
*
*@param[in]				- Base address of the I2Cx peripheral
*@param[in]				- Slave address
*@param[in]				- The requested operation on slave: read (1) or write (0)
*
*@return				- None
*
*@note					- This function is private (not runnable by app)
*
*/

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t Operation)
{
	if(Operation == AddrWrite)
	{
		SlaveAddr = SlaveAddr << 1;  // making space for the read/write bit
		SlaveAddr &= ~(1);			 // ensuring the read/write bit is set to write (0)
		pI2Cx->DR = SlaveAddr;
	}else
	{
		SlaveAddr = SlaveAddr << 1;  // making space for the read/write bit
		SlaveAddr |= 1;			     // ensuring the read/write bit is set to read (1)
		pI2Cx->DR = SlaveAddr;
	}
}


/*******************************************************
*@fn					- I2C_ClearADDRFlag
*
*@brief					- This function clears the ADDR flag for an I2Cx peripheral
*
*@param[in]				- Base address of the I2Cx peripheral
*@param[in]				-
*
*@return				- None
*
*@note					- This function is private (not runnable by app)
*
*/

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	// To clear ADDR flag a read must be performed on SR1 then SR2 registers
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;

	(void)dummyRead;	// Typecasting to avoid "unused variable" compiler error
}


/*******************************************************
*@fn					- I2C_ClearADDRFlagIT
*
*@brief					- This function clears the ADDR flag for interrupt based data send and receive
*
*@param[in]				- Pointer to I2C handle
*@param[in]				-
*
*@return				- None
*
*@note					- This function is private (not runnable by app)
*
*/

static void I2C_ClearADDRFlagIT(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	// Check for device mode
	if(pI2CHandle->pI2Cx->SR2 &(1 << I2C_SR2_MSL))
	{
		// Device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxLen == 1)
			{
				// Disable the Ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Clear the ADDR
				// A read must be performed on SR1 then SR2 registers to clear ADDR flag
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;	// Typecasting to avoid "unused variable" compiler error

			}else if(pI2CHandle->RxLen > 1)
			{
				// Clear the ADDR
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Clear the ADDR
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}

	}else
	{
		// Device is in slave mode
		// Clear the ADDR
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}


/*******************************************************
*@fn					- I2C_GenerateStopCondition
*
*@brief					- This function generates the stop condition for transfer of data on an I2Cx peripheral
*
*@param[in]				- Base address of the I2Cx peripheral
*@param[in]				-
*
*@return				- None
*
*@note					- This function is private (not runnable by app)
*
*/

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

}



/*
 * Peripheral Clock Setup
 */

/*******************************************************
*@fn					- I2C_PeriClockControl
*
*@brief					- This function enables or disables peripheral clock for an I2Cx peripheral
*
*@param[in]				- Base address of the I2Cx peripheral
*@param[in]				- ENABLE OR DISABLE macros
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*
 * Initialization and De-initialization
 */

/*******************************************************
*@fn					- I2C_Init
*
*@brief					- This function initializes an I2Cx peripheral
*
*@param[in]				- I2C handle structure
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// enable I2Cx peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// When PE = 0, HW resets Ack bit. The below code will not enable Ack bit
	// Enable Ack bit after enabling the peripheral (PE = 1)
	//tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	//pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg |= RCC_GetPCLK1Value()/ 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);     // bit 14 of OAR1 needs to be always 1 (requirement from manual)
	//tempreg &= ~(1<< I2C_OAR1_ADDMODE);  // to choose 7-bit slave address mode (default mode)
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <=I2C_SCL_SPEED_SM)
	{
		// speed mode is standard
		//tempreg &= ~(1 << I2C_CCR_FS);  // to choose standard mode (default mode)

		// calculate CCR value based on 50% duty cycle (Thigh = Tlow)
		ccr_value = (RCC_GetPCLK1Value()/(2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value &0xFFF);

	}else
	{
		// speed mode is fast
		tempreg |= (1 << I2C_CCR_FS); 		// to choose fast mode
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			// Duty Cycle 0: Tlow/Thigh = 2
			ccr_value = RCC_GetPCLK1Value()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}else
		{
			// Duty Cycle 1: Tlow/Thigh = 16/9
			ccr_value = RCC_GetPCLK1Value()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;


	// Trise Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <=I2C_SCL_SPEED_SM)
	{
		// Standard mode
		// Max rise time per I2C spec is 1000 ns
		tempreg = ((RCC_GetPCLK1Value() * 1000U)/1000000000U) + 1;

	}else
	{
		// Fast mode
		// Max rise time per I2C spec is 300 ns
		tempreg = ((RCC_GetPCLK1Value() * 300U)/1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}


/*******************************************************
*@fn					- I2C_DeInit
*
*@brief					- This function de-initializes an I2Cx peripheral
*
*@param[in]				- Base address of the I2Cx peripheral
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/*
 * Data send and receive
 */

/*******************************************************
*@fn					- I2C_MasterSendData
*
*@brief					- This function is used to send data from master to slave
*
*@param[in]				- Pointer to I2CHandle
*@param[in]				- Pointer to data buffer to send
*@param[in]				- Length of data to send
*@param[in]				- Slave address
*@param[in]				- Repeated Start (Sr) macros
*
*@return				- None
*
*@note					- This is a blocking call (also called polling call)
*
*/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that the start generation is completed by checking SB flag in the SR1
	// Note that until SB is cleared, SCL will be stretched (pulled to LOW). Thus the master and slave will be in wait state
	// SB is cleared by performing a read from SR1 register and write to DR register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));  // Polling for SB flag to be set. This is a read process from SR1

	// 3. Send the address of the slave with r/nw bit set to zero (write request). Total 8 bits to be sent.
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, AddrWrite);  // Slave address is written to DR. The SB bit will be cleared after

	//4. Confirm that address phase is complete by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); // Wait till ADDR flag is set

	// 5. Clear the ADDR flag according to its software sequence
	// Note that until ADDR is cleared, SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx); // ADDR is cleared by reading SR1 followed by SR2

	//6. Send data until Len becomes 0
	while(Len>0)
	{
		// Wait until TXE flag is set. TXE is set when DR is empty
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// 7. When Len becomes zero, wait for TXE = 1 and BTF = 1 before generating the STOP condition
	// BTF is set by HW when NOSTRETCH = 0 and in transmission when a new byte should be sent and DR has not been written yet (TxE=1)
	// BTF is cleared by SW by either a read or write in the DR register or by HW after a start or a stop condition in transmission or when PE=0.
	// When BTF = 1, SCL will be stretched (pulled to LOW)

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // Wait till TXE flag is set

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)); // Wait till BTF flag is set

	// 8. Generate STOP condition and master needs not to wait for the completion of the STOP condition

	if(Sr == I2C_DISABLE_SR)		// If repeated start not required then generate STOP condition
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}


/*******************************************************
*@fn					- I2C_MasterSendDataIT
*
*@brief					- This function is used to send data from master to slave
*
*@param[in]				- Pointer to I2CHandle
*@param[in]				- Pointer to data buffer to send
*@param[in]				- Length of data to send
*@param[in]				- Slave address
*@param[in]				- Repeated Start (Sr) macros
*
*@return				-
*
*@note					-
*
*/

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t state = pI2CHandle->TxRxState;

	if(state != I2C_BUSY_IN_TX && state != I2C_BUSY_IN_RX)
	{

		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;

		// Implement code to generate START condition
		// Note that generating the start condition resets TXE (and possibly RXNE as well?)
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

		// Implement code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement code to enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

	}
	return state;
}


/*******************************************************
*@fn					- I2C_MasterReceiveData
*
*@brief					- This function is used to receive data from slave by master
*
*@param[in]				- Pointer to I2CHandle
*@param[in]				- Pointer to data buffer. Data received will be stored in this buffer
*@param[in]				- Length of data to receive
*@param[in]				- Slave address
*@param[in]				- Repeated Start (Sr) macros
*
*@return				- None
*
*@note					- This is a blocking call (also called polling call)
*
*/

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm the START condition is completed by checking the SB flag in SR1 register
	// Note that until SB is cleared, SCL will be stretched (pulled to LOW). Thus the master and slave will be in wait state
	// SB is cleared by performing a read from SR1 register and write to DR register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));  // Polling for SB flag to be set. This is a read process from SR1

	// 3. Send the address of the slave with r/nw bit set to one (read request). Total 8 bits to be sent
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, AddrRead);  // Slave address will be written to DR. The SB bit will be cleared after

	// 4. Wait until the address phase is complete by checking the ADDR flag is set in SR1 register
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)); // Wait till ADDR flag is set

	// Procedure to read only 1 byte from slave
	if(Len == 1)
	{
		// Disable the Ack bit
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// Clear ADDR flag
		// Note that until ADDR is cleared, SCL will be stretched (pulled to LOW)
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx); // ADDR is cleared by reading SR1 followed by SR2

		// Wait until RXNE becomes 1. The RXNE is set when DR is filled
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// Generate STOP condition
		if(Sr == I2C_DISABLE_SR)		// If repeated start not required then generate STOP condition
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// Fetch data from DR and store in buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		pRxBuffer++;
	}

	// Procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx); // ADDR is cleared by reading SR1 followed by SR2

		// Read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--)
		{
			// Wait until RXNE becomes 1. The RXNE is set when DR is filled
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) // if last 2 bytes are remaining
			{
				// Disable the Ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Generate STOP condition
				if(Sr == I2C_DISABLE_SR)	 // If repeated start not required then generate STOP condition
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			// Fetch data from DR and store in buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}

	// Re-enable Acking if specified in the configuration of the I2Cx peripheral
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE )
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}


/*******************************************************
*@fn					- I2C_MasterReceiveDataIT
*
*@brief					- This function is used to receive data from slave by master
*
*@param[in]				- Pointer to I2CHandle
*@param[in]				- Pointer to data buffer. Data received will be stored in this buffer
*@param[in]				- Length of data to receive
*@param[in]				- Slave address
*@param[in]				- Repeated Start (Sr) macros
*
*@return				-
*
*@note					-
*
*/

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t state = pI2CHandle->TxRxState;

	if(state != I2C_BUSY_IN_TX && state != I2C_BUSY_IN_RX)
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;

		// Implement code to generate START condition
		// Note that generating the start condition resets TXE (and possibly RXNE as well?)
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

		// Implement code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement code to enable ITEVTEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
	}
	return state;
}


/*******************************************************
*@fn					- I2C_ManageAcking
*
*@brief					- This function enables/disables the Ack bit in the CR1 register of an I2Cx peripheral
*
*@param[in]				- I2Cx peripheral pointer
*@param[in]				- Enable/disable macros
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_DISABLE)
	{
		// Disable Ack bit
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

	}else
	{
		// Enable Ack bit
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}


/*******************************************************
*@fn					- I2C_GetFlagStatus
*
*@brief					- This function checks if a certain flag is set or not
*
*@param[in]				- I2Cx peripheral pointer
*@param[in]				- Flag to check its condition
*@param[in]				-
*
*@return				- Flag set or reset
*
*@note					- None
*
*/

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


/*
 * IRQ Configuration and ISR Handling
 */

/*******************************************************
*@fn					- I2C_IRQConfig
*
*@brief					- This function enables the IRQ number the I2Cx interrupt is arriving at
*
*@param[in]				- IRQ number to configure
*@param[in]				- Enable and disable macros for the IRQ number
*
*@return				- None
*
*@note					- None
*
*/

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI)
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
*@fn					- I2C_IRQPriorityConfig
*
*@brief					- This function configures the priority of the IRQ Number the I2Cx interrupt is arriving at
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;   			// to determine in what register the priority of IRQNumber resides (IPR1 to IPR4)
	uint8_t iprx_section = IRQNumber%4;		// to determine where in IPRx the priority of IRQNumber resides
	uint8_t shift_amount = (iprx_section*8) +(8- NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx)&= ~( 0xF<< shift_amount ); // clearing before writing the priority. 0xF is chosen since NO_PR_BITS_IMPLEMENTED = 4
	*(NVIC_PR_BASEADDR + iprx)|= ( IRQPriority<< shift_amount );  // every increment of an uint32_t pointer type is 4 bytes
}


/*******************************************************
*@fn					- I2C_MasterHandleTXEInterrupt
*
*@brief					- This function services an interrupt generated by TXE flag
*
*@param[in]				- Pointer to I2C handle
*@param[in]				-
*
*@return				- None
*
*@note					- This function is private (not runnable by app)
*
*/

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen>0)
	{
		// 1. Load the data to the DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		// 2. Decrement the TxLen
		pI2CHandle->TxLen--;

		// 3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}


/*******************************************************
*@fn					- I2C_MasterHandleRXNEInterrupt
*
*@brief					- This function services an interrupt generated by RXNE flag
*
*@param[in]				- Pointer to I2C handle
*@param[in]				-
*
*@return				- None
*
*@note					- This function is private (not runnable by app)
*
*/


static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxLen == 1)
	{
		// Fetch data from DR and store in buffer
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}

	if(pI2CHandle->RxLen > 1)
	{
		if(pI2CHandle->RxLen == 2) // if last 2 bytes are remaining to receive
		{
			// Disable the Ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}

		// Fetch data from DR and store in buffer
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}

	if(pI2CHandle->RxLen == 0)
	{
		// Close the I2C data reception and notify the application

		// 1. Generate STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)	 // If repeated start not required then generate STOP condition
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// 2. Close the I2C RX
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}


/*******************************************************
*@fn					- I2C_CloseSendData
*
*@brief					-
*
*@param[in]				-
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle-> TxRxState = I2C_READY;
	pI2CHandle-> pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}


/*******************************************************
*@fn					- I2C_CloseReceiveData
*
*@brief					-
*
*@param[in]				-
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Implement the code to disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle-> TxRxState = I2C_READY;
	pI2CHandle-> pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}


/*******************************************************
*@fn					- I2C_EV_IRQHandling
*
*@brief					-
*
*@param[in]				-
*@param[in]				-
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	// Check if event and buffer interrupts are enabled or not
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);


/**************************************Check for SB flag ********************************************/

	// 1. Handle for interrupt generated by SB event
	// Note that SB flag is applicable only in master mode

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if(temp1 && temp3)
	{
		// This block will not be executed in slave mode because slave SB is always zero
		// The next com step is executed here, which is address phase
		// The SR1 read done above in temp3 and DR write in the address phase will clear the SB flag
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr,AddrWrite);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr,AddrRead);
		}
	}


/**************************************Check for ADDR flag ********************************************/

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	// 2. Handle for interrupt generated by ADDR event
	// Note that when in master mode: address is sent
	// Note that when in slave mode: address matched with own address

	if(temp1 && temp3)
	{
		// ADDR flag interrupt needs service
		// Clear the ADDR flag
		I2C_ClearADDRFlagIT(pI2CHandle);
	}


/**************************************Check for BTF flag ********************************************/

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	// 3. Handle for interrupt generated by BTF event

	if (temp1 && temp3)
	{
		// BTF flag interrupt needs service
		// BTF flag is used in closing com in transmission state once all data is sent

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				// Make sure all data is already sent
				if(pI2CHandle->TxLen == 0)
				{
					// 1. Generate the STOP condition if repeated start is disabled

					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// 2. Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					// 3. Notify the application that the transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			; 	// we do nothing here since BTF flag is not useful in the process of data reception
		}
	}


/**************************************Check for STOPF flag ********************************************/

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

	// 4. Handle for interrupt generated by STOPF event
	// Note that STOPF flag is applicable only in slave mode
	// The below code block will not be executed by the master since STOPF will not be set in master mode

	if (temp1 && temp3)
	{
		// STOPF flag interrupt needs service
		// Clear the STOPF by read SR1 (done above in temp3) and writing to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}


/**************************************Check for TXE flag ********************************************/

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

	// 5. Handle for interrupt generated by TXE event

	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// The device mode is master (MSL in SR2 is set)
			// Transmit data if we are in transmission state
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			// The device mode is slave
			// Make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_SND);
			}
		}
	}


/**************************************Check for RXNE flag ********************************************/

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

	// 6. Handle for interrupt generated by RXNE event

	if(temp1 && temp2 && temp3)
	{
		// Check the device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// The device is master
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			// The device mode is slave
			// Make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}


/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
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

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);

/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	    I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error*************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure error*****************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);

	}

/**********************Check for Overrun/underrun error*************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun error

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Timeout error*********************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);

	}

}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	// Send the data
	pI2Cx->DR = data;
}


uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	// Receive the data
	return ((uint8_t) pI2Cx->DR);
}


void I2C_SlaveEnableDisableInterrupts(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		// Implement code to enable ITEVTEN control bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement code to enable ITBUFEN control bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// Implement code to enable ITERREN control bit
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}else
	{
		// Implement code to disable ITEVTEN control bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

		// Implement code to disable ITBUFEN control bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

		// Implement code to disable ITERREN control bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

/*
 * Other peripheral control APIs
 */

/*******************************************************
*@fn					- I2C_PeripheralControl
*
*@brief					- This function enables/disables the I2Cx peripheral
*
*@param[in]				- Base address to the I2C peripheral
*@param[in]				- ENABLE or DISABLE macro
*@param[in]				-
*
*@return				- None
*
*@note					- None
*
*/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}
