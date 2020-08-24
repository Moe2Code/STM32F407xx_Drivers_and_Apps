/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Sep 17, 2019
 *      Author: Mohammed
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"
//#include <stdint.h>


/*
 * This is a Configuration structure for a SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;			/* possible values from @SPI_DeviceMode */
	uint8_t SPI_BusConfig;			/* possible values from @SPI_BusConfig */
	uint8_t SPI_DFF;				/* possible values from @SPI_DFF */
	uint8_t SPI_CPHA;				/* possible values from @SPI_CPHA */
	uint8_t SPI_CPOL;				/* possible values from @SPI_CPOL */
	uint8_t SPI_SSM;				/* possible values from @SPI_SSM */
	uint8_t	SPI_SclkSpeed;			/* possible values from @SPI_Speed */
}SPI_Config_t;

/*
 * This is a Handle structure for a SPIx peripheral
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;			/* This holds the base address of the SPIx peripheral (x =1, 2, or 3)  */
	SPI_Config_t SPI_Config;		/* This holds the SPI peripheral configuration settings      			*/
	uint8_t	*pTxBuffer;				/* To store the app. This holds Tx buffer address 						*/
	uint8_t *pRxBuffer;				/* To store the app. This holds Rx buffer address 						*/
	uint32_t TxLen;					/* To store Tx length													*/
	uint32_t RxLen;					/* To store Rx length 													*/
	uint8_t TxState;				/* To store Tx state 													*/
	uint8_t	RxState;				/* To store Rx state 													*/
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 * SPI Device Modes
 */

#define SPI_DEVICE_MODE_SLAVE 				0
#define SPI_DEVICE_MODE_MASTER				1

/*
 * @SPI_BusConfig
 * SPI Device Configuration
 */

#define SPI_BUS_CFG_FD 						0  	// Full duplex
#define SPI_BUS_CFG_HD						1	// Half duplex
#define SPI_BUS_CFG_SMPLX_TXONLY			2	// Simplex transmit only
#define SPI_BUS_CFG_SMPLX_RXONLY			3	// Simplex receive only

/*
 * @SPI_DFF
 * SPI Device Data Frame Format
 */

#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/*
 * @SPI_Speed
 * SPI Device Clock Speed
 */

#define SPI_SCLK_SPEED_DIV2					0	// SCLK = PCLK/2
#define SPI_SCLK_SPEED_DIV4					1	// SCLK = PCLK/4
#define SPI_SCLK_SPEED_DIV8					2	// SCLK = PCLK/8
#define SPI_SCLK_SPEED_DIV16				3	// SCLK = PCLK/16
#define SPI_SCLK_SPEED_DIV32				4	// SCLK = PCLK/32
#define SPI_SCLK_SPEED_DIV64				5	// SCLK = PCLK/64
#define SPI_SCLK_SPEED_DIV128				6	// SCLK = PCLK/128
#define SPI_SCLK_SPEED_DIV256				7	// SCLK = PCLK/256

/*
 * @SPI_CPHA
 * SPI Device Clock Phase Configuration
 */

#define SPI_CPHA_HIGH						0  	// The first clock transition is the first data capture edge
#define SPI_CPHA_LOW						1	// The second clock transition is the first data capture edge

/*
 * @SPI_CPOL
 * SPI Device Clock Polarity Configuration
 */

#define SPI_CPOL_LOW						0	// Clock is low (0) when idle
#define SPI_CPOL_HIGH						1	// Clock is high (1) when idle

/*
 * @SPI_SSM
 * SPI Device Software Select Management
 */

#define SPI_SSM_DI							0	// Software select is disabled (hardware select is chosen)
#define SPI_SSM_EN							1	// SSM bit is set, the NSS pin input is replaced with the value from the SSI (Slave Select Internal) bit.

/*
 * SPI related status flags definitions
 */

#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_BSY_FLAG						(1 << SPI_SR_BSY)

/*
 * Possible SPI application states
 */

#define SPI_READY							0
#define SPI_BUSY_IN_RX						1
#define SPI_BUSY_IN_TX						2


/*
 * Possible SPI application events
 */

#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR				    4

/*****************************************************************************************************
 *                                      APIs supported by this driver
 *                    For more information about this APIs check the function definitions
 *****************************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Initialization and De-initialization
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral control APIs
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
