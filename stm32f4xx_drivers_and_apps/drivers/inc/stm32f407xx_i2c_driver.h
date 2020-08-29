/**
  ******************************************************************************
  * @file    stm32f407xx_i2c_driver.h
  * @author  Moe2Code
  * @brief   Header file of I2C module.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

/* Includes */
#include "stm32f407xx.h"


/*
 * This is a Configuration structure for a I2Cx peripheral
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;			/* possible values from @I2C_SCLSpeed      */
	uint8_t  I2C_DeviceAddress;		/* To store own address				  	   */
	uint8_t  I2C_AckControl;		/* possible values from @I2CACKControl     */
	uint8_t  I2C_FMDutyCycle;		/* possible values from @FMDutyCycle       */
}I2C_Config_t;


/*
 * This is a Handle structure for a I2Cx peripheral
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;			/* To hold the base address of the I2Cx peripheral (x =1, 2, or 3)  				*/
	I2C_Config_t I2C_Config;		/* To hold the I2Cx peripheral configuration settings				   				*/
	uint8_t		 *pTxBuffer;		/* To store the app Tx buffer address 												*/
	uint8_t		 *pRxBuffer;		/* To store the app Rx buffer address 												*/
	uint32_t     TxLen;				/* To store Tx data length 															*/
	uint32_t     RxLen;				/* To store Rx data length 															*/
	uint8_t      TxRxState;			/* To store the communication state. Possible values from @TxRxState 				*/
	uint8_t      DevAddr;			/* To store the slave address 														*/
	uint32_t     RxSize;			/* To store Rx size 																*/
	uint8_t      Sr;                /* To store the repeated start state. Possible values from @Repeated_Start_State 	*/
}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM 		100000   // Standard mode 100kHz
#define I2C_SCL_SPEED_FM2K		200000	 // Fast mode 200kHz
#define I2C_SCL_SPEED_FM4K		400000	 // Fast mode 400kHz


/*
 * @I2CACKControl
 */

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0


/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * @TxRxState
 */

#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

/*
 * @Repeated_Start_State
 */

#define I2C_DISABLE_SR			RESET
#define I2C_ENABLE_SR			SET


/*
 * I2C related status flags definitions
 */

#define I2C_FLAG_SB							(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR						(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF						(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10					    (1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF						(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE						(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE						(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR						(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO						(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF						    (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR						(1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT					(1 << I2C_SR1_TIMEOUT)


/*
 * I2C application events and errors macros
 */
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR  		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_SND			8
#define I2C_EV_DATA_RCV			9


/*****************************************************************************************************
 *                                      APIs supported by this driver
 *                    For more information about these APIs check the function definitions
 *****************************************************************************************************/

/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * Initialization and De-initialization
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Data send and receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);


/*
 * IRQ Configuration and ISR Handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other peripheral control APIs
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableInterrupts(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


/*
 * Application callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
