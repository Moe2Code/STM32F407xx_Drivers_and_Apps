/**
  ******************************************************************************
  * @file           : 013I2C_slave_tx_string.c
  * @author         : Moe2Code
  * @brief          : This is an application that sends a message from ST Discovery board (slave)
  * 				  to Arduino Uno (master) via I2C. The following pins configuration was used
  * 				  on ST Discovery
  * 				  PB6 -> SCL
  * 				  PB7 -> SDA
  * 				  Alternate Function Mode = 4	(I2C1/2/3)
  *******************************************************************************
*/

// Includes
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


// Defines
#define SLAVE_ADDR	0x69
#define MY_ADDR		SLAVE_ADDR


// Global variables
I2C_Handle_t I2C1Handle;
// Data to transmit from slave (STM board) to master (Arduino board)
uint8_t tx_buffer[32] = "I2C slave mode is fun..";		// Do not send more than 32 bytes (limitation of Arduino wire library)


// Simple delay function
void delay(void)
{
	for(uint32_t i=0; i<=500000; i++);
}


void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}


void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	//Duty cycle is only selectable in fast mode (FM)
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

	// Address is required for slaves only
	// Check I2C spec for reserved addresses in order to not use by mistake
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;

	I2C_Init(&I2C1Handle);
}


int main(void)
{
	//I2Cx pins init
	I2C1_GPIOInits();

	// I2Cx peripheral config
	I2C1_Inits();

	// Slave application is always in interrupt mode
	// IRQ configuration of I2C peripheral

	// Enable IRQ number for I2C event and error interrupts
	I2C_IRQInterruptConfig( IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig( IRQ_NO_I2C1_ER,ENABLE);

	// Configure IRQ priority for I2C event and error interrupts
	//I2C_IRQPriorityConfig(IRQ_NO_I2C1_EV,NVIC_IRQ_PRI10);
	//I2C_IRQPriorityConfig(IRQ_NO_I2C1_ER,NVIC_IRQ_PRI9);

	// To enable interrupt control bits for slave
	I2C_SlaveEnableDisableInterrupts(I2C1, ENABLE);

	// Enable I2Cx peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	// Enable Ack bit after enabling the peripheral (PE = 1)
	// Note that HW resets Ack bit when the peripheral is disabled (PE = 0)
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);

	return 0;
}


void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


uint8_t commandcode = 0;
uint8_t Cnt = 0;

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	// These static variables are private for this function
	// However the memory for them is allocated in global space
	//static uint8_t commandcode = 0;
	//static uint8_t Cnt = 0;

	if(AppEvent == I2C_EV_DATA_SND)
	{
		// Master requested some data. Slave has to send it
		if(commandcode == 0x51)
		{
			// Master has sent a read command after a write command of 0x51
			// This is a request to send data length

			I2C_SlaveSendData(pI2CHandle->pI2Cx,strlen((char*)tx_buffer));

		}else if(commandcode == 0x52)
		{
			// Master has sent a read command after a write command of 0x52
			// This is a request to send the data

			I2C_SlaveSendData(pI2CHandle->pI2Cx,tx_buffer[Cnt]);
			Cnt++;	// Incrementing the count to send the next byte of tx_buffer
		}

	}else if(AppEvent == I2C_EV_DATA_RCV)
	{
		// Master sent some data. Slave has to read it
		commandcode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if(AppEvent == I2C_ERROR_AF)
	{
		// This happens only during slave transmission. Master has sent NACK
		// Slave should understand that the master doesn't need more data

		// Invalidating/resetting
		commandcode = 0xFF;
		Cnt = 0;

	}else if(AppEvent == I2C_EV_STOP)
	{
		// This happens only during slave reception
		// Master has ended I2C communication with slave
	}
}
