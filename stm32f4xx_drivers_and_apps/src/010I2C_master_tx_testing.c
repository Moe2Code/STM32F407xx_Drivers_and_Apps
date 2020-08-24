/*
 * 010I2C_master_tx_testing.c
 *
 *  Created on: Sep 29, 2019
 *      Author: Mohammed
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define MYADDR		0x61
#define SlaveAddr	0x68  // Acquired from Arduino serial monitor

void delay(void)
{
	for(uint32_t i=0; i<=500000; i++);
}

I2C_Handle_t I2C1Handle;		// Global variable

// Data to send to Arduino slave
// You cannot send more than 32 bytes in one transmission
// This is due to limitation in the Arduino sketch WIRE library

uint8_t user_data[] = "I2C com from master (STM32F4)\n";

/*
 * PB6 -> SCL
 * PB9 -> SDA
 * Alternate Function Mode = 4	(I2C1/2/3)
 */


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
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	//Duty cycle is only selectable in fast mode (FM)
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

	// For master, this field is not required. Required for slaves only
	// Check I2C spec for reserved addresses in order to not use by mistake
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MYADDR;

	I2C_Init(&I2C1Handle);
}


void ButtonInit(void)
{
	GPIO_Handle_t GpioButton;
    GpioButton.pGPIOx = GPIOA;

	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;   //Not required however initialized to avoid errors

	GPIO_Init(&GpioButton);
}

int main(void)
{

	//I2Cx pins init
	I2C1_GPIOInits();

	// I2Cx peripheral config
	I2C1_Inits();

	// Button init
	ButtonInit();

	// Enable I2Cx peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	while(1)
	{
		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		// Send some data
		I2C_MasterSendData(&I2C1Handle, user_data, strlen((char*)user_data), SlaveAddr);
	}

	return 0;
}
