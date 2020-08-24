/*
 * 011I2C_master_rx_testing.c
 *
 *  Created on: Oct 02, 2019
 *      Author: Mohammed
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

#define MYADDR		0x61
#define SlaveAddr	0x68  // Acquired from Arduino serial monitor


// for semihosting to be able to use printf function
extern void initialise_monitor_handles();


void delay(void)
{
	for(uint32_t i=0; i<=500000; i++);
}

I2C_Handle_t I2C1Handle;		// Global variable

// Data received from Arduino slave will be stored in the rcv_buffer
uint8_t rcv_buffer[32];

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
	// for semihosting to be able to use printf function
	initialise_monitor_handles();

	printf("The application is running\n");

	//I2Cx pins init
	I2C1_GPIOInits();

	// I2Cx peripheral config
	I2C1_Inits();

	// Button init
	ButtonInit();

	// Enable I2Cx peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	// Enable Ack bit after enabling the peripheral (PE = 1)
	// Note that HW resets Ack bit when the peripheral is disabled (PE = 0)
	I2C_ManageAcking(I2C1Handle.pI2Cx, I2C_ACK_ENABLE);

	uint8_t commandcode;
	uint8_t data_len;

	while(1)
	{
		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		// Write command code 0x51 to request info on data length
		commandcode = 0x51;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SlaveAddr, I2C_ENABLE_SR);

		// Receive data length info from slave
		I2C_MasterReceiveData(&I2C1Handle, &data_len, 1, SlaveAddr, I2C_ENABLE_SR);

		// Write command code 0x52 to request the actual data
		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SlaveAddr, I2C_ENABLE_SR);

		// Receive the actual data
		I2C_MasterReceiveData(&I2C1Handle, rcv_buffer, data_len, SlaveAddr, I2C_DISABLE_SR);

		rcv_buffer[data_len] = '\0';   // null termination of array (required by %s format specifier)

		printf("DATA RECEIVED: %s", rcv_buffer);
	}
	return 0;
}