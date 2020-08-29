/**
  ******************************************************************************
  * @file           : 012I2C_master_rx_testingIT.c
  * @author         : Moe2Code
  * @brief          : This is an application that uses ST Discovery board as a master to receive data
  * 				  from Arduino Uno, slave, via I2C interrupt mode. The following pins configuration
  * 				  was used on ST Discovery
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
#define MYADDR		0x61
#define SlaveAddr	0x68  // Acquired from Arduino serial monitor


// Global variables
// Rx complete flag global variable
uint8_t rx_cmplt = RESET;
// I2C1 handle
I2C_Handle_t I2C1Handle;
// Data received from Arduino slave will be stored in the buffer below
uint8_t rcv_buffer[32];


// Semihosting prototype to be able to use printf function
extern void initialise_monitor_handles();


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
	// Executing semihosting initialize function to be able to use printf function
	initialise_monitor_handles();

	printf("The application is running\n");  // Must end with \n or \r\n

	//I2Cx pins init
	I2C1_GPIOInits();

	// I2Cx peripheral config
	I2C1_Inits();

	// Button init
	ButtonInit();

	// IRQ configuration of I2C peripheral

	// Enable IRQ number for I2C event and error interrupts
	I2C_IRQInterruptConfig( IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig( IRQ_NO_I2C1_ER,ENABLE);

	// Configure IRQ priority for I2C event and error interrupts
	I2C_IRQPriorityConfig(IRQ_NO_I2C1_EV,NVIC_IRQ_PRI10);
	I2C_IRQPriorityConfig(IRQ_NO_I2C1_ER,NVIC_IRQ_PRI9);


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
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SlaveAddr, I2C_ENABLE_SR) != I2C_READY );

		// Receive data length info from slave
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &data_len, 1, SlaveAddr, I2C_ENABLE_SR) != I2C_READY);

		// Write command code 0x52 to request the actual data
		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SlaveAddr, I2C_ENABLE_SR) != I2C_READY);

		// Receive the actual data
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buffer, data_len, SlaveAddr, I2C_DISABLE_SR) != I2C_READY);

		rx_cmplt = RESET;

		//Wait till all data received
		while(rx_cmplt != SET);

		rcv_buffer[data_len] = '\0';   // null termination of array (required by %s format specifier)
		printf("DATA RECEIVED: %s", rcv_buffer);

		rx_cmplt = RESET;
	}
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


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	if(AppEvent == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");

	}else if(AppEvent == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
		rx_cmplt = SET;

	}else if(AppEvent == I2C_ERROR_AF)
	{
		printf("ERROR: Ack failure\n");

		// In master mode Ack failure occurs when slave fails to send Ack
		// for the byte sent by the master

		I2C_CloseSendData(&I2C1Handle);

		// Generate the stop condition to release the bus
		I2C_GenerateStopCondition(I2C1Handle.pI2Cx);

		// Hang in infinite loop to not allow further app code from main to execute
		while(1);
	}
}

