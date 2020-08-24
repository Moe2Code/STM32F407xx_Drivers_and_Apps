/*
 * 008SPI_cmd_handling.c
 *
 *  Created on: Sep 24, 2019
 *      Author: Mohammed
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

// for semihosting to be able to use printf function
extern void initialise_monitor_handles();

// command codes

#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define	LED_OFF					0

// Arduino analog pins

#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

// Arduino LED

#define LED_PIN					9

void delay(void)
{
	for(uint32_t i=0; i<=500000; i++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * Alternate function mode: 5
 */


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CFG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;  // generates sclk of 2 MHz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;  // hardware slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}

void ButtonInit(void)
{
	GPIO_Handle_t GpioButton;
    GpioButton.pGPIOx = GPIOA;

	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;

	GPIO_Init(&GpioButton);
}


uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	// nack
	return 0;
}

int main (void)
{
	// for semihosting to be able to use printf function
	initialise_monitor_handles();

	printf("Application is running\n");

	ButtonInit();

	// this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize SPI2 peripheral parameters
	SPI2_Inits();

	printf("SPI initialized\n");

	/*
	 * making SSOE 1 does NSS output enable
	 * The NSS pin is automatically management by hardware
	 * i.e. when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);


	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;

	while(1)
	{
		// Send a command: 1. CMD_LED_CTRL	<pin no(1)>     <value(1)>

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		// enable SPI2 peripheral. To do after calling SPI2_Inits()
		SPI_PeripheralControl(SPI2, ENABLE);

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];  // 2 bytes arguments array

		// send the command to the slave
		// the slave will either acknowledge (ack) or not acknowledge (nack)
		SPI_SendData(SPI2, &commandcode, 1);

		// in SPI, when master or slave sends 1 byte, it also receives 1 byte of data
		// the transmission of 1 byte resulted in 1 garbage byte collection in Rx buffer
		// of the master and RXNE flag is set. So do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);


		// in SPI, the slave doesn't initiate the data transfer of ack/nack on its own.
		// thus, send a dummy byte to initiate the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_VerifyResponse(ackbyte))
		{
			// if command acknowledged, then send further arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL Executed\n");
		}


		// Send a command: 2. CMD_SENSOR_READ	<analog pin no(1)>

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		// send the command to the slave
		// the slave will either acknowledge (ack) or not acknowledge (nack)
		SPI_SendData(SPI2, &commandcode, 1);

		// in SPI, when master or slave sends 1 byte, it also receives 1 byte of data
		// the transmission of 1 byte resulted in 1 garbage byte collection in Rx buffer
		// of the master and RXNE flag is set. So do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// in SPI, the slave doesn't initiate the data transfer of ack/nack on its own.
		// thus, send a dummy byte to initiate the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t analog_read;

		if(SPI_VerifyResponse(ackbyte))
		{
			// if command acknowledged, then send further arguments
			args[0] = ANALOG_PIN0;

			SPI_SendData(SPI2, args, 1);

			// after sending args, dummy read is required to clear the RXNE
			// and to prepare to read the sensor values sent by the slave
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay so that the slave can be ready with the data
			// slave takes time to read the analog value. Slave does ADC conversion on that pin
			// master needs to wait before generating the dummy write to fetch the result

			delay();  // 200 ms delay is too much but that is ok

			// send a dummy byte to initiate the response from the slave providing
			// the sensor values
			SPI_SendData(SPI2, &dummy_write, 1);

			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d\n", analog_read);
		}


		// Send a command: 3. CMD_LED_READ	<pin no(1)>

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		// send the command to the slave
		// the slave will either acknowledge (ack) or not acknowledge (nack)
		SPI_SendData(SPI2, &commandcode, 1);

		// in SPI, when master or slave sends 1 byte, it also receives 1 byte of data
		// the transmission of 1 byte resulted in 1 garbage byte collection in Rx buffer
		// of the master and RXNE flag is set. So do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// in SPI, the slave doesn't initiate the data transfer of ack/nack on its own.
		// thus, send a dummy byte to initiate the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t Led_status;

		if(SPI_VerifyResponse(ackbyte))
		{
			// if command acknowledged, then send further arguments
			args[0] = LED_PIN;

			SPI_SendData(SPI2, args, 1);

			// after sending args, dummy read is required to clear the RXNE
			// and to prepare to read the sensor values sent by the slave
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// send a dummy byte to initiate the response from the slave
			// providing the LED status (ON or OFF)

			SPI_SendData(SPI2, &dummy_write, 1);
			SPI_ReceiveData(SPI2, &Led_status, 1);
			printf("COMMAND_LED_READ %d\n", Led_status);
		}

		// Send a command: 4. CMD_PRINT		  <len>		 <message(len)>

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		// send the command to the slave
		// the slave will either acknowledge (ack) or not acknowledge (nack)
		SPI_SendData(SPI2, &commandcode, 1);

		// in SPI, when master or slave sends 1 byte, it also receives 1 byte of data
		// the transmission of 1 byte resulted in 1 garbage byte collection in Rx buffer
		// of the master and RXNE flag is set. So do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// in SPI, the slave doesn't initiate the data transfer of ack/nack on its own.
		// thus, send a dummy byte to initiate the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t message[] = "This is the second SPI application!";

		if(SPI_VerifyResponse(ackbyte))
		{
			// first send data length information to slave. If length is bigger
			// than 255 then change to uint16_t and change length from 1 byte to 2.
			args[0] = strlen((char*)message);
			SPI_SendData(SPI2,args, 1);

			// send message
			SPI_SendData(SPI2, message, args[0]);

			printf("COMMAND_PRINT Executed\n");
		}

		// Send a command: 5. CMD_ID_READ

		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		// send the command to the slave
		// the slave will either acknowledge (ack) or not acknowledge (nack)
		SPI_SendData(SPI2, &commandcode, 1);

		// in SPI, when master or slave sends 1 byte, it also receives 1 byte of data
		// the transmission of 1 byte resulted in 1 garbage byte collection in Rx buffer
		// of the master and RXNE flag is set. So do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// in SPI, the slave doesn't initiate the data transfer of ack/nack on its own.
		// thus, send a dummy byte to initiate the response from the slave.
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t board_ID[10];
		uint32_t i=0;
		if(SPI_VerifyResponse(ackbyte))
		{
			for(i=0; i<10; i++)
			{
				// send dummy byte to initiate response from slave
				SPI_SendData(SPI2, &dummy_write, 1);
				// receive the bytes from the slave
				SPI_ReceiveData(SPI2, board_ID+i, 1);
			}

			board_ID[11] = '\0';   // null termination of array

			printf("COMMAND_ID_READ Executed. ID is %s \n", board_ID);
		}

		// confirm that SPI is not busy sending data before closing it
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		// disable the SPI2 peripheral clock
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("SPI Communication Closed\n");
	}
	return 0;
}

