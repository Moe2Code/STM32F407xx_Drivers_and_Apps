/**
  ******************************************************************************
  * @file           : 007SPI_txonly_arduino.c
  * @author         : Moe2Code
  * @brief          : This is an application that uses the ST Discovery board as a master to transmits data
  * 				  via SPI to Arduino Uno board, slave. The following pins configuration was used on ST
  * 				  Discovery
  * 				  PB14 --> SPI2_MISO
  * 				  PB15 --> SPI2_MOSI
  * 				  PB13 --> SPI2_SCLK
  * 				  PB12 --> SPI2_NSS
  * 				  Alternate function mode: 5
  *******************************************************************************
*/

// Includes
#include "stm32f407xx.h"
#include <string.h>


// Simple delay function
void delay(void)
{
	for(uint32_t i=0; i<=500000; i++);
}

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

	// MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

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


int main (void)
{

	char user_data[] = "Hello world. What is up? I am communicating with you!";

	ButtonInit();

	// initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// initialize SPI2 peripheral parameters
	SPI2_Inits();

	/*
	 * making SSOE 1 does NSS output enable
	 * The NSS pin is automatically management by hardware
	 * i.e. when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);


	while(1)
	{
		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		// enable SPI2 peripheral. To do after calling SPI2_Inits()
		SPI_PeripheralControl(SPI2, ENABLE);

		// first send data length information to slave. If dataLen is bigger than 255 then
		// change to uint16_t and change length from 1 byte to 2.
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// to send data
		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

		// confirm that SPI is not busy sending data before closing it
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		// disable the SPI2 peripheral clock
		SPI_PeripheralControl(SPI2,DISABLE);
	}

	return 0;
}
