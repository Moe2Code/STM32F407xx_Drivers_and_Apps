/**
  ******************************************************************************
  * @file           : 006SPI_tx_testing.c
  * @author         : Moe2Code
  * @brief          : This is an application that uses the ST Discovery board to transmits data
  * 				  via SPI. The transmission is verified to be successful using a logic analyzer.
  * 				  The following pins configuration was used on ST Discovery
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


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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

	// NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CFG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;  // generates sclk of 8 MHz
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;  // software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}


int main (void)
{
	char user_data[] = "Hello world";

	// initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// initialize SPI2 peripheral parameters
	SPI2_Inits();

	// this function makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	// enable SPI2 peripheral. To do after calling SPI2_Inits()
	SPI_PeripheralControl(SPI2, ENABLE);

	// to send data
	SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

	// confirm that SPI is not busy sending data before closing it
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	// disable the SPI2 peripheral clock
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1);

	return 0;
}
