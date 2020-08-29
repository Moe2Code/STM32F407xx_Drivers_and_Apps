/**
  ******************************************************************************
  * @file           : 015UART_tx.c
  * @author         : Moe2Code
  * @brief          : This is an application that sends one message to an Arduino Uno via
  * 				  UART using interrupt mode. The following pins configuration was used
  * 				  on ST Discovery
  * 				  PA2 --> USART2 TX
  * 				  PA3 --> USART2 RX
  * 				  Alternate function mode: 7
  *******************************************************************************
*/

// Includes
#include "stm32f407xx.h"
#include <string.h>


// Global variables
char msg[1024] = "UART Tx testing..\r\n";
USART_Handle_t USART2Handle;


// Simple delay function
void delay(void)
{
	for(uint32_t i=0; i<=500000; i++);
}


void USART2_GPIOInits(void)
{
	GPIO_Handle_t USART2Pins;

	USART2Pins.pGPIOx = GPIOA;
	USART2Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART2Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USART2Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART2Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART2Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// TX
	USART2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&USART2Pins);

	// RX
	USART2Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USART2Pins);
}

void USART2_Inits(void)
{

	USART2Handle.pUSARTx = USART2;
	USART2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS1;
	USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART2Handle);

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

	// Initializations of GPIOs and UART peripheral
	USART2_GPIOInits();

	USART2_Inits();

	USART_PeripheralControl(USART2, ENABLE);

	ButtonInit();

	while(1)
	{
		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		// Send message, msg, to Arduino Uno
		USART_SendData(&USART2Handle,(uint8_t*)msg, strlen(msg));
	}

	return 0;
}

