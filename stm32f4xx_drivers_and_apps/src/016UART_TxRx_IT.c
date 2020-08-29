/**
  ******************************************************************************
  * @file           : 016UART_TxRx_IT.c
  * @author         : Moe2Code
  * @brief          : This is an application that sends/receives one message from an Arduino Uno
  * 				  via UART using interrupt mode. The following pins configuration was used on
  * 				  ST Discovery
  * 				  PA2 --> USART2 TX
  * 				  PA3 --> USART2 RX
  * 				  Alternate function mode: 7
  *******************************************************************************
*/

// Includes
#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"


// Semihosting prototype to be able to use printf function
extern void initialise_monitor_handles();


// Global variables
USART_Handle_t USART2Handle;
char tx_msg[1024] = "UART testing..\r\n";
char rx_msg[1024];
uint8_t tx_cmplt = RESET;
uint8_t rx_cmplt = RESET;


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
	USART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
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
	// Executing semihosting initialize function to be able to use printf function
	initialise_monitor_handles();

	ButtonInit();

	USART2_GPIOInits();

	USART2_Inits();

	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

	USART_PeripheralControl(USART2, ENABLE);

	printf("Application is running\n");

	while(1)
	{
		// wait till button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-bouncing related issues. ~200ms of delay
		delay();

		while(USART_SendDataIT(&USART2Handle,(uint8_t*)tx_msg, strlen(tx_msg)) != USART_READY);

		while(USART_ReceiveDataIT(&USART2Handle,(uint8_t*)rx_msg, strlen(tx_msg)) != USART_READY);

		while(tx_cmplt != SET); // wait until the full Tx msg is transmitted

		tx_cmplt = RESET;

		printf("DATA TRANSMITTED: %s\n", tx_msg);

		while(rx_cmplt != SET); // wait until the full Rx msg is received

		rx_cmplt = RESET;

    	//just make sure that last byte should be null otherwise %s fails while printing
    	rx_msg[strlen(rx_msg)+ 1] = '\0';

		printf("DATA RECEIVED: %s\n", rx_msg);
	}

	return 0;
}


void USART2_IRQHandler(void)
{

	USART_IRQHandling(&USART2Handle);
}


void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{
	if(event == USART_EVENT_TX_CMPLT)
	{
		printf("Tx completed\n");
		tx_cmplt = SET;

	}else if(event == USART_EVENT_RX_CMPLT)
	{
		printf("Rx completed\n");
		rx_cmplt = SET;

	}else if(event == USART_EVENT_IDLE)
	{

	}else if(event == USART_EVENT_CTS)
	{

	}else if(event == USART_ERREVENT_PE)
	{

	}else if(event == USART_ERREVENT_FE)
	{

	}else if(event == USART_ERREVENT_NE)
	{

	}else if(event == USART_ERREVENT_ORE)
	{

	}
}
