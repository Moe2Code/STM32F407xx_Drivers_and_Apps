/**
  ******************************************************************************
  * @file           : 002LED_button.c
  * @author         : Moe2Code
  * @brief          : This is an application that uses blocking mode with GPIO to turn on/off
  * 				  an LED connected to PD12 when the button at PD5 is pushed. ST Discovery
  * 				  board is used in this application.
  *******************************************************************************
*/

// Includes
#include "stm32f407xx.h"


// Defines
#define HIGH		 	1
#define BTN_PRESSED 	HIGH


// Simple delay function
void delay(void)
{
	for(uint32_t i=0; i<=500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	GPIO_Init(&GpioLed);


	GPIO_Handle_t GpioButton;

	GpioButton.pGPIOx = GPIOA;

	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;

	GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);

	GPIO_Init(&GpioButton);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GpioButton.pGPIOx,GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_15);
		}
	}

	return 0;
}

