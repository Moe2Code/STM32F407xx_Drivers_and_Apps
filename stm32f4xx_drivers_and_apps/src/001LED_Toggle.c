/**
  ******************************************************************************
  * @file           : 001LED_Toggle.c
  * @author         : Moe2Code
  * @brief          : Simple application that toggles an LED on/off continuously on
  *                   an ST Discovery board
  *******************************************************************************
*/

// Includes
#include "stm32f407xx.h"


// Simple delay function
void delay(void)
{
	for(uint32_t i=0; i<=500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;     // was GPIO_PIN_NO_12
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;   // was GPIO_NO_PUPD
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;    // was GPIO_OP_TYPE_PP

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_14);   // was GPIO_PIN_NO_12
		delay();
	}

	return 0;
}
