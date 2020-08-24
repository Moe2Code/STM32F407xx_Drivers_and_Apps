/*
 * 001LED_Toggle.c
 *
 *  Created on: Sep 10, 2019
 *      Author: Mohammed
 */

#include "stm32f407xx.h"

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
