/*
 * 005button_interrupt.c
 *
 *  Created on: Sep 15, 2019
 *      Author: Mohammed
 */

#include<string.h>
#include "stm32f407xx.h"

#define HIGH		 	1
#define BTN_PRESSED 	HIGH

void delay(void)
{
	//this will introduce ~200 ms delay when system clock is 16 MHz
	for(uint32_t i=0; i<=500000/2; i++);
}


int main(void)
{

	GPIO_Handle_t GpioLed, GpioButton;

	memset(&GpioLed,0,sizeof(GpioLed));			// initializes every element of GpioLed to 0
	memset(&GpioButton,0,sizeof(GpioButton));

	GpioLed.pGPIOx = GPIOD;

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	GPIO_Init(&GpioLed);

	GpioButton.pGPIOx = GPIOD;

	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;   //To have falling edge trigger
	GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;

	GPIO_Init(&GpioButton);

	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	// GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI14); // just test code line

	while(1);

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
	delay();  // ~200 ms delay to wait till button de-bouncing is over
	GPIO_IRQHandling(GPIO_PIN_NO_5);  // Pin number for button, clear pending interrupt from EXTI line
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12); // Toggle LED at PD12

}




