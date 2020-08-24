/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Oct 10, 2019
 *      Author: Mohammed
 */

#include "stm32f407xx_rcc_driver.h"


uint16_t AHB_PreScalar[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScalar[8] = {2,4,8,16};
uint8_t APB2_PreScalar[8] = {2,4,8,16};


/*******************************************************
*@fn					- RCC_GetPCLK1Value
*
*@brief					- This function calculates the peripheral clock speed of APB1 bus
*
*@param[in]				- None
*
*
*@return				- None
*
*@note					- None
*
*/

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2)& 0x3);     		// to determine whether the clock source from HSI, HSE, or PLL

	if(clksrc == 0)
	{
		SystemClk = 16000000;		    	   // clock source is HSI
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;			       // clock source is HSE
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();   // clock source is PLL (not taught in this course)
	}

	// determining clock prescalar value for AHB
	temp = ((RCC->CFGR >>4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScalar[temp-8];
	}

	// determining clock prescalar value for APB1

	temp = ((RCC->CFGR >>10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScalar[temp-4];
	}

	pclk1 = (SystemClk/ahbp)/apb1p;				// clock for APB1 peripherals
	return pclk1;
}

/*******************************************************
*@fn					- RCC_GetPCLK2Value
*
*@brief					- This function calculates the peripheral clock speed of APB2 bus
*
*@param[in]				- None
*
*
*@return				- None
*
*@note					- None
*
*/

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;
	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = ((RCC->CFGR >> 2)& 0x3);     		// to determine whether the clock source from HSI, HSE, or PLL

	if(clksrc == 0)
	{
		SystemClk = 16000000;		    	   // clock source is HSI
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;			       // clock source is HSE
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();   // clock source is PLL (not taught in this course)
	}

	// determining clock prescalar value for AHB
	temp = ((RCC->CFGR >>4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScalar[temp-8];
	}

	// determining clock prescalar value for APB2

	temp = ((RCC->CFGR >>13) & 0x7);

	if(temp < 4)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB2_PreScalar[temp-4];
	}

	pclk2 = (SystemClk/ahbp)/apb2p;			 // clock for APB2 peripherals
	return pclk2;
}


/*******************************************************
*@fn					- RCC_GetPLLOutputClock
*
*@brief					- This function is a place holder to implement for clock source from PLL
*
*@param[in]				-
*
*@return				-
*
*@note					-
*
*/

uint32_t RCC_GetPLLOutputClock(void)		// clock source is PLL (not taught in this course)
{
	return 0;
}



