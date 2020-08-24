/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Oct 10, 2019
 *      Author: Mohammed
 */

#include "stm32f407xx.h"

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_


uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);



#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
