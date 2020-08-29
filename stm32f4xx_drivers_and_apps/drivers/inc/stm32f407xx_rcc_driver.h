/**
  ******************************************************************************
  * @file    stm32f407xx_rcc_driver.h
  * @author  Moe2Code
  * @brief   Header file of RCC module.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion */
#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

/* Includes */
#include "stm32f407xx.h"


/*****************************************************************************************************
 *                                      APIs supported by this driver
 *                    For more information about these APIs check the function definitions
 *****************************************************************************************************/

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);


#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
