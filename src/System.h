/**
 * @file      System.h
 * @brief     STM32F103C8T6 system initialisation
 * @copyright Copyright (c) 2020 STMicroelectronics.
 *            All rights reserved.
 *            This software component is licensed by ST under Ultimate
 *            Liberty license SLA0044, the "License"; You may not use
 *            this file except in compliance with the License.  You may
 *            obtain a copy of the License at:  www.st.com/SLA0044
 */
#pragma once

#include "stm32f1xx_hal.h"
#include "MCAL.h"

#define LED_Pin       GPIO_PIN_13 ///< LED pin
#define LED_GPIO_Port GPIOC       ///< LED GPIO port

int System_Init(void);
