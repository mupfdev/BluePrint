/**
 * @file      RTOS.c
 * @brief     FreeRTOS callbacks
 * @copyright Copyright (c) 2020 STMicroelectronics.
 *            All rights reserved.
 *            This software component is licensed by ST under Ultimate
 *            Liberty license SLA0044, the "License"; You may not use
 *            this file except in compliance with the License.  You may
 *            obtain a copy of the License at:  www.st.com/SLA0044
 */

#include "FreeRTOS.h"
#include "task.h"

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t  xIdleStack[configMINIMAL_STACK_SIZE];

/**
 * @brief Provide the memory for use by the RTOS Idle task.
 * @note  Required callback for configSUPPORT_STATIC_ALLOCATION.
 */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}
