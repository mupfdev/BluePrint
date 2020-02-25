// SPDX-License-Identifier: Beerware
/**
 * @file      main.c
 * @brief     A base project for the STM32F103C8T6 aka blue pill board.
 * @author    Michael Fitzmayer
 * @copyright "THE BEER-WARE LICENCE" (Revision 42)
 */

#include <stdint.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "System.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "task.h"

static TaskHandle_t hMainThread = NULL;

static void MainThread(void* pArg);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    static uint8_t u8BlinkDelay = 250;

    if (SYSTEM_OK != System_Init())
    {
        u8BlinkDelay = 100;
    }

    xTaskCreate(
        MainThread,
        "Main thread",
        configMINIMAL_STACK_SIZE,
        &u8BlinkDelay,
        osPriorityNormal,
        &hMainThread);

    osKernelStart();
    while(1);

    return EXIT_SUCCESS;
}

/**
 * @brief Main thread handler
 * @param pArg: Loop delay in ms
 */
static void MainThread(void* pArg)
{
    while(1)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        osDelay(*(uint16_t*)pArg);
    };
}
