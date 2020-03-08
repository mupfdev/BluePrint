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
#include "MCAL.h"
#include "System.h"
#include "cmsis_os.h"
#include "task.h"

static TaskHandle_t hMainThread = NULL;
static uint8_t      u8BlinkDelay = 250;

static void MainThread(void* pArg);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    if (0 != System_Init())
    {
        u8BlinkDelay = 100;
    }

    xTaskCreate(
        MainThread,
        "Main thread",
        configMINIMAL_STACK_SIZE,
        NULL,
        osPriorityNormal,
        &hMainThread);

    osKernelStart();
    while(1);

    return EXIT_SUCCESS;
}

/**
 * @brief Main thread handler
 * @param pArg: Unused
 */
static void MainThread(void* pArg)
{
    while(1)
    {
        GPIO_Toggle(GPIO_PORT_C, LED_Pin);
        osDelay(u8BlinkDelay);
    };
}
