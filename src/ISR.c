/**
 * @file      ISR.c
 * @brief     Interrupt service routines
 * @copyright Copyright (c) 2020 STMicroelectronics.
 *            All rights reserved.
 *            This software component is licensed by ST under Ultimate
 *            Liberty license SLA0044, the "License"; You may not use
 *            this file except in compliance with the License.  You may
 *            obtain a copy of the License at:  www.st.com/SLA0044
 */

#include "ISR.h"
#include "stm32f1xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim4;

/*
 * Cortex-M3 Processor Interruption and Exception Handlers
 */

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    while (1)
    {
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    while (1)
    {
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    while (1)
    {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
}

/* STM32F1xx Peripheral Interrupt Handlers
 * Add here the Interrupt Handlers for the used peripherals.
 * For the available peripheral interrupt handler names, please refer to
 * the startup file (startup_stm32f1xx.s).
 */

/**
 * @brief This function handles ADC1 and ADC2 global interrupts.
 */
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

/**
 * @brief This function handles SPI1 global interrupt.
 */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}
