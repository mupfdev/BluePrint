/**
 * @file      MSP.c
 * @brief     This file provides code for the MSP initialisation and
 *            de-initialisation codes
 * @copyright Copyright (c) 2020 STMicroelectronics.
 *            All rights reserved.
 *            This software component is licensed by ST under Ultimate
 *            Liberty license SLA0044, the "License"; You may not use
 *            this file except in compliance with the License.  You may
 *            obtain a copy of the License at:  www.st.com/SLA0044
 */

#include "stm32f1xx_hal.h"

/**
 * @brief Initialises the Global MSP.
 */
void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init
     * PendSV_IRQn interrupt configuration
     */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    // NOJTAG: JTAG-DP Disabled and SW-DP Enabled
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/**
 * @brief ADC MSP Initialisation
 * @param hadc: ADC handle pointer
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    if(ADC1 == hadc->Instance)
    {
        // Peripheral clock enable
        __HAL_RCC_ADC1_CLK_ENABLE();
        // ADC1 interrupt Init
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    }
}

/**
 * @brief  ADC MSP De-Initialisation
 * @param  hadc: ADC handle pointer
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
    if(ADC1 == hadc->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_ADC1_CLK_DISABLE();

        // ADC1 interrupt DeInit
        HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    }
}

/**
 * @brief CRC MSP Initialisation
 * @param hcrc: CRC handle pointer
 */
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
    if(CRC == hcrc->Instance)
    {
        // Peripheral clock enable
        __HAL_RCC_CRC_CLK_ENABLE();
    }

}

/**
 * @brief CRC MSP De-Initialisation
 * @param hcrc: CRC handle pointer
 */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
    if(CRC == hcrc->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_CRC_CLK_DISABLE();
    }
}

/**
 * @brief I2C MSP Initialisation
 * @param hi2c: I2C handle pointer
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if(I2C1 == hi2c->Instance)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* I²C1 GPIO Configuration
         *   PB6 ---> I2C1_SCL
         *   PB7 ---> I2C1_SDA
         */
        GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // Peripheral clock enable
        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

/**
 * @brief I2C MSP De-Initialisation
 * @param hi2c: I2C handle pointer
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    if(I2C1 == hi2c->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_I2C1_CLK_DISABLE();

        /* I2C1 GPIO Configuration
         *   PB6 ---> I2C1_SCL
         *   PB7 ---> I2C1_SDA
         */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    }
}

/**
 * @brief SPI MSP Initialisation
 * @param hspi: SPI handle pointer
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if(SPI1 == hspi->Instance)
    {
        // Peripheral clock enable
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* SPI1 GPIO Configuration
         *   PA5 ---> SPI1_SCK
         *   PA6 ---> SPI1_MISO
         *   PA7 ---> SPI1_MOSI
         */
        GPIO_InitStruct.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // SPI1 interrupt Init
        HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
    }
}

/**
 * @brief SPI MSP De-Initialisation
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
    if(SPI1 == hspi->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_SPI1_CLK_DISABLE();

        /* SPI1 GPIO Configuration
         *   PA5 ---> SPI1_SCK
         *   PA6 ---> SPI1_MISO
         *   PA7 ---> SPI1_MOSI
         */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

        /// SPI1 interrupt DeInit
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
    }
}
