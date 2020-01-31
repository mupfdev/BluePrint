/**
 * @file      System.c
 * @brief     STM32F103C8T6 system initialisation
 * @copyright Copyright (c) 2020 STMicroelectronics.
 *            All rights reserved.
 *            This software component is licensed by ST under Ultimate
 *            Liberty license SLA0044, the "License"; You may not use
 *            this file except in compliance with the License.  You may
 *            obtain a copy of the License at:  www.st.com/SLA0044
 */

#include "stm32f1xx_hal.h"
#include "System.h"

ADC_HandleTypeDef hadc1; ///< ADC handle
CRC_HandleTypeDef hcrc;  ///< CRC handle
I2C_HandleTypeDef hi2c1; ///< I²C handle
SPI_HandleTypeDef hspi1; ///< SPI handle

static void         System_GPIO_Init(void);
static SystemStatus System_ADC_Init(void);
static SystemStatus System_CRC_Init(void);
static SystemStatus System_I2C_Init(void);
static SystemStatus System_SPI_Init(void);

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called when TIM4 interrupt took place,
 *         inside HAL_TIM_IRQHandler(). It makes a direct call to
 *         HAL_IncTick() to increment a global variable "uwTick" used
 *         as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (TIM4 == htim->Instance)
    {
        HAL_IncTick();
    }
}

/**
 * @brief  System Initialisation Function
 * @return System status code
 */
SystemStatus System_Init(void)
{
    SystemStatus             eStatus           = SYSTEM_OK;
    RCC_OscInitTypeDef       RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = { 0 };

    /* Reset of all peripherals, Initialises the Flash interface and the
     * Systick.
     */
    HAL_Init();

    /** Initialises the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;

    if (HAL_OK != HAL_RCC_OscConfig(&RCC_OscInitStruct))
    {
        return SYSTEM_GENERAL_ERROR;
    }

    // Initialises the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK   |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1  |
        RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_OK != HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2))
    {
        return SYSTEM_GENERAL_ERROR;
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;

    if (HAL_OK != HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit))
    {
        return SYSTEM_GENERAL_ERROR;
    }

    // Initialise peripherals
    System_GPIO_Init();

    eStatus = System_ADC_Init();
    if (SYSTEM_OK != eStatus)
    {
        return eStatus;
    }

    eStatus = System_CRC_Init();
    if (SYSTEM_OK != eStatus)
    {
        return eStatus;
    }

    eStatus = System_I2C_Init();
    if (SYSTEM_OK != eStatus)
    {
        return eStatus;
    }

    eStatus = System_SPI_Init();
    return eStatus;
}

/**
 * @brief  ADC Initialisation Function
 * @return System status code
 */
static SystemStatus System_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    // Common config
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;

    if (HAL_OK != HAL_ADC_Init(&hadc1))
    {
        return SYSTEM_ADC_ERROR;
    }

    // Configure Regular Channel
    sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

    if (HAL_OK != HAL_ADC_ConfigChannel(&hadc1, &sConfig))
    {
        return SYSTEM_ADC_ERROR;
    }

    return SYSTEM_OK;
}

/**
 * @brief  CRC Initialisation Function
 * @return System status code
 */
static SystemStatus System_CRC_Init(void)
{
    hcrc.Instance = CRC;
    if (HAL_OK != HAL_CRC_Init(&hcrc))
    {
        return SYSTEM_CRC_ERROR;
    }

    return SYSTEM_OK;
}

/**
 * @brief  I²C Initialisation Function
 * @return System status code
 */
static SystemStatus System_I2C_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_OK != HAL_I2C_Init(&hi2c1))
    {
        return SYSTEM_I2C_ERROR;
    }

    return SYSTEM_OK;
}

/**
 * @brief  SPI Initialisation Function
 * @return System status code
 */
static SystemStatus System_SPI_Init(void)
{
    // SPI1 parameter configuration
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;

    if (HAL_OK != HAL_SPI_Init(&hspi1))
    {
        return SYSTEM_SPI_ERROR;
    }

    return SYSTEM_OK;
}

/**
 * @brief  GPIO Initialisation Function
 */
static void System_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = LED_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}
