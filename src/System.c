/**
 * @file       System.c
 * @brief      STM32F103C8T6 system initialisation
 * @copyright  Copyright (c) 2020 STMicroelectronics.
 *             All rights reserved.
 *             This software component is licensed by ST under Ultimate
 *             Liberty license SLA0044, the "License"; You may not use
 *             this file except in compliance with the License.  You may
 *             obtain a copy of the License at:  www.st.com/SLA0044
 * @page       SysOverview System overview
 * @section    GPIOConfig GPIO configuration
 * @subsection GPIO_ADC1 ADC 1
 *
 * @li PB0 ---> ADC1_IN8
 * @li PB1 ---> ADC1_IN9
 *
 * @subsection GPIO_I2C2 I²C 2
 *
 * @li PB10 ---> I2C2_SCL
 * @li PB11 ---> I2C2_SDA
 *
 * @subsection GPIO_SPI1 SPI 1
 *
 * @li PA5 ---> SPI1_SCK
 * @li PA6 ---> SPI1_MISO
 * @li PA7 ---> SPI1_MOSI
 *
 * @subsection GPIO_TIM1 TIM 1
 *
 * @li PA8 ---> TIM1_CH1
 *
 * @subsection GPIO_GENERIC Generic
 *
 * @li PC13 ---> LED
 *
 */

#include "stm32f1xx_hal.h"
#include "System.h"

ADC_HandleTypeDef hadc1; ///< ADC 1 handle
I2C_HandleTypeDef hi2c2; ///< I²C 2 handle
SPI_HandleTypeDef hspi1; ///< SPI 1 handle
TIM_HandleTypeDef htim1; ///< Timer 1 handle
TIM_HandleTypeDef htim4; ///< Timer 4 handle (Sys-Tick)

static void System_GPIO_Init(void);
static int  System_TIM1_Init(void);
static int  System_ADC1_Init(void);
static int  System_I2C2_Init(void);
static int  System_SPI1_Init(void);

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
int System_Init(void)
{
    int                      nStatus           = 0;
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
        return -1;
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
        return -1;
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;

    if (HAL_OK != HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit))
    {
        return -1;
    }

    // Initialise peripherals
    System_GPIO_Init();

    nStatus = System_TIM1_Init();
    if (0 != nStatus)
    {
        return nStatus;
    }

    if (HAL_OK != HAL_TIM_Base_Start(&htim1))
    {
        return -1;
    }

    nStatus = System_ADC1_Init();
    if (0 != nStatus)
    {
        return nStatus;
    }

    nStatus = System_I2C2_Init();
    if (0 != nStatus)
    {
        return nStatus;
    }

    nStatus = System_SPI1_Init();
    return nStatus;
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

/**
 * @brief  Timer 1 Initialisation Function
 * @return System status code
 */
static int System_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig   = { 0 };
    TIM_MasterConfigTypeDef        sMasterConfig        = { 0 };
    TIM_OC_InitTypeDef             sConfigOC            = { 0 };
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

    extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 72-1;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 0xFFFF-1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_OK != HAL_TIM_Base_Init(&htim1))
    {
        return -1;
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if (HAL_OK != HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig))
    {
        return -1;
    }

    if (HAL_OK != HAL_TIM_OC_Init(&htim1))
    {
        return -1;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_OK != HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig))
    {
        return -1;
    }

    sConfigOC.OCMode       = TIM_OCMODE_FORCED_ACTIVE;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_OK != HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1))
    {
        return -1;
    }

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_OK != HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig))
    {
        return -1;
    }

    HAL_TIM_MspPostInit(&htim1);

    return 0;
}

/**
 * @brief  ADC 1 Initialisation Function
 * @return System status code
 */
static int System_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    // Common config
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 4;

    if (HAL_OK != HAL_ADC_Init(&hadc1))
    {
        return -1;
    }

    // Configure Regular Channel
    sConfig.Channel      = ADC_CHANNEL_8;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    if (HAL_OK != HAL_ADC_ConfigChannel(&hadc1, &sConfig))
    {
        return -1;
    }

    // Configure Regular Channel
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank    = ADC_REGULAR_RANK_2;

    if (HAL_OK != HAL_ADC_ConfigChannel(&hadc1, &sConfig))
    {
        return -1;
    }

    // Configure Regular Channel
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank    = ADC_REGULAR_RANK_3;

    if (HAL_OK != HAL_ADC_ConfigChannel(&hadc1, &sConfig))
    {
        return -1;
    }

    // Configure Regular Channel
    sConfig.Rank = ADC_REGULAR_RANK_4;

    if (HAL_OK != HAL_ADC_ConfigChannel(&hadc1, &sConfig))
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  I²C 2 Initialisation Function
 * @return System status code
 */
static int System_I2C2_Init(void)
{
    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_OK != HAL_I2C_Init(&hi2c2))
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  SPI 1 Initialisation Function
 * @return Error code
 * @retval  0: OK
 * @retval -1: Error
 */
static int System_SPI1_Init(void)
{
    // SPI1 parameter configuration
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;

    if (HAL_OK != HAL_SPI_Init(&hspi1))
    {
        return -1;
    }

    return 0;
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
 * @brief Initialis the Global MSP.
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
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if(ADC1 == hadc->Instance)
    {
        // Peripheral clock enable
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* ADC1 GPIO Configuration
         *
         *   PB0 ---> ADC1_IN8
         *   PB1 ---> ADC1_IN9
         */
        GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // ADC1 interrupt Init
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    }
}

/**
 * @brief ADC MSP De-Initialisation
 * @param hadc: ADC handle pointer
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
    if (ADC1 == hadc->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_ADC1_CLK_DISABLE();

        /* ADC1 GPIO Configuration
         *
         *   PB0 ---> ADC1_IN8
         *   PB1 ---> ADC1_IN9
         */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);

        // ADC1 interrupt DeInit
        HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
    }
}

/**
 * @brief I2C MSP Initialisation
 * @param hi2c: I2C handle pointer
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    if(I2C2 == hi2c->Instance)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* I2C2 GPIO Configuration
         *
         *   PB10 ---> I2C2_SCL
         *   PB11 ---> I2C2_SDA
         */
        GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // Peripheral clock enable
        __HAL_RCC_I2C2_CLK_ENABLE();

        // I2C2 interrupt Init
        HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    }
}

/**
 * @brief I2C MSP De-Initialisation
 * @param hi2c: I2C handle pointer
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    if(I2C2 == hi2c->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_I2C2_CLK_DISABLE();

        /* I2C2 GPIO Configuration
         *
         *   PB10 ---> I2C2_SCL
         *   PB11 ---> I2C2_SDA
         */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);

        // I2C2 interrupt DeInit
        HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
        HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
    }
}

/**
 * @brief SPI MSP Initialisation
 * @param hspi: SPI handle pointer
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(SPI1 == hspi->Instance)
    {
        // Peripheral clock enable
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* SPI1 GPIO Configuration
         *
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
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
    if(SPI1 == hspi->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_SPI1_CLK_DISABLE();

        /* SPI1 GPIO Configuration
         *
         *   PA5 ---> SPI1_SCK
         *   PA6 ---> SPI1_MISO
         *   PA7 ---> SPI1_MOSI
         */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

        // SPI1 interrupt DeInit
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
    }
}

/**
 * @brief TIM_Base MSP Initialisation
 * @param htim_base: TIM_Base handle pointer
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(TIM1 == htim_base->Instance)
    {
        // Peripheral clock enable
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
}

/**
 * @brief TIM MSP Post-Initialisation
 * @param htim: TIM handle pointer
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if(TIM1 == htim->Instance)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* TIM1 GPIO Configuration
         *
         *   PA8 ---> TIM1_CH1
         */
        GPIO_InitStruct.Pin   = GPIO_PIN_8;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/**
 * @brief TIM_Base MSP De-Initialisation
 * @param htim_base: TIM_Base handle pointer
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if(TIM1 == htim_base->Instance)
    {
        // Peripheral clock disable
        __HAL_RCC_TIM1_CLK_DISABLE();
    }
}

/**
 * @brief  This function configures the TIM4 as a time base source.
 *         The time source is configured to have 1ms time base with a
 *         dedicated Tick interrupt priority.
 * @note   This function is called automatically at the beginning of
 *         program after reset by HAL_Init() or at any time when clock
 *         is configured, by HAL_RCC_ClockConfig().
 * @param  TickPriority: Tick interrupt priority.
 * @retval HAL status
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t           uwTimclock       = 0;
    uint32_t           uwPrescalerValue = 0;
    uint32_t           pFLatency;

    // Configure the TIM4 IRQ priority
    HAL_NVIC_SetPriority(TIM4_IRQn, TickPriority, 0);

    // Enable the TIM4 global Interrupt
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    // Enable TIM4 clock
    __HAL_RCC_TIM4_CLK_ENABLE();

    // Get clock configuration
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    // Compute TIM4 clock
    uwTimclock = 2*HAL_RCC_GetPCLK1Freq();

    /* Compute the prescaler value to have TIM4 counter clock equal to
     * 1MHz
     */
    uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);

    // Initialise TIM4
    htim4.Instance = TIM4;

    /* Initialise TIMx peripheral as follow:
     *   + Period = [(TIM4CLK/1000) - 1]. to have a (1/1000) s time base.
     *   + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
     *   + ClockDivision = 0
     *   + Counter direction = Up
     */
    htim4.Init.Period        = (1000000 / 1000) - 1;
    htim4.Init.Prescaler     = uwPrescalerValue;
    htim4.Init.ClockDivision = 0;
    htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;

    if(HAL_OK == HAL_TIM_Base_Init(&htim4))
    {
        // Start the TIM time Base generation in interrupt mode
        return HAL_TIM_Base_Start_IT(&htim4);
    }

    // Return function status
    return HAL_ERROR;
}

/**
 * @brief  Suspend Tick increment.
 * @note   Disable the tick increment by disabling TIM4 update
 *         interrupt.
 */
void HAL_SuspendTick(void)
{
    // Disable TIM4 update Interrupt
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
}

/**
 * @brief Resume Tick increment.
 * @note  Enable the tick increment by Enabling TIM4 update interrupt.
 */
void HAL_ResumeTick(void)
{
    // Enable TIM4 Update interrupt
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

/**
 * @brief Non-maskable interrupt handler
 */
void NMI_Handler(void)
{
}

/**
 * @brief Hard fault handler
 */
void HardFault_Handler(void)
{
    while (1) {}
}

/**
 * @brief Memory management fault handler
 */
void MemManage_Handler(void)
{
    while (1) {}
}

/**
 * @brief Prefetch fault, memory access fault handler
 */
void BusFault_Handler(void)
{
    while (1) {}
}

/**
 * @brief Undefined instruction or illegal state handler
 */
void UsageFault_Handler(void)
{
    while (1) {}
}

/**
 * @brief Debug monitor handler
 */
void DebugMon_Handler(void)
{
}

/*
 * STM32F1xx Peripheral Interrupt Handlers
 * Add here the Interrupt Handlers for the used peripherals.
 * For the available peripheral interrupt handler names, please refer to
 * the startup file (startup_stm32f1xx.s).
 */

/**
 * @brief ADC1 and ADC2 global interrupt handler
 */
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

/**
 * @brief TIM4 global interrupt handler
 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim4);
}

/**
 * @brief I2C2 event interrupt handler
 */
void I2C2_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c2);
}

/**
 * @brief I2C2 error interrupt handler.
 */
void I2C2_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c2);
}

/**
 * @brief SPI1 global interrupt handler
 */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1);
}
