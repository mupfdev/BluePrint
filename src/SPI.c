// SPDX-License-Identifier: Beerware
/**
 * @file      SPI.c
 * @brief     SPI interface
 * @author    Michael Fitzmayer
 * @copyright "THE BEER-WARE LICENCE" (Revision 42)
 */

#include <stdint.h>
#include "SPI.h"
#include "System.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;

/**
 * @brief  Transmit an amount via SPI
 * @param  pu8TxData
 *         Pointer to data buffer
 * @param  u16Size
 *         Amount of data to be sent
 * @return System status code
 */
SystemStatus SPI_Transmit(uint8_t* pu8TxData, uint16_t u16Size)
{
    if (HAL_OK != HAL_SPI_Transmit_IT(&hspi1, pu8TxData, u16Size))
    {
        return SYSTEM_SPI_ERROR;
    }

    return SYSTEM_OK;
}

/**
 * @brief  Receive an amount of data via SPI
 * @param  pu8RxData
 *         Pointer to data buffer
 * @param  u16Size
 *         Amount of data to be sent
 * @return System status code
 */
SystemStatus SPI_Receive(uint8_t* pu8RxData, uint16_t u16Size)
{
    if (HAL_OK != HAL_SPI_Receive_IT(&hspi1, pu8RxData, u16Size))
    {
        return SYSTEM_SPI_ERROR;
    }

    return SYSTEM_OK;
}

/**
 * @brief  Transmit and Receive an amount of data via SPI
 * @param  pu8TxData
 *         Pointer to transmission data buffer
 * @param  pu8RxData
 *         Pointer to reception data buffer
 * @param  u16Size
 *         Amount of data to be sent and received
 * @return System status code
 */
SystemStatus SPI_TransmitReceive(uint8_t* pu8TxData, uint8_t* pu8RxData, uint16_t u16Size)
{
    if (HAL_OK != HAL_SPI_TransmitReceive_IT(&hspi1, pu8TxData, pu8RxData, u16Size))
    {
        return SYSTEM_SPI_ERROR;
    }

    return SYSTEM_OK;
}
