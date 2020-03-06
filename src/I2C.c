// SPDX-License-Identifier: Beerware
/**
 * @file      I2C.c
 * @brief     I²C interface
 * @author    Michael Fitzmayer
 * @copyright "THE BEER-WARE LICENCE" (Revision 42)
 */

#include <stdint.h>
#include "System.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c2;

/**
 * @brief  Receive an amount via I²C
 * @param  u16DevAddress
 *         Target device address
 * @param  u16MemAddress
 *         Internal memory address
 * @param  pu8RxBuffer
 *         Pointer to data buffer
 * @param  u16Size
 *         Amount of data to be sent
 * @return System status code
 */
SystemStatus I2C_Receive(uint16_t u16DevAddress, uint16_t u16MemAddress, uint8_t* pu8RxBuffer, uint16_t u16Size)
{
    if (HAL_OK != HAL_I2C_Mem_Read_IT(
            &hi2c2,
            u16DevAddress,
            u16MemAddress,
            I2C_MEMADD_SIZE_16BIT,
            pu8RxBuffer,
            u16Size))
    {
        return SYSTEM_I2C_ERROR;
    }

    return SYSTEM_OK;
}

/**
 * @brief  Transmit an amount via I²C
 * @param  u16DevAddress
 *         Target device address
 * @param  u16MemAddress
 *         Internal memory address
 * @param  pu8TxBuffer
 *         Pointer to data buffer
 * @param  u16Size
 *         Amount of data to be sent
 * @return System status code
 */
SystemStatus I2C_Transmit(uint16_t u16DevAddress, uint16_t u16MemAddress, uint8_t* pu8TxBuffer, uint16_t u16Size)
{
    if (HAL_OK != HAL_I2C_Mem_Write_IT(
            &hi2c2,
            u16DevAddress,
            u16MemAddress,
            I2C_MEMADD_SIZE_16BIT,
            pu8TxBuffer,
            u16Size))
    {
        return SYSTEM_I2C_ERROR;
    }

    return SYSTEM_OK;
}
