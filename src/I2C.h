// SPDX-License-Identifier: Beerware
/**
 * @file  I2C.h
 * @brief I²C interface
 */
#pragma once

#include <stdint.h>
#include "System.h"

SystemStatus I2C_Receive(uint16_t u16DevAddress, uint16_t u16MemAddress, uint8_t* pu8RxBuffer, uint16_t u16Size);
SystemStatus I2C_Transmit(uint16_t u16DevAddress, uint16_t u16MemAddress, uint8_t* pu8TxBuffer, uint16_t u16Size);
