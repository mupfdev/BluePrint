// SPDX-License-Identifier: Beerware
/**
 * @file  SPI.h
 * @brief SPI interface
 */
#pragma once

#include <stdint.h>
#include "System.h"

SystemStatus SPI_Transmit(uint8_t* pu8TxData, uint16_t u16Size);
SystemStatus SPI_Receive(uint8_t* pu8RxData, uint16_t u16Size);
SystemStatus SPI_TransmitReceive(uint8_t* pu8TxData, uint8_t* pu8RxData, uint16_t u16Size);
