/**
 * @file      system_spi.c
 *
 * @brief     MCU SPI-related functions header file
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SYSTEM_SPI_H
#define _SYSTEM_SPI_H


#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "r_spi_cmsis_api.h"
#include "system.h"
#include "system_gpio.h"

//#include "stm32l4xx_ll_bus.h"
//#include "stm32l4xx_ll_gpio.h"
//#include "stm32l4xx_ll_spi.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif



void system_spi_init( ARM_DRIVER_SPI* spi) __attribute__ ((section(".ramfunc")));
void system_spi_stop( ARM_DRIVER_SPI* spi) __attribute__ ((section(".ramfunc")));

void system_spi_write( ARM_DRIVER_SPI* spi, const uint8_t* buffer, uint16_t length )  __attribute__ ((section(".ramfunc")));
void system_spi_read( ARM_DRIVER_SPI* spi, uint8_t* buffer, uint16_t length )  __attribute__ ((section(".ramfunc")));
void system_spi_write_read( ARM_DRIVER_SPI* spi, const uint8_t* cbuffer, uint8_t* rbuffer, uint16_t length )  __attribute__ ((section(".ramfunc")));
//void system_spi_write( SPI_TypeDef* spi, const uint8_t* buffer, uint16_t length );
//void system_spi_read( SPI_TypeDef* spi, uint8_t* buffer, uint16_t length );
//void system_spi_write_read( SPI_TypeDef* spi, const uint8_t* cbuffer, uint8_t* rbuffer, uint16_t length );

#ifdef __cplusplus
}
#endif

#endif
