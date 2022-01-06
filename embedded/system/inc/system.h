/**
 * @file      system.h
 *
 * @brief     MCU system-related functions header
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

#ifndef __SYSTEM_H
#define __SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "r_spi_cmsis_api.h"


typedef struct configuration
{
//    GPIO_TypeDef* port;
//    uint32_t      pin;
	uint8_t port;
	uint8_t pin;
} gpio_t;


//typedef struct lr1110_s
//{
//    ARM_DRIVER_SPI*       spi_id;
//    gpio_t                  nss;
//    gpio_t              reset;
//    gpio_t                  busy;
//    void*                event;
//    uint32_t                   spi;
//    uint16_t op_mode;
//} lr1110_t;

typedef struct
{
    float latitude;
    float longitude;
    float altitude;
} environment_location_t;


typedef struct
{
    ARM_DRIVER_SPI* spi;
    gpio_t       nss;
    gpio_t       reset;
    gpio_t       busy;
    gpio_t       irq;
} radio_t;

#define APP_ERR_HANDLER(x)    if (x) {while(1);}
#define APP_ERR_RETURN(x)      {if(x){return x;}}
#define UNUSED(x)  (void)(x)


#define GLOBAL_INT_DISABLE()                        \
    do {                                            \
        uint32_t  __int_state = __get_PRIMASK();    \
        __set_PRIMASK(1U);                          \

  /* One or more line breaks are required */

#define GLOBAL_INT_RESTORE()                        \
        __set_PRIMASK(__int_state);                 \
    } while(0)


#define HAL_DBG_TRACE_PRINTF( ... )  printf (  __VA_ARGS__ )

#define HAL_DBG_TRACE_INFO( ... )                                          \
do                                                                         \
{                                                                          \
    HAL_DBG_TRACE_PRINTF( "INFO : " );                                     \
    HAL_DBG_TRACE_PRINTF( __VA_ARGS__ );                                   \
} while ( 0 );

#define HAL_DBG_TRACE_MSG( msg )                                           \
do                                                                         \
{                                                                          \
    HAL_DBG_TRACE_PRINTF( msg );                                           \
} while ( 0 );

//#include "system_clock.h"
//#include "system_gpio.h"
//#include "system_spi.h"
//#include "system_uart.h"
//#include "system_i2c.h"
//#include "system_time.h"
//#include "system_lptim.h"

void system_init( radio_t* radio )  __attribute__ ((section(".ramfunc")));
void system_init_ISO2( )  __attribute__ ((section(".ramfunc")));
void system_uninit_ISO2(  )  __attribute__ ((section(".ramfunc")));
//void system_uart_init(radio_t* radio)  __attribute__ ((section(".ramfunc")));

char* delComma(char* date_buf)  __attribute__ ((section(".ramfunc")));
char* delCRLF(char* date_buf)  __attribute__ ((section(".ramfunc")));
void rtc_alarm_init(void) __attribute__ ((section(".ramfunc")));
int	print_uart(const char * format, ...);


#endif

#ifdef __cplusplus
}
#endif
