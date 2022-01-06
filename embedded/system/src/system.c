/**
 * @file      system.c
 *
 * @brief     MCU system-related functions
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
#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "config_mode.h"
#include "configuration.h"
#include "system.h"
#include "system_clock.h"
#include "system_gpio.h"
#include "system_i2c.h"
#include "system_it.h"
#include "system_lptim.h"
#include "system_spi.h"
#include "system_time.h"
#include "system_rtc.h"
#include "system_lpm.h"
#include "system_i2c.h"
//#include "system_uart_fifo.h"
#include "system_uart_fifo_low_level.h"
#include "system_i2c_simple_low_level.h"

#include "r_spi_cmsis_api.h"
#include "lr1110_hal.h"
#include "R_Driver_USART.h"


extern ARM_DRIVER_I2C Driver_I2C0;
//extern ARM_DRIVER_I2C Driver_I2C1;
extern ARM_DRIVER_SPI Driver_SPI0;
extern ARM_DRIVER_USART Driver_USART0;

extern uint32_t rtc_global_count;
extern UART_DRIVER_FIFO uart0_dev;


void system_init(radio_t* radio)
{

	radio->nss.port   = LR1110_NSS_PORT;
	radio->nss.pin    = LR1110_NSS_PIN;
	radio->reset.port = LR1110_RESET_PORT;
	radio->reset.pin  = LR1110_RESET_PIN;
	radio->busy.port  = LR1110_BUSY_PORT;
	radio->busy.pin  = LR1110_BUSY_PIN;
	radio->irq.port  = LR1110_IRQ_PORT;
	radio->irq.pin  = LR1110_IRQ_PIN;
	radio->spi = &Driver_SPI0;

//    R_LPM_ModuleStart(LPM_MSTP_SCI0);
//    R_LPM_ModuleStart(LPM_MSTP_SPI0);
//    R_LPM_ModuleStart(LPM_MSTP_RTC);
//    R_LPM_ModuleStart(LPM_MSTP_AGT0);


    system_clock_init( );
    system_gpio_init( );
    system_time_init( );
    system_lptim_init( );
    rtc_init( );
//    rtc_init( (void*)0);
	rtc_set_current_binary_time(10);
	rtc_counter_run(RTC_COUNTER_START);
	rtc_alarm_init();

	system_lptim_set_and_run(0xFFFFFFFF); // agtw0 32bit timer
	lr1110_hal_reset( radio );
    system_time_wait_ms( 1 );
}


void system_uart_init(UART_DRIVER_FIFO *p_uart_dev)
{
	int8_t result;


#if (BOARD_TYPE_EVK_TOKYOCOM  == 1)
	uart0_set_param(p_uart_dev);
#elif (BOARD_TYPE_EVK_TOKYOCOM  == 2)
	uart2_set_param(p_uart_dev);
#endif

//	result = uart_fifo_init(p_uart_dev, 38400);
	result = uart_fifo_init(p_uart_dev, 115200);
//	result = uart_fifo_init(p_uart_dev, 9600);
//	result = uart_fifo_init(p_uart_dev, 460800);
	APP_ERR_HANDLER(result);

    system_gpio_init_output( 2, 2, 1 );  // CTS
    system_gpio_init_input( 7, 4, SYSTEM_GPIO_NO_INTERRUPT ); // RTS
}

void system_uart_stop(UART_DRIVER_FIFO *p_uart_dev)
{
	int8_t result;
//	result = uart_fifo_uninit(p_uart_dev->sci_dev);
	result = uart_fifo_uninit(p_uart_dev);
	APP_ERR_HANDLER(result);
}


extern int8_t i2c_simple_channel;

void system_init_ISO2()
{
    system_spi_init(&Driver_SPI0);

#if (BOARD_TYPE_EVK_TOKYOCOM  == 1)
    system_i2c_init(&Driver_I2C0);
#elif (BOARD_TYPE_EVK_TOKYOCOM  == 2)
    I2C_Simple_Initialize(i2c_simple_channel, ARM_I2C_BUS_SPEED_STANDARD);
#endif


#if (TRACKER_RX_TX_UPDATE == 1 || TRACKER_RX_TX_UPDATE == 3)
    system_uart_init(&uart0_dev);
	uart_fifo_receive_start(&uart0_dev);
	uart_rx_fifo_flush(&uart0_dev);
#endif
}

void system_uninit_ISO2()
{
    system_spi_stop(&Driver_SPI0);

#if (BOARD_TYPE_EVK_TOKYOCOM  == 1)
    system_i2c_stop(&Driver_I2C0);
#elif (BOARD_TYPE_EVK_TOKYOCOM  == 2)
    I2C_Simple_Uninitialize(i2c_simple_channel);
#endif


#if (TRACKER_RX_TX_UPDATE == 1 || TRACKER_RX_TX_UPDATE == 3)
	system_uart_stop(&uart0_dev);
#endif
}

void rtc_alarm_init()
{
    uint32_t current_time;
	rtc_global_count=0;
    current_time = rtc_read_current_binary_time();
    rtc_set_alarm_binary_time (current_time);
    rtc_enable_alarms_binary (0x00000FFF); // RTC interrupt interval 4096sec
}



int	print_uart(const char * format, ...)
{

	char tbuf[256];
	va_list arg_ptr;
	int  len;
	va_start(arg_ptr,format);
	vsnprintf(tbuf,255,format,arg_ptr);
	va_end(arg_ptr);
	len = strlen(tbuf);
	uart_fifo_send_string(&uart0_dev, tbuf);
	return len;
}

