/**
 * @file      system_spi.c
 *
 * @brief     MCU SPI-related functions
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

#include "config_mode.h"
#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include "system.h"
#include "system_time.h"
#include "system_spi.h"
#include "r_lpm_api.h"
#include "r_spi_cmsis_api.h"

//static void callback(uint32_t event);

extern ARM_DRIVER_SPI Driver_SPI0;
extern ARM_DRIVER_SPI Driver_SPI1;

void spi0_callback(uint32_t event)  __attribute__ ((section(".ramfunc")));
void spi1_callback(uint32_t event)  __attribute__ ((section(".ramfunc")));

static uint8_t spi_eventWait;



static int err;

//extern ARM_DRIVER_SPI Driver_SPI0;


void system_spi_init( ARM_DRIVER_SPI* spi)
{
	GLOBAL_INT_DISABLE( );
	if (spi == &Driver_SPI0) {
		  err = spi->Initialize(spi0_callback); 
	} else if (spi == &Driver_SPI1){
		  err = spi->Initialize(spi1_callback);
	} else {
		err=1;
	}

  APP_ERR_HANDLER(err);

  err = spi->PowerControl(ARM_POWER_FULL); 
  APP_ERR_HANDLER(err);

#if (TRACKER_RX_TX_UPDATE == 2)
  err = spi->Control(ARM_SPI_MODE_MASTER  | 
                                       ARM_SPI_DATA_BITS(8) | 
                                         ARM_SPI_SS_MASTER_UNUSED | 
                                           ARM_SPI_CPOL0_CPHA0  | 
                                           ARM_SPI_MSB_LSB,1000000);//  MSB first 1Mbps
#elif (TRACKER_RX_TX_UPDATE == 1 || TRACKER_RX_TX_UPDATE == 3)
  err = spi->Control(ARM_SPI_MODE_MASTER  |
                                       ARM_SPI_DATA_BITS(8) |
                                         ARM_SPI_SS_MASTER_UNUSED |
                                           ARM_SPI_CPOL0_CPHA0  |
                                           ARM_SPI_MSB_LSB,8000000);  // MSB first 8Mbps
#endif

  APP_ERR_HANDLER(err);
  
  GLOBAL_INT_RESTORE();
}

void system_spi_stop( ARM_DRIVER_SPI* spi)
{
	if (spi == &Driver_SPI0) {
        err = spi->Uninitialize();
	    R_LPM_ModuleStop(LPM_MSTP_SPI0);
	} else if (spi == &Driver_SPI1){
        err = spi->Uninitialize();
	    R_LPM_ModuleStop(LPM_MSTP_SPI1);
	} else {
		err=1;
	}
    APP_ERR_HANDLER(err);
}

void system_spi_write_read( ARM_DRIVER_SPI* spi, const uint8_t* cbuffer, uint8_t* rbuffer, uint16_t length )
//uint16_t hal_spi_in_out( const uint32_t id, const uint16_t outData )
{
  int err;
//  uint8_t rxData = 0;
  uint32_t timeout = 0x020000;
  
  if (length ==0) {return;}
  spi_eventWait = 0; 
  
  err = spi->Transfer(&cbuffer, &rbuffer, length);
  APP_ERR_HANDLER(err);
  
  while((spi_eventWait == 0) && (--timeout != 0));
  
  APP_ERR_HANDLER(spi_eventWait != ARM_SPI_EVENT_TRANSFER_COMPLETE);
  
//  system_time_wait_ms( 10 );
  return;
}

void system_spi_write( ARM_DRIVER_SPI* spi, const uint8_t* buffer, uint16_t length )
//uint16_t hal_spi_in_out( const uint32_t id, const uint16_t outData )
{
  int err;
//  uint8_t rxData = 0;
  uint32_t timeout = 0x02000000;
  
  if (length ==0) {return;}
  spi_eventWait = 0; 
  
  err = spi->Send(buffer, length);
  APP_ERR_HANDLER(err);
  
  while((spi_eventWait == 0) && (--timeout != 0));
  
  APP_ERR_HANDLER(spi_eventWait != ARM_SPI_EVENT_TRANSFER_COMPLETE);
//  system_time_wait_ms( 10 );
  return;
}

void system_spi_read( ARM_DRIVER_SPI* spi, uint8_t* buffer, uint16_t length )
//uint16_t hal_spi_in_out( const uint32_t id, const uint16_t outData )
{
  int err;
//  uint8_t rxData = 0;
  uint32_t timeout = 0x02000000;
  
  if (length ==0) {return;}
  spi_eventWait = 0; 
  
  err = spi->Receive(buffer, length);
  APP_ERR_HANDLER(err);
  
  while((spi_eventWait == 0) && (--timeout != 0));
  
  APP_ERR_HANDLER(spi_eventWait != ARM_SPI_EVENT_TRANSFER_COMPLETE);
  
//  system_time_wait_ms( 10 );
  return;
}

void spi0_callback(uint32_t event)
{
  spi_eventWait  = event;
}

void spi1_callback(uint32_t event)
{
  spi_eventWait  = event;
}



