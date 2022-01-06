/*!
* \file      i2c-board.c
*
* \brief     Target board I2C driver implementation
*
* \copyright Revised BSD License, see section \ref LICENSE.
*
* \code
*                ______                              _
*               / _____)             _              | |
*              ( (____  _____ ____ _| |_ _____  ____| |__
*               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
*               _____) ) ____| | | || |_| ____( (___| | | |
*              (______/|_____)_|_|_| \__)_____)\____)_| |_|
*              (C)2013-2017 Semtech
*
* \endcode
*
* \author    Miguel Luis ( Semtech )
*
* \author    Gregory Cristian ( Semtech )
*/
//#include "sensor_poc_config.h"
//#include "board-config.h"
//#include "RE01_1500KB.h"
//#include "utilities.h"
//#include "i2c-board.h"
#include "config_mode.h"
#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include "system.h"
#include "system_i2c.h"
#include "r_i2c_cmsis_api.h"
#include "r_lpm_api.h"

/*!
*  The value of the maximal timeout for I2C waiting loops
*/
#define TIMEOUT_MAX                                 0x8000

volatile static uint8_t i2c_eventWait;


extern ARM_DRIVER_I2C Driver_I2C0;
extern ARM_DRIVER_I2C Driver_I2C1;


void i2c0_callback(uint32_t event) __attribute__ ((section(".ramfunc")));
void i2c1_callback(uint32_t event) __attribute__ ((section(".ramfunc")));


void system_i2c_init( ARM_DRIVER_I2C *i2c_obj)
{
	  int err;
	if (i2c_obj == &Driver_I2C0) {
	    err=i2c_obj->Initialize(i2c0_callback); 
	} else if (i2c_obj == &Driver_I2C1){
	    err=i2c_obj->Initialize(i2c1_callback); 
	} else {
		err=1;
	}
  APP_ERR_HANDLER(err);

  err = i2c_obj->PowerControl(ARM_POWER_FULL); 
  APP_ERR_HANDLER(err);

  err = i2c_obj->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
//  err = i2c_obj->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  APP_ERR_HANDLER(err);


}

void system_i2c_stop( ARM_DRIVER_I2C* i2c_obj)
{
	  int err;
	if (i2c_obj == &Driver_I2C0) {
        err = i2c_obj->Uninitialize();
	    R_LPM_ModuleStop(LPM_MSTP_RIIC0);
	} else if (i2c_obj == &Driver_I2C1){
        err = i2c_obj->Uninitialize();
	    R_LPM_ModuleStop(LPM_MSTP_RIIC1);
	} else {
		err=1;
	}
    APP_ERR_HANDLER(err);
}


// Repeated .. True-> Don't Send Stop Condition(Exist next data)  , False-> Send Stop Condition(Final Data)
uint8_t system_i2c_write( ARM_DRIVER_I2C *i2c_obj,  uint8_t deviceAddr,  uint8_t *buffer, uint8_t size , const bool repeated )
{
volatile   uint8_t status = ARM_DRIVER_ERROR;
volatile   uint32_t timeout = 0x020000;
  
  i2c_eventWait = 0;
  
  status = i2c_obj->MasterTransmit(deviceAddr, buffer, size, repeated);
  
  while(i2c_eventWait == 0 && --timeout);
  
  return ((status == ARM_DRIVER_OK) && (i2c_eventWait != 0)) ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;
}


uint8_t system_i2c_read(ARM_DRIVER_I2C* i2c_obj,  uint8_t deviceAddr, uint8_t *buffer, uint8_t size , const bool repeated)
{
volatile   uint8_t status = ARM_DRIVER_ERROR;
volatile   uint32_t timeout = 0x020000;
  
  i2c_eventWait = 0;
  
  status = i2c_obj->MasterReceive(deviceAddr, buffer, size, repeated);
  
  while(i2c_eventWait == 0 && --timeout);
  
  return ((status == ARM_DRIVER_OK) && (i2c_eventWait != 0)) ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;
//  return ((status == ARM_DRIVER_OK) && (i2c_eventWait != 0)) ? SUCCESS : FAIL;
}


void i2c0_callback(uint32_t event)
{
  i2c_eventWait = (uint8_t)event;
}

void i2c1_callback(uint32_t event)
{
  i2c_eventWait = (uint8_t)event;
}
