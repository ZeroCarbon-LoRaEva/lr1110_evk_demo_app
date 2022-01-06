/***********************************************************************************************************************
* System Name      : UART simple I2C mode sample software for low level code
* File Name        : main.c
* Version          : 1.03
* Device           : RE01 256KB
* Abstract         : Demo source file
* Tool-Chain       : IAR compiler v8.40.2 and GCC v6
* IDE              : IAR EWARM and Renesas e2 studio
* OS               : not use
* H/W Platform     : Evaluation Kit RE01 256KB
* Description      : [Terminal connection]
*                  :  P106(SSDA0) - EEPROM SSDA pin
*                  :  P105(SSCL0) - EEPROM SSCL pin
*                  : [Operation]
*                  : (1) When SW2 is pressed, the following operations are performed.
*                  :   (1)-1: If SCI is stopped, LED0 turn on and initialize SCI as follows.:
*                  :             SCI0: Simple I2C mode,  communication speed 400kbps
*                  :   (1)-2: Write 10 bytes data to EEPROM
*                  :   (1)-3: Read 10 bytes data from EEPROM
*                  :   (1)-4: LED1 turns on when the read data and written data match
*                  : (2) When SW1 is pressed, LED0 turn off and execute SCI0 termination processing
* Limitation       : none
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 26.12.2019 1.00     First Release
*         : 19.03.2020 1.02     Replaced with RE01 256KB Group CMSIS Driver Package Rev.0.80.
*         : 30.06.2020 1.03     Replaced with RE01 256KB Group CMSIS Driver Package Rev.1.00.
*                               Change the port used.
***********************************************************************************************************************/


#ifndef _SYSTEM_I2C_SIMPLE_H
#define _SYSTEM_I2C_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif



/***********************************************************************************************************************
Macros
***********************************************************************************************************************/



/*  Enum for  State */
typedef enum {
    I2C_STATE_UNINITIALIZE,                             /* I2C STATE: I2C uninitialize               */
    I2C_STATE_RDY,                                      /* I2C STATE: I2C ready                      */
    I2C_STATE_START,                                    /* I2C STATE: I2C generate start condition   */
    I2C_STATE_SND_DEVICE_W,                             /* I2C STATE: I2C send device address(W)     */
    I2C_STATE_SND_ADDR_H,                               /* I2C STATE: I2C send data address(H)       */
    I2C_STATE_SND_ADDR_L,                               /* I2C STATE: I2C send data address(L)       */
    I2C_STATE_SND_DATA,                                 /* I2C STATE: I2C send data                  */
    I2C_STATE_RESTART,                                  /* I2C STATE: I2C generate restart condition */
    I2C_STATE_SND_DEVICE_R,                             /* I2C STATE: I2C send device address(R)     */
    I2C_STATE_RCV_DATA,                                 /* I2C STATE: I2C receive data               */
    I2C_STATE_STOP,                                     /* I2C STATE: I2C generate stop condition    */
    I2C_STATE_ERROR                                     /* I2C STATE: I2C error occurred             */
} e_sci_state_t;

/***********************************************************************************************************************
Private global variables and functions
***********************************************************************************************************************/

// bus speed ARM_I2C_BUS_SPEED_STANDARD ..100kbps
//           ARM_I2C_BUS_SPEED_FAST  .. 400kbps
//addr .. 7bit address
// isc_channel 0, 2, or 4

int8_t I2C_Simple_Initialize(int8_t i2c_channel, int8_t bus_speed); 
int8_t I2C_Simple_Uninitialize(int8_t i2c_channel);                                            /* I2C close */
int8_t I2C_Simple_MasterTransmit(int8_t i2c_channel, uint8_t addr, uint8_t const * const p_data_out, uint32_t size);      /* I2C  write */
int8_t I2C_Simple_MasterReceive(int8_t i2c_channel, uint8_t  addr, uint8_t *p_data_in, uint32_t size);                     /* I2C read */
int8_t I2C_Simple_MasterTransmit_no_Wait(int8_t i2c_channel, uint8_t addr, uint8_t const * const p_data_out, uint32_t size);      /* I2C  write */
int8_t I2C_Simple_MasterReceive_no_Wait(int8_t i2c_channel, uint8_t  addr, uint8_t *p_data_in, uint32_t size);                     /* I2C read */
int8_t I2C_Simple_Wait_Ready(int8_t i2c_channel, int32_t timeout); // timeout(msec)


#ifdef __cplusplus
}
#endif

#endif


