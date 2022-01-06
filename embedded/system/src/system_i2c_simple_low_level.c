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

/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "RE01_256KB.h"
#include "r_core_cfg.h"
#include "r_system_api.h"
#include "r_lpm_api.h"
#include "R_Driver_I2C.h"
#include "pin.h"
#include "system_i2c_simple_low_level.h"





/***********************************************************************************************************************
***********************************************************************************************************************/

#define EEP_PRV_WRITE       (0x00)                      /* EEPROM Write */
#define EEP_PRV_READ        (0x01)                      /* EEPROM Read */
#define I2C_PRV_ACK         (0)                         /* EEPROM ACK signal */


#define SCI_USE_PCLKA (1)
#define SCI_USE_PCLKB (2)


#define FUNC_LOCATION_I2C_SIMPLE   __attribute__ ((section(".ramfunc"))) /* @suppress("Macro expansion") */
//#define FUNC_LOCATION_I2C_SIMPLE


/***********************************************************************************************************************
*
* I2C channel selector
*
***********************************************************************************************************************/



int8_t I2C0_Simple_Initialize(int8_t bus_speed); 
int8_t I2C0_Simple_Uninitialize(void);                                            /* I2C close */
int8_t I2C0_Simple_MasterTransmit_no_Wait(uint8_t addr, uint8_t const * const p_data_out, uint32_t size);      /* I2C  write */
int8_t I2C0_Simple_MasterReceive_no_Wait(uint8_t  addr, uint8_t *p_data_in, uint32_t size);                     /* I2C read */
int8_t I2C0_Simple_Wait_Ready(int32_t timeout); // timeout(msec)


int8_t I2C2_Simple_Initialize(int8_t bus_speed);
int8_t I2C2_Simple_Uninitialize(void);                                            /* I2C close */
int8_t I2C2_Simple_MasterTransmit_no_Wait(uint8_t addr, uint8_t const * const p_data_out, uint32_t size);      /* I2C  write */
int8_t I2C2_Simple_MasterReceive_no_Wait(uint8_t  addr, uint8_t *p_data_in, uint32_t size);                     /* I2C read */
int8_t I2C2_Simple_Wait_Ready(int32_t timeout); // timeout(msec)


int8_t I2C4_Simple_Initialize(int8_t bus_speed);
int8_t I2C4_Simple_Uninitialize(void);                                            /* I2C close */
int8_t I2C4_Simple_MasterTransmit_no_Wait(uint8_t addr, uint8_t const * const p_data_out, uint32_t size);      /* I2C  write */
int8_t I2C4_Simple_MasterReceive_no_Wait(uint8_t  addr, uint8_t *p_data_in, uint32_t size);                     /* I2C read */
int8_t I2C4_Simple_Wait_Ready(int32_t timeout); // timeout(msec)



FUNC_LOCATION_I2C_SIMPLE int8_t I2C_Simple_Initialize(int8_t i2c_channel, int8_t bus_speed)
{
	volatile int8_t result;

	switch(i2c_channel) {
		case 0:
			result = I2C0_Simple_Initialize(bus_speed);
			break;
		case 2:
			result = I2C2_Simple_Initialize(bus_speed);
			break;
		case 4:
			result = I2C4_Simple_Initialize(bus_speed);
			break;
		default:
			result = -1;
			break;
	}
	return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t   I2C_Simple_Uninitialize(int8_t i2c_channel)
{
	volatile int8_t result;
	switch(i2c_channel) {
		case 0:
			result = I2C0_Simple_Uninitialize();
			break;
		case 2:
			result = I2C2_Simple_Uninitialize();
			break;
		case 4:
			result = I2C4_Simple_Uninitialize();
			break;
		default:
			result = -1;
			break;
	}
	return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t   I2C_Simple_MasterTransmit(int8_t i2c_channel, uint8_t addr, uint8_t const * const p_data_out, uint32_t size)
{
	volatile int8_t result;
	volatile int32_t timeout;
	timeout=3000;  // Timeout 3sec
	switch(i2c_channel) {
		case 0:
			result = I2C0_Simple_MasterTransmit_no_Wait(addr, p_data_out, size);
			if (result) {goto Skip;}
			result = I2C0_Simple_Wait_Ready(timeout);
			break;
		case 2:
			result = I2C2_Simple_MasterTransmit_no_Wait(addr, p_data_out, size);
			if (result) {goto Skip;}
			result = I2C2_Simple_Wait_Ready(timeout);
			break;
		case 4:
			result = I2C4_Simple_MasterTransmit_no_Wait(addr, p_data_out, size);
			if (result) {goto Skip;}
			result = I2C4_Simple_Wait_Ready(timeout);
			break;
		default:
			result = -1;
			break;
	}
Skip:
	return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t   I2C_Simple_MasterReceive(int8_t i2c_channel, uint8_t  addr, uint8_t *p_data_in, uint32_t size)
{
	volatile int8_t result;
	volatile int32_t timeout;
	timeout=3000;  // Timeout 3sec
	switch(i2c_channel) {
		case 0:
			result = I2C0_Simple_MasterReceive_no_Wait(addr, p_data_in, size);
			if (result) {goto Skip;}
			result = I2C0_Simple_Wait_Ready(timeout);
			break;
		case 2:
			result = I2C2_Simple_MasterReceive_no_Wait(addr, p_data_in, size);
			if (result) {goto Skip;}
			result = I2C2_Simple_Wait_Ready(timeout);
			break;
		case 4:
			result = I2C4_Simple_MasterReceive_no_Wait(addr, p_data_in, size);
			if (result) {goto Skip;}
			result = I2C4_Simple_Wait_Ready(timeout);
			break;
		default:
			result = -1;
			break;
	}
Skip:
	return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t   I2C_Simple_MasterTransmit_no_Wait(int8_t i2c_channel, uint8_t addr, uint8_t const * const p_data_out, uint32_t size)
{
	volatile int8_t result;
	switch(i2c_channel) {
		case 0:
			result = I2C0_Simple_MasterTransmit_no_Wait(addr, p_data_out, size);
			break;
		case 2:
			result = I2C2_Simple_MasterTransmit_no_Wait(addr, p_data_out, size);
			break;
		case 4:
			result = I2C4_Simple_MasterTransmit_no_Wait(addr, p_data_out, size);
			break;
		default:
			result = -1;
			break;
	}
	return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t   I2C_Simple_MasterReceive_no_Wait(int8_t i2c_channel, uint8_t  addr, uint8_t *p_data_in, uint32_t size)
{
	volatile int8_t result;
	switch(i2c_channel) {
		case 0:
			result = I2C0_Simple_MasterReceive_no_Wait(addr, p_data_in, size);
			break;
		case 2:
			result = I2C2_Simple_MasterReceive_no_Wait(addr, p_data_in, size);
			break;
		case 4:
			result = I2C4_Simple_MasterReceive_no_Wait(addr, p_data_in, size);
			break;
		default:
			result = -1;
			break;
	}
	return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t I2C_Simple_Wait_Ready(int8_t i2c_channel, int32_t timeout)
{
	volatile int8_t result;
	switch(i2c_channel) {
		case 0:
			result = I2C0_Simple_Wait_Ready(timeout);
			break;
		case 2:
			result = I2C2_Simple_Wait_Ready(timeout);
			break;
		case 4:
			result = I2C4_Simple_Wait_Ready(timeout);
			break;
		default:
			result = -1;
			break;
	}
	return result;
}


/***********************************************************************************************************************
* Function Name: i2c_init
* Description  : SCI0 simple I2C mode initialization
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/

/***********************************************************************************************************************
*
*  I2C using SCI2
*
***********************************************************************************************************************/

void i2c0_simple_gen_stop_condition(void);                               /* I2C Start generation of stop condition */
void i2c0_simple_txi_callback(void);                                     /* I2C Transmit interrupt callback */
void i2c0_simple_rxi_callback(void);                                     /* I2C Receive interrupt callback */
void i2c0_simple_sti_callback(void);                                     /* I2C STI interrupt callback */
void i2c0_simple_eri_callback(void);                                     /* I2C ERI interrupt callback */
int8_t i2c_simple_baud_rate(int8_t clock_source, int8_t bus_speed, uint8_t *pBRR, uint8_t *pn);


static int8_t s_i2c0_simple_state;     /* I2C status */

static uint8_t const *sp_i2c0_simple_data_out;                                  /* I2C send data pointer */
static uint8_t *sp_i2c0_simple_data_in;                                         /* I2C receive data pointer */
static uint8_t s_i2c0_simple_address;                                           /* I2C EEPROM address  */
static uint8_t s_i2c0_simple_eep_mode;                                          /* I2C EEPROM Read/Write mode */
static uint8_t s_i2c0_simple_size;                                              /* I2C send/receive size  */
static uint8_t s_i2c0_simple_counter = 0;                                       /* I2C send/receive counter  */

static uint8_t i2c0_prv_device_addr;                       /* I2C Device address */


FUNC_LOCATION_I2C_SIMPLE int8_t i2c_simple_baud_rate(int8_t clock_source, int8_t bus_speed, uint8_t *pBRR, uint8_t *pn)
{
	int8_t result;
	static uint32_t pclka_freq;
	static uint32_t pclkb_freq;
	volatile uint32_t sci_freq;
	volatile uint8_t BRR;
	

	if (clock_source == SCI_USE_PCLKA) {
	    R_SYS_SystemClockFreqGet(&pclka_freq);
	    sci_freq = pclka_freq / 1000000;
	} else if (clock_source == SCI_USE_PCLKB) {
	    R_SYS_PeripheralClockFreqGet(&pclkb_freq); /* Store pclk frequency */ // @suppress("Cast comment")
	    sci_freq = pclkb_freq / 1000000;
	}

    if  ((sci_freq < 2 && bus_speed == ARM_I2C_BUS_SPEED_STANDARD) 
      || (sci_freq < 8 && bus_speed == ARM_I2C_BUS_SPEED_FAST)) {
		return -1;
	}

    if (sci_freq > 32) {
    	*pn = 0x01; //PCLK/4 (n = 1).
        if (bus_speed == ARM_I2C_BUS_SPEED_STANDARD) {
            BRR = ((sci_freq*5/64) & 0xFF);
        } else {
        	BRR = ((sci_freq*1/64) & 0xFF);
        }
    } else {
    	*pn = 0x00; //PCLK/1 (n = 0).
        if (bus_speed == ARM_I2C_BUS_SPEED_STANDARD) {
        	BRR = ((sci_freq*20/64) & 0xFF);
        } else {
        	BRR = ((sci_freq*5/64) & 0xFF);
        }
    }
    if (BRR>0) {
    	BRR = BRR -1;
    }
    *pBRR = BRR;
	return 0;
}


FUNC_LOCATION_I2C_SIMPLE int8_t I2C0_Simple_Wait_Ready(int32_t timeout)
{
	int32_t ic;
	int8_t result;
	result=0;
	timeout *=10;
	for (ic=0 ; ;ic++) {
		if (ic > timeout && timeout != 0) {
			result = -2; break;
		}
		if (I2C_STATE_ERROR == s_i2c0_simple_state) {
			result = -1; break;
		}
//		if (I2C_STATE_RDY == s_i2c0_simple_state) {
		if (1 == s_i2c0_simple_state) {
			result = 0; break;
		}
	   R_SYS_SoftwareDelay(100, SYSTEM_DELAY_UNITS_MICROSECONDS);
   }

	return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t I2C0_Simple_Initialize(int8_t bus_speed)
{
	int8_t result;
	static uint8_t BRR;
	static uint8_t n;

    /* Lock SCI0 resource */
    if (0 != R_SYS_ResourceLock(SYSTEM_LOCK_SCI0))
    {

        /* If the result of the resource lock is an error, do not initialize */
        while (1)
        {
            ;   /* Intentionally empty braces. */
        }
    }

    /* Start SCI0 module */
    R_LPM_ModuleStart(LPM_MSTP_SCI0);

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is disabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is disabled.
       b4    - RE   - Receive Enable - Serial transmission is disabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI0->SCR = 0x00;

    /* Set SSDA, SSCL pin */
    R_SCI_Pinset_CH0();

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
       b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state.
       b3    - IICSTIF    - Issuing of Start, Restart, or Stop Condition Completed Flag - 
                               No requests made for generating conditions, or a condition is being generated
       b2    - IICSTPREQ  - Stop Condition Generation - Do not generate stop condition.
       b1    - IICRSTAREQ - Restart Condition Generation - Do not generate restart condition.
       b0    - IICSTREQ   - Start Condition Generation - Do not generate start condition. */
    SCI0->SIMR3 = 0xF0;


    /* SCMR  - Smart Card Mode Register
       b7    - BCP2 - Base Clock Pulse 2.
       b6:b5 - Reserved - This bit is read as 1. The write value should be 1.
       b4    - CHR1 - Character Length 1 - Transmit/receive in 8-bit data length.
       b3    - SDIR - Transmitted/Received Data Transfer Direction - Transfer with MSB first.
       b2    - SINV - Transmitted/Received Data Invert - TDR contents are transmitted as they are. Receive data
                                                         is stored as it is in RDR.
       b1    - Reserved - This bit is read as 1. The write value should be 1.
       b0    - SMIF - Smart Card Interface Mode Select - Non-smart card interface mode. */
    SCI0->SCMR = 0xFA;

	result = i2c_simple_baud_rate(SCI_USE_PCLKA, bus_speed, &BRR, &n);
	if (result !=0) {return result;}
	
    /* SMR   - Serial Mode Register for Non-Smart Card Interface Mode - The set value depends on the variable.
       b7    - CM  - Communications Mode - Asynchronous mode or simple I2C mode
       b6    - CHR - Character Length - Transmit/receive in 8-bit data length.
       b5    - PE  - Parity Enable - Parity bit addition is not performed.
       b4    - PM  - Parity Mode - Selects even parity.
       b3    - STOP - Stop Bit Length - 1 stop bit.
       b2    - MP  - Multi-Processor Mode - Multi-processor communications function is disabled.
       b1:b0 - Clock Select - PCLK/4 (n = 1) PCLK/1 (n = 0) */
    SCI0->SMR = 0x00 | n; 

    /* BRR - Bit Rate Register 
       b7:b0 - Adjusts the bit rate(Set to 100kbps, Width at high=4.375us(min), Width at low=5.00us(min)).  */
//    SCI0->BRR = (5-1);
    SCI0->BRR = BRR;

    /* SEMR  - Serial Extended Mode Register
       b7    - RXDESEL - Asynchronous Start Bit Edge Detection Select - Valid only in asynchronous mode.
       b6    - BGDM - Baud Rate Generator Double-Speed Mode Select - Valid only in asynchronous mode.
       b5    - NFEN - Digital Noise Filter Function Enable - The noise cancellation function for the SSCLn and
                                                             SSDAn input signals is enabled.
       b4    - ABCS - Asynchronous Mode Base Clock Select - Valid only in asynchronous mode.
       b3    - ABCSE - Asynchronous Mode Extended Base Clock Select1 - Valid only in asynchronous mode.
       b2    - BRME - Bit Rate Modulation Enable - Bit rate modulation function is disabled.
       b1:b0 - Reserved - This bit is read as 0. The write value should be 0. */
    SCI0->SEMR = 0x20;

    /* SNFR  - Noise Filter Setting Register 
       b7:b3 - Reserved - This bit is read as 0. The write value should be 0.
       b2:b0 - NFCS - Noise Filter Clock Select - The clock signal divided by 4 is used with the noise filter */
    SCI0->SNFR = 0x03;

    /* SIMR1 - I2C Mode Register 1
       b7:b3 - IICDL - SSDAn Output Delay Select - The following cycles are of the clock signal from the on-chip
                                                   baud rate generator: No output delay
       b2:b1 - Reserved - This bit is read as 0. The write value should be 0.
       b0    - IICM - Simple I2C Mode Select - Simple I2C mode. */
    SCI0->SIMR1 = 0x01;

    /* SIMR2 - I2C Mode Register 2
       b7:b6 - Reserved - This bit is read as 0. The write value should be 0.
       b5    - IICACKT - ACK Transmission Data - Transmission of NACK and reception of ACK/NACK.
       b4:b2 - Reserved - This bit is read as 0. The write value should be 0.
       b1    - IICCSC - Clock Synchronization - Synchronize with the clock signal.
       b0    - IICINTM - I2C Interrupt Mode Select - Use reception and transmission interrupts. */
    SCI0->SIMR2 = 0x23;

    /* SPMR - SPI Mode Register
       b7   - CKPH - Clock Phase Select - Do not delay clock.
       b6   - CKPOL - Clock Polarity Select - Do not invert clock polarity.
       b5   - Reserved  - This bit is read as 0. The write value should be 0.
       b4   - MFF - Mode Fault Flag - No mode fault error.
       b3   - Reserved - This bit is read as 0. The write value should be 0.
       b2   - MSS - Master Slave Select - Transmission is through
                                          the TXDn pin and reception is through the RXDn pin (master mode).
       b1   - CTSE - CTS Enable - CTS function is disabled (RTS output function is enabled).
       b0   - SSE - SSn Pin Function Enable - SSn pin function is disabled. */
    SCI0->SPMR = 0x00;

    /* Set to SCI0 TXI irq event link and interrupt enable */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI, 0x10, i2c0_simple_txi_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI);

    /* Set to SCI0 STI(TEI) irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI, 0x0F, i2c0_simple_sti_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);

    /* Set to SCI0 RXI irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI, 0x10, i2c0_simple_rxi_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI);

    /* Set to SCI0 ERI irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI, 0x10, i2c0_simple_eri_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);

    s_i2c0_simple_state = I2C_STATE_RDY;

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is enabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is enabled.
       b4    - RE   - Receive Enable - Serial transmission is enabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is enabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI0->SCR = 0xB4;

    return 0;
}   /* End of function i2c_init() */

/***********************************************************************************************************************
* Function Name: i2c_close
* Description  : SCI0 simple I2C mode uninitialize
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C0_Simple_Uninitialize(void)
{
    volatile uint8_t dummy_read;

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is disabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is disabled.
       b4    - RE   - Receive Enable - Serial transmission is disabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI0->SCR = 0x00;

    /* Error flag reading and clear */
    dummy_read = SCI0->SSR;

    /* SSR - Serial Status Register for Non-Smart Card Interface Mode and non-FIFO Mode
       b7  - TDRE - Transmit Data Empty Flag - No transmission data in TDR register
       b6  - RDRF - Receive Data Full Flag - No received data in RDR register
       b5  - ORER - Overrun Error Flag - No overrun error occurred
       b4  - FER - Framing Error Flag - No framing error occurred
       b3  - PER - Parity Error Flag - No parity error occurred
       b2  - TEND - Transmit End Flag
       b1  - MPB - Multi-Processor
       b0  - MPBT - Multi-Processor Bit Transfer - Data transmission cycles */
    SCI0->SSR = 0x80;

    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);

    /* Clear TXI1 interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI);

    /* Clear RXI1 interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI);

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);

    /* Clear ERI interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);

    /* Release simple SPI pins */
    R_SCI_Pinclr_CH0();

    /* Stop SCI0 module */
    R_LPM_ModuleStop(LPM_MSTP_SCI0);

    /* Unlock SCI0 resource */
    R_SYS_ResourceUnlock(SYSTEM_LOCK_SCI0);

    s_i2c0_simple_state = I2C_STATE_UNINITIALIZE;

	return 0;
}   /* End of function i2c_close() */

/***********************************************************************************************************************
* Function Name: i2c_write
* Description  : Writing to EEPROM in simple I2C mode
* Arguments    : addr       : EEPROM write address
*                p_data_out : Pointer to write data
*                size       : Number of write data
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C0_Simple_MasterTransmit_no_Wait(uint8_t addr, uint8_t const * const p_data_out, uint32_t size)
{
    if ((I2C_STATE_RDY != s_i2c0_simple_state) && (I2C_STATE_ERROR != s_i2c0_simple_state))
    {
        return -1;
    }
    i2c0_prv_device_addr = addr;

    sp_i2c0_simple_data_out = p_data_out;                   /* Set EEPROM Write data pointer        */
    s_i2c0_simple_size  = size;                             /* Set EEPROM Write size                */
//    s_i2c0_simple_address = addr;                           /* Set EEPROM Write address             */
    s_i2c0_simple_eep_mode = EEP_PRV_WRITE;                 /* EEPROM Write mode                    */
    s_i2c0_simple_counter = 0;                              /* Clear I2C counter                    */

    s_i2c0_simple_state = I2C_STATE_START;                  /* Set generate start condition state   */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b0    - IICSTREQ   - Start Condition Generation - Generate start condition. */
    SCI0->SIMR3 = 0x51;

	return 0;

}   /* End of function i2c_write() */

/***********************************************************************************************************************
* Function Name: i2c_read
* Description  : Read to EEPROM in simple I2C mode
* Arguments    : addr       : EEPROM read address
*                p_data_in  : Pointer to read data
*                size       : Number of read data
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C0_Simple_MasterReceive_no_Wait(uint8_t addr, uint8_t *p_data_in, uint32_t size)
{
    if ((I2C_STATE_RDY != s_i2c0_simple_state) && (I2C_STATE_ERROR != s_i2c0_simple_state))
    {
        return -1;
    }

    i2c0_prv_device_addr = addr;
    sp_i2c0_simple_data_in = p_data_in;                     /* Set EEPROM Read data pointer         */
    s_i2c0_simple_size  = size;                             /* Set EEPROM Read size                 */
//    s_i2c0_simple_address = addr;                           /* Set EEPROM Read address              */
    s_i2c0_simple_eep_mode = EEP_PRV_READ;                  /* EEPROM Read mode                     */
    s_i2c0_simple_counter = 0;                              /* Clear I2C counter                    */

//    s_i2c0_simple_state = I2C_STATE_START;                  /* Set generate start condition state   */
    s_i2c0_simple_state = I2C_STATE_RESTART;                  /* Set generate start condition state   */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b0    - IICSTREQ   - Start Condition Generation - Generate start condition. */
    SCI0->SIMR3 = 0x51;

	return 0;
}   /* End of function i2c_read() */

/***********************************************************************************************************************
* Function Name: i2c_gen_stop_condition
* Description  : Start generation of stop condition
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c0_simple_gen_stop_condition(void)
{

    s_i2c0_simple_state = I2C_STATE_STOP;                   /* Set generate stop condition state    */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b2    - IICSTPREQ  - Stop Condition Generation - Do not generate stop condition.  */
    SCI0->SIMR3 = 0x54;

}   /* End of function i2c_gen_stop_condition() */

/***********************************************************************************************************************
* Function Name: i2c_txi_callback
* Description  : SCI0 txi interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c0_simple_txi_callback(void)
{

    /* Check I2C state */
    switch (s_i2c0_simple_state)
    {
        case I2C_STATE_SND_DEVICE_W:

            /* STATE: I2C send device address(W)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI0->SISR_b.IICACKR)
            {

                /* Transition to upper address output state when ACK is received */
                s_i2c0_simple_state = I2C_STATE_SND_ADDR_H;
                SCI0->TDR = (uint8_t)(s_i2c0_simple_address >> 8);
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c0_simple_gen_stop_condition();
            }
            break;

        case I2C_STATE_SND_DEVICE_R:

            /* STATE: I2C send device address(R)  */
            /* Transition to data reception state */
            if (I2C_PRV_ACK != SCI0->SISR_b.IICACKR)
            {

                /* Generate stop condition when receiving NACK */
                i2c0_simple_gen_stop_condition();
                break;
            }

            s_i2c0_simple_state = I2C_STATE_RCV_DATA;

            /* SIMR2 - I2C Mode Register 2
               b5    - IICACKT - ACK Transmission Data - ACK transmission.  */
            SCI0->SIMR2_b.IICACKT = 0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are enabled.  */
            SCI0->SCR_b.RIE = 1;

        /* Perform the following processing  */
        case I2C_STATE_RCV_DATA:

            /* STATE: I2C receive data  */
            /* Check receive counter */
            if ((s_i2c0_simple_counter + 1) == s_i2c0_simple_size)
            {

                /* When the next data is the final data */
                /* SIMR2 - I2C Mode Register 2
                   b5    - IICACKT - ACK Transmission Data - ACK transmission.  */
                SCI0->SIMR2_b.IICACKT = 1;
                SCI0->TDR = 0xFF;
                s_i2c0_simple_counter ++;
            }
            else if (s_i2c0_simple_counter >= s_i2c0_simple_size)
            {

                /* When final data is completed */
                i2c0_simple_gen_stop_condition();

            }
            else
            {
                SCI0->TDR = 0xFF;
                s_i2c0_simple_counter ++;
            }

            break;

        case I2C_STATE_SND_ADDR_H:

            /* STATE: I2C send data address(H)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI0->SISR_b.IICACKR)
            {

                /* Transition to lower address output state when ACK is received */
                s_i2c0_simple_state = I2C_STATE_SND_ADDR_L;
                SCI0->TDR = (uint8_t)(s_i2c0_simple_address & 0xFF);
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c0_simple_gen_stop_condition();
            }

            break;

        case I2C_STATE_SND_ADDR_L:

            /* STATE: I2C send data address(L)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI0->SISR_b.IICACKR)
            {

                /* Check EEPROM Read/Write mode */
                if (EEP_PRV_WRITE == s_i2c0_simple_eep_mode)
                {

                    /* In write mode, transition to data transmission state */
                    s_i2c0_simple_state = I2C_STATE_SND_DATA;
                    SCI0->TDR = sp_i2c0_simple_data_out[s_i2c0_simple_counter];
                    s_i2c0_simple_counter ++;
                }
                else
                {

                    /* In read mode, transition to the restart condition output state */
                    s_i2c0_simple_state = I2C_STATE_RESTART;

                    /* SIMR3   - I2C Mode Register 3
                       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
                       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
                       b1    - IICRSTAREQ - Restart Condition Generation - Generate restart condition. */
                    SCI0->SIMR3 = 0x52;
                }
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c0_simple_gen_stop_condition();
            }

            break;

        case I2C_STATE_SND_DATA:

            /* STATE: I2C send data  */
            /* Check ACK reception/final data output */
            if ((I2C_PRV_ACK == SCI0->SISR_b.IICACKR) && (s_i2c0_simple_size > s_i2c0_simple_counter))
            {

                /* Next data write */
                SCI0->TDR = sp_i2c0_simple_data_out[s_i2c0_simple_counter];
                s_i2c0_simple_counter ++;
            }
            else
            {

                /* Generate stop condition when receiving NACK or final data output */
                i2c0_simple_gen_stop_condition();
            }
            break;

        default:

            /* In case of an invalid state, a stop condition is output. */
            i2c0_simple_gen_stop_condition();
            break;
    }
}   /* End of function i2c_txi_callback() */

/***********************************************************************************************************************
* Function Name: i2c_rxi_callback
* Description  : SCI0 rxi interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c0_simple_rxi_callback(void)
{

    *sp_i2c0_simple_data_in++ = SCI0->RDR;

    }   /* End of function i2c_rxi_callback() */

/***********************************************************************************************************************
* Function Name: i2c_sti_callback
* Description  : SCI0 sti interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c0_simple_sti_callback(void)
{

    /* SIMR3   - I2C Mode Register 3
       b3    - IICSTIF    - Issuing of Start, Restart, or Stop Condition Completed Flag - 
                               No requests made for generating conditions, or a condition is being generated */
    SCI0->SIMR3_b.IICSTIF=0;

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);

    switch (s_i2c0_simple_state)
    {
        case I2C_STATE_START:

            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Serial clock output.
               b5:b4 - IICSDAS    - SSDAn Output Select - Serial data output.  */
            SCI0->SIMR3 = 0x00;

            /* STATE: I2C generate start condition  */
//            s_i2c0_simple_state = I2C_STATE_SND_DEVICE_W;
            s_i2c0_simple_state = I2C_STATE_SND_DATA;
  
            SCI0->TDR = ((i2c0_prv_device_addr<<1) | EEP_PRV_WRITE);
            break;

        case I2C_STATE_RESTART:

            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Serial clock output.
               b5:b4 - IICSDAS    - SSDAn Output Select - Serial data output.  */
            SCI0->SIMR3 = 0x00;

            /* STATE: I2C generate restart condition  */
            s_i2c0_simple_state = I2C_STATE_SND_DEVICE_R;
            SCI0->TDR = ((i2c0_prv_device_addr<<1) | EEP_PRV_READ);
            break;

        case I2C_STATE_STOP:

            /* STATE: I2C generate stop condition  */
            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
               b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state. */
            SCI0->SIMR3 = 0xF0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.  */
            SCI0->SCR_b.RIE = 0;

            /* Check if all transmission / reception is completed */
            if (s_i2c0_simple_size == s_i2c0_simple_counter)
            {
                s_i2c0_simple_state = I2C_STATE_RDY;
            }
            else
            {
                s_i2c0_simple_state = I2C_STATE_ERROR;
            }
            break;

        default:

            /* In case of an invalid state, i2c stop. */
            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
               b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state. */
            SCI0->SIMR3 = 0xF0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.  */
            SCI0->SCR_b.RIE = 0;

            s_i2c0_simple_state = I2C_STATE_ERROR;
            break;
    }

}   /* End of function i2c_sti_callback() */

FUNC_LOCATION_I2C_SIMPLE void i2c0_simple_eri_callback(void)
{

}




/***********************************************************************************************************************
*
*  I2C using SCI2
*
***********************************************************************************************************************/

static int8_t s_i2c2_simple_state;     /* I2C status */

static uint8_t const *sp_i2c2_simple_data_out;                                  /* I2C send data pointer */
static uint8_t *sp_i2c2_simple_data_in;                                         /* I2C receive data pointer */
static uint8_t s_i2c2_simple_address;                                           /* I2C EEPROM address  */
static uint8_t s_i2c2_simple_eep_mode;                                          /* I2C EEPROM Read/Write mode */
static uint8_t s_i2c2_simple_size;                                              /* I2C send/receive size  */
static uint8_t s_i2c2_simple_counter = 0;                                       /* I2C send/receive counter  */

static uint8_t i2c2_prv_device_addr;                       /* I2C Device address */


/***********************************************************************************************************************
***********************************************************************************************************************/

void i2c2_simple_gen_stop_condition(void);                               /* I2C Start generation of stop condition */
void i2c2_simple_txi_callback(void);                                     /* I2C Transmit interrupt callback */
void i2c2_simple_rxi_callback(void);                                     /* I2C Receive interrupt callback */
void i2c2_simple_sti_callback(void);                                     /* I2C STI interrupt callback */
void i2c2_simple_eri_callback(void);                                     /* I2C ERI interrupt callback */


/***********************************************************************************************************************
* Function Name: i2c_init
* Description  : SCI2 simple I2C mode initialization
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/

FUNC_LOCATION_I2C_SIMPLE int8_t I2C2_Simple_Wait_Ready(int32_t timeout)
{
    int32_t ic;
    int8_t result;
    result=0;
    timeout *=10;
    for (ic=0 ; ;ic++) {
        if (ic > timeout && timeout != 0) {
            result = -2; break;
        }
        if (I2C_STATE_ERROR == s_i2c2_simple_state) {
            result = -1; break;
        }
//      if (I2C_STATE_RDY == s_i2c2_simple_state) {
        if (1 == s_i2c2_simple_state) {
            result = 0; break;
        }
       R_SYS_SoftwareDelay(100, SYSTEM_DELAY_UNITS_MICROSECONDS);
   }

    return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t I2C2_Simple_Initialize(int8_t bus_speed)
{
    int8_t result;
    static uint8_t BRR;
    static uint8_t n;

    /* Lock SCI2 resource */
    if (0 != R_SYS_ResourceLock(SYSTEM_LOCK_SCI2))
    {

        /* If the result of the resource lock is an error, do not initialize */
        while (1)
        {
            ;   /* Intentionally empty braces. */
        }
    }

    /* Start SCI2 module */
    R_LPM_ModuleStart(LPM_MSTP_SCI2);

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is disabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is disabled.
       b4    - RE   - Receive Enable - Serial transmission is disabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI2->SCR = 0x00;

    /* Set SSDA, SSCL pin */
    R_SCI_Pinset_CH2();

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
       b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state.
       b3    - IICSTIF    - Issuing of Start, Restart, or Stop Condition Completed Flag - 
                               No requests made for generating conditions, or a condition is being generated
       b2    - IICSTPREQ  - Stop Condition Generation - Do not generate stop condition.
       b1    - IICRSTAREQ - Restart Condition Generation - Do not generate restart condition.
       b0    - IICSTREQ   - Start Condition Generation - Do not generate start condition. */
    SCI2->SIMR3 = 0xF0;


    /* SCMR  - Smart Card Mode Register
       b7    - BCP2 - Base Clock Pulse 2.
       b6:b5 - Reserved - This bit is read as 1. The write value should be 1.
       b4    - CHR1 - Character Length 1 - Transmit/receive in 8-bit data length.
       b3    - SDIR - Transmitted/Received Data Transfer Direction - Transfer with MSB first.
       b2    - SINV - Transmitted/Received Data Invert - TDR contents are transmitted as they are. Receive data
                                                         is stored as it is in RDR.
       b1    - Reserved - This bit is read as 1. The write value should be 1.
       b0    - SMIF - Smart Card Interface Mode Select - Non-smart card interface mode. */
    SCI2->SCMR = 0xFA;

    result = i2c_simple_baud_rate(SCI_USE_PCLKB, bus_speed, &BRR, &n);
    if (result !=0) {return result;}
    
    /* SMR   - Serial Mode Register for Non-Smart Card Interface Mode - The set value depends on the variable.
       b7    - CM  - Communications Mode - Asynchronous mode or simple I2C mode
       b6    - CHR - Character Length - Transmit/receive in 8-bit data length.
       b5    - PE  - Parity Enable - Parity bit addition is not performed.
       b4    - PM  - Parity Mode - Selects even parity.
       b3    - STOP - Stop Bit Length - 1 stop bit.
       b2    - MP  - Multi-Processor Mode - Multi-processor communications function is disabled.
       b1:b0 - Clock Select - PCLK/4 (n = 1) PCLK/1 (n = 0) */
    SCI2->SMR = 0x00 | n; 

    /* BRR - Bit Rate Register 
       b7:b0 - Adjusts the bit rate(Set to 100kbps, Width at high=4.375us(min), Width at low=5.00us(min)).  */
//    SCI2->BRR = (5-1);
    SCI2->BRR = BRR;

    /* SEMR  - Serial Extended Mode Register
       b7    - RXDESEL - Asynchronous Start Bit Edge Detection Select - Valid only in asynchronous mode.
       b6    - BGDM - Baud Rate Generator Double-Speed Mode Select - Valid only in asynchronous mode.
       b5    - NFEN - Digital Noise Filter Function Enable - The noise cancellation function for the SSCLn and
                                                             SSDAn input signals is enabled.
       b4    - ABCS - Asynchronous Mode Base Clock Select - Valid only in asynchronous mode.
       b3    - ABCSE - Asynchronous Mode Extended Base Clock Select1 - Valid only in asynchronous mode.
       b2    - BRME - Bit Rate Modulation Enable - Bit rate modulation function is disabled.
       b1:b0 - Reserved - This bit is read as 0. The write value should be 0. */
    SCI2->SEMR = 0x20;

    /* SNFR  - Noise Filter Setting Register 
       b7:b3 - Reserved - This bit is read as 0. The write value should be 0.
       b2:b0 - NFCS - Noise Filter Clock Select - The clock signal divided by 4 is used with the noise filter */
    SCI2->SNFR = 0x03;

    /* SIMR1 - I2C Mode Register 1
       b7:b3 - IICDL - SSDAn Output Delay Select - The following cycles are of the clock signal from the on-chip
                                                   baud rate generator: No output delay
       b2:b1 - Reserved - This bit is read as 0. The write value should be 0.
       b0    - IICM - Simple I2C Mode Select - Simple I2C mode. */
    SCI2->SIMR1 = 0x01;

    /* SIMR2 - I2C Mode Register 2
       b7:b6 - Reserved - This bit is read as 0. The write value should be 0.
       b5    - IICACKT - ACK Transmission Data - Transmission of NACK and reception of ACK/NACK.
       b4:b2 - Reserved - This bit is read as 0. The write value should be 0.
       b1    - IICCSC - Clock Synchronization - Synchronize with the clock signal.
       b0    - IICINTM - I2C Interrupt Mode Select - Use reception and transmission interrupts. */
    SCI2->SIMR2 = 0x23;

    /* SPMR - SPI Mode Register
       b7   - CKPH - Clock Phase Select - Do not delay clock.
       b6   - CKPOL - Clock Polarity Select - Do not invert clock polarity.
       b5   - Reserved  - This bit is read as 0. The write value should be 0.
       b4   - MFF - Mode Fault Flag - No mode fault error.
       b3   - Reserved - This bit is read as 0. The write value should be 0.
       b2   - MSS - Master Slave Select - Transmission is through
                                          the TXDn pin and reception is through the RXDn pin (master mode).
       b1   - CTSE - CTS Enable - CTS function is disabled (RTS output function is enabled).
       b0   - SSE - SSn Pin Function Enable - SSn pin function is disabled. */
    SCI2->SPMR = 0x00;

    /* Set to SCI2 TXI irq event link and interrupt enable */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI, 0x1A, i2c2_simple_txi_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI);

    /* Set to SCI2 STI(TEI) irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI, 0x19, i2c2_simple_sti_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);

    /* Set to SCI2 RXI irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI, 0x1A, i2c2_simple_rxi_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI);

    /* Set to SCI2 RXI irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI, 0x19, i2c2_simple_eri_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);

    s_i2c2_simple_state = I2C_STATE_RDY;

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is enabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is enabled.
       b4    - RE   - Receive Enable - Serial transmission is enabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is enabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI2->SCR = 0xB4;

    return 0;
}   /* End of function i2c_init() */

/***********************************************************************************************************************
* Function Name: i2c_close
* Description  : SCI2 simple I2C mode uninitialize
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C2_Simple_Uninitialize(void)
{
    volatile uint8_t dummy_read;

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is disabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is disabled.
       b4    - RE   - Receive Enable - Serial transmission is disabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI2->SCR = 0x00;

    /* Error flag reading and clear */
    dummy_read = SCI2->SSR;

    /* SSR - Serial Status Register for Non-Smart Card Interface Mode and non-FIFO Mode
       b7  - TDRE - Transmit Data Empty Flag - No transmission data in TDR register
       b6  - RDRF - Receive Data Full Flag - No received data in RDR register
       b5  - ORER - Overrun Error Flag - No overrun error occurred
       b4  - FER - Framing Error Flag - No framing error occurred
       b3  - PER - Parity Error Flag - No parity error occurred
       b2  - TEND - Transmit End Flag
       b1  - MPB - Multi-Processor
       b0  - MPBT - Multi-Processor Bit Transfer - Data transmission cycles */
    SCI2->SSR = 0x80;

    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);

    /* Clear TXI1 interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI);

    /* Clear RXI1 interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI);

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);

    /* Release simple SPI pins */
    R_SCI_Pinclr_CH2();

    /* Stop SCI2 module */
    R_LPM_ModuleStop(LPM_MSTP_SCI2);

    /* Unlock SCI2 resource */
    R_SYS_ResourceUnlock(SYSTEM_LOCK_SCI2);

    s_i2c2_simple_state = I2C_STATE_UNINITIALIZE;

	return 0;

}   /* End of function i2c_close() */

/***********************************************************************************************************************
* Function Name: i2c_write
* Description  : Writing to EEPROM in simple I2C mode
* Arguments    : addr       : EEPROM write address
*                p_data_out : Pointer to write data
*                size       : Number of write data
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C2_Simple_MasterTransmit_no_Wait(uint8_t addr, uint8_t const * const p_data_out, uint32_t size)
{
    if ((I2C_STATE_RDY != s_i2c2_simple_state) && (I2C_STATE_ERROR != s_i2c2_simple_state))
    {
        return -1;
    }
    i2c2_prv_device_addr = addr;

    sp_i2c2_simple_data_out = p_data_out;                   /* Set EEPROM Write data pointer        */
    s_i2c2_simple_size  = size;                             /* Set EEPROM Write size                */
//    s_i2c2_simple_address = addr;                           /* Set EEPROM Write address             */
    s_i2c2_simple_eep_mode = EEP_PRV_WRITE;                 /* EEPROM Write mode                    */
    s_i2c2_simple_counter = 0;                              /* Clear I2C counter                    */

    s_i2c2_simple_state = I2C_STATE_START;                  /* Set generate start condition state   */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b0    - IICSTREQ   - Start Condition Generation - Generate start condition. */
    SCI2->SIMR3 = 0x51;

	return 0;
}   /* End of function i2c_write() */

/***********************************************************************************************************************
* Function Name: i2c_read
* Description  : Read to EEPROM in simple I2C mode
* Arguments    : addr       : EEPROM read address
*                p_data_in  : Pointer to read data
*                size       : Number of read data
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C2_Simple_MasterReceive_no_Wait(uint8_t addr, uint8_t *p_data_in, uint32_t size)
{
    if ((I2C_STATE_RDY != s_i2c2_simple_state) && (I2C_STATE_ERROR != s_i2c2_simple_state))
    {
        return -1;
    }

    i2c2_prv_device_addr = addr;
    sp_i2c2_simple_data_in = p_data_in;                     /* Set EEPROM Read data pointer         */
    s_i2c2_simple_size  = size;                             /* Set EEPROM Read size                 */
//    s_i2c2_simple_address = addr;                           /* Set EEPROM Read address              */
    s_i2c2_simple_eep_mode = EEP_PRV_READ;                  /* EEPROM Read mode                     */
    s_i2c2_simple_counter = 0;                              /* Clear I2C counter                    */

//    s_i2c2_simple_state = I2C_STATE_START;                  /* Set generate start condition state   */
    s_i2c2_simple_state = I2C_STATE_RESTART;                  /* Set generate start condition state   */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b0    - IICSTREQ   - Start Condition Generation - Generate start condition. */
    SCI2->SIMR3 = 0x51;

	return 0;
}   /* End of function i2c_read() */

/***********************************************************************************************************************
* Function Name: i2c_gen_stop_condition
* Description  : Start generation of stop condition
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c2_simple_gen_stop_condition(void)
{

    s_i2c2_simple_state = I2C_STATE_STOP;                   /* Set generate stop condition state    */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b2    - IICSTPREQ  - Stop Condition Generation - Do not generate stop condition.  */
    SCI2->SIMR3 = 0x54;

}   /* End of function i2c_gen_stop_condition() */

/***********************************************************************************************************************
* Function Name: i2c_txi_callback
* Description  : SCI2 txi interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c2_simple_txi_callback(void)
{

    /* Check I2C state */
    switch (s_i2c2_simple_state)
    {
        case I2C_STATE_SND_DEVICE_W:

            /* STATE: I2C send device address(W)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI2->SISR_b.IICACKR)
            {

                /* Transition to upper address output state when ACK is received */
                s_i2c2_simple_state = I2C_STATE_SND_ADDR_H;
                SCI2->TDR = (uint8_t)(s_i2c2_simple_address >> 8);
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c2_simple_gen_stop_condition();
            }
            break;

        case I2C_STATE_SND_DEVICE_R:

            /* STATE: I2C send device address(R)  */
            /* Transition to data reception state */
            if (I2C_PRV_ACK != SCI2->SISR_b.IICACKR)
            {

                /* Generate stop condition when receiving NACK */
                i2c2_simple_gen_stop_condition();
                break;
            }

            s_i2c2_simple_state = I2C_STATE_RCV_DATA;

            /* SIMR2 - I2C Mode Register 2
               b5    - IICACKT - ACK Transmission Data - ACK transmission.  */
            SCI2->SIMR2_b.IICACKT = 0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are enabled.  */
            SCI2->SCR_b.RIE = 1;

        /* Perform the following processing  */
        case I2C_STATE_RCV_DATA:

            /* STATE: I2C receive data  */
            /* Check receive counter */
            if ((s_i2c2_simple_counter + 1) == s_i2c2_simple_size)
            {

                /* When the next data is the final data */
                /* SIMR2 - I2C Mode Register 2
                   b5    - IICACKT - ACK Transmission Data - ACK transmission.  */
                SCI2->SIMR2_b.IICACKT = 1;
                SCI2->TDR = 0xFF;
                s_i2c2_simple_counter ++;
            }
            else if (s_i2c2_simple_counter >= s_i2c2_simple_size)
            {

                /* When final data is completed */
                i2c2_simple_gen_stop_condition();

            }
            else
            {
                SCI2->TDR = 0xFF;
                s_i2c2_simple_counter ++;
            }

            break;

        case I2C_STATE_SND_ADDR_H:

            /* STATE: I2C send data address(H)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI2->SISR_b.IICACKR)
            {

                /* Transition to lower address output state when ACK is received */
                s_i2c2_simple_state = I2C_STATE_SND_ADDR_L;
                SCI2->TDR = (uint8_t)(s_i2c2_simple_address & 0xFF);
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c2_simple_gen_stop_condition();
            }

            break;

        case I2C_STATE_SND_ADDR_L:

            /* STATE: I2C send data address(L)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI2->SISR_b.IICACKR)
            {

                /* Check EEPROM Read/Write mode */
                if (EEP_PRV_WRITE == s_i2c2_simple_eep_mode)
                {

                    /* In write mode, transition to data transmission state */
                    s_i2c2_simple_state = I2C_STATE_SND_DATA;
                    SCI2->TDR = sp_i2c2_simple_data_out[s_i2c2_simple_counter];
                    s_i2c2_simple_counter ++;
                }
                else
                {

                    /* In read mode, transition to the restart condition output state */
                    s_i2c2_simple_state = I2C_STATE_RESTART;

                    /* SIMR3   - I2C Mode Register 3
                       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
                       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
                       b1    - IICRSTAREQ - Restart Condition Generation - Generate restart condition. */
                    SCI2->SIMR3 = 0x52;
                }
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c2_simple_gen_stop_condition();
            }

            break;

        case I2C_STATE_SND_DATA:

            /* STATE: I2C send data  */
            /* Check ACK reception/final data output */
            if ((I2C_PRV_ACK == SCI2->SISR_b.IICACKR) && (s_i2c2_simple_size > s_i2c2_simple_counter))
            {

                /* Next data write */
                SCI2->TDR = sp_i2c2_simple_data_out[s_i2c2_simple_counter];
                s_i2c2_simple_counter ++;
            }
            else
            {

                /* Generate stop condition when receiving NACK or final data output */
                i2c2_simple_gen_stop_condition();
            }
            break;

        default:

            /* In case of an invalid state, a stop condition is output. */
            i2c2_simple_gen_stop_condition();
            break;
    }
}   /* End of function i2c_txi_callback() */

/***********************************************************************************************************************
* Function Name: i2c_rxi_callback
* Description  : SCI2 rxi interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c2_simple_rxi_callback(void)
{

    *sp_i2c2_simple_data_in++ = SCI2->RDR;

    }   /* End of function i2c_rxi_callback() */

/***********************************************************************************************************************
* Function Name: i2c_sti_callback
* Description  : SCI2 sti interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c2_simple_sti_callback(void)
{

    /* SIMR3   - I2C Mode Register 3
       b3    - IICSTIF    - Issuing of Start, Restart, or Stop Condition Completed Flag - 
                               No requests made for generating conditions, or a condition is being generated */
    SCI2->SIMR3_b.IICSTIF=0;

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);

    switch (s_i2c2_simple_state)
    {
        case I2C_STATE_START:

            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Serial clock output.
               b5:b4 - IICSDAS    - SSDAn Output Select - Serial data output.  */
            SCI2->SIMR3 = 0x00;

            /* STATE: I2C generate start condition  */
//            s_i2c2_simple_state = I2C_STATE_SND_DEVICE_W;
            s_i2c2_simple_state = I2C_STATE_SND_DATA;
  
            SCI2->TDR = ((i2c2_prv_device_addr<<1) | EEP_PRV_WRITE);
            break;

        case I2C_STATE_RESTART:

            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Serial clock output.
               b5:b4 - IICSDAS    - SSDAn Output Select - Serial data output.  */
            SCI2->SIMR3 = 0x00;

            /* STATE: I2C generate restart condition  */
            s_i2c2_simple_state = I2C_STATE_SND_DEVICE_R;
            SCI2->TDR = ((i2c2_prv_device_addr<<1) | EEP_PRV_READ);
            break;

        case I2C_STATE_STOP:

            /* STATE: I2C generate stop condition  */
            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
               b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state. */
            SCI2->SIMR3 = 0xF0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.  */
            SCI2->SCR_b.RIE = 0;

            /* Check if all transmission / reception is completed */
            if (s_i2c2_simple_size == s_i2c2_simple_counter)
            {
                s_i2c2_simple_state = I2C_STATE_RDY;
            }
            else
            {
                s_i2c2_simple_state = I2C_STATE_ERROR;
            }
            break;

        default:

            /* In case of an invalid state, i2c stop. */
            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
               b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state. */
            SCI2->SIMR3 = 0xF0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.  */
            SCI2->SCR_b.RIE = 0;

            s_i2c2_simple_state = I2C_STATE_ERROR;
            break;
    }

}   /* End of function i2c_sti_callback() */

FUNC_LOCATION_I2C_SIMPLE void i2c2_simple_eri_callback(void)
{

}



/***********************************************************************************************************************
*
*  I2C using SCI4
*
***********************************************************************************************************************/

static int8_t s_i2c4_simple_state;     /* I2C status */

static uint8_t const *sp_i2c4_simple_data_out;                                  /* I2C send data pointer */
static uint8_t *sp_i2c4_simple_data_in;                                         /* I2C receive data pointer */
static uint8_t s_i2c4_simple_address;                                           /* I2C EEPROM address  */
static uint8_t s_i2c4_simple_eep_mode;                                          /* I2C EEPROM Read/Write mode */
static uint8_t s_i2c4_simple_size;                                              /* I2C send/receive size  */
static uint8_t s_i2c4_simple_counter = 0;                                       /* I2C send/receive counter  */

static uint8_t i2c4_prv_device_addr;                       /* I2C Device address */


/***********************************************************************************************************************
***********************************************************************************************************************/

void i2c4_simple_gen_stop_condition(void);                               /* I2C Start generation of stop condition */
void i2c4_simple_txi_callback(void);                                     /* I2C Transmit interrupt callback */
void i2c4_simple_rxi_callback(void);                                     /* I2C Receive interrupt callback */
void i2c4_simple_sti_callback(void);                                     /* I2C STI interrupt callback */
void i2c4_simple_eri_callback(void);                                     /* I2C ERI interrupt callback */


/***********************************************************************************************************************
* Function Name: i2c_init
* Description  : SCI4 simple I2C mode initialization
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/



FUNC_LOCATION_I2C_SIMPLE int8_t I2C4_Simple_Wait_Ready(int32_t timeout)
{
    int32_t ic;
    int8_t result;
    result=0;
    timeout *=10;
    for (ic=0 ; ;ic++) {
        if (ic > timeout && timeout != 0) {
            result = -2; break;
        }
        if (I2C_STATE_ERROR == s_i2c4_simple_state) {
            result = -1; break;
        }
//      if (I2C_STATE_RDY == s_i2c4_simple_state) {
        if (1 == s_i2c4_simple_state) {
            result = 0; break;
        }
       R_SYS_SoftwareDelay(100, SYSTEM_DELAY_UNITS_MICROSECONDS);
   }

    return result;
}

FUNC_LOCATION_I2C_SIMPLE int8_t I2C4_Simple_Initialize(int8_t bus_speed)
{
    int8_t result;
    static uint8_t BRR;
    static uint8_t n;

    /* Lock SCI4 resource */
    if (0 != R_SYS_ResourceLock(SYSTEM_LOCK_SCI4))
    {

        /* If the result of the resource lock is an error, do not initialize */
        while (1)
        {
            ;   /* Intentionally empty braces. */
        }
    }

    /* Start SCI4 module */
    R_LPM_ModuleStart(LPM_MSTP_SCI4);

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is disabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is disabled.
       b4    - RE   - Receive Enable - Serial transmission is disabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI4->SCR = 0x00;

    /* Set SSDA, SSCL pin */
    R_SCI_Pinset_CH4();

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
       b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state.
       b3    - IICSTIF    - Issuing of Start, Restart, or Stop Condition Completed Flag - 
                               No requests made for generating conditions, or a condition is being generated
       b2    - IICSTPREQ  - Stop Condition Generation - Do not generate stop condition.
       b1    - IICRSTAREQ - Restart Condition Generation - Do not generate restart condition.
       b0    - IICSTREQ   - Start Condition Generation - Do not generate start condition. */
    SCI4->SIMR3 = 0xF0;


    /* SCMR  - Smart Card Mode Register
       b7    - BCP2 - Base Clock Pulse 2.
       b6:b5 - Reserved - This bit is read as 1. The write value should be 1.
       b4    - CHR1 - Character Length 1 - Transmit/receive in 8-bit data length.
       b3    - SDIR - Transmitted/Received Data Transfer Direction - Transfer with MSB first.
       b2    - SINV - Transmitted/Received Data Invert - TDR contents are transmitted as they are. Receive data
                                                         is stored as it is in RDR.
       b1    - Reserved - This bit is read as 1. The write value should be 1.
       b0    - SMIF - Smart Card Interface Mode Select - Non-smart card interface mode. */
    SCI4->SCMR = 0xFA;

    result = i2c_simple_baud_rate(SCI_USE_PCLKB, bus_speed, &BRR, &n);
    if (result !=0) {return result;}
    
    /* SMR   - Serial Mode Register for Non-Smart Card Interface Mode - The set value depends on the variable.
       b7    - CM  - Communications Mode - Asynchronous mode or simple I2C mode
       b6    - CHR - Character Length - Transmit/receive in 8-bit data length.
       b5    - PE  - Parity Enable - Parity bit addition is not performed.
       b4    - PM  - Parity Mode - Selects even parity.
       b3    - STOP - Stop Bit Length - 1 stop bit.
       b2    - MP  - Multi-Processor Mode - Multi-processor communications function is disabled.
       b1:b0 - Clock Select - PCLK/4 (n = 1) PCLK/1 (n = 0) */
    SCI4->SMR = 0x00 | n; 

    /* BRR - Bit Rate Register 
       b7:b0 - Adjusts the bit rate(Set to 100kbps, Width at high=4.375us(min), Width at low=5.00us(min)).  */
//    SCI4->BRR = (5-1);
    SCI4->BRR = BRR;

    /* SEMR  - Serial Extended Mode Register
       b7    - RXDESEL - Asynchronous Start Bit Edge Detection Select - Valid only in asynchronous mode.
       b6    - BGDM - Baud Rate Generator Double-Speed Mode Select - Valid only in asynchronous mode.
       b5    - NFEN - Digital Noise Filter Function Enable - The noise cancellation function for the SSCLn and
                                                             SSDAn input signals is enabled.
       b4    - ABCS - Asynchronous Mode Base Clock Select - Valid only in asynchronous mode.
       b3    - ABCSE - Asynchronous Mode Extended Base Clock Select1 - Valid only in asynchronous mode.
       b2    - BRME - Bit Rate Modulation Enable - Bit rate modulation function is disabled.
       b1:b0 - Reserved - This bit is read as 0. The write value should be 0. */
    SCI4->SEMR = 0x20;

    /* SNFR  - Noise Filter Setting Register 
       b7:b3 - Reserved - This bit is read as 0. The write value should be 0.
       b2:b0 - NFCS - Noise Filter Clock Select - The clock signal divided by 4 is used with the noise filter */
    SCI4->SNFR = 0x03;

    /* SIMR1 - I2C Mode Register 1
       b7:b3 - IICDL - SSDAn Output Delay Select - The following cycles are of the clock signal from the on-chip
                                                   baud rate generator: No output delay
       b2:b1 - Reserved - This bit is read as 0. The write value should be 0.
       b0    - IICM - Simple I2C Mode Select - Simple I2C mode. */
    SCI4->SIMR1 = 0x01;

    /* SIMR2 - I2C Mode Register 2
       b7:b6 - Reserved - This bit is read as 0. The write value should be 0.
       b5    - IICACKT - ACK Transmission Data - Transmission of NACK and reception of ACK/NACK.
       b4:b2 - Reserved - This bit is read as 0. The write value should be 0.
       b1    - IICCSC - Clock Synchronization - Synchronize with the clock signal.
       b0    - IICINTM - I2C Interrupt Mode Select - Use reception and transmission interrupts. */
    SCI4->SIMR2 = 0x23;

    /* SPMR - SPI Mode Register
       b7   - CKPH - Clock Phase Select - Do not delay clock.
       b6   - CKPOL - Clock Polarity Select - Do not invert clock polarity.
       b5   - Reserved  - This bit is read as 0. The write value should be 0.
       b4   - MFF - Mode Fault Flag - No mode fault error.
       b3   - Reserved - This bit is read as 0. The write value should be 0.
       b2   - MSS - Master Slave Select - Transmission is through
                                          the TXDn pin and reception is through the RXDn pin (master mode).
       b1   - CTSE - CTS Enable - CTS function is disabled (RTS output function is enabled).
       b0   - SSE - SSn Pin Function Enable - SSn pin function is disabled. */
    SCI4->SPMR = 0x00;

    /* Set to SCI4 TXI irq event link and interrupt enable */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI, 0x1B, i2c4_simple_txi_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI);

    /* Set to SCI4 STI(TEI) irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI, 0x1B, i2c4_simple_sti_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);

    /* Set to SCI4 RXI irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI, 0x1B, i2c4_simple_rxi_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI);

    /* Set to SCI4 RXI irq event link */
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI, 0x1A, i2c4_simple_eri_callback);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI, 3);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);

    s_i2c4_simple_state = I2C_STATE_RDY;

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is enabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is enabled.
       b4    - RE   - Receive Enable - Serial transmission is enabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is enabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI4->SCR = 0xB4;

    return 0;
}   /* End of function i2c_init() */

/***********************************************************************************************************************
* Function Name: i2c_close
* Description  : SCI4 simple I2C mode uninitialize
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C4_Simple_Uninitialize(void)
{
    volatile uint8_t dummy_read;

    /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
       b7    - TIE  - Transmit Interrupt Enable - A TXI interrupt request is disabled.
       b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.
       b5    - TE   - Transmit Enable - Serial transmission is disabled.
       b4    - RE   - Receive Enable - Serial transmission is disabled.
       b3    - MPIE - Multi-Processor Interrupt Enable - Normal reception.
       b2    - TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled.
       b1:b0 - CKE[1:0] - Clock Enable - Internal clock. The SCKn pin functions as the clock output pin. */
    SCI4->SCR = 0x00;

    /* Error flag reading and clear */
    dummy_read = SCI4->SSR;

    /* SSR - Serial Status Register for Non-Smart Card Interface Mode and non-FIFO Mode
       b7  - TDRE - Transmit Data Empty Flag - No transmission data in TDR register
       b6  - RDRF - Receive Data Full Flag - No received data in RDR register
       b5  - ORER - Overrun Error Flag - No overrun error occurred
       b4  - FER - Framing Error Flag - No framing error occurred
       b3  - PER - Parity Error Flag - No parity error occurred
       b2  - TEND - Transmit End Flag
       b1  - MPB - Multi-Processor
       b0  - MPBT - Multi-Processor Bit Transfer - Data transmission cycles */
    SCI4->SSR = 0x80;

    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);

    /* Clear TXI1 interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI);

    /* Clear RXI1 interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI);

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);

    /* Release simple SPI pins */
    R_SCI_Pinclr_CH4();

    /* Stop SCI4 module */
    R_LPM_ModuleStop(LPM_MSTP_SCI4);

    /* Unlock SCI4 resource */
    R_SYS_ResourceUnlock(SYSTEM_LOCK_SCI4);

    s_i2c4_simple_state = I2C_STATE_UNINITIALIZE;

	return 0;

}   /* End of function i2c_close() */

/***********************************************************************************************************************
* Function Name: i2c_write
* Description  : Writing to EEPROM in simple I2C mode
* Arguments    : addr       : EEPROM write address
*                p_data_out : Pointer to write data
*                size       : Number of write data
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C4_Simple_MasterTransmit_no_Wait(uint8_t addr, uint8_t const * const p_data_out, uint32_t size)
{
    if ((I2C_STATE_RDY != s_i2c4_simple_state) && (I2C_STATE_ERROR != s_i2c4_simple_state))
    {
        return -1;
    }
    i2c4_prv_device_addr = addr;

    sp_i2c4_simple_data_out = p_data_out;                   /* Set EEPROM Write data pointer        */
    s_i2c4_simple_size  = size;                             /* Set EEPROM Write size                */
//    s_i2c4_simple_address = addr;                           /* Set EEPROM Write address             */
    s_i2c4_simple_eep_mode = EEP_PRV_WRITE;                 /* EEPROM Write mode                    */
    s_i2c4_simple_counter = 0;                              /* Clear I2C counter                    */

    s_i2c4_simple_state = I2C_STATE_START;                  /* Set generate start condition state   */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b0    - IICSTREQ   - Start Condition Generation - Generate start condition. */
    SCI4->SIMR3 = 0x51;

	return 0;
}   /* End of function i2c_write() */

/***********************************************************************************************************************
* Function Name: i2c_read
* Description  : Read to EEPROM in simple I2C mode
* Arguments    : addr       : EEPROM read address
*                p_data_in  : Pointer to read data
*                size       : Number of read data
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE int8_t I2C4_Simple_MasterReceive_no_Wait(uint8_t addr, uint8_t *p_data_in, uint32_t size)
{
    if ((I2C_STATE_RDY != s_i2c4_simple_state) && (I2C_STATE_ERROR != s_i2c4_simple_state))
    {
        return -1;
    }

    i2c4_prv_device_addr = addr;
    sp_i2c4_simple_data_in = p_data_in;                     /* Set EEPROM Read data pointer         */
    s_i2c4_simple_size  = size;                             /* Set EEPROM Read size                 */
//    s_i2c4_simple_address = addr;                           /* Set EEPROM Read address              */
    s_i2c4_simple_eep_mode = EEP_PRV_READ;                  /* EEPROM Read mode                     */
    s_i2c4_simple_counter = 0;                              /* Clear I2C counter                    */

//    s_i2c4_simple_state = I2C_STATE_START;                  /* Set generate start condition state   */
    s_i2c4_simple_state = I2C_STATE_RESTART;                  /* Set generate start condition state   */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b0    - IICSTREQ   - Start Condition Generation - Generate start condition. */
    SCI4->SIMR3 = 0x51;

	return 0;

}   /* End of function i2c_read() */

/***********************************************************************************************************************
* Function Name: i2c_gen_stop_condition
* Description  : Start generation of stop condition
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c4_simple_gen_stop_condition(void)
{

    s_i2c4_simple_state = I2C_STATE_STOP;                   /* Set generate stop condition state    */

    /* SIMR3   - I2C Mode Register 3
       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
       b2    - IICSTPREQ  - Stop Condition Generation - Do not generate stop condition.  */
    SCI4->SIMR3 = 0x54;

}   /* End of function i2c_gen_stop_condition() */

/***********************************************************************************************************************
* Function Name: i2c_txi_callback
* Description  : SCI4 txi interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c4_simple_txi_callback(void)
{

    /* Check I2C state */
    switch (s_i2c4_simple_state)
    {
        case I2C_STATE_SND_DEVICE_W:

            /* STATE: I2C send device address(W)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI4->SISR_b.IICACKR)
            {

                /* Transition to upper address output state when ACK is received */
                s_i2c4_simple_state = I2C_STATE_SND_ADDR_H;
                SCI4->TDR = (uint8_t)(s_i2c4_simple_address >> 8);
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c4_simple_gen_stop_condition();
            }
            break;

        case I2C_STATE_SND_DEVICE_R:

            /* STATE: I2C send device address(R)  */
            /* Transition to data reception state */
            if (I2C_PRV_ACK != SCI4->SISR_b.IICACKR)
            {

                /* Generate stop condition when receiving NACK */
                i2c4_simple_gen_stop_condition();
                break;
            }

            s_i2c4_simple_state = I2C_STATE_RCV_DATA;

            /* SIMR2 - I2C Mode Register 2
               b5    - IICACKT - ACK Transmission Data - ACK transmission.  */
            SCI4->SIMR2_b.IICACKT = 0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are enabled.  */
            SCI4->SCR_b.RIE = 1;

        /* Perform the following processing  */
        case I2C_STATE_RCV_DATA:

            /* STATE: I2C receive data  */
            /* Check receive counter */
            if ((s_i2c4_simple_counter + 1) == s_i2c4_simple_size)
            {

                /* When the next data is the final data */
                /* SIMR2 - I2C Mode Register 2
                   b5    - IICACKT - ACK Transmission Data - ACK transmission.  */
                SCI4->SIMR2_b.IICACKT = 1;
                SCI4->TDR = 0xFF;
                s_i2c4_simple_counter ++;
            }
            else if (s_i2c4_simple_counter >= s_i2c4_simple_size)
            {

                /* When final data is completed */
                i2c4_simple_gen_stop_condition();

            }
            else
            {
                SCI4->TDR = 0xFF;
                s_i2c4_simple_counter ++;
            }

            break;

        case I2C_STATE_SND_ADDR_H:

            /* STATE: I2C send data address(H)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI4->SISR_b.IICACKR)
            {

                /* Transition to lower address output state when ACK is received */
                s_i2c4_simple_state = I2C_STATE_SND_ADDR_L;
                SCI4->TDR = (uint8_t)(s_i2c4_simple_address & 0xFF);
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c4_simple_gen_stop_condition();
            }

            break;

        case I2C_STATE_SND_ADDR_L:

            /* STATE: I2C send data address(L)  */
            /* Check ACK */
            if (I2C_PRV_ACK == SCI4->SISR_b.IICACKR)
            {

                /* Check EEPROM Read/Write mode */
                if (EEP_PRV_WRITE == s_i2c4_simple_eep_mode)
                {

                    /* In write mode, transition to data transmission state */
                    s_i2c4_simple_state = I2C_STATE_SND_DATA;
                    SCI4->TDR = sp_i2c4_simple_data_out[s_i2c4_simple_counter];
                    s_i2c4_simple_counter ++;
                }
                else
                {

                    /* In read mode, transition to the restart condition output state */
                    s_i2c4_simple_state = I2C_STATE_RESTART;

                    /* SIMR3   - I2C Mode Register 3
                       b7:b6 - IICSCLS    - SSCLn Output Select - Generate a start, restart, or stop condition
                       b5:b4 - IICSDAS    - SSDAn Output Select - Generate a start, restart, or stop condition
                       b1    - IICRSTAREQ - Restart Condition Generation - Generate restart condition. */
                    SCI4->SIMR3 = 0x52;
                }
            }
            else
            {

                /* Generate stop condition when receiving NACK */
                i2c4_simple_gen_stop_condition();
            }

            break;

        case I2C_STATE_SND_DATA:

            /* STATE: I2C send data  */
            /* Check ACK reception/final data output */
            if ((I2C_PRV_ACK == SCI4->SISR_b.IICACKR) && (s_i2c4_simple_size > s_i2c4_simple_counter))
            {

                /* Next data write */
                SCI4->TDR = sp_i2c4_simple_data_out[s_i2c4_simple_counter];
                s_i2c4_simple_counter ++;
            }
            else
            {

                /* Generate stop condition when receiving NACK or final data output */
                i2c4_simple_gen_stop_condition();
            }
            break;

        default:

            /* In case of an invalid state, a stop condition is output. */
            i2c4_simple_gen_stop_condition();
            break;
    }
}   /* End of function i2c_txi_callback() */

/***********************************************************************************************************************
* Function Name: i2c_rxi_callback
* Description  : SCI4 rxi interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c4_simple_rxi_callback(void)
{

    *sp_i2c4_simple_data_in++ = SCI4->RDR;

    }   /* End of function i2c_rxi_callback() */

/***********************************************************************************************************************
* Function Name: i2c_sti_callback
* Description  : SCI4 sti interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
FUNC_LOCATION_I2C_SIMPLE void i2c4_simple_sti_callback(void)
{

    /* SIMR3   - I2C Mode Register 3
       b3    - IICSTIF    - Issuing of Start, Restart, or Stop Condition Completed Flag - 
                               No requests made for generating conditions, or a condition is being generated */
    SCI4->SIMR3_b.IICSTIF=0;

    /* Clear STI1(TEI1) interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);

    switch (s_i2c4_simple_state)
    {
        case I2C_STATE_START:

            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Serial clock output.
               b5:b4 - IICSDAS    - SSDAn Output Select - Serial data output.  */
            SCI4->SIMR3 = 0x00;

            /* STATE: I2C generate start condition  */
//            s_i2c4_simple_state = I2C_STATE_SND_DEVICE_W;
            s_i2c4_simple_state = I2C_STATE_SND_DATA;
  
            SCI4->TDR = ((i2c4_prv_device_addr<<1) | EEP_PRV_WRITE);
            break;

        case I2C_STATE_RESTART:

            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Serial clock output.
               b5:b4 - IICSDAS    - SSDAn Output Select - Serial data output.  */
            SCI4->SIMR3 = 0x00;

            /* STATE: I2C generate restart condition  */
            s_i2c4_simple_state = I2C_STATE_SND_DEVICE_R;
            SCI4->TDR = ((i2c4_prv_device_addr<<1) | EEP_PRV_READ);
            break;

        case I2C_STATE_STOP:

            /* STATE: I2C generate stop condition  */
            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
               b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state. */
            SCI4->SIMR3 = 0xF0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.  */
            SCI4->SCR_b.RIE = 0;

            /* Check if all transmission / reception is completed */
            if (s_i2c4_simple_size == s_i2c4_simple_counter)
            {
                s_i2c4_simple_state = I2C_STATE_RDY;
            }
            else
            {
                s_i2c4_simple_state = I2C_STATE_ERROR;
            }
            break;

        default:

            /* In case of an invalid state, i2c stop. */
            /* SIMR3   - I2C Mode Register 3
               b7:b6 - IICSCLS    - SSCLn Output Select - Drive the SSCLn pin to high-impedance state.
               b5:b4 - IICSDAS    - SSDAn Output Select - Drive the SSDAn pin to high-impedance state. */
            SCI4->SIMR3 = 0xF0;

            /* SCR   - Serial Control Register for Non-Smart Card Interface Mode
               b6    - RIE  - Receive Interrupt Enable - RXI and ERI interrupt requests are disabled.  */
            SCI4->SCR_b.RIE = 0;

            s_i2c4_simple_state = I2C_STATE_ERROR;
            break;
    }

}   /* End of function i2c_sti_callback() */

FUNC_LOCATION_I2C_SIMPLE void i2c4_simple_eri_callback(void)
{

}

