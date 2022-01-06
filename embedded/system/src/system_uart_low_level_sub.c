/***********************************************************************************************************************
* System Name      : UART sample software for low level code
* File Name        : main.c
* Version          : 1.02
* Device           : RE01 1500KB
* Abstract         : Demo source file
* Tool-Chain       : IAR compiler v8.40.2 and GCC v6
* IDE              : IAR EWARM and Renesas e2 studio
* OS               : not use
* H/W Platform     : Evaluation Kit RE01 1500KB
* Description      : [Terminal connection]
*                  :  P812(TXD4)
*                  :  P813(RXD4)
*                  : [Operation]
*                  : The following steps for communicating with PC terminal software.
*                  : (1) After releasing reset, initialize SCI4 as follows:
*                  :         SCI4: communication speed 38400bps, no parity, 1 stop bit
*                  : (2) Send the following data.
*                  :     "Command?
*                  :      1:LED ON
*                  :      2:LED OFF
*                  :      3:UART STOP"
*                  : (3) After the submission is complete, 
*                  :     prohibit the transmission, and allow the reception.
*                  : (4) If the data received in (3) is 1 or 2, perform LED operation.
*                  :     Then permission the transmission, and allow the reception.
*                  :     And go to (2).
*                  : (5) If the data received in (3) is 3, SCI4 end processing.
*                  : (6) When SW2 input is detected, initialize SCI4 and go to (2)
* Limitation       : none
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version  Description
*         : 26.12.2019 1.00     First Release
*           28. 2.2020 1.01     Remove unnecessary processing
*                               TIE, TE setting method modification
*         : 19.03.2020 1.02     Replaced with RE01 256KB Group CMSIS Driver Package Rev.0.80.
*                               Fixed the problem that command 3 became invalid 
*                               when resetting UART after stopping UART at 3 reception.
***********************************************************************************************************************/
/***********************************************************************************************************************
Includes   <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "RE01_256KB.h"
//#include "RE01_1500KB.h"
#include "r_core_cfg.h"
#include "r_system_api.h"
#include "r_lpm_api.h"
#include "pin.h"
#include "r_usart_cmsis_api.h"
#include "system_uart_fifo_low_level.h"
#include "system_uart_low_level_sub.h"


#define GLOBAL_INT_DISABLE()                        \
    do {                                            \
        uint32_t  __int_state = __get_PRIMASK();    \
        __set_PRIMASK(1U);                          \

  /* One or more line breaks are required */

#define GLOBAL_INT_RESTORE()                        \
        __set_PRIMASK(__int_state);                 \
    } while(0)


extern FIFO_HEADER uart0_tx_fifo;
extern FIFO_HEADER uart0_rx_fifo;
extern FIFO_HEADER uart2_tx_fifo;
extern FIFO_HEADER uart2_rx_fifo;
extern FIFO_HEADER uart4_tx_fifo;
extern FIFO_HEADER uart4_rx_fifo;

uint8_t bulk_receive_buffer[257];
int8_t bulk_receive_mode = 0;
int16_t bulk_count = 0;


/***********************************************************************************************************************
Macros
***********************************************************************************************************************/

typedef enum                                               /* SCI state management */
{
    SCI_STATE_STOP = 0,                                    /* SCI stopping */
    SCI_STATE_BUSY,                                        /* Sending / receiving */
    SCI_STATE_RDY,                                         /* Waiting to send */
}e_sci_state_t;



/***********************************************************************************************************************
* Function Name: main
* Description  : Main Function
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/

__attribute__ ((section(".ramfunc"))) void uart_event_callback_dum()
{
}


/***********************************************************************************************************************
* Function Name: sci0
***********************************************************************************************************************/
extern ARM_DRIVER_USART Driver_USART0;

volatile uint8_t s_sci0_state = SCI_STATE_STOP;      /* SCI state management */

void sci0_send_callback(void)            __attribute__ ((section(".ramfunc"))); /* Transmit interrupt callback */
void sci0_send_end_callback(void)        __attribute__ ((section(".ramfunc"))); /* Transmit end interrupt callback */
void sci0_receive_callback(void)         __attribute__ ((section(".ramfunc"))); /* Receive interrupt callback */
void sci0_receive_error_callback(void)   __attribute__ ((section(".ramfunc"))); /* Receive error interrupt callback */


/***********************************************************************************************************************
* Function Name: sci0_init
* Description  : SCI initialize
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/

int8_t sci0_init_low_level(
		uint32_t baud_rate, 
		uint32_t data_bits,
		uint32_t parity,
		uint32_t stop_bits,
		uint32_t flow_control
	)
{
    volatile int8_t  result;

    result = Driver_USART0.Initialize(uart_event_callback_dum);
    if (result != ARM_DRIVER_OK) {return(result);}
    result = Driver_USART0.PowerControl(ARM_POWER_FULL);
    if (result != ARM_DRIVER_OK) {return(result);}
    result = Driver_USART0.Control(
   						(ARM_USART_MODE_ASYNCHRONOUS |    /* Work1 Please select argument of UART (Asynchronous) */
                         data_bits    |    /* 8 Data bits */
                         parity       |    /* No Parity */
                         stop_bits    |    /* 1 Stop bit */
                         flow_control  )     /* No Flow Control */
                        ,baud_rate);
    if (result != ARM_DRIVER_OK) {return(result);}

    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);

    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI,0x10,sci0_send_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI,0x0F,sci0_send_end_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI,0x10,sci0_receive_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI,0x10,sci0_receive_error_callback);

    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI,0);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI,0);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI,0);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI,0);


    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TXI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_RXI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);

    R_SCI_Pinset_CH0();

    s_sci0_state = SCI_STATE_RDY;
	return result;
}



int8_t sci0_uninit_low_level(void)
{
    volatile int8_t  result;
    result = Driver_USART0.Uninitialize();
    R_SCI_Pinclr_CH0();
    s_sci0_state = SCI_STATE_STOP;
	return result;
}



/***********************************************************************************************************************
* Function Name: send_start
* Description  : Send start
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//int8_t sci0_send_start_low_level_FIFO()
int8_t sci0_send_start_low_level()

{
	GLOBAL_INT_DISABLE();
    /* Check state */
    if (SCI_STATE_RDY != s_sci0_state)
    {

        /* If SCI0 is not ready to be sent, don't start sending */
//        return -1;
    } else {

    /* TE - Transmit Enable - Serial transmission is enabled */
    /* TIE - Transmit Interrupt Enable - A TXI interrupt request is enabled */
      SCI0->SCR |= 0xA0;

    /* State setting - sending */
      s_sci0_state = SCI_STATE_BUSY;
	}
    R_SYS_SoftwareDelay(10, SYSTEM_DELAY_UNITS_MILLISECONDS);

	GLOBAL_INT_RESTORE();
	return 0;
}   /* End of function send_start() */



/***********************************************************************************************************************
* Function Name: send_start
* Description  : Send start
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//int8_t sci0_receive_start_low_level_no_FIFO()
int8_t sci0_receive_start_low_level()
{
	return 0;
}


/***********************************************************************************************************************
* Function Name: send_callback
* Description  : Transmit interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//void sci0_send_callback_FIFO(void)
void sci0_send_callback(void)
{
    GLOBAL_INT_DISABLE();
    if (fifo_local_empty(&uart0_tx_fifo)) {
        uart0_tx_fifo.send_end = true;

        /* TIE - Transmit Interrupt Enable - A TXI interrupt request is disabled */
        SCI0->SCR_b.TIE = 0;

        /* TEIE - Transmit End Interrupt Request Enable - The SCIn_TXI interrupt request is enabled. */
        SCI0->SCR_b.TEIE = 1;
    } else {

        if (s_sci0_state != SCI_STATE_BUSY) {
        	sci0_send_start_low_level();
        }
    	uart0_tx_fifo.cdat = fifo_local_pop(&uart0_tx_fifo);
//        SCI0->TDR_b.TDR = uart0_tx_fifo.cdat;
        SCI0->TDR = uart0_tx_fifo.cdat;
        uart0_tx_fifo.send_end = false;
    }
    GLOBAL_INT_RESTORE();
}   /* End of function send_callback() */


/***********************************************************************************************************************
* Function Name: send_end_callback
* Description  : Transmit end interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void sci0_send_end_callback(void)
{

    /* TE - Transmit Enable - Serial transmission is disabled */
    SCI0->SCR_b.TE = 0;

    /* TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled */
    SCI0->SCR_b.TEIE = 0;

    /* Clear interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_TEI);

    /* RIE - Receive Interrupt Enable - RXI and ERI interrupt requests are enable */
    SCI0->SCR_b.RIE = 1;

    /* RE - Receive Enable - Serial reception is enabled */
    SCI0->SCR_b.RE = 1;

    /* State setting - waiting to send */
    s_sci0_state = SCI_STATE_RDY;

}   /* End of function send_end_callback() */


/***********************************************************************************************************************
* Function Name: receive_callback
* Description  : Receive interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//void sci0_receive_callback_FIFO(void)
void sci0_receive_callback(void)
{
	char c0;
if (bulk_receive_mode) {
	bulk_receive_buffer[bulk_count++] = SCI0->RDR;
} else {

    GLOBAL_INT_DISABLE();
	if (fifo_local_full(&uart0_rx_fifo) != true) {
      c0 = SCI0->RDR;
//      if (c0 != 0) {
        fifo_local_push( &uart0_rx_fifo, c0 );
//      }
    }
    GLOBAL_INT_RESTORE();
}
}   /* End of function receive_callback() */


/***********************************************************************************************************************
* Function Name: receive_error_callback
* Description  : Receive error interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void sci0_receive_error_callback(void)
{
    volatile uint8_t dummy_read;

    /* Error flag reading and clear */
    dummy_read = SCI0->SSR;

    /* SSR - Serial Status Register for Non-Smart Card Interface Mode and non-FIFO Mode
        b7  - TDRE - Transmit Data Empty Flag - No transmit data in the TDR register
        b6  - RDRF - Receive Data Full Flag - Received data in the RDR register
        b5  - ORER - Overrun Error Flag - No overrun error occurred
        b4  - FER - Framing Error Flag - No framing error occurred
        b3  - PER - Parity Error Flag - No parity error occurred
        b2  - TEND - Transmit End Flag - A character is being transmitted
        b1  - MPB - Multi-Processor - Data transmission cycles
        b0  - MPBT - Multi-Processor Bit Transfer - Data transmission cycles */
    SCI0->SSR = 0xC0;

    /* Clear interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI0_ERI);

}   /* End of function receive_error_callback() */




/***********************************************************************************************************************
* Function Name: sci2
***********************************************************************************************************************/
extern ARM_DRIVER_USART Driver_USART2;

volatile uint8_t s_sci2_state = SCI_STATE_STOP;      /* SCI state management */

void sci2_send_callback(void)            __attribute__ ((section(".ramfunc"))); /* Transmit interrupt callback */
void sci2_send_end_callback(void)        __attribute__ ((section(".ramfunc"))); /* Transmit end interrupt callback */
void sci2_receive_callback(void)         __attribute__ ((section(".ramfunc"))); /* Receive interrupt callback */
void sci2_receive_error_callback(void)   __attribute__ ((section(".ramfunc"))); /* Receive error interrupt callback */


/***********************************************************************************************************************
* Function Name: sci2_init
* Description  : SCI initialize
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/

int8_t sci2_init_low_level(
		uint32_t baud_rate, 
		uint32_t data_bits,
		uint32_t parity,
		uint32_t stop_bits,
		uint32_t flow_control
	)
{
    volatile int8_t  result;

    result = Driver_USART2.Initialize(uart_event_callback_dum);
    if (result != ARM_DRIVER_OK) {return(result);}
    result = Driver_USART2.PowerControl(ARM_POWER_FULL);
    if (result != ARM_DRIVER_OK) {return(result);}
    result = Driver_USART2.Control(
   						(ARM_USART_MODE_ASYNCHRONOUS |    /* Work1 Please select argument of UART (Asynchronous) */
                         data_bits    |    /* 8 Data bits */
                         parity       |    /* No Parity */
                         stop_bits    |    /* 1 Stop bit */
                         flow_control  )     /* No Flow Control */
                        ,baud_rate);
    if (result != ARM_DRIVER_OK) {return(result);}

    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);

    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI,0x1A,sci2_send_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI,0x19,sci2_send_end_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI,0x1A,sci2_receive_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI,0x19,sci2_receive_error_callback);

    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TXI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_RXI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);
    R_SCI_Pinset_CH2();

    s_sci2_state = SCI_STATE_RDY;
	return result;
}

int8_t sci2_uninit_low_level(void)
{
    volatile int8_t  result;
    result = Driver_USART2.Uninitialize();
    R_SCI_Pinclr_CH2();
    s_sci2_state = SCI_STATE_STOP;
	return result;
}



/***********************************************************************************************************************
* Function Name: send_start
* Description  : Send start
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//int8_t sci2_send_start_low_level_FIFO()
int8_t sci2_send_start_low_level()

{
	GLOBAL_INT_DISABLE();
    /* Check state */
    if (SCI_STATE_RDY != s_sci2_state)
    {

        /* If SCI2 is not ready to be sent, don't start sending */
//        return -1;
    } else {

    /* TE - Transmit Enable - Serial transmission is enabled */
    /* TIE - Transmit Interrupt Enable - A TXI interrupt request is enabled */
      SCI2->SCR |= 0xA0;

    /* State setting - sending */
      s_sci2_state = SCI_STATE_BUSY;
	}

	GLOBAL_INT_RESTORE();
	return 0;
}   /* End of function send_start() */



/***********************************************************************************************************************
* Function Name: send_start
* Description  : Send start
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//int8_t sci2_receive_start_low_level_no_FIFO()
int8_t sci2_receive_start_low_level()
{
	return 0;
}


/***********************************************************************************************************************
* Function Name: send_callback
* Description  : Transmit interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//void sci2_send_callback_FIFO(void)
void sci2_send_callback(void)
{
    if (fifo_local_empty(&uart2_tx_fifo)) {
        uart2_tx_fifo.send_end = true;

        /* TIE - Transmit Interrupt Enable - A TXI interrupt request is disabled */
        SCI2->SCR_b.TIE = 0;

        /* TEIE - Transmit End Interrupt Request Enable - The SCIn_TXI interrupt request is enabled. */
        SCI2->SCR_b.TEIE = 1;
    } else {

        if (s_sci2_state != SCI_STATE_BUSY) {
        	sci2_send_start_low_level();
        }
    	uart2_tx_fifo.cdat = fifo_local_pop(&uart2_tx_fifo);
//        SCI2->TDR_b.TDR = uart2_tx_fifo.cdat;
        SCI2->TDR = uart2_tx_fifo.cdat;
        uart2_tx_fifo.send_end = false;
    }
}   /* End of function send_callback() */


/***********************************************************************************************************************
* Function Name: send_end_callback
* Description  : Transmit end interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void sci2_send_end_callback(void)
{

    /* TE - Transmit Enable - Serial transmission is disabled */
    SCI2->SCR_b.TE = 0;

    /* TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled */
    SCI2->SCR_b.TEIE = 0;

    /* Clear interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_TEI);

    /* RIE - Receive Interrupt Enable - RXI and ERI interrupt requests are enable */
    SCI2->SCR_b.RIE = 1;

    /* RE - Receive Enable - Serial reception is enabled */
    SCI2->SCR_b.RE = 1;

    /* State setting - waiting to send */
    s_sci2_state = SCI_STATE_RDY;

}   /* End of function send_end_callback() */

/***********************************************************************************************************************
* Function Name: receive_callback
* Description  : Receive interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//void sci2_receive_callback_FIFO(void)
void sci2_receive_callback(void)
{
	char c0;

if (bulk_receive_mode) {
	bulk_receive_buffer[bulk_count++] = SCI2->RDR;
} else {

    GLOBAL_INT_DISABLE();

    if (fifo_local_full(&uart2_rx_fifo) != true) {
      c0 = SCI2->RDR;
//      if (c0 != 0) {
        fifo_local_push( &uart2_rx_fifo, c0 );
//      }
    }
    GLOBAL_INT_RESTORE();
}
}   /* End of function receive_callback() */


/***********************************************************************************************************************
* Function Name: receive_error_callback
* Description  : Receive error interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void sci2_receive_error_callback(void)
{
    volatile uint8_t dummy_read;

    /* Error flag reading and clear */
    dummy_read = SCI2->SSR;

    /* SSR - Serial Status Register for Non-Smart Card Interface Mode and non-FIFO Mode
        b7  - TDRE - Transmit Data Empty Flag - No transmit data in the TDR register
        b6  - RDRF - Receive Data Full Flag - Received data in the RDR register
        b5  - ORER - Overrun Error Flag - No overrun error occurred
        b4  - FER - Framing Error Flag - No framing error occurred
        b3  - PER - Parity Error Flag - No parity error occurred
        b2  - TEND - Transmit End Flag - A character is being transmitted
        b1  - MPB - Multi-Processor - Data transmission cycles
        b0  - MPBT - Multi-Processor Bit Transfer - Data transmission cycles */
    SCI2->SSR = 0xC0;

    /* Clear interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI2_ERI);

}   /* End of function receive_error_callback() */


/***********************************************************************************************************************
* Function Name: sci4
***********************************************************************************************************************/
extern ARM_DRIVER_USART Driver_USART4;

volatile uint8_t s_sci4_state = SCI_STATE_STOP;      /* SCI state management */

void sci4_send_callback(void)            __attribute__ ((section(".ramfunc"))); /* Transmit interrupt callback */
void sci4_send_end_callback(void)        __attribute__ ((section(".ramfunc"))); /* Transmit end interrupt callback */
void sci4_receive_callback(void)         __attribute__ ((section(".ramfunc"))); /* Receive interrupt callback */
void sci4_receive_error_callback(void)   __attribute__ ((section(".ramfunc"))); /* Receive error interrupt callback */


/***********************************************************************************************************************
* Function Name: sci4_init
* Description  : SCI initialize
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/

int8_t sci4_init_low_level(
		uint32_t baud_rate, 
		uint32_t data_bits,
		uint32_t parity,
		uint32_t stop_bits,
		uint32_t flow_control
	)
{
    volatile int8_t  result;

    result = Driver_USART4.Initialize(uart_event_callback_dum);
    if (result != ARM_DRIVER_OK) {return(result);}
    result = Driver_USART4.PowerControl(ARM_POWER_FULL);
    if (result != ARM_DRIVER_OK) {return(result);}
    result = Driver_USART4.Control(
   						(ARM_USART_MODE_ASYNCHRONOUS |    /* Work1 Please select argument of UART (Asynchronous) */
                         data_bits    |    /* 8 Data bits */
                         parity       |    /* No Parity */
                         stop_bits    |    /* 1 Stop bit */
                         flow_control  )     /* No Flow Control */
                        ,baud_rate);
    if (result != ARM_DRIVER_OK) {return(result);}

    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI);
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);

    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI,0x1B,sci4_send_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI,0x1B,sci4_send_end_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI,0x1B,sci4_receive_callback);
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI,0x1A,sci4_receive_error_callback);

    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TXI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_RXI);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);
    R_SCI_Pinset_CH4();

    s_sci4_state = SCI_STATE_RDY;
	return result;
}

int8_t sci4_uninit_low_level(void)
{
    volatile int8_t  result;
    result = Driver_USART4.Uninitialize();
    R_SCI_Pinclr_CH4();
    s_sci4_state = SCI_STATE_STOP;
	return result;
}



/***********************************************************************************************************************
* Function Name: send_start
* Description  : Send start
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//int8_t sci4_send_start_low_level_FIFO()
int8_t sci4_send_start_low_level()

{
	GLOBAL_INT_DISABLE();
    /* Check state */
    if (SCI_STATE_RDY != s_sci4_state)
    {

        /* If SCI4 is not ready to be sent, don't start sending */
//        return -1;
    } else {

    /* TE - Transmit Enable - Serial transmission is enabled */
    /* TIE - Transmit Interrupt Enable - A TXI interrupt request is enabled */
      SCI4->SCR |= 0xA0;

    /* State setting - sending */
      s_sci4_state = SCI_STATE_BUSY;
	}

	GLOBAL_INT_RESTORE();
	return 0;
}   /* End of function send_start() */



/***********************************************************************************************************************
* Function Name: send_start
* Description  : Send start
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//int8_t sci4_receive_start_low_level_no_FIFO()
int8_t sci4_receive_start_low_level()
{
	return 0;
}


/***********************************************************************************************************************
* Function Name: send_callback
* Description  : Transmit interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//void sci4_send_callback_FIFO(void)
void sci4_send_callback(void)
{
    if (fifo_local_empty(&uart4_tx_fifo)) {
        uart4_tx_fifo.send_end = true;

        /* TIE - Transmit Interrupt Enable - A TXI interrupt request is disabled */
        SCI4->SCR_b.TIE = 0;

        /* TEIE - Transmit End Interrupt Request Enable - The SCIn_TXI interrupt request is enabled. */
        SCI4->SCR_b.TEIE = 1;
    } else {

        if (s_sci4_state != SCI_STATE_BUSY) {
        	sci4_send_start_low_level();
        }
    	uart4_tx_fifo.cdat = fifo_local_pop(&uart4_tx_fifo);
//        SCI4->TDR_b.TDR = uart4_tx_fifo.cdat;
        SCI4->TDR = uart4_tx_fifo.cdat;
        uart4_tx_fifo.send_end = false;
    }
}   /* End of function send_callback() */


/***********************************************************************************************************************
* Function Name: send_end_callback
* Description  : Transmit end interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void sci4_send_end_callback(void)
{

    /* TE - Transmit Enable - Serial transmission is disabled */
    SCI4->SCR_b.TE = 0;

    /* TEIE - Transmit End Interrupt Enable - A TEI interrupt request is disabled */
    SCI4->SCR_b.TEIE = 0;

    /* Clear interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_TEI);

    /* RIE - Receive Interrupt Enable - RXI and ERI interrupt requests are enable */
    SCI4->SCR_b.RIE = 1;

    /* RE - Receive Enable - Serial reception is enabled */
    SCI4->SCR_b.RE = 1;

    /* State setting - waiting to send */
    s_sci4_state = SCI_STATE_RDY;

}   /* End of function send_end_callback() */

/***********************************************************************************************************************
* Function Name: receive_callback
* Description  : Receive interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
//void sci4_receive_callback_FIFO(void)
void sci4_receive_callback(void)
{
	char c0;
if (bulk_receive_mode) {
	bulk_receive_buffer[bulk_count++] = SCI4->RDR;
} else {

    GLOBAL_INT_DISABLE();


    if (fifo_local_full(&uart4_rx_fifo) != true) {
      c0 = SCI4->RDR;
//      if (c0 != 0) {
        fifo_local_push( &uart4_rx_fifo, c0 );
//      }
    }
    GLOBAL_INT_RESTORE();
}
}   /* End of function receive_callback() */


/***********************************************************************************************************************
* Function Name: receive_error_callback
* Description  : Receive error interrupt callback
* Arguments    : none
* Return Value : none
***********************************************************************************************************************/
void sci4_receive_error_callback(void)
{
    volatile uint8_t dummy_read;

    /* Error flag reading and clear */
    dummy_read = SCI4->SSR;

    /* SSR - Serial Status Register for Non-Smart Card Interface Mode and non-FIFO Mode
        b7  - TDRE - Transmit Data Empty Flag - No transmit data in the TDR register
        b6  - RDRF - Receive Data Full Flag - Received data in the RDR register
        b5  - ORER - Overrun Error Flag - No overrun error occurred
        b4  - FER - Framing Error Flag - No framing error occurred
        b3  - PER - Parity Error Flag - No parity error occurred
        b2  - TEND - Transmit End Flag - A character is being transmitted
        b1  - MPB - Multi-Processor - Data transmission cycles
        b0  - MPBT - Multi-Processor Bit Transfer - Data transmission cycles */
    SCI4->SSR = 0xC0;

    /* Clear interrupt request */
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);
    R_NVIC_ClearPendingIRQ(SYSTEM_CFG_EVENT_NUMBER_SCI4_ERI);

}   /* End of function receive_error_callback() */


