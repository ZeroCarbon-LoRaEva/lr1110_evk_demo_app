/*************************************************

UART send receive 
using fifo buffer
without CMSIS Driver

*************************************************/


#define GLOBAL_INT_DISABLE()                        \
    do {                                            \
        uint32_t  __int_state = __get_PRIMASK();    \
        __set_PRIMASK(1U);                          \

  /* One or more line breaks are required */

#define GLOBAL_INT_RESTORE()                        \
        __set_PRIMASK(__int_state);                 \
    } while(0)





//#include "RE01_1500KB.h"
#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "r_usart_cmsis_api.h"
#include "system_uart_fifo_low_level.h"
#include "system_uart_low_level_sub.h"


#define UART_LOW_LEVEL (1)
//#define UART_LOW_LEVEL (0)


extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART2;
extern ARM_DRIVER_USART Driver_USART4;
extern ARM_DRIVER_USART Driver_USART5;


void fifo_local_init( FIFO_HEADER *fifo, char *buffer, uint16_t size );
void fifo_local_push( FIFO_HEADER *fifo, char c );
char fifo_local_pop( FIFO_HEADER *fifo  );
//  If empty , return true
int8_t fifo_local_empty(FIFO_HEADER *fifo);

//  If empty , return true
int8_t fifo_local_full(FIFO_HEADER *fifo);
void fifo_local_flush( FIFO_HEADER *fifo );
//char  fifo_receive_char(  FIFO_HEADER *fifo);





void fifo_local_init( FIFO_HEADER *fifo, char *buffer, uint16_t size )
{
    GLOBAL_INT_DISABLE();
    fifo->start = 0;
    fifo->next = 0;
    fifo->data = buffer;
    fifo->size = size;
    fifo->cdat = 0;
    fifo->send_end = true;
    GLOBAL_INT_RESTORE();
}

void fifo_local_push( FIFO_HEADER *fifo, char c )
{
//    volatile int16_t src, dst;
    volatile int16_t dst;
    GLOBAL_INT_DISABLE();

    dst = fifo->next;
    fifo->data[dst] = c;
    dst++;
    if (dst >= fifo->size) {
            dst = 0;
    } 
    fifo->data[dst] = 0; // Terminate Buffer
    fifo->next = dst;
    GLOBAL_INT_RESTORE();
}

char fifo_local_pop( FIFO_HEADER *fifo  )
{
    volatile char c;
//    volatile int16_t src, dst;
    volatile int16_t src;

    GLOBAL_INT_DISABLE();

    src = fifo->start;
    c = fifo->data[src];
    src++;
    if (src >= fifo->size) {
            src = 0;
    } 
    fifo->start = src;
    GLOBAL_INT_RESTORE();

    return(c);
}

//  If empty , return true
int8_t fifo_local_empty(FIFO_HEADER *fifo)
{
	int8_t result;
    GLOBAL_INT_DISABLE();
    if (fifo->next == fifo->start) {
        result = true;
    }
    else {
        result = false;
    }
    GLOBAL_INT_RESTORE();
    return result;
}

//  If empty , return true
int8_t fifo_local_full(FIFO_HEADER *fifo)
{
    //    volatile int16_t src, dst;
    volatile int16_t dst;
    int8_t result;

    GLOBAL_INT_DISABLE();
    dst = fifo->next;
    dst++;
    if (dst >= fifo->size) {
            dst = 0;
    }
    if (dst == fifo->start) {result = true;}
    else {result =  false;}
    GLOBAL_INT_RESTORE();
    return result;
}

extern int16_t bulk_count;

void fifo_local_flush( FIFO_HEADER *fifo )
{
    GLOBAL_INT_DISABLE();
    fifo->start = 0;
    fifo->next = 0;
    bulk_count = 0;

    GLOBAL_INT_RESTORE();

}


//char  fifo_receive_char(  FIFO_HEADER *fifo)
//{
//	return fifo_local_pop(fifo);
//}

int16_t  fifo_receive_line(  FIFO_HEADER *fifo, char* line_buf, uint16_t max_size, int32_t timeout)
{
    volatile char c;
    volatile uint8_t result;
    volatile int16_t ic, src;
    volatile int32_t i32;

    timeout *=10;
    if (max_size < 2) {return 0;}
    if (max_size > UART_DRIVER_FIFO_BUFFER_SIZE) {max_size = UART_DRIVER_FIFO_BUFFER_SIZE-2;}

    for (ic = 0; ic < fifo->size; ic++) {
        for (i32 = 0; ; i32++) {
        	if ((timeout !=0) && (i32 > timeout)) {
                line_buf[ic] = 0;
        		ic = -1;
        		goto Skip;
        	}
        	result = fifo_local_empty(fifo);
			if (!(result)) {break;}
			R_SYS_SoftwareDelay(100, SYSTEM_DELAY_UNITS_MICROSECONDS);
		}
		c = fifo_local_pop(fifo  );
        line_buf[ic] = c;
        if (c == 0x0D || c == 0x0A || c == 0x00) {
           break;
        }
	}
    line_buf[ic] = '\0';
Skip:
    return ic;
}

int16_t  fifo_receive_line_BAD3(  FIFO_HEADER *fifo, char* line_buf, uint16_t max_size)
{
    volatile char c;
    volatile int16_t ic, src;

    if (max_size < 2) {return 0;}
    if (max_size > UART_DRIVER_FIFO_BUFFER_SIZE) {max_size = UART_DRIVER_FIFO_BUFFER_SIZE-2;}

    max_size--;
    GLOBAL_INT_DISABLE();
    src =fifo->start;

    for (ic = 0; ic < fifo->size; ic++) {
		if (ic >= max_size) {
        	ic=0;
			break;
		}
        if (src == fifo->next) {
        	ic=0;
        	break;
        }
        c = fifo->data[src];
        line_buf[ic] = c;
        src++;
        if (src >= fifo->size) {
            src = 0;
        } 
        if (c == 0x0D || c == 0x0A || c == 0x00) {
           ic++;
           fifo->start=src;
           break;
        }
    }
    GLOBAL_INT_RESTORE();
    line_buf[ic] = '\0';
    return ic;
}


void uart_fifo_callback_sub(uint32_t event, ARM_DRIVER_USART* p_uart_dev, FIFO_HEADER* p_tx_fifo, FIFO_HEADER* p_rx_fifo)
{
//    volatile uint8_t result;
    volatile char c0;
  /** Check event */
    switch( event )
    {
        case ARM_USART_EVENT_SEND_COMPLETE:
            {

                if (fifo_local_empty(p_tx_fifo)) {
                    p_tx_fifo->send_end = true;
                } else {
                    p_tx_fifo->cdat = fifo_local_pop(p_tx_fifo);
                    p_uart_dev->Send(&(p_tx_fifo->cdat), 1);
                    p_tx_fifo->send_end = false;
                }

               /* Describe the process when sending is completed */
                /* set flag to send complete status */
            }
        break;
        
        case ARM_USART_EVENT_RECEIVE_COMPLETE:

            /* Describe processing when receiving is completed */
            {
              if (fifo_local_full(p_rx_fifo) != true) {
            	c0 = p_rx_fifo->cdat;
                if (c0 != 0) {
                  fifo_local_push( p_rx_fifo, c0 );
                }
              }
            p_uart_dev->Receive(&(p_rx_fifo->cdat), 1);
            }
        break;
        
        default:
            {
            /* Resume reception when a reception error occurs */
            }
        break;
    }

}   /* End of function usart_callback() */



int8_t uart_fifo_init_sub(ARM_DRIVER_USART* p_uart_dev, void (*p_callback)(uint32_t), 
		uint32_t baud_rate, 
		uint32_t data_bits,
		uint32_t parity,
		uint32_t stop_bits,
		uint32_t flow_control
		)
{
    volatile int8_t  result;


    result = p_uart_dev->Initialize(p_callback);
    if (result != ARM_DRIVER_OK) {return(result);}
  
    /** Initialize USART PowerControl **/
    result = p_uart_dev->PowerControl(ARM_POWER_FULL);
    if (result != ARM_DRIVER_OK) {return(result);}

    /** Initialize USART transmition settings **/
    result = p_uart_dev->Control((ARM_USART_MODE_ASYNCHRONOUS |    /* Work1 Please select argument of UART (Asynchronous) */
                         data_bits    |    /* 8 Data bits */
                         parity       |    /* No Parity */
                         stop_bits    |    /* 1 Stop bit */
                         flow_control  )     /* No Flow Control */
                        ,baud_rate);
    if (result != ARM_DRIVER_OK) {return(result);}

    /** Enable send for UART2 Debug   **/
    result = p_uart_dev->Control(ARM_USART_CONTROL_TX_RX,1);

    return (result);
}




int8_t  uart_fifo_send_start_sub(ARM_DRIVER_USART* p_uart_dev, FIFO_HEADER* p_fifo, char* send_buf, uint16_t size)
{
    volatile int8_t  result;
    volatile int32_t i,j;
//    volatile char c;

    if (size == 0) { return ARM_DRIVER_ERROR;}

    result = ARM_DRIVER_OK;
    for( i = 0; i < size; i++ )
    {
    	for (j=0; ; j++) {
    		if (j > 100000) {
    			return ARM_DRIVER_ERROR; // Timeout 100sec
    		}
    		result = fifo_local_full(p_fifo);
    		if (result != true) {break;}
    		R_SYS_SoftwareDelay(1, SYSTEM_DELAY_UNITS_MILLISECONDS);
    	}
        fifo_local_push(p_fifo, send_buf[i]);

        GLOBAL_INT_DISABLE();
        if (p_fifo->send_end == true) {
        	p_fifo->cdat = fifo_local_pop(p_fifo);
        	result = p_uart_dev->Send(&(p_fifo->cdat), 1);
        	p_fifo->send_end = false;
        }
        GLOBAL_INT_RESTORE();
    }
	return result;
}

int8_t  uart_fifo_send_start_trigger(UART_DRIVER_FIFO* p_uart_dev, FIFO_HEADER* p_fifo)
{
    volatile int8_t  result;
        GLOBAL_INT_DISABLE();
        if (p_fifo->send_end == true) {
//        	p_fifo->cdat = fifo_local_pop(p_fifo);
        	result = p_uart_dev->p_send_start();
        	p_fifo->send_end = false;
        }
        GLOBAL_INT_RESTORE();
        return result;
}

int8_t  uart_fifo_send_start_low_level_sub(UART_DRIVER_FIFO* p_uart_dev, FIFO_HEADER* p_fifo, char* send_buf, uint16_t size)
{
    volatile int8_t  result;
    volatile int32_t i,j;
//    volatile char c;

    if (size == 0) { return ARM_DRIVER_ERROR;}

    result = ARM_DRIVER_OK;
    for( i = 0; i < size; i++ )
    {
    	for (j=0; ; j++) {
    		if (j > 30000) {
    			result = ARM_DRIVER_ERROR;
    			goto Skip ; // Timeout 100sec
    		}
    		result = fifo_local_full(p_fifo);
    		if (result != true) {break;}
			uart_fifo_send_start_trigger(p_uart_dev, p_fifo);
    		R_SYS_SoftwareDelay(1, SYSTEM_DELAY_UNITS_MILLISECONDS);
    	}
        fifo_local_push(p_fifo, send_buf[i]);
    }
	uart_fifo_send_start_trigger(p_uart_dev, p_fifo);
Skip:
	return result;
}



int8_t  uart_fifo_send_start_sub_BAD(ARM_DRIVER_USART* p_uart_dev, FIFO_HEADER* p_fifo, char* send_buf, uint16_t size)
{
    volatile int8_t  result;
    volatile int16_t i;
//    volatile char c;
//    volatile int16_t retryCount;

    if (size == 0) { return ARM_DRIVER_ERROR;}

    for( i = 0; i < size; i++ )
    {
        while (fifo_local_full(p_fifo)) {}
        fifo_local_push(p_fifo, send_buf[i]);
    }
    
  GLOBAL_INT_DISABLE();
    p_fifo->cdat = fifo_local_pop(p_fifo);
    result = p_uart_dev->Send(&(p_fifo->cdat), 1);
    p_fifo->send_end = false;
  GLOBAL_INT_RESTORE();

	return result;
}

int8_t  uart_fifo_send_finished_sub(FIFO_HEADER* p_fifo)
{
	return p_fifo->send_end;
}


int8_t  uart_fifo_receive_start_sub(ARM_DRIVER_USART* p_uart_dev, FIFO_HEADER* p_fifo)
{
    volatile int8_t  result;
    result = p_uart_dev->Receive(&(p_fifo->cdat), 1);
	return result;

}

int8_t  uart_fifo_receive_abort_sub(ARM_DRIVER_USART* p_uart_dev)
{
    volatile int8_t  result;
    result = p_uart_dev->Control( ARM_USART_ABORT_RECEIVE, 0);
	return result;

}

#if 0
//  Return  >=1 .. Read bytes
//            0 .. No CR or LF
//           -1 .. Buffer is empty
int8_t    uart_fifo_receive_line_no_wait_sub(FIFO_HEADER* p_fifo, char* receive_buf, uint16_t max_size)
{
    volatile int8_t result;
    receive_buf[0] = '\0';
    result = fifo_local_empty(p_fifo); 
    if (result) {return -1;}
    result = fifo_receive_line(p_fifo, receive_buf, max_size);
    return (result);
}
#endif

int8_t    uart_fifo_receive_line_wait_sub(FIFO_HEADER* p_fifo, char* receive_buf, uint16_t max_size, int32_t timeout)
{
    volatile uint32_t ic;
    volatile int8_t result;
	volatile int32_t timeout2;
    char c0;

	timeout2 = timeout;
//	for (ic=0; ic < timeout; ic++) {
	for (ic=0; ; ic++) {
		if (timeout !=0 && ic >= timeout) {return -1;}
		receive_buf[0] = '\0';
    	result = fifo_receive_line(p_fifo, receive_buf, max_size, timeout2);
		if (result >0) {
		              c0=receive_buf[0];
		              if (c0 != 0 && c0 != 0x0D && c0 != 0x0A) {
		                break;
		              }
		} else if (result <0) {
            break;
		}
		R_SYS_SoftwareDelay(1, SYSTEM_DELAY_UNITS_MILLISECONDS);
	}
    return (result);
}


int8_t uart_fifo_uninit_sub(ARM_DRIVER_USART* p_uart_dev)
{
    volatile int8_t result;
    result = p_uart_dev->Uninitialize();
	return result;
}

char uart_fifo_receive_char_sub(FIFO_HEADER* p_fifo)
{
	volatile char c2;
	volatile int8_t result1;
    result1 = fifo_local_empty(p_fifo);
__NOP();
    if (result1 != true) {
		c2=fifo_local_pop(p_fifo);
		__NOP();
	} else {
		c2='\0';
	}
    return c2;
}


/*******************************************************

   The following functions are needed for each different SCIn

*******************************************************/

FIFO_HEADER uart0_tx_fifo;
FIFO_HEADER uart0_rx_fifo;
char uart0_tx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];
char uart0_rx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];


void uart0_fifo_callback(uint32_t event)
{
  GLOBAL_INT_DISABLE();
  uart_fifo_callback_sub(event, &Driver_USART0, &uart0_tx_fifo, &uart0_rx_fifo);
  GLOBAL_INT_RESTORE();
}


void uart0_set_param(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	p_uart_drv_fifo->sci_dev = &Driver_USART0;
	p_uart_drv_fifo->tx_fifo = &uart0_tx_fifo;
	p_uart_drv_fifo->rx_fifo = &uart0_rx_fifo;
	p_uart_drv_fifo->tx_buf = &uart0_tx_buf[0];
	p_uart_drv_fifo->rx_buf = &uart0_rx_buf[0];
	p_uart_drv_fifo->p_callback      = uart0_fifo_callback;

	p_uart_drv_fifo->p_initialize    = sci0_init_low_level;
	p_uart_drv_fifo->p_uninitialize  = sci0_uninit_low_level;
	p_uart_drv_fifo->p_send_start    = sci0_send_start_low_level;
	p_uart_drv_fifo->p_receive_start = sci0_receive_start_low_level;
}

FIFO_HEADER uart2_tx_fifo;
FIFO_HEADER uart2_rx_fifo;
char uart2_tx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];
char uart2_rx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];

void uart2_fifo_callback(uint32_t event)
{
  GLOBAL_INT_DISABLE();
  uart_fifo_callback_sub(event, &Driver_USART2, &uart2_tx_fifo, &uart2_rx_fifo);
  GLOBAL_INT_RESTORE();
}


void uart2_set_param(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	p_uart_drv_fifo->sci_dev = &Driver_USART2;
	p_uart_drv_fifo->tx_fifo = &uart2_tx_fifo;
	p_uart_drv_fifo->rx_fifo = &uart2_rx_fifo;
	p_uart_drv_fifo->tx_buf = &uart2_tx_buf[0];
	p_uart_drv_fifo->rx_buf = &uart2_rx_buf[0];
	p_uart_drv_fifo->p_callback = uart2_fifo_callback;

	p_uart_drv_fifo->p_initialize    = sci2_init_low_level;
	p_uart_drv_fifo->p_uninitialize  = sci2_uninit_low_level;
	p_uart_drv_fifo->p_send_start    = sci2_send_start_low_level;
	p_uart_drv_fifo->p_receive_start = sci2_receive_start_low_level;
}

FIFO_HEADER uart4_tx_fifo;
FIFO_HEADER uart4_rx_fifo;
char uart4_tx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];
char uart4_rx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];

void uart4_fifo_callback(uint32_t event)
{
  GLOBAL_INT_DISABLE();
  uart_fifo_callback_sub(event, &Driver_USART4, &uart4_tx_fifo, &uart4_rx_fifo);
  GLOBAL_INT_RESTORE();
}

void uart4_set_param(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	p_uart_drv_fifo->sci_dev = &Driver_USART4;
	p_uart_drv_fifo->tx_fifo = &uart4_tx_fifo;
	p_uart_drv_fifo->rx_fifo = &uart4_rx_fifo;
	p_uart_drv_fifo->tx_buf = &uart4_tx_buf[0];
	p_uart_drv_fifo->rx_buf = &uart4_rx_buf[0];
	p_uart_drv_fifo->p_callback = uart4_fifo_callback;

	p_uart_drv_fifo->p_initialize    = sci4_init_low_level;
	p_uart_drv_fifo->p_uninitialize  = sci4_uninit_low_level;
	p_uart_drv_fifo->p_send_start    = sci4_send_start_low_level;
	p_uart_drv_fifo->p_receive_start = sci4_receive_start_low_level;
}

#if 0
FIFO_HEADER uart5_tx_fifo;
FIFO_HEADER uart5_rx_fifo;
char uart5_tx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];
char uart5_rx_buf[UART_DRIVER_FIFO_BUFFER_SIZE];

void uart5_fifo_callback(uint32_t event)
{
  GLOBAL_INT_DISABLE();
  uart_fifo_callback_sub(event, &Driver_USART5, &uart5_tx_fifo, &uart5_rx_fifo);
  GLOBAL_INT_RESTORE();
}

void uart5_set_param(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	p_uart_drv_fifo->sci_dev = &Driver_USART5;
	p_uart_drv_fifo->tx_fifo = &uart5_tx_fifo;
	p_uart_drv_fifo->rx_fifo = &uart5_rx_fifo;
	p_uart_drv_fifo->tx_buf = &uart5_tx_buf[0];
	p_uart_drv_fifo->rx_buf = &uart5_rx_buf[0];
	p_uart_drv_fifo->p_callback = uart5_fifo_callback;

	p_uart_drv_fifo->p_initialize    = sci5_init_low_level;
	p_uart_drv_fifo->p_uninitialize  = sci5_uninit_low_level;
	p_uart_drv_fifo->p_send_start    = sci5_send_start_low_level;
	p_uart_drv_fifo->p_receive_start = sci5_receive_start_low_level;
}

#endif

/****************************

  Global functions

****************************/


int8_t uart_fifo_init(UART_DRIVER_FIFO* p_uart_drv_fifo,  uint32_t baud_rate)
{
    volatile int8_t  result;
    fifo_local_init( p_uart_drv_fifo->rx_fifo, p_uart_drv_fifo->rx_buf, UART_DRIVER_FIFO_BUFFER_SIZE );
    fifo_local_init( p_uart_drv_fifo->tx_fifo, p_uart_drv_fifo->tx_buf, UART_DRIVER_FIFO_BUFFER_SIZE );

#if (UART_LOW_LEVEL == 0)
	result =  uart_fifo_init_sub( p_uart_drv_fifo->sci_dev, p_uart_drv_fifo->p_callback, 
				baud_rate, 
				ARM_USART_DATA_BITS_8,
				ARM_USART_PARITY_NONE,
				ARM_USART_STOP_BITS_1,
				ARM_USART_FLOW_CONTROL_NONE
				);
#else 
	result =  p_uart_drv_fifo->p_initialize(
				baud_rate, 
				ARM_USART_DATA_BITS_8,
				ARM_USART_PARITY_NONE,
				ARM_USART_STOP_BITS_1,
				ARM_USART_FLOW_CONTROL_NONE
				);
#endif
	return result;
}


int8_t uart_fifo_init_custom(UART_DRIVER_FIFO* p_uart_drv_fifo,
		uint32_t baud_rate, 
		uint32_t data_bits,
		uint32_t parity,
		uint32_t stop_bits,
		uint32_t flow_control
		)
{
    volatile int8_t  result;
    fifo_local_init( p_uart_drv_fifo->rx_fifo, p_uart_drv_fifo->rx_buf, UART_DRIVER_FIFO_BUFFER_SIZE );
    fifo_local_init( p_uart_drv_fifo->tx_fifo, p_uart_drv_fifo->tx_buf, UART_DRIVER_FIFO_BUFFER_SIZE );

#if (UART_LOW_LEVEL == 0)
	result =  uart_fifo_init_sub( p_uart_drv_fifo->sci_dev, p_uart_drv_fifo->p_callback, 
				baud_rate, 
				data_bits,
				parity,
				stop_bits,
				flow_control
				);
#else 
	result =  p_uart_drv_fifo->p_initialize( 
				baud_rate, 
				data_bits,
				parity,
				stop_bits,
				flow_control
				);
#endif
	return result;
}



int8_t uart_fifo_uninit(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
    volatile int8_t result;
#if (UART_LOW_LEVEL == 0)
    result = uart_fifo_uninit_sub(p_uart_drv_fifo->sci_dev);
#else 
    result = p_uart_drv_fifo->p_uninitialize();
#endif
    return result;
}



//int8_t uart_fifo_init_default_sub(ARM_DRIVER_USART* p_uart_drv_fifo, void (*p_callback)(uint32_t), uint32_t baud_rate, uint32_t stop_bits)
//{
//    volatile int8_t  result;
//	result =  uart_fifo_init_sub( p_uart_drv_fifo, p_callback, 
//				baud_rate, 
//				ARM_USART_DATA_BITS_8,
//				ARM_USART_PARITY_NONE,
//				ARM_USART_STOP_BITS_1,
//				ARM_USART_FLOW_CONTROL_NONE
//				);
//	return result;
//}


int8_t uart_fifo_send_start(UART_DRIVER_FIFO* p_uart_drv_fifo, char* buf, uint16_t size)
{
	int8_t result;
#if (UART_LOW_LEVEL == 0)
    result = uart_fifo_send_start_sub(p_uart_drv_fifo->sci_dev, p_uart_drv_fifo->tx_fifo, buf, size);
#else 
    result = uart_fifo_send_start_low_level_sub(p_uart_drv_fifo, p_uart_drv_fifo->tx_fifo, buf, size);
#endif
    return result;
}


int8_t uart_fifo_send_string(UART_DRIVER_FIFO *p_uart_dev, char* send_buf)
{
	uint16_t len;
	len = strlen(send_buf);
    return uart_fifo_send_start(p_uart_dev, send_buf, len);
}


int8_t  uart_fifo_send_finished(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	return p_uart_drv_fifo->tx_fifo->send_end;
}


int8_t uart_fifo_receive_start(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	int8_t result;
#if (UART_LOW_LEVEL == 0)
    result = uart_fifo_receive_start_sub(p_uart_drv_fifo->sci_dev, p_uart_drv_fifo->rx_fifo);
#else 
//    result = p_uart_drv_fifo->p_receive_start( p_uart_drv_fifo->rx_fifo);
	result = 0;
#endif
    return result;
}

int8_t  uart_fifo_receive_abort(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	int8_t result;
#if (UART_LOW_LEVEL == 0)
	result = uart_fifo_receive_abort_sub(p_uart_drv_fifo->sci_dev);
#else 
	result = 0;
#endif
    return result;
}

int8_t uart_fifo_receive_line(UART_DRIVER_FIFO* p_uart_drv_fifo, char* buf, uint16_t max_size, int32_t timeout)
{
	int8_t result;
    result = uart_fifo_receive_line_wait_sub(p_uart_drv_fifo->rx_fifo, buf,  max_size, timeout);
    return result;
}

#if 0
int8_t uart_fifo_receive_line_no_wait(UART_DRIVER_FIFO* p_uart_drv_fifo, char* buf, uint16_t max_size)
{
	int8_t result;
    result = uart_fifo_receive_line_no_wait_sub(p_uart_drv_fifo->rx_fifo, buf,  max_size);
    return result;
}
#endif

volatile int16_t start0;
volatile int16_t next0;

char uart_fifo_receive_char(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
start0=    p_uart_drv_fifo->rx_fifo->start;
next0=    p_uart_drv_fifo->rx_fifo->next;

	int8_t result;
	result =  uart_fifo_receive_char_sub(p_uart_drv_fifo->rx_fifo);
    return result;
}

void uart_rx_fifo_flush(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
    return fifo_local_flush(p_uart_drv_fifo->rx_fifo);
}

void uart_tx_fifo_flush(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
    return fifo_local_flush(p_uart_drv_fifo->tx_fifo);
}
 


void uart_rx_fifo_wait_empty(UART_DRIVER_FIFO* p_uart_dev)
{
	while(fifo_local_empty(p_uart_dev->rx_fifo) != true) {
	}
    return;
}

void uart_tx_fifo_wait_empty(UART_DRIVER_FIFO* p_uart_dev)
{
	while(fifo_local_empty(p_uart_dev->tx_fifo) != true) {
	}
    return;
}


int8_t uart_rx_fifo_empty(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	return fifo_local_empty(p_uart_drv_fifo->rx_fifo);
}

char uart_rx_fifo_pop(UART_DRIVER_FIFO* p_uart_drv_fifo)
{
	return fifo_local_pop(p_uart_drv_fifo->rx_fifo);
}

