#ifndef _UART_DRIVER_FIFO
#define _UART_DRIVER_FIFO

#include "r_usart_cmsis_api.h"

//#define       UART_BAUD_RATE  (115200)
//#define       UART_BAUD_RATE  (460800)
#define       UART_DRIVER_FIFO_BUFFER_SIZE  (256)

//#define ARM_DRIVER_OK      0 
//#define ARM_DRIVER_ERROR             -1
//#define ARM_DRIVER_ERROR_TIMEOUT     -3


typedef struct Fifo_ring
{
    uint16_t start;
    uint16_t next;
    char *data;
    uint16_t size;
    char cdat;
    int8_t send_end;
} FIFO_HEADER;


typedef struct arm_driver_UART_DRIVER_FIFO
{
  ARM_DRIVER_USART* sci_dev;
  FIFO_HEADER* tx_fifo;
  FIFO_HEADER* rx_fifo;
  char* tx_buf;
  char* rx_buf;
  void (*p_callback)(uint32_t);
// for low level function
  int8_t (*p_initialize)(
  		uint32_t baud_rate, 
		uint32_t data_bits,
		uint32_t parity,
		uint32_t stop_bits,
		uint32_t flow_control
  		);
  int8_t (*p_uninitialize)(void);
  int8_t (*p_send_start)(void);
  int8_t (*p_receive_start)(void);
  int8_t (*p_send_finish)(void);
  int8_t (*p_receive_finish)(void);
} UART_DRIVER_FIFO;


void uart0_set_param(UART_DRIVER_FIFO* p_uart_dev);
void uart2_set_param(UART_DRIVER_FIFO* p_uart_dev);
void uart4_set_param(UART_DRIVER_FIFO* p_uart_dev);
void uart5_set_param(UART_DRIVER_FIFO* p_uart_dev);

int8_t uart_fifo_init(UART_DRIVER_FIFO* p_uart_dev,  uint32_t baud_rate);
int8_t uart_fifo_init_custom(UART_DRIVER_FIFO* p_uart_dev,
		uint32_t baud_rate, 
		uint32_t data_bits,
		uint32_t parity,
		uint32_t stop_bits,
		uint32_t flow_control
		);
int8_t uart_fifo_uninit(UART_DRIVER_FIFO* p_uart_dev);

int8_t uart_fifo_send_start(UART_DRIVER_FIFO* p_uart_dev, char* buf, uint16_t size);
int8_t  uart_fifo_send_finished(UART_DRIVER_FIFO* p_uart_dev);
int8_t uart_fifo_receive_start(UART_DRIVER_FIFO* p_uart_dev);
int8_t  uart_fifo_receive_abort(UART_DRIVER_FIFO* p_uart_dev);
int8_t uart_fifo_receive_line_no_wait(UART_DRIVER_FIFO* p_uart_dev, char* buf, uint16_t max_size);
int8_t uart_fifo_receive_line(UART_DRIVER_FIFO* p_uart_dev, char* buf, uint16_t max_size, uint32_t timeout);
char uart_fifo_receive_char(UART_DRIVER_FIFO* p_uart_dev);
void uart_rx_fifo_flush(UART_DRIVER_FIFO* p_uart_dev);
void uart_tx_fifo_flush(UART_DRIVER_FIFO* p_uart_dev);
void uart_rx_fifo_wait_empty(UART_DRIVER_FIFO* p_uart_dev);

int8_t uart_rx_fifo_empty(UART_DRIVER_FIFO* p_uart_dev);
char uart_rx_fifo_pop(UART_DRIVER_FIFO* p_uart_dev);

//UART_DRIVER_FIFO uart2_dev;
//UART_DRIVER_FIFO uart4_dev;
//UART_DRIVER_FIFO uart5_dev;

void fifo_local_init( FIFO_HEADER *fifo, char *buffer, uint16_t size );
void fifo_local_push( FIFO_HEADER *fifo, char c );
char fifo_local_pop( FIFO_HEADER *fifo  );
//  If empty , return true
int8_t fifo_local_empty(FIFO_HEADER *fifo);

//  If empty , return true
int8_t fifo_local_full(FIFO_HEADER *fifo);
void fifo_local_flush( FIFO_HEADER *fifo );
int16_t  fifo_receive_line(  FIFO_HEADER *fifo, char* line_buf, uint16_t max_size);
//char  fifo_receive_char(  FIFO_HEADER *fifo);

int8_t uart_fifo_send_string(UART_DRIVER_FIFO *p_uart_dev, char* send_buf);
void uart_tx_fifo_wait_empty(UART_DRIVER_FIFO* p_uart_dev);
void uart_rx_fifo_wait_empty(UART_DRIVER_FIFO* p_uart_dev);


#endif

