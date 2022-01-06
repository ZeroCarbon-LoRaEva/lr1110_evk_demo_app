#include <stdint.h>
#include <stdbool.h>
//#include "RE01_1500KB.h"
#include "RE01_256KB.h"

#ifndef _UART_LOW_LEVEL_SUB
#define _UART_LOW_LEVEL_SUB



int8_t sci0_init_low_level(
	uint32_t baud_rate, uint32_t data_bits,uint32_t parity,uint32_t stop_bits,uint32_t flow_control
	)  __attribute__ ((section(".ramfunc")));            /* SCI0 initialize */
int8_t sci0_uninit_low_level(void) __attribute__ ((section(".ramfunc"))) ;
int8_t sci0_send_start_low_level(void) __attribute__ ((section(".ramfunc")));            /* Enable transmission */
int8_t sci0_receive_start_low_level(void) __attribute__ ((section(".ramfunc"))) ;            /* Enable transmission */

int8_t sci2_init_low_level(
	uint32_t baud_rate, uint32_t data_bits,uint32_t parity,uint32_t stop_bits,uint32_t flow_control
	)  __attribute__ ((section(".ramfunc")));            /* SCI0 initialize */
int8_t sci2_uninit_low_level(void)  __attribute__ ((section(".ramfunc")));
int8_t sci2_send_start_low_level(void) __attribute__ ((section(".ramfunc")));            /* Enable transmission */
int8_t sci2_receive_start_low_level(void)  __attribute__ ((section(".ramfunc")));            /* Enable transmission */

int8_t sci4_init_low_level(
	uint32_t baud_rate, uint32_t data_bits,uint32_t parity,uint32_t stop_bits,uint32_t flow_control
	)  __attribute__ ((section(".ramfunc")));            /* SCI0 initialize */
int8_t sci4_uninit_low_level(void) __attribute__ ((section(".ramfunc"))) ;
int8_t sci4_send_start_low_level(void) __attribute__ ((section(".ramfunc")));            /* Enable transmission */
int8_t sci4_receive_start_low_level(void) __attribute__ ((section(".ramfunc"))) ;            /* Enable transmission */





#endif
