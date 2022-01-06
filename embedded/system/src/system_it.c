/**
 * @file      system_it.c
 *
 * @brief     MCU interrupt-related functions
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

/* Includes ------------------------------------------------------------------*/
#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "system_it.h"
//#include "stm32l4xx_ll_dma.h"
#include "configuration.h"
#include <stdbool.h>
#include "system_time.h"
#include "system_lptim.h"
//#include "system_uart_fifo.h"
#include "system_uart_fifo_low_level.h"

//extern void SupervisorInterruptHandlerGui( bool is_down );
//extern void SupervisorInterruptHandlerDemo( void );
//extern void TimerHasElapsed( void );
//extern void lv_tick_inc( uint32_t );


int8_t irq3_event_flag;
int8_t irq4_event_flag;
int8_t accel_event_flag;
int8_t irq7_event_flag;
int8_t irq9_event_flag;


/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler( void ) {}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler( void )
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while( 1 )
    {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler( void )
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while( 1 )
    {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler( void )
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while( 1 )
    {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler( void )
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while( 1 )
    {
    }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler( void ) {}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler( void ) {}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler( void ) {}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
/*
  * @note   In the default implementation the System Timer (Systick) is used as source of time base.
  *         The Systick configuration is based on MSI clock, as MSI is the clock
  *         used after a system Reset and the NVIC configuration is set to Priority group 4.
  *         Once done, time base tick starts incrementing: the tick variable counter is incremented
  *         each 1ms in the SysTick_Handler() interrupt handler.
*/
void SysTick_Handler( void )
{
    system_time_IncreaseTicker( );
//    lv_tick_inc( 1 );
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (EXTI), for the */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xx.s).                                               */
/******************************************************************************/

/**
 * @brief  This function handles external lines 10 to 15 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI4_IRQHandler( void )
{
}

/**
 * @brief  This function handles external lines 10 to 15 interrupt request.
 * @param  None
 * @retval None
 */
void EXTI15_10_IRQHandler( void )
{
}

/**
 * @brief  This function handles DMA1 interrupt request.
 * @param  None
 * @retval None
 */
void DMA1_Channel7_IRQHandler( void )
{
}

/**
 * @brief  This function handles DMA1 interrupt request.
 * @param  None
 * @retval None
 */
void DMA1_Channel6_IRQHandler( void )
{
}

/**
 * @brief This function handles LPTIM1 global interrupt.
 */
void LPTIM1_IRQHandler( void )
{
}

void IRQ3_IRQHandler( void )
{
#if (BOARD_TYPE_EVK_TOKYOCOM == 2)
	accel_event_flag = true;
#else
	irq3_event_flag = true;
#endif
}

//int8_t irq4_event_flag;
void IRQ4_IRQHandler( void )
{
	irq4_event_flag = true;
}

//int8_t accel_event_flag;
void IRQ7_IRQHandler( void )
{
#if (BOARD_TYPE_EVK_TOKYOCOM == 1 && BOARD_TYPE1_TX_VERSION == 1)
	accel_event_flag = true;
#else
	irq7_event_flag = true;
#endif
}
//int8_t irq9_event_flag;
void IRQ9_IRQHandler( void )
{
#if (BOARD_TYPE_EVK_TOKYOCOM == 1 && BOARD_TYPE1_TX_VERSION == 2)
	accel_event_flag = true;
#else
	irq9_event_flag = true;
#endif
}
