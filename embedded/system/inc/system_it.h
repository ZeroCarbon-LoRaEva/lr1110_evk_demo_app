/**
 * @file      system_it.h
 *
 * @brief     MCU interrupt-related functions header
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

#ifndef __SYSTEM_IT_H
#define __SYSTEM_IT_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "stm32l4xx_ll_exti.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler( void )  __attribute__ ((section(".ramfunc")));
void HardFault_Handler( void )  __attribute__ ((section(".ramfunc")));
void MemManage_Handler( void )  __attribute__ ((section(".ramfunc")));
void BusFault_Handler( void )  __attribute__ ((section(".ramfunc")));
void UsageFault_Handler( void )  __attribute__ ((section(".ramfunc")));
void SVC_Handler( void )  __attribute__ ((section(".ramfunc")));
void DebugMon_Handler( void )  __attribute__ ((section(".ramfunc")));
void PendSV_Handler( void )  __attribute__ ((section(".ramfunc")));
void SysTick_Handler( void )  __attribute__ ((section(".ramfunc")));
void EXTI4_IRQHandler( void )  __attribute__ ((section(".ramfunc")));
void EXTI15_10_IRQHandler( void ) __attribute__ ((section(".ramfunc")));
void DMA1_Channel7_IRQHandler( void ) __attribute__ ((section(".ramfunc")));
void DMA1_Channel6_IRQHandler( void ) __attribute__ ((section(".ramfunc")));
void LPTIM1_IRQHandler( void ) __attribute__ ((section(".ramfunc")));
void IRQ3_IRQHandler( void ) __attribute__ ((section(".ramfunc")));
void IRQ4_IRQHandler( void ) __attribute__ ((section(".ramfunc")));
void IRQ7_IRQHandler( void ) __attribute__ ((section(".ramfunc")));
void IRQ9_IRQHandler( void ) __attribute__ ((section(".ramfunc")));

#ifdef __cplusplus
}
#endif

#endif
