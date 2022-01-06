/**
 * @file      system_gpio.c
 *
 * @brief     MCU GPIO-related functions
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

#include "RE01_256KB.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "config_mode.h"
#include "configuration.h"
#include "system.h"
#include "system_it.h"
#include "system_gpio.h"

//#include "stm32l476xx.h"
//#include "stm32l4xx_ll_bus.h"
//#include "stm32l4xx_ll_exti.h"
//#include "stm32l4xx_ll_system.h"
//#include "stm32l4xx_ll_gpio.h"

#if (BOARD_TYPE_EVK_TOKYOCOM  == 1)

void GpioMcuSetInterrupt(uint16_t port, uint16_t pin, int16_t irqMode, int8_t irqPriority )
{

  if (0x04 == port && 0x9 == pin)  // .. IRQ9
  {
	ICU->IRQCR9_b.IRQMD = 0x1; // rising edge interrupt (0x0 for falling edge)
	R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ9);
    R_ICU_Pinset_CH9();
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ9, 0x1F, (system_int_cb_t)IRQ9_IRQHandler);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ9);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ9, 1);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ9);
  }
  else if (0x06 == port && 0x4 == pin)  // F_CS .. IRQ3
//	  else if (F_CS == pin)  // F_CS .. IRQ3
  {
	    ICU->IRQCR3_b.IRQMD = 0x1; // rising edge interrupt (0x0 for falling edge)
	R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3);
    R_ICU_Pinset_CH3();
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3, 0x1, (system_int_cb_t)IRQ3_IRQHandler);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3, 1);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3);
    
    
  }
  else if (0x02 == port && 0x4 == pin)  // .. IRQ7
//	  else if (F_CS == pin)  // F_CS .. IRQ3
  {
	ICU->IRQCR7_b.IRQMD = 0x1; // rising edge interrupt (0x0 for falling edge)
	R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7);
    R_ICU_Pinset_CH7();
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7, 0x13, (system_int_cb_t)IRQ7_IRQHandler);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7, 1);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7);
    
    
  }
  else
  {
    // no other GPIO interrupts configured
  }     
}


#elif (BOARD_TYPE_EVK_TOKYOCOM  == 2)
void GpioMcuSetInterrupt(uint16_t port, uint16_t pin, int16_t irqMode, int8_t irqPriority )
{

  if (0x01 == port && 0xD == pin)  //P113 Trigger SW  .. IRQ3
  {
    ICU->IRQCR3_b.IRQMD = 0x0; // 0x0 for falling edge)
    R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3);
    R_ICU_Pinset_CH3();
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3, 0x1, (system_int_cb_t)IRQ3_IRQHandler);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3, 1);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ3);
    
    
  }
  else if (0x02 == port && 0x4 == pin)  // .. IRQ7
//	  else if (F_CS == pin)  // F_CS .. IRQ3
  {
	ICU->IRQCR7_b.IRQMD = 0x1; // rising edge interrupt (0x0 for falling edge)
	R_NVIC_DisableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7);
    R_ICU_Pinset_CH7();
    R_SYS_IrqEventLinkSet(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7, 0x13, (system_int_cb_t)IRQ7_IRQHandler);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7);
    R_NVIC_SetPriority(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7, 1);
    R_NVIC_EnableIRQ(SYSTEM_CFG_EVENT_NUMBER_PORT_IRQ7);
    
    
  }
  else
  {
    // no other GPIO interrupts configured
  }     
}

#endif

void system_gpio_init_input(uint16_t port, uint16_t pin, system_gpio_interrupt_t irq_mode )
//void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const gpio_pull_mode_t pull_mode, const gpio_irq_mode_t irq_mode,  hal_gpio_irq_t* irq )
{
	uint32_t *pAddr;
	uint32_t tempReg;
//	uint8_t port;

//	port = (pin & 0xf0) >> 4;
	pAddr = (uint32_t *) (PFS_BASE + (0x40 * port) + (0x04 * (0x0f & pin) ));


   tempReg = 0;
//   if (pull_mode == HAL_GPIO_PULL_MODE_UP) {
//       tempReg |= 0x01 << 4;
//   } else if (pull_mode == HAL_GPIO_PULL_MODE_DOWN) {
//      tempReg |= 0x01 << 5;
//   }
   R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_MPC);
   *pAddr = tempReg;
   R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_MPC);

   if (irq_mode == SYSTEM_GPIO_RISING || irq_mode == SYSTEM_GPIO_FALLING || irq_mode == SYSTEM_GPIO_BOTH) {
     GpioMcuSetInterrupt(port,  pin, irq_mode, 1);
   }

   R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_MPC);
   tempReg = *pAddr;
   if (irq_mode == SYSTEM_GPIO_RISING) {
       tempReg |= 0x05 << 12;
   } else if (irq_mode == SYSTEM_GPIO_FALLING) {
       tempReg |= 0x06 << 12;
   } else if (irq_mode == SYSTEM_GPIO_BOTH) {
       tempReg |= 0x07 << 12;
   }
   *pAddr = tempReg;
   R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_MPC);

}

void system_gpio_init_output( uint16_t port, uint16_t pin, uint8_t value )
{
  uint32_t *pAddr = (uint32_t *) (PFS_BASE + (0x40 * port) + (0x04 * (0x0f & pin) ));
//  uint32_t tempReg = *pAddr;
  
  R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_MPC);

  *pAddr = 0x04 | (value & 0x01);
  R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_MPC);
  
}


void system_gpio_init( void )
{
    system_gpio_init_output( LR1110_LED_SCAN_PORT, LR1110_LED_SCAN_PIN, 0 );
    system_gpio_init_output( LR1110_LED_TX_PORT, LR1110_LED_TX_PIN, 0 );
    system_gpio_init_output( LR1110_LED_RX_PORT, LR1110_LED_RX_PIN, 0 );

    system_gpio_init_output( LR1110_RESET_PORT, LR1110_RESET_PIN, 1 );
    system_gpio_init_output( LR1110_NSS_PORT, LR1110_NSS_PIN, 1 );


#if (BOARD_TYPE_EVK_TOKYOCOM  == 1)
    system_gpio_init_input( LR1110_IRQ_PORT, LR1110_IRQ_PIN, SYSTEM_GPIO_NO_INTERRUPT ); //LR1110 interrupt  P604
    system_gpio_init_input( LR1110_IRQ_DUM_PORT, LR1110_IRQ_DUM_PIN, SYSTEM_GPIO_RISING ); //LR1110 interrupt P603 IRQ3
    system_gpio_init_input( ACCELERATOR_IRQ_PORT, ACCELERATOR_IRQ_PIN, SYSTEM_GPIO_RISING );// PORT409 IRQ9 
#elif (BOARD_TYPE_EVK_TOKYOCOM  == 2)
    system_gpio_init_input( LR1110_IRQ_PORT, LR1110_IRQ_PIN, SYSTEM_GPIO_RISING );  //LR1110 interrupt P204 IRQ7
    system_gpio_init_input( ACCELERATOR_IRQ_PORT, ACCELERATOR_IRQ_PIN, SYSTEM_GPIO_FALLING );// TRIGGER(SW) P113 IRQ3
#endif


    system_gpio_init_input( LR1110_BUSY_PORT, LR1110_BUSY_PIN, SYSTEM_GPIO_NO_INTERRUPT );

//    system_gpio_init_input( TOUCH_IRQ_PORT, TOUCH_IRQ_PIN, SYSTEM_GPIO_RISING );

    system_gpio_init_output( DISPLAY_NSS_PORT, DISPLAY_NSS_PIN, 1 );
    system_gpio_init_output( DISPLAY_DC_PORT, DISPLAY_DC_PIN, 0 );

//    system_gpio_init_input( TOUCH_IRQ_PORT, TOUCH_IRQ_PIN, SYSTEM_GPIO_RISING ); // PORT204 IRQ7





    system_gpio_init_output( LR1110_LNA_PORT, LR1110_LNA_PIN, 0 );

    system_gpio_init_output( DCDC_EN_PORT, DCDC_EN_PIN, 1 );


//    system_gpio_init_input( BUTTON_BLUE_PORT, BUTTON_BLUE_PIN, SYSTEM_GPIO_NO_INTERRUPT );

    // Initialize the NSS of unused flash device that may interact on the SPI
    // bus
//    system_gpio_init_output( FLASH_NSS_PORT, FLASH_NSS_PIN, 1 );
}

void system_gpio_init_direction_state( const gpio_t gpio, const system_gpio_pin_direction_t direction,
                                       const system_gpio_pin_state_t state )
{
    switch( direction )
    {
    case SYSTEM_GPIO_PIN_DIRECTION_INPUT:
        system_gpio_init_input( gpio.port, gpio.pin, SYSTEM_GPIO_NO_INTERRUPT );
        break;
    case SYSTEM_GPIO_PIN_DIRECTION_OUTPUT:
        system_gpio_init_output( gpio.port, gpio.pin, state );
        break;
    default:
        break;
    }
}




void system_gpio_set_pin_state( gpio_t gpio, const system_gpio_pin_state_t state )
//void hal_gpio_set_value( const hal_gpio_pin_names_t pin_const, const uint32_t value )
//void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
  uint8_t pin = gpio.pin;
  uint8_t port = gpio.port;
  uint32_t *pAddr = (uint32_t *) (PORT0_BASE + (0x20 * port) ); 
  
  if (state)
  {
    *pAddr |= 0x1 << (pin + 16);
  }
  else
  {
    *pAddr &= ~( 0x1 << (pin +16) );
  }
}




system_gpio_pin_state_t system_gpio_get_pin_state( gpio_t gpio )
//uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin_const )
//uint32_t GpioMcuRead( Gpio_t *obj )
{

  uint8_t pin = gpio.pin;
  uint8_t port = gpio.port;

  uint32_t *pAddr = (uint32_t *) (PORT0_BASE + (0x20 * port) + 4); 
  
  uint32_t value = *pAddr & (0x1 << pin); 
  
  return (value > 0? 1 : 0);
}





void system_gpio_wait_for_state( gpio_t gpio, system_gpio_pin_state_t state )
{
	system_gpio_pin_state_t current_state;
	for (;;) {
    	current_state = system_gpio_get_pin_state(gpio );
		if (current_state == state) {break;}
	}
}



#if (BOARD_TYPE_EVK_TOKYOCOM  == 1)

static uint32_t reg_P300;
static uint32_t reg_P010;
static uint32_t reg_P815;
static uint32_t reg_P011;
static uint32_t reg_P000;
static uint32_t reg_P001;
static uint32_t reg_P003;


void system_uninit_PORT()
{
    R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_MPC);
    
    reg_P300 = PFS->P300PFS;
    
    reg_P010 = PFS->P010PFS;
    reg_P815 = PFS->P815PFS;
    reg_P011 = PFS->P011PFS;
    
    reg_P000 = PFS->P000PFS;
    reg_P001 = PFS->P001PFS;
    reg_P003 = PFS->P003PFS;
    
    
    
    PFS->P300PFS = 0x04;
    PFS->P010PFS = 0x04;
    PFS->P815PFS = 0x04;
    PFS->P011PFS = 0x04;
    PFS->P000PFS = 0x04;
    PFS->P001PFS = 0x04;
    PFS->P003PFS = 0x04;
    
    R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_MPC);
}

void system_restore_PORT()
{
    R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_MPC);
    
    PFS->P300PFS = reg_P300;
    PFS->P010PFS = reg_P010;
    PFS->P815PFS = reg_P815;
    PFS->P011PFS = reg_P011;
    PFS->P000PFS = reg_P000;
    PFS->P001PFS = reg_P001;
    PFS->P003PFS = reg_P003;
    
    R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_MPC);
}

#elif (BOARD_TYPE_EVK_TOKYOCOM  == 2)

static uint32_t reg_P006;
static uint32_t reg_P104;
static uint32_t reg_P105;
static uint32_t reg_P106;
static uint32_t reg_P107;
static uint32_t reg_P108;
static uint32_t reg_P109;
static uint32_t reg_P111;
static uint32_t reg_P112;
static uint32_t reg_P202;
static uint32_t reg_P203;
static uint32_t reg_P204;

void system_uninit_PORT()
{
    R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_MPC);
    
    reg_P006 = PFS->P006PFS;
    reg_P104 = PFS->P104PFS;
    reg_P105 = PFS->P105PFS;
    reg_P106 = PFS->P106PFS;
    reg_P107 = PFS->P107PFS;
    reg_P108 = PFS->P108PFS;
    reg_P109 = PFS->P109PFS;
    reg_P111 = PFS->P111PFS;
    reg_P112 = PFS->P112PFS;
    reg_P202 = PFS->P202PFS;
    reg_P203 = PFS->P203PFS;
    reg_P204 = PFS->P204PFS;
    
    
    
    PFS->P006PFS = 0x0;
    PFS->P104PFS = 0x0;
    PFS->P105PFS = 0x0;
    PFS->P106PFS = 0x0;
    PFS->P107PFS = 0x0;
    PFS->P108PFS = 0x0;
    PFS->P109PFS = 0x0;
    PFS->P111PFS = 0x0;
    PFS->P112PFS = 0x0;
    PFS->P202PFS = 0x0;
    PFS->P203PFS = 0x0;
    PFS->P204PFS = 0x0;

//    PFS->P113PFS = 0x0;
//    PFS->P012PFS = 0x0;
//    PFS->P001PFS = 0x0;
//    PFS->P002PFS = 0x0;
//    PFS->P003PFS = 0x0;
//    PFS->P004PFS = 0x0;
//    PFS->P005PFS = 0x0;


//    PFS->P100PFS = 0x0;
//    PFS->P101PFS = 0x0;
//    PFS->P102PFS = 0x0;
//    PFS->P103PFS = 0x0;


//    PFS->P200PFS = 0x0;
//    PFS->P201PFS = 0x0;
    
    R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_MPC);
}

void system_restore_PORT()
{
    R_SYS_RegisterProtectDisable(SYSTEM_REG_PROTECT_MPC);
    
    PFS->P006PFS = reg_P006;
    PFS->P104PFS = reg_P104;
    PFS->P105PFS = reg_P105;
    PFS->P106PFS = reg_P106;
    PFS->P107PFS = reg_P107;
    PFS->P108PFS = reg_P108;
    PFS->P109PFS = reg_P109;
    PFS->P111PFS = reg_P111;
    PFS->P112PFS = reg_P112;
    PFS->P202PFS = reg_P202;
    PFS->P203PFS = reg_P203;
    PFS->P204PFS = reg_P204;
    
    R_SYS_RegisterProtectEnable(SYSTEM_REG_PROTECT_MPC);
}
#endif

