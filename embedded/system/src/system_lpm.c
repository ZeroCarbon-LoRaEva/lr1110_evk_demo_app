/**********************************************************************************************************************
* File Name    : sys_lpm.c
* Description  : Low power mode control
**********************************************************************************************************************/
#include "config_mode.h"
#include "RE01_256KB.h"
#include "system.h"
#include "r_system_api.h"
#include "r_lpm_api.h"
#include "system_lpm.h"

/**************************************************************************//**
* @brief Setup the MCU low power mode
*
* @retval       
******************************************************************************/


static st_lpm_sstby_cfg_t sstby_cfg;

// MINPWON
void system_lpm_setup_rtc(void)
{
    int err = 0;
//    sstby_cfg.power_supply = LPM_OPE_TO_SSTBY_EXFPWON_VBB;         /* Transit from ALLPWON OPE to EXFPWON SSTBY VBB */
    sstby_cfg.power_supply = LPM_OPE_TO_SSTBY_MINPWON_VBB;         /* Transit from ALLPWON OPE to MINPWON SSTBY VBB */
    sstby_cfg.speed = LPM_SSTBY_SPEED_MODE_OFF;             /* Transit time is not shortened */
//  sstby_cfg.wup   = LPM_SSTBY_WUP_ACCEL  | LPM_SSTBY_WUP_RTCALM ;  /* Wakeup interrupt : PORT_IRQ3 PORT_IRQ7  RTC_Alarm*/
    sstby_cfg.wup   = LPM_SSTBY_WUP_ACCEL;                           /* Wakeup interrupt : PORT_IRQ3 PORT_IRQ7  */
    err = R_LPM_SSTBYModeSetup(&sstby_cfg);
    APP_ERR_HANDLER(err);
}

// EXFPWON
void system_lpm_setup_agt(void)
{
    int err = 0;
    sstby_cfg.power_supply = LPM_OPE_TO_SSTBY_EXFPWON_VBB;         /* Transit from ALLPWON OPE to EXFPWON SSTBY VBB */
//    sstby_cfg.power_supply = LPM_OPE_TO_SSTBY_MINPWON_VBB;         /* Transit from ALLPWON OPE to MINPWON SSTBY VBB */
    sstby_cfg.speed = LPM_SSTBY_SPEED_MODE_OFF;             /* Transit time is not shortened */
    sstby_cfg.wup   = LPM_SSTBY_WUP_LR1110 | LPM_SSTBY_WUP_AGT1UD;  /* Wakeup interrupt : PORT_IRQ3 PORT_IRQ7  RTC_Alarm*/
    err = R_LPM_SSTBYModeSetup(&sstby_cfg);
    APP_ERR_HANDLER(err);
}

void system_lpm_prepare_VBB(void)
{
    int err = 0;
    err = R_LPM_PowerSupplyModeExfpwonSet();
    APP_ERR_HANDLER(err);

    err = R_SYS_SystemClockSOSCSet();
//    err = R_SYS_SystemClockLOCOSet();
    APP_ERR_HANDLER(err);

    /* Select Sub-osc clock as the system clock source */
    err = R_SYS_SystemClockSOSCSet();
//    err = R_SYS_SystemClockLOCOSet();
    APP_ERR_HANDLER(err);

    err = R_SYS_HighSpeedClockStop();
    APP_ERR_HANDLER(err);

    err = R_SYS_MediumSpeedClockStop();
    APP_ERR_HANDLER(err);
    /* Stop HOCO oscillation */

    err = R_SYS_LowSpeedClockStop();
    APP_ERR_HANDLER(err);

    while(0x01 == (0x01 & R_SYS_OscStabilizationFlagGet()));

    /* Transit to SOSC-speed mode */
    err = R_SYS_LowSpeedModeSet();
//    err = R_SYS_32kHzSpeedModeSet();
    APP_ERR_HANDLER(err);

    /* Transit from NORMAL mode to VBB mode */
    err = R_LPM_BackBiasModeEntry();
    APP_ERR_HANDLER(err);
}



void system_lpm_prepare(void)
{
#if (TRACKER_RX_TX_UPDATE == 1 || TRACKER_RX_TX_UPDATE == 3)
        /* Set divider value for system clock and peripheral module clock */
        /* When transitioning from MINPWON to ALLPWON, the clock must be set to 4 MHz or less */
        /* before executing the WFE instruction due to limitation */
    int err = 0;
        err = R_SYS_SystemClockDividerSet(SYSTEM_CLOCK_DIV_8, SYSTEM_CLOCK_DIV_8);
        APP_ERR_HANDLER(err);
#endif
}

/**************************************************************************//**
* @brief Emter the MCU low power mode  
*
* @retval       
******************************************************************************/
//__attribute__ ((section(".ramfunc"))) void system_lpm_enter(void)

void system_lpm_enter(void)
{
    int err = 0;
        /* Entry EXFPWON SSTBY VBB mode */
    err = R_LPM_SSTBYModeEntry();
    APP_ERR_HANDLER(err);
}

/**************************************************************************//**
* @brief Exit the MCU low power mode      
*
* @retval       
******************************************************************************/
//__attribute__ ((section(".ramfunc"))) void system_lpm_exit(void)

void system_lpm_exit(void)
{
#if (TRACKER_RX_TX_UPDATE == 1 || TRACKER_RX_TX_UPDATE == 3)
    int err = 0;
    /* Set divider value for system clock and peripheral module clock */
    /* Since the clock was set to 4MHz when transitioning from MINPWON to ALLPWON, */
    /* the clock setting was changed back to 32MHz */
    err = R_SYS_SystemClockDividerSet(SYSTEM_CLOCK_DIV_1, SYSTEM_CLOCK_DIV_1);
    APP_ERR_HANDLER(err);
#endif
}


void system_lpm_wait_agt(void)
{
	system_lpm_prepare();
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_RTC_ALM);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_ACCEL);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_LR1110);
	system_lpm_enter();
	system_lpm_exit();
}

void system_lpm_wait_rtc(void)
{
	system_lpm_prepare();
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_AGT1_AGTI);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_ACCEL);
    R_SYS_IrqStatusClear(SYSTEM_CFG_EVENT_NUMBER_PORT_LR1110);
	system_lpm_enter();
	system_lpm_exit();
}

