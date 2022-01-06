/**********************************************************************************************************************
* File Name    : sys_lpm.h
* Description  : Low power mode control function declaration
**********************************************************************************************************************/

#ifndef SYS_LPM_H_
#define SYS_LPM_H_

void system_lpm_setup_rtc(void) __attribute__ ((section(".ramfunc")));
void system_lpm_setup_agt(void) __attribute__ ((section(".ramfunc")));
void system_lpm_prepare(void)  __attribute__ ((section(".ramfunc")));
void system_lpm_enter(void) __attribute__ ((section(".ramfunc"))) ;
void system_lpm_exit(void) __attribute__ ((section(".ramfunc"))) ;
void system_lpm_wait_rtc(void) __attribute__ ((section(".ramfunc")));
void system_lpm_wait_agt(void) __attribute__ ((section(".ramfunc")));
void system_lpm_prepare_VBB(void) __attribute__ ((section(".ramfunc")));

#endif /* SYS_LPM_H_ */
