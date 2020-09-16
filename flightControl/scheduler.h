#ifndef __SCHEDULER_H
#define	__SCHEDULER_H

#include "r_cg_macrodriver.h"
#include "r_cg_timer.h"

void Loop(void);

void SysTick_Handler(void);

void delay(void);
void delayms(uint16_t ms);

#endif 