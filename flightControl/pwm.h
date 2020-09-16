#ifndef __PWM_H
#define	__PWM_H

#include "r_cg_macrodriver.h"

void SetPWM(int16_t PWM1,int16_t PWM2,int16_t PWM3,int16_t PWM4);

void PWM_Init(void);

void PWM_ud(void);

void zhilicl(void);

void outshezhi(float S,float P,float X );

#endif 
