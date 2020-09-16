#include "pwm.h"
#include "scheduler.h"
#include "imu.h"
#include "ms5611.h"

#define PWMMAX 22300
#define PWMMIN 16000
float base1=20700,base2=20850,base3=20700,base4=20870;
float YAW_P=0,YAW_D=0,PITCH_P=50,PITCH_D=5,ROLL_P=50,ROLL_D=5;
void SetPWM(int16_t PWM1,int16_t PWM2,int16_t PWM3,int16_t PWM4)
{		
	if(PWM1 > PWMMAX)	PWM1 = PWMMAX;
	if(PWM2 > PWMMAX)	PWM2 = PWMMAX;
	if(PWM3 > PWMMAX)	PWM3 = PWMMAX;
	if(PWM4 > PWMMAX)	PWM4 = PWMMAX;
	if(PWM1 < PWMMIN)	PWM1 = PWMMIN;
	if(PWM2 < PWMMIN)	PWM2 = PWMMIN;
	if(PWM3 < PWMMIN)	PWM3 = PWMMIN;
	if(PWM4 < PWMMIN)	PWM4 = PWMMIN;
	
	TDR01 = PWM1;
	TDR02 = PWM2;
	TDR03 = PWM3;
	TDR04 = PWM4;
}

void PWM_Init(void)
{
	SetPWM(PWMMIN, PWMMIN, PWMMIN, PWMMIN);
	delayms(5000);
}
void PWM_ud(void)
{  
    float PWM11,PWM22,PWM33,PWM44;   
    if(up_flag==100 && press<=98.2){
       pwm11=base1+50;
       pwm22=base2+50;
       pwm33=base3+50;
       pwm44=base4+50;
     }
    if(down_flag==100 && press>=98){
       pwm11=base1-50;
       pwm22=base2-50;
       pwm33=base3-50;
       pwm44=base4-50;
     }
   SetPWM(PWM11, PWM22, PWM33, PWM44);
}
/************************zitaipid*****************************/
void zhilicl(void) 
{
   float Xjiaoclout,Pjiaoclout,Sjiaoclout;
   float YAW_last,YAW_llast,YAW_last_err,YAW_llast_err,YAWd;
   float PITCH_last,PITCH_llast,PITCH_last_err,PITCH_llast_err,PITCHd;
   float ROLL_last,ROLL_llast,ROLL_last_err,ROLL_llast_err,ROLLd;
   
   YAW_llast=YAW_last;
   YAW_last=Angle[YAW];
   YAW_llast_err=YAW_last-YAW_llast;
   YAW_last_err=Angle[YAW]-YAW_last;
   YAWd=YAW_last_err-YAW_llast_err;
   
   PITCH_llast=PITCH_last;
   PITCH_last=Angle[PITCH];
   PITCH_llast_err=PITCH_last-PITCH_llast;
   PITCH_last_err=Angle[PITCH]-PITCH_last;
   PITCHd=PITCH_last_err-PITCH_llast_err;
   
   
   ROLL_llast=ROLL_last;
   ROLL_last=Angle[ROLL];
   ROLL_llast_err=ROLL_last-ROLL_llast;
   ROLL_last_err=Angle[ROLL]-ROLL_last;
   ROLLd=ROLL_last_err-ROLL_llast_err;
   
   
   Xjiaoclout=Angle[YAW]*YAW_P+YAWd*YAW_D;
   Pjiaoclout=Angle[PITCH]*PITCH_P+PITCHd*PITCH_D;
   Sjiaoclout=Angle[ROLL]*ROLL_P+ROLLd*ROLL_D;
   
   outshezhi(Sjiaoclout,Pjiaoclout,Xjiaoclout);
}
/*******************dianjishuchuPWM****************************/
void outshezhi(float S,float P,float X ) 
{ 
    float PWM1,PWM2,PWM3,PWM4;
       PWM1=base1-S+P-X;
       PWM2=base2-S-P+X;
       PWM3=base3+S+P+X;
       PWM4=base4+S-P-X; 
     SetPWM(PWM1, PWM2, PWM3, PWM4);
}