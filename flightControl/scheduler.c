#include "scheduler.h"
#include "mpu6050.h"
#include "imu.h"
#include "led.h"
#include "data_transfer.h"
#include "ms5611.h"
#include "pwm.h"

uint32_t systime = 0;
uint16_t cnt_2ms,cnt_5ms,cnt_10ms,cnt_20ms,cnt_50ms,up_flag,down_flag;

static void Loop_500Hz(void)	//2ms
{	
		
}

static void Loop_200Hz(void)	//5ms
{
         zhilicl();    
}

static void Loop_100Hz(void)	//10ms
{
	Read_MPU6050();	

	Acc_LowPassFilter_1st(Acc.X,Acc.Y,Acc.Z);

	GetAttitude();	
}

static void Loop_50Hz(void)	//20ms
{       
      
	Baro_update();
	
	Pilot_Light();
}

static void Loop_20Hz(void)	//50ms
{	
	Data_Exchange();		
}

void Loop(void)
{
	
	if(cnt_2ms >= 2){
		Loop_500Hz();
		cnt_2ms = 0;
	}		
	if(cnt_5ms >= 5){	
		Loop_200Hz();
		cnt_5ms = 0;
	}
	if(cnt_10ms >= 10){		
		Loop_100Hz();
		cnt_10ms = 0;
	}
	if(cnt_20ms >= 20){		
		Loop_50Hz();
		cnt_20ms = 0;
	}
	if(cnt_50ms >= 50){		
		Loop_20Hz();
		cnt_50ms = 0;
	}
	if(systime >= 10000){
	R_TAU0_Channel0_Stop();
	R_TAU0_Channel7_Stop();
	}
	
	if(systime >= 15000){
   	up_flag=100;
        }
        if(systime >= 20000){
   	down_flag=100;
        }	

}

void SysTick_Handler(void)
{
	systime++;
	cnt_2ms++;
	cnt_5ms++;
	cnt_10ms++;
	cnt_20ms++;
	cnt_50ms++;

}

void delay(void)
{
	uint16_t i,j;
	for(i = 0;i<50000;i++)
		for(j = 0;j<50;j++);
}

void delayms(uint16_t ms)
{
	uint32_t now = systime;
	while (systime - now < ms);	
}

