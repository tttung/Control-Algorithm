/*==================================================================================================
	tb49final
	
	从UART4接收控制命令，根据控制命令在200ms内反馈对应的数据帧结果。
	从UART5定时接收二进制数据并解析和组装，供需要时发送。
	
	written by Jichen At Mar. 2016
==================================================================================================*/



/*--------------------------------------------------------------------------------------------------
	引用文件定义
--------------------------------------------------------------------------------------------------*/
#ifndef STM32F10x_HEADER
	#define STM32F10x_HEADER
	#include "stm32f10x.h"
#endif

#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "..\bsp\jccGPIO.h"
#include "..\bsp\jccTIM.h"
#include "..\bsp\jccUSART.h"
#include "..\bsp\jccNVIC.h"

#include "..\lib\jccProtocol.h"
#include "..\lib\jccCommon.h"
#include "..\lib\jccGPS.h"



/*--------------------------------------------------------------------------------------------------
	定义外部高速振荡器时钟
	这里外部晶振为8MHz
--------------------------------------------------------------------------------------------------*/
#define HSE_VALUE			((uint32_t)8000000)



/*--------------------------------------------------------------------------------------------------
	定义系统时钟SYSCLK
	
	在.s中Reset_Handler部分可以看到，MCU复位后首先执行的是SystemInit()函数。
	MDK中把这部分已经写好，设定不同的宏定义可以实现不同的RCC设置。
	因此复位后的系统时钟部分不再需要自己编写，只需根据需要设置如下宏定义：
	
	#define SYSCLK_FREQ_HSE		HSE_VALUE
	#define SYSCLK_FREQ_24MHz	24000000
	#define SYSCLK_FREQ_36MHz	36000000
	#define SYSCLK_FREQ_48MHz	48000000
	#define SYSCLK_FREQ_56MHz	56000000
	#define SYSCLK_FREQ_72MHz	72000000
	
	事实上IDE根据你所选芯片已经设置好了最常用的时钟参数如：8MHz振荡器和72MHz系统时钟。
--------------------------------------------------------------------------------------------------*/
#define SYSCLK_FREQ_72MHz	72000000



/*--------------------------------------------------------------------------------------------------
	宏定义
--------------------------------------------------------------------------------------------------*/
#define SIZE_RECVBUF1_LEN		4096									// 为了优化环形缓冲区指针运算，最好用2的整数次方。

#define LOOP1(val)				((val) & (SIZE_RECVBUF1_LEN - 1))		// 通过位运算获取缓冲区指针（序号），只取低N位，所以缓冲区大小必须是2的整数次方
#define NEXT1(pt)				(LOOP1((pt) + 1))
#define PREV1(pt)				(LOOP1((pt) - 1))

#define POS1(pt, pos)			(LOOP1((pt) + (pos)))
#define LEN1(hd, ft)			(ft >= hd ? ft - hd + 1 : SIZE_RECVBUF1_LEN + ft - hd + 1)





/*--------------------------------------------------------------------------------------------------
	全局变量定义
--------------------------------------------------------------------------------------------------*/
u8 g_bufUART1[SIZE_RECVBUF1_LEN];
u16 g_rdUART1, g_wrUART1;



// 这部分特别注意一下，在STM32系统中栈内存不能过大，更不要超过全局变量的堆内存大小，否则编译器生产的程序会莫名其妙的死掉。


USART1_dat usart1_dat;
Date_package date_package;



/*--------------------------------------------------------------------------------------------------
	全局函数
--------------------------------------------------------------------------------------------------*/
u16 Checksum16_LOOP(u8* buf, u16 pos, u16 len)
{
	u16 i, checksum16 = 0;
	
	for(i = 0; i < len; i++)
	{
		checksum16 += buf[POS1(pos, i)];
	}
	
	return checksum16;
}

u16 Checksum8_LOOP(u8* buf, u16 pos, u16 len)
{
	u16 i, checksum8 = 0;
	
	for(i = 0; i < len; i++)
	{
		checksum8 += buf[POS1(pos, i)];
	}
	
	return checksum8;
}


u8 date_checksum8_LOOP(u8* buf, u16 len)
{
	u8 checksum8 = 0;
	u16 i;
	
	for(i = 0; i < len ; i++)
	{
		checksum8 = checksum8 ^ buf[i];
	}
	
	return checksum8;
}


/*--------------------------------------------------------------------------------------------------
	系统主函数
	在SystemInit函数后才会执行
	use usart2 for GPS
	use uart4 for b49
	use stm32f103re
	PA2 <-> U2_TX(16)
	PA3 <-> U2_RX(17)
	PB10 <-> U3_TX(29)
	PB11 <-> U3_RX(30)
	PC10 <-> U4_TX(51)
	PC11 <-> U4_RX(52)
	PC12 <-> U5_TX(53)
	PD2  <-> U5_RX(54)
--------------------------------------------------------------------------------------------------*/
int main(void)
{
	u16 dupWrite, lenRead, sizeFrame, hd, ft;
	u16 errCHK5, errCHK4;
	u16 i, j, k;
	u32 checksum32;
	u16 checksum16;
	u8 checksum8 = 0;
	u8 find;
	u8 *pos, cmd, acc, sum, cnt;

	
	//
	jccNVICPriorityGroupConfig(2);
	jccNVICPriorityConfig(1, 1, USART1_IRQn, 2);		// 
	jccNVICPriorityConfig(1, 1, USART2_IRQn, 2);		// 
	jccNVICPriorityConfig(1, 1, USART3_IRQn, 2);		// 
	jccNVICPriorityConfig(1, 1, UART4_IRQn, 2);		// 
	jccNVICPriorityConfig(1, 1, UART5_IRQn, 2);		// 
	
	jccUSART1Init(72, 115200);
	jccUSART2Init(36, 115200);
	jccUSART3Init(36, 115200);
	jccUART4Init(36, 115200);
	jccUART5Init(36, 115200);
	
	// 初始化变量

	
	
								
	// 主循环
	while(1)
	{
		
		// 解析微电子所协议
		dupWrite = g_wrUART1;
		if(g_rdUART1 != dupWrite)
		{
			lenRead = LEN1(g_rdUART1, LOOP1(dupWrite - 1));		// 获取当前缓冲区中的数据量
			if(lenRead >= 26)									// 缓冲区中数据量至少要比最短帧的长度大
			{
				// 在数据中找帧头
				find = 0;
				for(i = 0; i <= lenRead - 6; i++)				// 没有必要找到最后，因为要在当前序号往后加5，形如：BA CE FF 00 01 02
				{
					if(0xEB == g_bufUART1[POS1(g_rdUART1, i)] && 0x90 == g_bufUART1[POS1(g_rdUART1, i + 1)]) 
					{
						//
						sizeFrame = g_bufUART1[POS1(g_rdUART1, i + 2)];	
						
						//
						
							hd = i;							// header idx
							ft = hd + sizeFrame - 1;	// footer idx
							
							// 缓冲区数据不够，等待更多数据
							if(ft >= lenRead)				
							{
								find = 1;
								g_rdUART1 = POS1(g_rdUART1, hd);
								break;
							}
							
							// 数据够长，那么校验数据
							checksum8 = Checksum8_LOOP(g_bufUART1, POS1(g_rdUART1, hd + 2), sizeFrame - 3);
							
						 if(	checksum8 == g_bufUART1[POS1(g_rdUART1, ft)])
  						{
//							if(checksum8 == g_bufUART1[POS1(g_rdUART1, ft)])		
//							{
//								for(j = hd; j <= ft; j++)
//								{
//									dr = g_bufUART1[POS1(g_rdUART1, j)];
//									jccUSART1TxChar(dr);
//								}
							
					
								// 拷贝数据
								pos = (u8*)&usart1_dat;
								for(j = hd; j <= ft; j++, pos++)
								{
									*pos = g_bufUART1[POS1(g_rdUART1, j)];
								}
								
								// 组装数据
								date_package.hdFrame[0] = 0xEB;
								date_package.hdFrame[1] = 0x90;
								date_package.lenFrame = 0x26;
								date_package.work_state = usart1_dat.work_state;
								date_package.numSV = usart1_dat.numSV;
								date_package.runtime = usart1_dat.runtime;
								date_package.msecond = usart1_dat.msecond;
								date_package.lon = usart1_dat.lon*1000000;
								date_package.lat = usart1_dat.lat*1000000;
								date_package.height = usart1_dat.height*10;
								date_package.velN = usart1_dat.velN*10;
								date_package.velU = usart1_dat.velU*10;
								date_package.velE = usart1_dat.velE*10;
								date_package.GDOP = usart1_dat.GDOP*100;		
								
						   	date_package.checksum = date_checksum8_LOOP((u8*)&date_package, sizeof(date_package));
								
								
						    jccUSART1TxBuf((u8*)&date_package, sizeof(date_package));
								
								//jccUSART1TxBuf((u8*)&usart1_dat, sizeof(usart1_dat));
								
								
								if(ft + 1 > lenRead - 6)
								{
									find = 2;
									g_rdUART1 = POS1(g_rdUART1, ft + 1);
									break;
								}
								else
								{
									i = ft;					// 指向尾部，由于i会自加，所以这里不要+1						
								}
							}	
					    else
    					{
								i = hd + 2;
							}
						}

				} // for len - HD1LEN
				
				// 没找到数据
				if(!find)
				{
					g_rdUART1 = POS1(g_rdUART1, (lenRead - 6 + 1));			
				} // !find
			} // if len >= 26
		} // if g_rdUART1 != g_wrUART1
		
		
	}
	
}




/*--------------------------------------------------------------------------------------------------
	中断服务函数
	
--------------------------------------------------------------------------------------------------*/

void USART1_IRQHandler(void)
{
	if(USART1->SR & 0x20)  //Received data is ready to be read
	{	
			
	  // get the recv byte
		g_bufUART1[g_wrUART1] = USART1->DR;
		
		// next byte
		g_wrUART1 = NEXT1(g_wrUART1);	
		
			
	}	
	
}

void USART2_IRQHandler(void)
{
if(USART2->SR & 0x20) //Received data is ready to be read
	{
	
	u8 dat = USART2->DR;
	 
  jccUSART2TxChar(dat);	
		
	}
}


void USART3_IRQHandler(void)
{
	if(USART3->SR & 0x20) //Received data is ready to be read
	{
		u8 dat = USART3->DR;
	 
	  jccUSART3TxChar(dat);
	}
}

// 与九院对接
void UART4_IRQHandler(void)
{
	if(UART4->SR & 0x20)   //Received data is ready to be read
	{
		u8 dat = UART4->DR;
	  
	  jccUART4TxChar(dat);
		
	}
}


// 与微电子所对接
void UART5_IRQHandler(void)
{
	
	if(UART5->SR & 0x20)  //Received data is ready to be read
	{
		u8 dat = UART5->DR;
	  
	  jccUART5TxChar(dat);
		
	}
	
}


