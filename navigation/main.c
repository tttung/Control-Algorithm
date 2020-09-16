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
#define SIZE_RECVBUF5_LEN		4096									// 为了优化环形缓冲区指针运算，最好用2的整数次方。

#define LOOP5(val)				((val) & (SIZE_RECVBUF5_LEN - 1))		// 通过位运算获取缓冲区指针（序号），只取低N位，所以缓冲区大小必须是2的整数次方
#define NEXT5(pt)				(LOOP5((pt) + 1))
#define PREV5(pt)				(LOOP5((pt) - 1))

#define POS5(pt, pos)			(LOOP5((pt) + (pos)))
#define LEN5(hd, ft)			(ft >= hd ? ft - hd + 1 : SIZE_RECVBUF5_LEN + ft - hd + 1)


#define SIZE_RECVBUF4_LEN		512										// 为了优化环形缓冲区指针运算，最好用2的整数次方。

#define LOOP4(val)				((val) & (SIZE_RECVBUF4_LEN - 1))		// 通过位运算获取缓冲区指针（序号），只取低N位，所以缓冲区大小必须是2的整数次方
#define NEXT4(pt)				(LOOP4((pt) + 1))
#define PREV4(pt)				(LOOP4((pt) - 1))

#define POS4(pt, pos)			(LOOP4((pt) + (pos)))
#define LEN4(hd, ft)			(ft >= hd ? ft - hd + 1 : SIZE_RECVBUF4_LEN + ft - hd + 1)



/*--------------------------------------------------------------------------------------------------
	全局变量定义
--------------------------------------------------------------------------------------------------*/
u8 g_bufUART5[SIZE_RECVBUF5_LEN];
u16 g_rdUART5, g_wrUART5;

u8 g_bufUART4[SIZE_RECVBUF4_LEN];
u16 g_rdUART4, g_wrUART4;

// 这部分特别注意一下，在STM32系统中栈内存不能过大，更不要超过全局变量的堆内存大小，否则编译器生产的程序会莫名其妙的死掉。
CSIP_NAVPV csipPV;
CSIP_NAVSOL csipSOL;
CSIP_GPSINFO_HEADER csipGpsHeader;
CSIP_GPSINFO_ITEM csipGpsItem[12];
CSIP_GPSION csipION;
CSIP_GPSEPH csipEPH;
	
CASIC_21H cas21;
CASIC_46H cas46;
CASIC_47H cas47[12];
CASIC_47H dupcas47[12];

u8 saCount, saList[12];
u32 g_cntRecv, g_cntFrame;


/*--------------------------------------------------------------------------------------------------
	全局函数
--------------------------------------------------------------------------------------------------*/
u16 CASIC_Checksum16_LOOP(u8* buf, u16 pos, u16 len)
{
	u16 i, checksum16 = 0;
	
	for(i = 0; i < len; i++)
	{
		checksum16 += buf[POS4(pos, i)];
	}
	
	return checksum16;
}

u32 CSIP_Checksum32_LOOP(u8* buf, u16 pos, u16 len)
{
	u32 checksum32 = 0;
	u16 i;
	
	for(i = 0; i < (len / 4); i++)
	{
		checksum32 += buf[POS5(pos, i * 4)] + (buf[POS5(pos, i * 4 + 1)] << 8) + (buf[POS5(pos, i * 4 + 2)] << 16) + (buf[POS5(pos, i * 4 + 3)] << 24);
	}
	
	return checksum32;
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
	u16 errCHK5, errCHK4, fra0102, fra0103, fra0120, fra0806, fra0807, send22H;
	u16 i, j, k;
	u32 checksum32;
	u16 checksum16;
	u8 find;
	u8 *pos, cmd, acc, sum, cnt;
	u8 idx36, idx37;
	u8 flagPOS, assbPOS, assbEPH, dupEPH;
	u16 gpsWN;
	u32 gpsTOW;
	
	//
	jccNVICPriorityGroupConfig(2);
	jccNVICPriorityConfig(1, 1, UART4_IRQn, 2);		// 
	jccNVICPriorityConfig(1, 1, UART5_IRQn, 2);		// 
	
	jccUART4Init(36, 115200);
	jccUART5Init(36, 115200);
	
	// 初始化变量
	flagPOS = 0;
	assbPOS = 0;
	assbEPH = 0;
	dupEPH = 0;
	saCount = 0;
	
	errCHK5 = 0;
	errCHK4 = 0;
	fra0102 = 0;
	fra0103 = 0; 
	fra0120 = 0; 
	fra0806 = 0;
	fra0807 = 0;
	send22H = 0;
	
	// 主循环
	while(1)
	{
		// 解析微电子所协议
		dupWrite = g_wrUART5;
		if(g_rdUART5 != dupWrite)
		{
			lenRead = LEN5(g_rdUART5, LOOP5(dupWrite - 1));		// 获取当前缓冲区中的数据量
			if(lenRead >= 26)									// 缓冲区中数据量至少要比最短帧的长度大
			{
				// 在数据中找帧头
				find = 0;
				for(i = 0; i <= lenRead - 6; i++)				// 没有必要找到最后，因为要在当前序号往后加5，形如：BA CE FF 00 01 02
				{
					if(0xBA == g_bufUART5[POS5(g_rdUART5, i)] && 0xCE == g_bufUART5[POS5(g_rdUART5, i + 1)])
					{
						//
						sizeFrame = g_bufUART5[POS5(g_rdUART5, i + 2)] + (g_bufUART5[POS5(g_rdUART5, i + 3)] << 8);	
						
						//
						if(0x01 == g_bufUART5[POS5(g_rdUART5, i + 4)] && 0x02 == g_bufUART5[POS5(g_rdUART5, i + 5)])			// 找到01 02帧头，它的size=72
						{
							hd = i;							// header idx
							ft = hd + sizeFrame + 10 - 1;	// footer idx
							
							// 缓冲区数据不够，等待更多数据
							if(ft >= lenRead)				
							{
								find = 1;
								g_rdUART5 = POS5(g_rdUART5, hd);
								break;
							}
							
							// 数据够长，那么校验数据
							checksum32 = CSIP_Checksum32_LOOP(g_bufUART5, POS5(g_rdUART5, hd + 2), sizeFrame + 4);
							if(checksum32 == (g_bufUART5[POS5(g_rdUART5, ft)] << 24) + (g_bufUART5[POS5(g_rdUART5, ft - 1)] << 16) + 
								(g_bufUART5[POS5(g_rdUART5, ft - 2)] << 8) + (g_bufUART5[POS5(g_rdUART5, ft - 3)]))		
							{
//								for(j = hd; j <= ft; j++)
//								{
//									dr = g_bufUART5[POS5(g_rdUART5, j)];
//									jccUART5TxChar(dr);
//								}
								fra0102++;
								g_cntFrame += sizeFrame;
								
								// 拷贝数据
								pos = (u8*)&csipSOL;
								for(j = hd; j <= ft; j++, pos++)
								{
									*pos = g_bufUART5[POS5(g_rdUART5, j)];
								}
								
								// 组装数据
								gpsWN = csipSOL.week & 0x3FF;
								gpsTOW = (u32)(csipSOL.tow / 6);
								cas21.secWeek = csipSOL.tow;
								cas21.valWeek = csipSOL.week;
								if(6 == csipSOL.posValid || 7 == csipSOL.posValid)
								{
									cas21.valMode = 0xF;
								}
								else
								{
									cas21.valMode = csipSOL.posValid;
								}
								
								assbPOS = 1;
								
								//
								if(ft + 1 > lenRead - 6)
								{
									find = 2;
									g_rdUART5 = POS5(g_rdUART5, ft + 1);
									break;
								}
								else
								{
									i = ft;					// 指向尾部，由于i会自加，所以这里不要+1						
								}
							}
							else
							{
								errCHK5++;
								
								//
								i = hd + 2;
							}
						}
						else if(0x01 == g_bufUART5[POS5(g_rdUART5, i + 4)] && 0x03 == g_bufUART5[POS5(g_rdUART5, i + 5)])		// 找到01 03帧头，它的size=80
						{
							hd = i;							// header idx
							ft = hd + sizeFrame + 10 - 1;	// footer idx
							if(ft >= lenRead)				// 缓冲区数据不够，等待更多数据
							{
								find = 1;
								g_rdUART5 = POS5(g_rdUART5, hd);
								break;
							}
							
							// 数据够长，那么校验数据	
							checksum32 = CSIP_Checksum32_LOOP(g_bufUART5, POS5(g_rdUART5, hd + 2), sizeFrame + 4);
							if(checksum32 == (g_bufUART5[POS5(g_rdUART5, ft)] << 24) + (g_bufUART5[POS5(g_rdUART5, ft - 1)] << 16) + 
								(g_bufUART5[POS5(g_rdUART5, ft - 2)] << 8) + (g_bufUART5[POS5(g_rdUART5, ft - 3)]))									
							{
//								for(j = hd; j <= ft; j++, pos++)
//								{
//									dr = g_bufUART5[POS5(g_rdUART5, j)];
//									jccUART5TxChar(dr);
//								}
								fra0103++;
								g_cntFrame += sizeFrame;
								
								// 拷贝数据
								pos = (u8*)&csipPV;
								for(j = hd; j <= ft; j++, pos++)
								{
									*pos = g_bufUART5[POS5(g_rdUART5, j)];
								}
								
								// 组装数据
								cas21.valPDOP = (u16)(csipPV.pDop * 10);
								cas21.valLon = csipPV.lon * pi / 180.0;
								cas21.valLat = csipPV.lat * pi / 180.0;
								cas21.h = csipPV.height;
								cas21.Vn = csipPV.velN;
								cas21.Ve = csipPV.velE;
								cas21.Vu = csipPV.velU;
								assbPOS++;
								
								//
								if(ft + 1 > lenRead - 6)
								{
									find = 2;
									g_rdUART5 = POS5(g_rdUART5, ft + 1);
									break;
								}
								else
								{
									i = ft;					// 指向尾部，由于i会自加，所以这里不要+1	
								}
							}
							else
							{
								errCHK5++;
								
								//
								i = hd + 2;
							}
						}
						else if(0x01 == g_bufUART5[POS5(g_rdUART5, i + 4)] && 0x20 == g_bufUART5[POS5(g_rdUART5, i + 5)])		// 找到01 20帧头，它的size=8+12N
						{
							hd = i;							// header idx
							ft = hd + sizeFrame + 10 - 1;	// footer idx
							if(ft >= lenRead)				// 缓冲区数据不够，等待更多数据
							{
								find = 1;
								g_rdUART5 = POS5(g_rdUART5, hd);
								break;
							}
							
							// 数据够长，那么校验数据	
							checksum32 = CSIP_Checksum32_LOOP(g_bufUART5, POS5(g_rdUART5, hd + 2), sizeFrame + 4);
							if(checksum32 == (g_bufUART5[POS5(g_rdUART5, ft)] << 24) + (g_bufUART5[POS5(g_rdUART5, ft - 1)] << 16) + 
								(g_bufUART5[POS5(g_rdUART5, ft - 2)] << 8) + (g_bufUART5[POS5(g_rdUART5, ft - 3)]))		
							{
//								for(j = hd; j <= ft; j++, pos++)
//								{
//									dr = g_bufUART5[POS5(g_rdUART5, j)];
//									jccUART5TxChar(dr);
//								}
								fra0120++;
								g_cntFrame += sizeFrame;
								
								// 拷贝数据
								acc = sizeof(CSIP_GPSINFO_HEADER);
								pos = (u8*)&csipGpsHeader;
								for(j = 0; j < acc; j++, pos++)
								{
									*pos = g_bufUART5[POS5(g_rdUART5, hd + j)];
								}
								
								// 中科微模块可见星数量有可能出现大于12的情况（这时有低仰角的星出现），只能通过仰角来控制其有效性
								cnt = csipGpsHeader.numViewSv;

								// 初始化记录
								saCount = 0;
								for(j = 0; j < 12; j++)
								{
									saList[j] = 0;
								}
								for(j = 0; j < 60; j++)
								{
									cas21.chnSat[j] = 0;
								}
								
								//
								hd += acc;
								sum = sizeof(CSIP_GPSINFO_ITEM);
								for(j = 0; j < cnt; j++)
								{
									// 拷贝一个通道信息
									pos = (u8*)&csipGpsItem[j];
									for(k = 0; k < sum; k++, pos++)
									{
										*pos = g_bufUART5[POS5(g_rdUART5, hd + j * sum + k)];
									}
									
									// 组装数据，通过仰角来过滤通道卫星数量，否则会出现大于12的情况
									if(csipGpsItem[j].elev > 5)
									{
										cas21.chnSat[j * 5] = csipGpsItem[j].svid;
										cas21.chnSat[j * 5 + 1] = csipGpsItem[j].flags;
										cas21.chnSat[j * 5 + 2] = csipGpsItem[j].elev + 128;
										cas21.chnSat[j * 5 + 3] = csipGpsItem[j].azim / 2;
										cas21.chnSat[j * 5 + 4] = csipGpsItem[j].CN0;
										
										// 累加参与定位的卫星数据
										if(13 == csipGpsItem[j].flags)
										{
											// 记录所有可见卫星
											saList[saCount] = csipGpsItem[j].svid;
											
											saCount++;
										}
									}
								} // for j < cnt
								
								// 组装数据
								cas21.hdFrame[0] = '$';
								cas21.hdFrame[1] = 'B';
								cas21.hdFrame[2] = 'I';
								cas21.hdFrame[3] = 'N';
								cas21.idFrame = 0x21;
								cas21.lenFrame = 107;
								cas21.ftFrame[0] = 0x0D;
								cas21.ftFrame[1] = 0x0A;
								cas21.satActive = saCount;
								
								pos = (u8*)&cas21;
								pos += 8;
								checksum16 = 0;
								for(j = 0; j < 107; j++, pos++)		// calculate checksum
								{
									checksum16 += *pos;
								}
								cas21.checksum = checksum16;	
								
								assbPOS++;							// 组装完成
								
								//
								if(1 == flagPOS && 3 == assbPOS)
								{
									send22H++;
									jccUART4TxBuf((u8*)&cas21, sizeof(CASIC_21H));
									
									//
//									jccUART4TxChar(fra0102 & 0xFF);
//									jccUART4TxChar((fra0102 >> 8) & 0xFF);
//									jccUART4TxChar(fra0103 & 0xFF);
//									jccUART4TxChar((fra0103 >> 8) & 0xFF);
//									jccUART4TxChar(fra0120 & 0xFF);
//									jccUART4TxChar((fra0120 >> 8) & 0xFF);
//									jccUART4TxChar(fra0806 & 0xFF);
//									jccUART4TxChar((fra0806 >> 8) & 0xFF);
//									jccUART4TxChar(fra0807 & 0xFF);
//									jccUART4TxChar((fra0807 >> 8) & 0xFF);
//									jccUART4TxChar(send22H & 0xFF);
//									jccUART4TxChar((send22H >> 8) & 0xFF);
//									jccUART4TxChar(0x0D);
//									jccUART4TxChar(0x0A);
									
//									jccUART4TxChar(g_cntFrame & 0xFF);
//									jccUART4TxChar((g_cntFrame >> 8) & 0xFF);
//									jccUART4TxChar((g_cntFrame >> 16) & 0xFF);
//									jccUART4TxChar((g_cntFrame >> 24) & 0xFF);
//									jccUART4TxChar(g_cntRecv & 0xFF);
//									jccUART4TxChar((g_cntRecv >> 8) & 0xFF);
//									jccUART4TxChar((g_cntRecv >> 16) & 0xFF);
//									jccUART4TxChar((g_cntRecv >> 24) & 0xFF);
//									jccUART4TxChar(errCHK4 & 0xFF); 		// 输出“九院指令校验错误”的计数低字节
//									jccUART4TxChar((errCHK4 >> 8) & 0xFF); 	// 输出“九院指令校验错误”的计数高字节	
//									jccUART4TxChar(errCHK5 & 0xFF); 		// 输出“九院指令校验错误”的计数低字节
//									jccUART4TxChar((errCHK5 >> 8) & 0xFF); 	// 输出“九院指令校验错误”的计数高字节	
//									jccUART4TxChar(0x0D);
//									jccUART4TxChar(0x0A);	
								}
											
								assbPOS = 0;
								
								//
								if(ft + 1 > lenRead - 6)
								{
									find = 2;
									g_rdUART5 = POS5(g_rdUART5, ft + 1);
									break;
								}
								else
								{
									i = ft;					// 指向尾部，由于i会自加，所以这里不要+1									
								}
							}
							else
							{
								errCHK5++;
								
								//
								i = hd + 2;
							}
						}
						else if(0x08 == g_bufUART5[POS5(g_rdUART5, i + 4)] && 0x06 == g_bufUART5[POS5(g_rdUART5, i + 5)])		// 找到08 06帧头，它的size=16
						{
							// 先收到08 07 再收到08 06，因此收到08 06代表收完08 07
							// 组装好了各个卫星的星历帧数据，记录卫星星历个数，等待有请求时再发
							if(assbEPH)									
							{			
								dupEPH = (assbEPH > 12 ? 12 : assbEPH);
								
								for(j = 0; j < dupEPH; j++)
								{						
									dupcas47[j] = cas47[j];
								}
								
								assbEPH = 0;
								
//								// 为了200ms以内响应位置请求，立刻把当前星历信息发送出去
//								for(j = 0; j < dupEPH; j++)
//								{
//									dupcas47[j].idxFrame = 0;
//									
//									pos = (u8*)&dupcas47[j];
//									pos += 8;
//									checksum16 = 0;
//									for(k = 0; k < 122; k++, pos++)			// calculate checksum
//									{
//										checksum16 += *pos;
//									}
//									dupcas47[j].checksum = checksum16;
//									
//									jccUART4TxBuf((u8*)&dupcas47[j], sizeof(CASIC_47H));
//								} // for
							}
								
							//
							hd = i;						// header idx
							ft = hd + sizeFrame + 10 - 1;	// footer idx
							if(ft >= lenRead)				// 缓冲区数据不够，等待更多数据
							{
								find = 1;
								g_rdUART5 = POS5(g_rdUART5, hd);
								break;
							}
							
							// 数据够长，那么校验数据	
							checksum32 = CSIP_Checksum32_LOOP(g_bufUART5, POS5(g_rdUART5, hd + 2), sizeFrame + 4);
							if(checksum32 == (g_bufUART5[POS5(g_rdUART5, ft)] << 24) + (g_bufUART5[POS5(g_rdUART5, ft - 1)] << 16) + 
								(g_bufUART5[POS5(g_rdUART5, ft - 2)] << 8) + (g_bufUART5[POS5(g_rdUART5, ft - 3)]))		
							{
//								for(j = hd; j <= ft; j++, pos++)
//								{
//									dr = g_bufUART5[POS5(g_rdUART5, j)];
//									jccUART5TxChar(dr);
//								}
								fra0806++;
								g_cntFrame += sizeFrame;
								
								// 拷贝数据
								pos = (u8*)&csipION;
								for(j = hd; j <= ft; j++, pos++)
								{
									*pos = g_bufUART5[POS5(g_rdUART5, j)];
								}
								
								// 组装数据
								cas46.hdFrame[0] = '$';
								cas46.hdFrame[1] = 'B';
								cas46.hdFrame[2] = 'I';
								cas46.hdFrame[3] = 'N';
								cas46.idFrame = 0x46;
								cas46.lenFrame = 66;
								cas46.ftFrame[0] = 0x0D;
								cas46.ftFrame[1] = 0x0A;
								cas46.idxFrame = 0;
								
								cas46.alpha0 = csipION.alpha[0] / 1073741824.0;
								cas46.alpha1 = csipION.alpha[1] / 134217728.0;
								cas46.alpha2 = csipION.alpha[2] / 16777216.0;
								cas46.alpha3 = csipION.alpha[3] / 16777216.0;
								cas46.beta0 = csipION.beta[0] * 2048.0;
								cas46.beta1 = csipION.beta[1] * 16384.0;
								cas46.beta2 = csipION.beta[2] * 65536.0;
								cas46.beta3 = csipION.beta[3] * 65536.0;
								cas46.flagFrame = (3 == csipION.valid ? 1 : 0);
								
								//
								if(ft + 1 > lenRead - 6)
								{
									find = 2;
									g_rdUART5 = POS5(g_rdUART5, ft + 1);
									break;
								}
								else
								{
									i = ft;					// 指向尾部，由于i会自加，所以这里不要+1						
								}
							}
							else
							{
								errCHK5++;
								
								//
								i = hd + 2;
							}
						}
						else if(0x08 == g_bufUART5[POS5(g_rdUART5, i + 4)] && 0x07 == g_bufUART5[POS5(g_rdUART5, i + 5)])		// 找到08 07帧头，它的size=72
						{
							hd = i;							// header idx
							ft = hd + sizeFrame + 10 - 1;	// footer idx
							if(ft >= lenRead)				// 缓冲区数据不够，等待更多数据
							{
								find = 1;
								g_rdUART5 = POS5(g_rdUART5, i);
								break;
							}
							
							// 数据够长，那么校验数据	
							checksum32 = CSIP_Checksum32_LOOP(g_bufUART5, POS5(g_rdUART5, hd + 2), sizeFrame + 4);
							if(checksum32 == (g_bufUART5[POS5(g_rdUART5, ft)] << 24) + (g_bufUART5[POS5(g_rdUART5, ft - 1)] << 16) + 
								(g_bufUART5[POS5(g_rdUART5, ft - 2)] << 8) + (g_bufUART5[POS5(g_rdUART5, ft - 3)]))		
							{
//								for(j = hd; j <= ft; j++, pos++)
//								{
//									dr = g_bufUART5[POS5(g_rdUART5, j)];
//									jccUART5TxChar(dr);
//								}
								fra0807++;
								g_cntFrame += sizeFrame;
								
								// 拷贝数据
								pos = (u8*)&csipEPH;
								for(j = hd; j <= ft; j++, pos++)
								{
									*pos = g_bufUART5[POS5(g_rdUART5, j)];
								}
								
								// 先保证当前星的星历有效
								if(3 == csipEPH.valid)
								{					
									for(j = 0; j < saCount; j++)
									{
										// 当前星历为参与解算卫星的星历，并且仰角大于5度
										if(saList[j] == csipEPH.svid)
										{
											// 组装数据
											cas47[assbEPH].hdFrame[0] = '$';
											cas47[assbEPH].hdFrame[1] = 'B';
											cas47[assbEPH].hdFrame[2] = 'I';
											cas47[assbEPH].hdFrame[3] = 'N';
											cas47[assbEPH].idFrame = 0x47;
											cas47[assbEPH].lenFrame = 122;
											cas47[assbEPH].ftFrame[0] = 0x0D;
											cas47[assbEPH].ftFrame[1] = 0x0A;
											cas47[assbEPH].idxFrame = 0;
											cas47[assbEPH].idSat = csipEPH.svid;
											
											acc = CASIC_MakeTLW(0, (u32*)&cas47[assbEPH].wordFrame1[0], 0x8B, 0x347);
											acc = CASIC_MakeHOW(acc, (u32*)&cas47[assbEPH].wordFrame1[1], gpsTOW, 1);
											acc = CASIC_MakeSub1Word3(0, (u32*)&cas47[assbEPH].wordFrame1[2], gpsWN, csipEPH.ura, csipEPH.health, csipEPH.iodc);
											acc = CASIC_MakeSub1Word4(acc, (u32*)&cas47[assbEPH].wordFrame1[3]);
											acc = CASIC_MakeSub1Word5(acc, (u32*)&cas47[assbEPH].wordFrame1[4]);
											acc = CASIC_MakeSub1Word6(acc, (u32*)&cas47[assbEPH].wordFrame1[5]);
											acc = CASIC_MakeSub1Word7(acc, (u32*)&cas47[assbEPH].wordFrame1[6], csipEPH.tgd);
											acc = CASIC_MakeSub1Word8(acc, (u32*)&cas47[assbEPH].wordFrame1[7], csipEPH.iodc, csipEPH.toc);
											acc = CASIC_MakeSub1Word9(acc, (u32*)&cas47[assbEPH].wordFrame1[8], csipEPH.af2, csipEPH.af1);
											acc = CASIC_MakeSub1Word10(acc, (u32*)&cas47[assbEPH].wordFrame1[9], csipEPH.af0);
											
											acc = CASIC_MakeTLW(0, (u32*)&cas47[assbEPH].wordFrame2[0], 0x8B, 0x347);
											acc = CASIC_MakeHOW(acc, (u32*)&cas47[assbEPH].wordFrame2[1], gpsTOW, 2);
											acc = CASIC_MakeSub2Word3(0, (u32*)&cas47[assbEPH].wordFrame2[2], csipEPH.iodc & 0xFF, csipEPH.crs);
											acc = CASIC_MakeSub2Word4(acc, (u32*)&cas47[assbEPH].wordFrame2[3], csipEPH.deltan, csipEPH.M0);
											acc = CASIC_MakeSub2Word5(acc, (u32*)&cas47[assbEPH].wordFrame2[4], csipEPH.M0);
											acc = CASIC_MakeSub2Word6(acc, (u32*)&cas47[assbEPH].wordFrame2[5], csipEPH.cuc, csipEPH.es);
											acc = CASIC_MakeSub2Word7(acc, (u32*)&cas47[assbEPH].wordFrame2[6], csipEPH.es);
											acc = CASIC_MakeSub2Word8(acc, (u32*)&cas47[assbEPH].wordFrame2[7], csipEPH.cus, csipEPH.sqra);
											acc = CASIC_MakeSub2Word9(acc, (u32*)&cas47[assbEPH].wordFrame2[8], csipEPH.sqra);
											acc = CASIC_MakeSub2Word10(acc, (u32*)&cas47[assbEPH].wordFrame2[9], csipEPH.toe);
											
											acc = CASIC_MakeTLW(0, (u32*)&cas47[assbEPH].wordFrame3[0], 0x8B, 0x347);
											acc = CASIC_MakeHOW(0, (u32*)&cas47[assbEPH].wordFrame3[1], gpsTOW, 3);
											acc = CASIC_MakeSub3Word3(0, (u32*)&cas47[assbEPH].wordFrame3[2], csipEPH.cic, csipEPH.OMEGA0);
											acc = CASIC_MakeSub3Word4(0, (u32*)&cas47[assbEPH].wordFrame3[3], csipEPH.OMEGA0);
											acc = CASIC_MakeSub3Word5(0, (u32*)&cas47[assbEPH].wordFrame3[4], csipEPH.cis, csipEPH.i0);
											acc = CASIC_MakeSub3Word6(0, (u32*)&cas47[assbEPH].wordFrame3[5], csipEPH.i0);
											acc = CASIC_MakeSub3Word7(0, (u32*)&cas47[assbEPH].wordFrame3[6], csipEPH.crc, csipEPH.omega);
											acc = CASIC_MakeSub3Word8(0, (u32*)&cas47[assbEPH].wordFrame3[7], csipEPH.omega);
											acc = CASIC_MakeSub3Word9(0, (u32*)&cas47[assbEPH].wordFrame3[8], csipEPH.OMEGA);
											acc = CASIC_MakeSub3Word10(0, (u32*)&cas47[assbEPH].wordFrame3[9], csipEPH.iodc & 0xFF, csipEPH.IDOT);
											
											assbEPH++;	// 组装完成
													
											break;
										}
									} // for
								} // if
								
								//
								if(ft + 1 > lenRead - 6)
								{
									find = 2;
									g_rdUART5 = POS5(g_rdUART5, ft + 1);
									break;
								}
								else
								{
									i = ft;					// 指向尾部，由于i会自加，所以这里不要+1						
								}
							}
							else
							{
								errCHK5++;
								
								//
								i = hd + 2;
							}
						} // else if 08 07
					} // if BA CE
				} // for len - HD1LEN
				
				// 没找到数据
				if(!find)
				{
					g_rdUART5 = POS5(g_rdUART5, (lenRead - 6 + 1));			
				} // !find
			} // if len >= 26
		} // if g_rdUART5 != g_wrUART5
		
		
		// 解析九院指令
		dupWrite = g_wrUART4;
		if(g_rdUART4 != dupWrite)
		{
			lenRead = LEN4(g_rdUART4, LOOP4(dupWrite - 1));	// 获取当前缓冲区中的数据量
			if(lenRead >= 13)
			{
				// 在数据中找帧头
				find = 0;
				for(i = 0; i <= lenRead - 8; i++)		// 没有必要找到最后，因为要再当前序号往后加7，形如：$BINXXXX
				{
					if('$' == g_bufUART4[POS4(g_rdUART4, i)] && 'B' == g_bufUART4[POS4(g_rdUART4, i + 1)] && 
						'I' == g_bufUART4[POS4(g_rdUART4, i + 2)] && 'N' == g_bufUART4[POS4(g_rdUART4, i + 3)])
					{
						sizeFrame = g_bufUART4[POS4(g_rdUART4, i + 6)];
						
						hd = i;						// header idx
						ft = hd + sizeFrame + 12 - 1;	// footer idx
						if(ft >= lenRead)				// 缓冲区数据不够，等待更多数据
						{
							find = 1;
							g_rdUART4 = POS4(g_rdUART4, hd);
							break;
						}
						
						checksum16 = CASIC_Checksum16_LOOP(g_bufUART4, POS4(g_rdUART4, hd + 8), sizeFrame);
						if(checksum16 == (g_bufUART4[POS4(g_rdUART4, ft - 2)] << 8) + (g_bufUART4[POS4(g_rdUART4, ft - 3)]))		// check OK
						{
							if(0x22 == (g_bufUART4[POS4(g_rdUART4, hd + 4)]) + (g_bufUART4[POS4(g_rdUART4, hd + 5)] << 8))			// 22H = POS
							{
								cmd = g_bufUART4[POS4(g_rdUART4, hd + 8)];
								if(cmd)
								{
									if(0 == flagPOS)
									{
										// 为了200ms以内响应位置请求，立刻把当前定位信息发送出去
										jccUART4TxBuf((u8*)&cas21, sizeof(CASIC_21H));
										
										flagPOS = 1;							
									}
								}
							}
							else if(0x36 == (g_bufUART4[POS4(g_rdUART4, hd + 4)]) + (g_bufUART4[POS4(g_rdUART4, hd + 5)] << 8))		// 36H = ION
							{
								idx36 = g_bufUART4[POS4(g_rdUART4, hd + 8)];
								cmd = g_bufUART4[POS4(g_rdUART4, hd + 9)];
								if(cmd)
								{
									// 为了200ms以内响应位置请求，立刻把当前电离层信息发送出去
									cas46.idxFrame = idx36;
									
									pos = (u8*)&cas46;
									pos += 8;
									checksum16 = 0;
									for(j = 0; j < 66; j++, pos++)			// calculate checksum
									{
										checksum16 += *pos;
									}
									cas46.checksum = checksum16;
									
									jccUART4TxBuf((u8*)&cas46, sizeof(CASIC_46H));
								}
							}
							else if(0x37 == (g_bufUART4[POS4(g_rdUART4, hd + 4)]) + (g_bufUART4[POS4(g_rdUART4, hd + 5)] << 8))		// 37H = EPH
							{
								idx37 = g_bufUART4[POS4(g_rdUART4, hd + 8)];
								cmd = g_bufUART4[POS4(g_rdUART4, hd + 9)];
								if(cmd)
								{
									// 为了200ms以内响应位置请求，立刻把当前星历信息发送出去
									for(j = 0; j < dupEPH; j++)
									{
										dupcas47[j].idxFrame = idx37;
										
										pos = (u8*)&dupcas47[j];
										pos += 8;
										checksum16 = 0;
										for(k = 0; k < 122; k++, pos++)			// calculate checksum
										{
											checksum16 += *pos;
										}
										dupcas47[j].checksum = checksum16;
												
										jccUART4TxBuf((u8*)&dupcas47[j], sizeof(CASIC_47H));
									} // for
								} // if
							}
							else if(0xEE == (g_bufUART4[POS4(g_rdUART4, hd + 4)]) + (g_bufUART4[POS4(g_rdUART4, hd + 5)] << 8))		// EEH = ERR		
							{
								jccUART4TxChar(errCHK4 & 0xFF); 		// 输出“九院指令校验错误”的计数低字节
								jccUART4TxChar((errCHK4 >> 8) & 0xFF); 	// 输出“九院指令校验错误”的计数高字节	
								jccUART4TxChar(errCHK5 & 0xFF); 		// 输出“九院指令校验错误”的计数低字节
								jccUART4TxChar((errCHK5 >> 8) & 0xFF); 	// 输出“九院指令校验错误”的计数高字节	
								jccUART4TxChar(0x0D);
								jccUART4TxChar(0x0A);						
							}
						}
						else
						{
							errCHK4++;
						}
						
						//
						if(ft + 1 > lenRead - 8)
						{
							find = 2;
							g_rdUART4 = POS4(g_rdUART4, ft + 1);
							break;
						}
						else
						{
							i = ft;					// 指向尾部，由于i会自加，所以这里不要+1						
						}
					}
				}
				
				// 没找到数据
				if(!find)
				{
					g_rdUART4 = POS4(g_rdUART4, (lenRead - 8 + 1));			
				} // if !find
			} // if len >= 13
		} // if g_rdUART4 != g_wrUART4
		
	} // while main
}



/*--------------------------------------------------------------------------------------------------
	中断服务函数
	
--------------------------------------------------------------------------------------------------*/
// 与九院对接
void UART4_IRQHandler(void)
{
	if(UART4->SR & 0x20)
	{
		// get the recv byte
		g_bufUART4[g_wrUART4] = UART4->DR;
		
		// next byte
		g_wrUART4 = NEXT4(g_wrUART4);
	}
}



// 与微电子所对接
void UART5_IRQHandler(void)
{
	if(UART5->SR & 0x20)
	{
		// get the recv byte
		g_bufUART5[g_wrUART5] = UART5->DR;
		
		// next byte
		g_wrUART5 = NEXT5(g_wrUART5);
		
		//g_cntRecv++;
	}
}


