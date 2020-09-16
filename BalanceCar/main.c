#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "LQ12864.h"
#define   deadout  10
#define   maxout   3000
#define   minout   -3000
//����//
#define PRINT_AD        (1)       //���ô��ڴ�ӡ�������ͣ�0:��ӡ��ֵ�����ݣ�1����ӡADֵ
#define WINDOW_WIDTH    (128)     //���ô��ڴ�ӡ�����ظ��������128����С0
float OutData[4] = { 0 };
//�Ƕȿ���
#define   zhonglijiaochangshu  0.1871
#define   jiaosuduchangshu     0.062
#define   buchangxishu         4
#define   jiaochangshu         0.05
#define   jiaoduyushe          1300
float G,left,right,chejiao,zjiaodu,jiaosudu,jiaojifen,jiaoclout,Z,g,z,jiaosuduyushe;
//�ٶȿ���
#define   CARSPEEDCHANGLIANG   0.02
#define   SPEEDCZZQ            0.01
float carspeedyushe,WTCARSPEEDYUSHE,CARSPEEDYUSHE,TCARSPEEDYUSHE,speedtjzq,cd;
float speedout,carspeed,maichong,OLDspeedout,NEWspeedout,speedczjf,suduczzq,sududuzhi;
int Emaichong,CD;
//CCD
#define TSL1401_SI(x)   (PTH_PTH6=(x))
#define TSL1401_CLK(x)  (PTH_PTH7=(x))
uint ADCValue(uchar channel);
uint ADCValue1(uchar channel);
void TSL1401_GetLine(uchar *pixel);
void delay();
uchar gPixel[128] = {0};
//�ܵ�����
long int yuzhi,max,min;
float zuidiyuzhi=20;
float zuidayuzhi=240;
uchar SS[128]=0;
float zuo_flag,you_flag,zhongxian,duquzhongxian,bianduqu;
float  fangxiang,newchazhiout,oldchazhiout,d_chazhi,yunxu=0,Right,Left,oldRight,oldLeft;
float  fangxiangzhouqi,fangxiangzq;
float  Right1,Left1,zuo_flag1,you_flag1,newchazhi,oldchazhi;
long int yeshu=0,qisu,anjian;
//���ڲ���//
float WfangxiangP=32,WfangxiangD=1.45,fangxiangP=30,fangxiangD=1.45;
float speedI=10,speedP=500,jiaodu_P=145,jiaodu_D=10;


//������ƺ���//
void Dis_Num(byte x,byte y , uint num,byte N);
void yejing(void);
void Dly_ms(int ms);
void shedig(void);
void outshezhi(float left,float right);
void dianjisudu(void);
void dianjiout(void);
void zhilicl(void);
void jiaodujisuan(void);
void Speed_Read(void);
void speed_out(void);
void speedcontrol(void);
void ADduqu(void);
void dongtaiyuzhi(void);
void erzhihua(void);
void zuobiao(void);
void ZXJS(void);
void qphzj(void);
void paodaoshuchu(void);
int abs(int data);
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void Dis_Float(byte Y,byte X,double real,byte N);
unsigned char uart_getchar(void);
volatile unsigned int TimeCount[5] = {0};
volatile unsigned int Set_Count = 0,setcount = 0;
//Ӳ���趨����//  
void IO(void);
void PWM(void);
void ATD(void);
void PLL(void);
void TimInit (void);
void PIT(void);
void UART0_Init(void);
void UART0_SendByte(byte ch);
void UART0_SendPacket(byte *pBuf,int pBuf_Length);
void uart_putchar ( unsigned char c);
void uart_putstr(char ch[]);
unsigned char uart_getchar(void);
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void OutPut_Data(void);
void update(void);

void main(void) 
{ 
 DisableInterrupts;
 duquzhongxian=64;
 TCARSPEEDYUSHE=21;
 WTCARSPEEDYUSHE=21;
 TimInit(); 
 PWM();
 PLL();
 ATD();
 PIT();
 IO();
 UART0_Init();
 yejing();
 EnableInterrupts;
}
int abs(int data) 
{
   int date;
   if(data>=0)
    date=data;
   else
    date=-data;
   return date;
}
void IO(void) 
{  
  DDRJ=0x00; //����IOΪ����
  DDRK=0x00; //����IOΪ����
  DDRB=0x00; //����IOΪ����
  DDRM=0x00; //����IOΪ����
  PERM=0xff; //����M������ʹ������������
  PPSM=0x00; //����M������ʹ����������
  DDRH=0xff; //����IOΪ���
  PTH_PTH7= 0; //����IO������Ϊ0
  PTH_PTH6= 0; //����IO������Ϊ0
}
void PWM(void) //PWM��ʼ��
{  
    PWME=0;
    PWMCAE=0;
    PWMPOL=0Xaa;
    PWMPRCLK=0X00;
    PWMCTL=0Xf0;
    PWMSCLA=1;
    PWMSCLB=1;
    PWMCLK=0Xaa;
    PWMPER67=3000;
    PWMPER45=3000;
    PWMPER23=3000;
    PWMPER01=3000;  
    PWME=0Xaa;       
}
//AD��ʼ����AD��ȡ//
void ATD(void) 
{
  ATD0CTL4 = 0x0F; 
  ATD0CTL3 = 0x88;    
  ATD0CTL2 = 0x40;  
  ATD0DIEN = 0x00;
}
uint ADCValue(uchar channel)
{
  //�ݴ�A/Dת���Ľ��
  uint temp;
  ATD0CTL1 = 0x4F;                         
	ATD0CTL5 = channel;
	//ȡA/Dת�����                                  
  while(!ATD0STAT0_SCF);
  temp = ATD0DR0;
	return  temp;
}
uint ADCValue1(uchar channel)
{
  //�ݴ�A/Dת���Ľ��
  uint temp;
  ATD0CTL1 = 0x0F;                         
	ATD0CTL5 = channel;
	//ȡA/Dת�����                                  
  while(!ATD0STAT0_SCF);
  temp = ATD0DR0;
	return  temp;
}
//��������//
void shedig(void) 
{
   if(PTJ_PTJ0==0)
   {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ0==0)
      {
      jiaosuduyushe=G;
      }
   }
   if(PORTK_PK4==1)
   {
       yeshu=0; 
   }
   if(PORTK_PK4==0)
   {
      yeshu=1;
   }
   if(PORTK_PK5==0)
   {
     if(PTJ_PTJ1==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ1==0)
      {
       jiaodu_P=jiaodu_P+1;
      } 
     }
     if(PTJ_PTJ6==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ6==0)
      {
       jiaodu_P=jiaodu_P-1;
      } 
     }
     if(PTJ_PTJ7==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ7==0)
      {
       jiaodu_D=jiaodu_D+0.1;
      } 
     }
     if(PORTK_PK0==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PORTK_PK0==0)
      {
       jiaodu_D=jiaodu_D-0.1;
      } 
     }
   }
   if(PORTK_PK7==0)
   {
     if(PTJ_PTJ1==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ1==0)
      {
       speedP=speedP+1;
      } 
     }
     if(PTJ_PTJ6==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ6==0)
      {
       speedP=speedP-1;
      } 
     }
     if(PTJ_PTJ7==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ7==0)
      {
       speedI=speedI+0.1;
      } 
     }
     if(PORTK_PK0==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PORTK_PK0==0)
      {
       speedI=speedI-0.1;
      } 
     }
   }
   if(PTM_PTM0==0)
   {
     if(PTJ_PTJ1==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ1==0)
      {
       fangxiangP=fangxiangP+0.1;
      } 
     }
     if(PTJ_PTJ6==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ6==0)
      {
       fangxiangP=fangxiangP-0.1;
      } 
     }
     if(PTJ_PTJ7==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ7==0)
      {
       fangxiangD=fangxiangD+0.01;
      } 
     }
     if(PORTK_PK0==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PORTK_PK0==0)
      {
       fangxiangD=fangxiangD-0.01;
      } 
     }
   }
   if(PTM_PTM1==0)
   {
     if(PTJ_PTJ1==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ1==0)
      {
       WfangxiangP=WfangxiangP+0.1;
      } 
     }
     if(PTJ_PTJ6==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ6==0)
      {
       WfangxiangP=WfangxiangP-0.1;
      } 
     }
     if(PTJ_PTJ7==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ7==0)
      {
       WfangxiangD=WfangxiangD+0.01;
      } 
     }
     if(PORTK_PK0==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PORTK_PK0==0)
      {
       WfangxiangD=WfangxiangD-0.01;
      } 
     }
   }
   if(PTM_PTM2==0)
   {
     if(PTJ_PTJ1==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ1==0)
      {
       TCARSPEEDYUSHE=TCARSPEEDYUSHE+0.1;
      } 
     }
     if(PTJ_PTJ6==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ6==0)
      {
       TCARSPEEDYUSHE=TCARSPEEDYUSHE-0.1;
      } 
     }
     if(PTJ_PTJ7==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PTJ_PTJ7==0)
      {
       WTCARSPEEDYUSHE=WTCARSPEEDYUSHE+0.1;
      } 
     }
     if(PORTK_PK0==0) 
     {
      DisableInterrupts;
      Dly_ms(200);
      if(PORTK_PK0==0)
      {
       WTCARSPEEDYUSHE=WTCARSPEEDYUSHE-0.1;
      } 
     }
   }
   EnableInterrupts;
}
//��ʾ���趨//
 void yejing(void) 
{
  unsigned int m= 0;
  static unsigned char flage = 1;
  static unsigned int Count = 0,speedlow = 0;
  uchar j;
  byte i=0;  
  DDRA=0XFF;   
  LCD_Init();  
  for(;;) 
  {  
      if(yeshu==0){
      LCD_P6x8Str(0,0,"rl:");                
      Dis_Num(18,0,left,6);     
      LCD_P6x8Str(0,1,"ll:");    
      Dis_Num(18,1,right,6);
      LCD_P6x8Str(0,2,"jp:");    
      Dis_Float(18,2,jiaodu_P,6);
      LCD_P6x8Str(0,3,"jd:");    
      Dis_Float(18,3,jiaodu_D,6);
      LCD_P6x8Str(0,4,"cj:");
      Dis_Num(18,4,chejiao,6);
      LCD_P6x8Str(0,5,"jd:");
      Dis_Num(18,5,jiaosuduyushe,6);
      LCD_P6x8Str(0,6,"zz:");
      Dis_Num(18,6,Z,6);  
      LCD_P6x8Str(0,7,"mc:");
      Dis_Num(18,7,CD,6);   
      LCD_P6x8Str(58,0,"sdI:");   
      Dis_Float(80,0,speedI,6);
      LCD_P6x8Str(58,1,"sdP:");
      Dis_Float(80,1,speedP,6);
      LCD_P6x8Str(58,2,"Ysd:");
      Dis_Float(80,2,TCARSPEEDYUSHE,6);
      LCD_P6x8Str(58,3,"sdJ:");
      Dis_Num(80,3,speedczjf,6);
      LCD_P6x8Str(58,4,"mc:");
      Dis_Num(80,4,Emaichong,6);
      LCD_P6x8Str(58,5,"aj:");
      Dis_Num(80,5,anjian,6);
      LCD_P6x8Str(58,6,"wsd:");
      Dis_Float(80,6,WTCARSPEEDYUSHE,6);
      LCD_P6x8Str(58,7,"sjz");
      Dis_Num(80,7,yuzhi,6);
      } 
      else{
      LCD_P6x8Str(0,0,"fp:");                
      Dis_Float(18,0,fangxiangP,6);     
      LCD_P6x8Str(0,1,"fd:");    
      Dis_Float(18,1,fangxiangD,6);
      LCD_P6x8Str(0,2,"rz:");    
      Dis_Num(18,2,Right,6);
      LCD_P6x8Str(0,3,"lz:");    
      Dis_Num(18,3,Left,6);
      LCD_P6x8Str(0,4,"sp:");
      Dis_Num(18,4,d_chazhi,6);  
      LCD_P6x8Str(0,5,"mx:");
      Dis_Num(18,5,max,6);  
      LCD_P6x8Str(0,6,"mn:");
      Dis_Num(18,6,min,6); 
      LCD_P6x8Str(0,7,"aj:");
      Dis_Num(18,7,anjian,6); 
      LCD_P6x8Str(58,0,"yz:");
      Dis_Num(80,0,yuzhi,6);
      LCD_P6x8Str(58,1,"sjz:");
      Dis_Num(80,1,zhongxian,6); 
      LCD_P6x8Str(58,2,"wp:");
      Dis_Float(80,2,WfangxiangP,6);
      LCD_P6x8Str(58,3,"wd:");
      Dis_Float(80,3,WfangxiangD,6);
      LCD_P6x8Str(58,4,"zi:");
      Dis_Num(80,4,zuidiyuzhi,6);
      LCD_P6x8Str(58,5,"za:");
      Dis_Num(80,5,zuidayuzhi,6);   
      LCD_P6x8Str(58,6,"ysd:");
      Dis_Float(80,6,TCARSPEEDYUSHE,6);  
      LCD_P6x8Str(58,7,"Wsd:");
      Dis_Float(80,7,WTCARSPEEDYUSHE,6); 
      }
      shedig();
      //��ӡ���
    /*for(j=(64-WINDOW_WIDTH/2); j<(64+WINDOW_WIDTH/2); j++)
    {
       #if(PRINT_AD==1)          //���ڷ���ADֵ������������CCD��������
           if(gPixel[j]==0xFF) 
               gPixel[j] = 0xFE; //����FF��FE�滻����
            UART0_SendByte(gPixel[j]);
       #else                     //���ڷ��Ͷ�ֵ���������ô��ڵ���
           if(gPixel[j]>yuzhi)
             UART0_SendByte(250);
        else
            UART0_SendByte(0);
       #endif     
    }
    UART0_SendByte(0xFF);*/
    update();
     
  }   
}
//���ڷ���//
//---------------------------------------------------------------------
// �������ܣ�UART0_Init��ʼ��
// ��ʽ������  ��
// ��������ֵ����   
//---------------------------------------------------------------------
void UART0_Init(void)
{
  SCI0CR1 = 0x00; 
  SCI0CR2 = 0x2C;     //�����ж�ʹ�ܣ����ͽ���ʹ��
  SCI0BD  = 0x2b;     //���������ó�115200
                      //When IREN = 0 then 
                      //SCI baud rate = SCI bus clock / (16 x SBR[12:0])
}
//---------------------------------------------------------------------
// �������ܣ�SCI0����һ���ֽ�����
// ��ʽ������  byte ch�����͵�һ���ֽ�����
// ��������ֵ����   
//---------------------------------------------------------------------
void UART0_SendByte(byte ch)
{
  while(!(SCI0SR1&0x80));
  SCI0DRL = ch; 
}
//---------------------------------------------------------------------
// �������ܣ�SCI0�����ַ�������
// ��ʽ������   byte *pBuff     ���ͻ�����
//              int Length �����ֽڵĳ��� 
// ��������ֵ����   
//---------------------------------------------------------------------
void UART0_SendPacket(byte *pBuf,int pBuf_Length) 
{
  int i;
  for(i=0;i<pBuf_Length;i++)
  {
    while(!(SCI0SR1&0x80));
    SCI0DRL=*(pBuf+i); 
  }
}
/*********************************************************************
*************************����ʾ����***********************************
*********************************************************************/
void uart_putchar ( unsigned char c)
{
  
  while(!(SCI0SR1&0x80)) ; 		    //keep waiting when not empty  
  SCI0DRL=c;
}
void uart_putstr(char ch[])
{
  unsigned char ptr=0;
  while(ch[ptr]){
      uart_putchar((unsigned char)ch[ptr++]);
  } 
}
unsigned char uart_getchar(void)
{
	//printf("uart_getchar\n");
	 byte res=0;
   while(!(SCI0SR1&0x80)) ; 		 //keep waiting when not empty  
   return (SCI0DRL);

}
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
void OutPut_Data(void)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
   {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
   }
   
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  for(i=0;i<10;i++)
    uart_putchar(databuf[i]);
}
void update(void) 
{

  OutData[0] =zjiaodu;
  OutData[1] =carspeed;
  OutData[2] =chejiao;
  OutData[3] =zhongxian; 
  OutPut_Data();
}
/*********************************************************************
**************************С����ʾ************************************
*********************************************************************/
void Dis_Float(byte x,byte y,double real,byte N) 
{
   int j;
   byte   n[9]={0};  
   long int   real_int=0;
   real_int=real*1000;
   n[0]=(real_int/10000000)%10; 
   n[1]=(real_int/1000000)%10;
   n[2]=(real_int/100000)%10;
   n[3]=(real_int/10000)%10; 
   n[4]=(real_int/1000)%10; 
   n[5]=(real_int/100)%10;
   n[6]=(real_int/10)%10;
   n[7]=(real_int/1)%10;                         
   n[8]='\0';
   n[7]=n[6];
   n[6]=n[5];
   n[5]='.'; 
   for(j=0;j<8;j++) n[j]=n[j]+16+32;   
   LCD_P6x8Str(x,y,&n[8-N]); 
}
 
//��ʾ����̬��ʾ//
void Dis_Num(byte x,byte y , uint num,byte N) 
{
  byte j=0;
  byte n[9]={0};
  n[0]=(num/10000000)%10;
  n[1]=(num/1000000)%10;
  n[2]=(num/100000)%10;
  n[3]=(num/10000)%10; 
  n[4]=(num/1000)%10;
  n[5]=(num/100)%10;
  n[6]=(num/10)%10;
  n[7]=num%10;
  n[8]='\0';
  for(j=0;j<8;j++) n[j]=n[j]+16+32;
  LCD_P6x8Str(x,y,&n[8-N]);//��ACSII����ж�ȡ�ֽڣ�Ȼ��д��Һ��
}
void Dly_ms(int ms)
{
   int ii,jj;
   if (ms<1) ms=1;
   for(ii=0;ii<ms;ii++)
   for(jj=0;jj<4000;jj++);    
}
//----------------------------------------------------------------------
//�� �� ��:��ʱ���� delay(), DelayMs()                                                     
//��    ��:��ʱ����                                                              
//��    ��:��                                                                                       
//----------------------------------------------------------------------
void delay()
{
  unsigned int i;
  for(i=0;i<10;i++)
  {
    asm("nop");
  }
}
//----------------------------------------------------------------------
//�� �� ��:TSL1401_GetLine                                                       
//��    ��:���AD��������                                       
//��    ��:*pixel  ��õ�����ֵ                          
//��    ��:��                                                                                       
//----------------------------------------------------------------------
void TSL1401_GetLine(uchar *pixel)
{
  uchar i;
  //��ʼSI
  TSL1401_SI(0) ;
  TSL1401_CLK(0);
  delay();
  TSL1401_SI(1);
  delay();
  TSL1401_CLK(1);
  delay();
  TSL1401_SI(0);
  delay();
  pixel[0]=(uchar)ADCValue1(1);
  TSL1401_CLK(0);
  
  //�ɼ���2~128����
  for(i=1; i<128; i++)
  {
    delay();
    TSL1401_CLK(1);
    delay(); 
    pixel[i]=(uchar)ADCValue1(1);
    TSL1401_CLK(0);
  }
  
  //���͵�129��clk
   delay();
   TSL1401_CLK(1);
   delay(); 
   TSL1401_CLK(0);
   delay(); 
}
 /*************************************************************/
/*                    ��̬��ֵ               */
/*************************************************************/  
void dongtaiyuzhi(void)
{
  
        int i;
        max=0;
        min=255;                               
        for(i=10;i<119;i++)
        {
          if(gPixel[i]>max) max=gPixel[i];
          if(gPixel[i]<min) min=gPixel[i];
        }  
        yuzhi=(int)(max+min)/2;  
        if(yuzhi<zuidiyuzhi)yuzhi=zuidiyuzhi;
        if(yuzhi>zuidayuzhi)yuzhi=zuidayuzhi;
}
/*************************************************************/
/*                   ��ֵ��               */
/*************************************************************/  
void erzhihua(void) 
{
  uchar i;
  for(i=0;i<128;i++) 
  {
      if(gPixel[i]<yuzhi) 
      {
        SS[i]=0;
      } 
      else
      {
        SS[i]=255;
      }
  } 
}
//*******************������**************************//
void qphzj(void)
{
  
   
  
}
/*************************************************************/
/*                   ��ȡ��������               */
/*************************************************************/  
void zuobiao(void)
{
  int i;
  if(bianduqu<10){
  bianduqu++;
  }
  if(bianduqu==10) {
  duquzhongxian=zhongxian;
  }
  oldRight=Right;
  oldLeft=Left;
  zuo_flag=0;
  you_flag=0;
  zuo_flag1=0;
  you_flag1=0;  
  for(i=duquzhongxian;i<122;i++) 
  {
    if(SS[i-1]==255 && SS[i]==255 && SS[i+1]==0) 
    {
       Right=(int)i;
       you_flag=1;   
       break;
    } 
  }
     
  for(i=duquzhongxian;i>5;i--) 
  {  
    if(SS[i+1]==255 && SS[i]==255 && SS[i-1]==0) 
    {
    
       Left=(int)i;
       zuo_flag=1;
       break;
    }    
  }
  /*  for(i=71;i>57;i--) 
  {  
    if(SS[i+1]==255 && SS[i]==255 && SS[i-1]==0) 
    {
    
       Left1=(int)i;
       zuo_flag1=1;
       break;
    }
  }
  for(i=57;i<71;i++) 
  {
    if(SS[i-1]==255 && SS[i]==255 && SS[i+1]==0) 
    {
       Right1=(int)i;
       you_flag1=1;   
       break;
    } 
  }*/
}
/***********************************************************
**********************�ܵ�����******************************
***********************************************************/
void ZXJS(void) 
{
   int m,j=0,n=0,a,b;
   if(zuo_flag1==0&&you_flag1==0&&zuo_flag==0&&you_flag==0) 
  {
     zhongxian=(Left+Right)/2;
  }
  if(zuo_flag==1&&you_flag==1) //��ͨ���//
  {
     CARSPEEDYUSHE=TCARSPEEDYUSHE;
     //fangxiangP=20;
    //fangxiangD=1.5;
     zhongxian=(Left+Right)/2;
     for(m=20;m<50;m++)//�ϰ���//
      {
        if(SS[m]==0)
        j=j+1;
      }
      if(j>25) 
      {
          zhongxian=100;
      }
       for(m=110;m>80;m--)//�ϰ���//
      {
        if(SS[m]==0)
        n=n+1;
      }
      if(n>25) 
      {
          zhongxian=40;
      }
  }
  
  if(zuo_flag1==1&&you_flag1==1) //���ĺ���//
  {
     zhongxian=(Left1+Right1)/2;
  }
  
  if(zuo_flag==1&&you_flag==0) //������ҹ�,90��ֱ��//
  {
    //speedczjf=0;
    //fangxiangP=20;
    //fangxiangD=1.0;
    b=Left-oldLeft;
    zhongxian=zhongxian+b;
    CARSPEEDYUSHE=WTCARSPEEDYUSHE;
  }
  if(zuo_flag==0&&you_flag==1)//��������,90��ֱ��//
  {
    //speedczjf=0;
    //fangxiangP=20;
    //fangxiangD=1.0;
    a=Right-oldRight;
    zhongxian=zhongxian+a;
    CARSPEEDYUSHE=WTCARSPEEDYUSHE;
  }
}
/*************************************************************
*********************�ܵ��������*****************************
**************************************************************/
void paodaolukuan(void)
{  
  int chazhi,chazhi1;
  chazhi=64-zhongxian;
  chazhi1=chazhi*fangxiangP+(980-d_chazhi)*fangxiangD;
  oldchazhiout=newchazhiout;
  newchazhiout=chazhi1;
}
/*******************�ܵ����***************/
void paodaoshuchu(void){
    float zhi;
    zhi=newchazhiout-oldchazhiout;
    fangxiang=zhi*(fangxiangzhouqi+1)/10+oldchazhiout;  
}
//Z��G���˲�//
void ADduqu(void)
{     
     int i;
     long int k=0,m=0,d=0,c=0;
     for(i=0;i<20;i++) {
     g=ADCValue(5); 
      k=g+k;
     }
     G=k*0.05;
     for(i=0;i<20;i++) {
       z=ADCValue(7); 
       m=z+m;
     }
     Z=m*0.05;
     for(i=0;i<20;i++) {
       d=ADCValue(2); 
       c=d+c;
     }
     d_chazhi=c*0.05; 
}
//�Ƕȼ���//
void jiaodujisuan(void)
{    
   float  jiaozhi;
   zjiaodu=(jiaoduyushe-Z)*zhonglijiaochangshu;
   jiaosudu=(jiaosuduyushe-G)*jiaosuduchangshu;
   chejiao=jiaojifen;
   jiaozhi=(zjiaodu-chejiao)/buchangxishu;
   jiaojifen+=(jiaosudu+jiaozhi)*jiaochangshu;
   zhilicl();
}
//ֱ������//
void zhilicl(void) 
{
   float  zhi;
   zhi=chejiao*jiaodu_P+jiaosudu*jiaodu_D;
   /*if(zhi>zuidazhi)    �޷�����
     zhi=zuidazhi;
   else(zhi<zuixiaozhi)
     zhi=zuixiaozhi; */
   jiaoclout=zhi;
}
// ���������ۼ���//
void TimInit (void) 
{
 PACNT=0x0000;
 PACTL=0X00;
 PACNT=0X0000;//���������ۼ�����ֵ
 PACTL_PAEN=1;
}
//�ٶȶ�ȡ����//
 void Speed_Read(void)
{
  cd=PORTB;
 PORTA_PA4=1;
 maichong=PACNT;         //��ȡ�Ĵ���������ֵ
 PACNT=0x0000;           //����Ĵ���������ֵ 
 PORTA_PA4=0;            //4520��ֵ
 if(PORTK_PK2==1)//��� 
 {
   cd=-cd;
 }
 if(PORTK_PK3==1)//�ұ� 
 {
   maichong=-maichong;
 }
 Emaichong+=maichong;
 CD+=cd;             
}
//�ٶȵ���//
void speedcontrol(void) 
{
  float chesu,P,I; 
  carspeed=(Emaichong+CD)/2;
  Emaichong=0;
  CD=0;
  carspeed=carspeed*CARSPEEDCHANGLIANG;
  if(carspeedyushe<CARSPEEDYUSHE){
     carspeedyushe=carspeedyushe+1;
  }
  if(carspeedyushe>CARSPEEDYUSHE){
     carspeedyushe=carspeedyushe-1;
  }
  chesu=carspeedyushe-carspeed;
 /* if(abs(chesu)<5) {
    speedI=7;
  } 
  else if(abs(chesu)<10) 
  {
    speedI=5; 
  }  
  else if(abs(chesu)<15) 
  {
     speedI=3;
  } 
  else if(abs(chesu)<20)
  {
     speedI=1;
  } 
  else
  {
     speedI=0;
  }*/
  P=chesu*speedP;
  I=chesu*speedI;
  speedczjf+=I;
  OLDspeedout=NEWspeedout;
  NEWspeedout=P+speedczjf;
}
void speed_out(void){
  float zhispeed;
  zhispeed=NEWspeedout-OLDspeedout;
  speedout=zhispeed*(speedtjzq+1)*SPEEDCZZQ+OLDspeedout;
}
//�������趨//
void dianjiout(void) 
{ 
    left=jiaoclout-speedout-fangxiang;
    right=jiaoclout-speedout+fangxiang;
   dianjisudu();
}
//������ѹ����//
void dianjisudu(void) 
{
  if(left>0)
  left+=deadout;
  else
  left-=deadout;
  if(right>0)
  right+=deadout;
  else
  right-=deadout;
  if(left>maxout)
  left=maxout;
  if(left<minout)
  left=minout;
  if(right>maxout)
  right=maxout;
  if(right<minout)
  right=minout;
  outshezhi(left,right);
}
//������תPWM//
void outshezhi(float lefta,float righta) 
{  
          if(lefta>0) 
         { 
             PWMDTY23=lefta;    
             PWMDTY01=0;
          } 
         else
         { 
             PWMDTY23=0;    
            PWMDTY01=(0-lefta); 
         }
         if(righta>0) 
        {
            PWMDTY67=righta;
            PWMDTY45=0;
        } 
        else
       {
           PWMDTY67=0;
           PWMDTY45=(0-righta);  
       }
}
//���໷�趨//
void PLL(void)
{
    CLKSEL=0X00;			         //disengage PLL to system
    PLLCTL_PLLON=1;			       //turn on PLL
    SYNR =0xc0 | 0x09;         // Fvco=2*osc*(1+SYNRDIV)/(1+REFDIV)
                               //Fpll=Fvco/(2*POSTDIV) Fbus=Fpll/2
    REFDV=0x80 | 0x01; 
    POSTDIV=0x00;              //pllclock=2*osc*(1+SYNRDIV)/(1+REFDIV)=160MHz;
    _asm(nop);                 //BUS CLOCK=80M
    _asm(nop);
    while(!(CRGFLG_LOCK==1));	 //when pll is steady ,then use it;
    CLKSEL_PLLSEL =1;		       //engage PLL to system; 
    PLLCTL_PCE=1;   
}
//PIT�ж�����//
void PIT(void) 
{
 PITMTLD0=49;     //Ϊ0ͨ��8λ��������ֵ
 PITLD0=1599;     //Ϊ0ͨ��16λ��������ֵ   
 PITMUX_PMUX0=0;   //��0ͨ��ʹ��΢������0
 PITCE_PCE0=1;     //��0ͨ������������ 
 PITCFLMT=0X80;    //ʹ�������ж϶�ʱ��
 PITINTE_PINTE0=1; //0ͨ����ʱ����ʱ�жϱ�ʹ��
}
//�жϷ������//
#pragma CODE_SEG __NEAR_SEG NON_BANKED
void interrupt 66 pit0(void)
{
  PITTF_PTF0=1;
  sududuzhi++;
  speedtjzq=speedtjzq+1;
  speed_out();
  fangxiangzhouqi=fangxiangzhouqi+1;
  paodaoshuchu();
  if(qisu<5000) {
     if(chejiao<30&&chejiao>-30)
     qisu=qisu+1;  
  }
  if(qisu==5000){
    CARSPEEDYUSHE=TCARSPEEDYUSHE;
   yunxu=2;
  }
  if(sududuzhi==1)
  {
   ADduqu();
   jiaodujisuan();
   dianjiout();
  }
  if(sududuzhi==2)
  {
   if(PTM_PTM3==1) 
    {
    Speed_Read();
    suduczzq++;
    if(suduczzq>19) 
    {
    speedcontrol();
    suduczzq=0;
    speedtjzq=0;
    }
    }
  }
  if(sududuzhi==3)
  {
    if(PTM_PTM4==1) 
    {
       if(yunxu==2)
       {
          fangxiangzq=fangxiangzq+1;
          if(fangxiangzq==2)
          { //�������ֵ
              TSL1401_GetLine(gPixel);
          }
       }
    }
  }
  if(sududuzhi==4)
  {
    if(PTM_PTM4==1) 
    {
      if(yunxu==2)
      { 
        if(fangxiangzq==2)
        {
           dongtaiyuzhi();
           erzhihua(); 
           qphzj();
           zuobiao();
           ZXJS();
        }
      }
    }
  }
  if(sududuzhi==5)
  {
    if(PTM_PTM4==1) 
    {
       if(yunxu==2)
       {
          if(fangxiangzq==2)
          {
              ADduqu();
              paodaolukuan();
              fangxiangzq=0;
              fangxiangzhouqi=0;
          } 
       }
    }
  sududuzhi=0; 
  }
  if(Emaichong>3500)
  {
     sududuzhi=6;
     PWMDTY23=0;    
     PWMDTY01=0;
     PWMDTY45=0;    
     PWMDTY67=0;
  }
}
#pragma CODE_SEG DEFAULT