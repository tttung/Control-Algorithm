`timescale 1ns / 1ps
/**--------------文件信息------------------------------------------------------------------------------
;**文   件   名: pwm.v
;**描        述: Verilog HDL语言在EPM7128STC100-10上，产生两路互差180度相位的PWM信号。该PWM信号经过
;**              消毛刺处理，在过流、过压和过热保护时刻自动关闭PWM信号。用MAX+plusII10.1编译。
;**--------------历史版本信息--------------------------------------------------------------------------
;** 版  本: V1.1
;**         由于V1.0占用的逻辑门太多只能在2500门芯片上实现，故改变PWM算法期望在1200门芯片上实现
;**--------------历史版本信息--------------------------------------------------------------------------
;** 版  本: V1.2
;**         当输入震荡时,PWM-A,PWM-B不同步（满占空比1300）
;**---------------------------------------------------------------------------------------------------*/

module pwm (clock,data_in,pwm_out,rst_n);
  // 端口声明
     input clock;                          // 定义时钟入端口
	  input rst_n;
     input[11:0] data_in;                   // 定义12位宽数据输入端口
     output pwm_out;                     // 定义PWM_A输出端口

  // 端口变量类型定义
                               // 被声明为input或inout型的变量只能被定义为线网型
     wire[11:0] data_in;                                    
     wire  pwm_out;                       // PWM_A输出寄存器类型
	 
	 // wire clock; 
                                           // 对output型端口如果不加定义则默认为wire型
	  
  // 内部寄存器定义
     reg[12:0] counter1;                    // 时钟计数寄存器,初始值默认是0 
  //   reg[11:0] counter2;  
  //   reg flag ;                            // 标志寄存器,初始值默认是0 
     reg pwm_A;

  // 任务主体
     always @(posedge clock)               
                                           // 定义clock信号上升沿触发
                                           // (2500*2)*40KHz=200MHz的clock信号，产生160KHz三角波
        // begin
             
          //   if (flag == 1)                // 后半周期三角波上升斜坡
         //    begin 
          //        if (counter2 < 2500)
          //            counter2 <= counter2 + 1;
          //        else
          //       begin
          //             counter2 <= 0;
          //             flag <= 0;          // 设前半周期三角波上升斜坡标志
          //        end
          //   end
          //   else                          // 前半周期三角波上升斜坡
             begin
                  if (counter1 < 5000)      // 死区设置为5.4us
                      counter1 <= counter1 + 1;                  
                  else     
                  begin
                       counter1 <= 0;
                     //  flag <= 1;          // 设后半周期三角波上升斜坡标志
                  end

             end 
       //  end

     always @(counter1) // or counter2
        // begin
           //  begin
             //     if(flag==0)
                  begin
                       if(counter1 < 3500)
                            pwm_A <= 1;        // 是，则输出高电平
                       else
                            pwm_A <= 0;        // 否则，输出低电平
                  end
						//else
                 // begin
                 //      if(counter2 < 150)
                  //          pwm_A <= 1;        // 是，则输出高电平
                  //     else
                  //          pwm_A <= 0;        // 否则，输出低电平
                //  end
          //   end
        // end
         
     filter U1(.cp(clock),.x_in(pwm_A),.y_out(pwm_out)); // 调用PWM-A滤波模块 
 
 endmodule


