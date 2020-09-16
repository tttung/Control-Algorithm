`timescale 1ns / 1ps
/**********************************************************************************
;** 函数名称: filter.v
;** 功能描述: 去除信号中毛刺（小于2个时钟cp脉宽的信号）
;** 输　入:   cp    :时钟信号
;**           x_in  :带有毛刺的信号
;** 输　出 :  y_out :去除毛刺后的信号
;** 全局变量: y_out
;** 调用模块: 无
;**-------------------------------------------------------------------------------*/

module filter ( cp, x_in,y_out);
  // 端口声明
     input cp;                   // 定义时钟入口   
     input x_in;                 // 消抖前输入信号
     output y_out;               // 消抖后输出信号

  // 端口变量类型定义
     wire cp;                    // 被声明为input或inout型的变量只能被定义为线网型
     wire x_in;                  // 线网型变量表明连接关系
     reg y_out;                  // y_out为输出寄存器类型
  
  // 内部寄存器定义
     reg [1:0] q;                // 定义一个1位、3个元素的数组缓冲寄存器
     reg [1:0] sum;              // 定义求和寄存器
     integer i;                  // 计数变量

  // 任务主体
     always @(posedge cp)        // 输入采用存入数据缓冲器
          begin
          //     q[2] = q[1];
               q[1] = q[0];
               q[0] = x_in;
          end
     always @(posedge cp)
          begin
               sum = 0;
               for(i=0;i<2;i=i+1)
               begin
                   if (q[i] == 1)
                       sum = sum +1;
                   else
                       sum = sum;
               end
               if (sum > 1)     // 大于1个时钟脉宽的信号认为不是毛刺信号
                   y_out <= 1;
               else
                   y_out <= 0;
           end 

  endmodule

