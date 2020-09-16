`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Create Date:    16:53:00 04/18/2018 
// Design Name: 
// Module Name:    Control 
// Project Name: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module control(clk, reset, ena, ad1_in, ad1_clk, ad2_in, ad2_clk, pwmout);
   input            clk;
   input         	  reset;
	input         	  ena;   
	input   [11:0]   ad1_in;
 	input   [11:0]   ad2_in;
	output           pwmout;
	output 		     ad1_clk;
	output 		     ad2_clk;
	
	reg              ov1; //初始化结束信号
   reg              ov2; //从初始化后的32个个体中选择全局最优结束信号
   reg              ov3; //局部，全局最优更新结束信号
   reg              ov4; //个体速度计算并判断阀值结束信号
   reg              ov5; //个体位置计算并判断阀值结束信号
	
   reg              st1; 
   reg              st2; 
   reg              st3;
   reg              st4;
	reg              st5;
	
	wire   [2:0]     address1;	//ini_particle inout
	wire   [15:0]    x1;
	wire   [15:0]    v;
	
	reg    [15:0]    x2;			//fitness inout
	wire   [15:0]    p2;
	wire   [2:0]     addr2;
	
	reg    [15:0]    Vin;		//update inout
   reg    [15:0]    P3;
   reg    [15:0]    G;
   reg    [15:0]    Xin;
	reg    [2:0]     address3;
   wire   [31:0]    Vout;
   wire   [31:0]    Xout;
	
	reg    [15:0]    p_fit4;		//gbest inout
	reg    [1:0]     addr4;
	wire 	 [2:0]     address4;
   wire   [15:0]    gb_fit;
	
	reg    [15:0]    x_fit;		//pbest inout
	reg    [1:0]     addr5;	
	wire 	 [2:0]     address5;
   wire   [15:0]    p_fit5;

   parameter        states_idle = 7'b0000001,
						  states_st1  = 7'b0000010,
                    states_st2  = 7'b0000100,
                    states_st3  = 7'b0001000,
                    states_st4  = 7'b0010000,
                    states_st5  = 7'b0100000,
                    states_stop = 7'b1000000;
   reg    [6:0]     current_state;
	reg    [5:0]     cnt;
	
	ini_particle ini_particle1(
	.clk_P(clk),
	.reset(rst_n),
	.ena(st1),
	.address(address1),
	.x(x1),
	.v(v)
	);
	
	fitness fitness1(
	.clk_P(clk),
	.rst_n(rst_n),
	.ena(st2),
	.ad1_in(ad1_in),
	.ad2_in(ad2_in),
	.ad1_clk(ad1_clk),
	.ad2_clk(ad2_clk),
	.address(address1),
	.addr(addr2),
	.pwmout(pwmout),
	.x(x2),
	.p(p2)
	);
	
	update update1(
	.clk(clk),
	.rst_n(rst_n),
	.ena(st3),
	.Vin(Vin),	
   .P(p3),
   .G(G),
   .Xin(Xin),
	.address(address3),
   .Vout(Vout),
   .Xout(Xout)
	);
	
	gbest gbest1(
	.clk_P(clk),
	.reset(rst_n),
	.ena(st4),
	.p_fit(p_fit4), 
	.addr(addr4), 
	.gb_fit(gb_fit), 
	.address(address4)
	);
	
	pbest pbest1(
	.clk_P(clk),
	.reset(rst_n),
	.ena(st5),
	.x_fit(x_fit), 
	.addr(addr5), 
	.p_fit(p_fit5), 
	.address(address5)
	);	
	
   always @(posedge clk) //系统主状态机
      if (reset == 1'b1)
         current_state <= states_idle;
      else 
         case (current_state)
            states_idle :      //系统复位
               begin
						st1 <= 1'b0;
                  st2 <= 1'b0;
                  st3 <= 1'b0;
                  st4 <= 1'b0;
						st5 <= 1'b0;
                  current_state <= states_st1;
               end
            states_st1 :       //群体初始化
               begin
                  st1 <= 1'b1;
                  st2 <= 1'b0;
                  st3 <= 1'b0;
                  st4 <= 1'b0;
						st5 <= 1'b0;
						x2  <= x1;
                  if (ov1 == 1'b1)
                  begin
                     current_state <= states_st2;
							st1 <= 1'b0;
                  end
                  else
                     current_state <= states_st1;
               end
            states_st2 :       //个体适应值计算
               begin
                  st1 <= 1'b0;
                  st2 <= 1'b1;
                  st3 <= 1'b0;
                  st4 <= 1'b0;
						st5 <= 1'b0;
                  if (ov2 == 1'b1)
                  begin
                     current_state <= states_st3;
							st2 <= 1'b0;
                  end
                  else
                     current_state <= states_st2;
               end
            states_st3 :        //个体速度位置更新
               begin
                  st1 <= 1'b0;
                  st2 <= 1'b0;
                  st3 <= 1'b1;
                  st4 <= 1'b0;
						st5 <= 1'b0;
                  if (ov3 == 1'b1)
                  begin
                     current_state <= states_st4;
                     st3 <= 1'b0;
                  end
                     current_state <= states_st3;
               end
            states_st4 :         //从个体中选择局部最优
               begin
                  st1 <= 1'b0;
                  st2 <= 1'b0;
                  st3 <= 1'b0;
                  st4 <= 1'b1;
						st5 <= 1'b0;
                  if (ov4 == 1'b1)
                  begin
                     current_state <= states_st2;
                     st4 <= 1'b0;
                  end
                  else
                     current_state <= states_st4;
               end
            states_st5 :         //从个体中选择全局最优
               begin
                  st1 <= 1'b0;
                  st2 <= 1'b0;
                  st3 <= 1'b0;
                  st4 <= 1'b0;
						st5 <= 1'b1;
                  if (ov5 == 1'b1)
                  begin
                     st5 <= 1'b0;
                     current_state <= states_st1;
							cnt <= cnt + 1;
							if(cnt > 59)
							current_state <= states_stop;
                  end
               end
            states_stop :         //迭代结束
               begin
                  st1 <= 1'b0;
                  st2 <= 1'b0;
                  st3 <= 1'b0;
                  st4 <= 1'b0;
						st5 <= 1'b0;
						cnt <= 1'b0;
               end
            default :
               current_state <= 1'bxxxxxxx;
         endcase
	
endmodule
