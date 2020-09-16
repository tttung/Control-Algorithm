`timescale 1ns / 1ps
/* 
Filename    : V_update_pipe.v
Description : Demo how to write: Vout = Vin + C1 * R1 * (P-Xin) + C2 * R2 * (G-Xin) 
with pipeline
八位小数
Release     : 4/16/2018 1.0
*/

module update (
   input          clk,
   input          rst_n,
   input          ena,
	input  [2:0]   address,
   input  [15:0]  Vin,
   input  [15:0]  P,
   input  [15:0]  G,
   input  [15:0]  Xin,
  
   output [31:0]  Vout,
   output [31:0]  Xout
);

	reg    [15:0]  a;
	reg    [15:0]  ao;
	reg    [15:0]  b;
	reg    [15:0]  c;
	reg    [15:0]  d;
	reg    [7:0]   w;

	reg    [15:0]  RNG1;
	reg    [15:0]  RNG2;
	reg    [15:0]  error1;
	reg    [15:0]  error2;

	reg    [31:0]  m0;
	reg    [31:0]  m1;

	reg    [31:0]  sum1;
	reg    [31:0]  sum2;

	reg    [5:0]   number;
	reg    [5:0]   delta;


	random rand(
	.clk_P(clk),
	.reset(rst_n),
	.rand_num(rng1)
	);
	
	random rando(
	.clk_P(clk),
	.reset(rst_n),
	.rand_num(rng2)
	);

always@(posedge clk or negedge rst_n) 
	begin
		if (!rst_n) 
			begin
				a <= 0;
				ao <= 0;
				b <= 0;
				c <= 0;
				d <= 0;
				delta <= 0;
				m0 <= 0;
				m1 <= 0;
				RNG1 <= 0;
				RNG2 <= 0;
				error1 <= 0;
				error2 <= 0;
			end
		else if(ena == 1'b1)
			begin
				a <= Vin;
				b <= P;
				c <= G;
				d <= Xin;	
				if (number < 60)
					begin
						RNG1 <= rng1 << 1;
						RNG2 <= rng2 << 1;
						error1 <= b-a;  //粒子与局部最优差值
						error2 <= c-a;  //粒子与全局最优差值
						m0 <=  RNG1 * error1;
						m1 <=  RNG2 * error1;
						ao <= a * w;
						
						sum1 <= ao + m0 + m1;
						sum2 <= a + d;
						
					   delta <= number * 8'b00000010;	//线性递减0.0078125
						w <= 8'b11001100 - delta;	 //惯性权值=0.8-delta
						number <= number + 1;
					end
				else
					begin
						number <= 0;
					end
			end
    end		
	 
assign Vout = sum1;
assign Xout = sum2;

endmodule