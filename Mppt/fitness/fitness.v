`timescale 1ns / 1ps
//--------------------------------------------------------------------------------------------
// 31 2018 21:08:29
//      Component name  : fitness
//--------------------------------------------------------------------------------------------

module fitness(clk_P, rst_n, ena, ad1_in, ad1_clk, ad2_in, ad2_clk, address, x, p, addr, pwmout);
   input            clk_P;
	input            rst_n;
	input            ena;
   input  [2:0]     address;
	input            x;
	input  [11:0]    ad1_in;
 	input  [11:0]    ad2_in;  
	
	output 		     ad1_clk;
	output 		     ad2_clk;
	
	output [31:0]    p;
   reg    [31:0]    p;
	output [2:0]     addr;
	reg    [2:0]     addr;
	output 		     pwmout;
	reg    [2:0]     cuttent_state;
	
	wire   [16:0]    uo;
	wire   [16:0]    io;
	wire             clock; 
	wire   		     locked;	
	
	reg    [16:0]    u;
	reg    [16:0]    i;
	
	reg              wr_en; // input wr_en
   reg              rd_en; // input rd_en
	wire             dout;
   wire             full; // output full
   wire             empty; // output empty
  
	parameter 		  states_idle = 6'b000001,
                    states_st1 = 6'b000010,
                    states_st2 = 6'b000100,
                    states_st3 = 6'b001000,
                    states_st4 = 6'b010000,
                    states_st5 = 6'b100001;
                    
	pwm pwmo(
	.clock(clk_P),
	.rst_n(rst_n),
	.data_in(x),
	.pwm_out(pwmout)
	);
	
   ad9226_test AD(
	.clk50m(clk_P),
	.reset_n(rst_n),
	.ad_ch1(uo),
	.ad_ch2(io),
	.ad1_clk(ad1_clk),
	.ad2_clk(ad2_clk)
	);	
	
	fifo pwm_fitness (
  .rst(rst_n), // input rst
  .wr_clk(clk_P), // input wr_clk
  .rd_clk(clk_P), // input rd_clk
  .din(p), // input [11 : 0] din
  .wr_en(wr_en), // input wr_en
  .rd_en(rd_en), // input rd_en
  .dout(dout), // output [11 : 0] dout
  .full(full), // output full
  .empty(empty) // output empty
);

	always @(posedge clk_P)
		begin
			cuttent_state <= address;
			u <= uo;
			i <= io;
		end 
	
   always @(posedge clk_P)
		begin
			if (rst_n == 1'b1)
				begin
					p <= 0;
					addr <= 0;
				end
			else if(ena == 1'b1)
				begin
				case(cuttent_state)
					states_idle :
						begin
							p <= 0;
							addr <= 0;
						end
					states_st1 :
						begin
							p <= u * i;
							addr <= 1;
						end
					states_st2 :
						begin
							p <= u * i;
							addr <= 2;
						end
					states_st3 :
						begin
							p <= u * i;
							addr <= 3;
						end
					states_st4 :
						begin
							p <= u * i;
							addr <= 4;
						end
					states_st5 :
						begin
							p <= u * i;
							addr <= 5;
						end
					default :
						begin
							p <= 0;
							addr <= 0;
						end
				endcase	
				end
      end
   
endmodule
