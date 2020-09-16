`timescale 1ns / 1ps
//--------------------------------------------------------------------------------------------
//  31 2018 18:13:05
//      Component name  : ini_particle
//--------------------------------------------------------------------------------------------

module ini_particle(clk_P, reset, ena, address, x, v);
   input          clk_P;
   input          reset;
   input          ena;
	
   output [2:0] address;
   reg [2:0]    address;
   output [15:0]  x;
   reg [15:0]     x;
   output [15:0]  v;
   reg [15:0]     v;
	
	random rand(
	.clk_P(clk),
	.reset(rst_n),
	.rand_num(rng1)
	);
	
	random rand0(
	.clk_P(clk),
	.reset(rst_n),
	.rand_num(rng2)
	);
	
   always @(posedge clk_P or posedge reset)
      if (reset == 1'b1)
			begin
				x <= {16{1'b0}};
				v <= {16{1'b0}};
				address <= {8{1'b0}};
			//	over <= 1'b0;
			end
      else if(ena == 1'b1)
			begin
				if (address < 3'b110)
				begin
					x <= rng1;
					v <= rng2;
					address <= address + 1;
				end
				//else
				//	over <= 1'b1;
			end
   
endmodule
