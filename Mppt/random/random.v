`timescale 1ns / 1ps
//--------------------------------------------------------------------------------------------
//  31 2018 20:42:14
//      Component name  : random
//--------------------------------------------------------------------------------------------

module random(clk_P, reset, rand_num);
   input        clk_P;
   input        reset;
	output [15:0] rand_num;
	
   reg [15:0]    z;
   wire   qz;
   reg [15:0] rand_num;
	
   always @(*)
   begin
      if (reset == 1'b1)
      begin
         z <= 16'b1001001001000100;//seed
         rand_num <= {16{1'b0}};
      end
      else 
      begin
         z[15:1] <= z[14:0];
         z[0] <= qz;
         rand_num <= {z[1], z[3], z[5], z[7], z[9], z[11], z[13], z[15],
         z[0], z[2], z[4], z[6], z[8], z[10], z[12], z[14]};
      end
   end
   assign qz = z[15] ^ z[14] ^ z[12] ^ z[3]; //反馈函数（抽头序列1101000000001000）
	
endmodule
