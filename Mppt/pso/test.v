`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   21:21:28 04/24/2018
// Design Name:   control
// Module Name:   E:/FPGA/project/all/pso/test.v
// Project Name:  pso
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: control
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module test;

	// Inputs
	reg clk;
	reg reset;
	reg ena;
	reg [11:0] ad1_in;
	reg [11:0] ad2_in;

	// Outputs
	wire pwmout;

	// Instantiate the Unit Under Test (UUT)
	control uut (
		.clk(clk), 
		.reset(reset), 
		.ena(ena), 
		.ad1_in(ad1_in), 
		.ad2_in(ad2_in), 
		.pwmout(pwmout)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		reset = 0;
		ena = 0;
		ad1_in = 0;
		ad2_in = 0;

		// Wait 100 ns for global reset to finish
		#100;
      reset = 1; 
		#2000;	
		// Add stimulus here
		always #10 clk = ~ clk;

	end
      
endmodule

