`timescale 1ns / 1ps
//--------------------------------------------------------------------------------------------
//  31 2018 21:15:42
//      Component name  : gbest
//--------------------------------------------------------------------------------------------

module gbest(clk_P, reset, ena, p_fit, addr, gb_fit, address);
   input            clk_P;
   input            reset;
	input            ena;
   input  [15:0]    p_fit;
	input  [1:0]     addr;
	
   output [2:0]     address;
	reg 	 [2:0]     address;
	
	reg  	 [15:0]    p_fit1;
//	reg  	 [15:0]    p_fit2;
//	reg  	 [15:0]    p_fit3;
	reg 	 [1:0]     addri;
	
   output [15:0]    gb_fit;
   reg    [15:0]    gb_fit;
   reg    [15:0]    gb_fit1;
	
   parameter   	  states_idle = 4'b0001,
                    states_st1 = 4'b0010,
                    states_st2 = 4'b0100,
						  states_st3 = 4'b1000;
						  
   
	always @(posedge clk_P)
		begin
			addri <= addr;
			p_fit1 <= p_fit;
		end
		
   always @(posedge clk_P)
   begin
      if (reset == 1'b1)
      begin
         gb_fit <= 0;
       //  best <= 0;
			address <= 0;
      end
      else if(ena == 1'b1)
         case (addri)
            states_idle :
               begin
						gb_fit <= 0;
						address <= 0;
               end
            states_st1 :
               begin
                  gb_fit1 <= p_fit1;
               end
				states_st2 :
               begin
			//			p_fit2 <= p_fit;
                  if (gb_fit1 < p_fit1)
                     gb_fit1 <= p_fit1;
               end
				states_st3 :
               begin
						if (p_fit1 < gb_fit1)
                     gb_fit <= gb_fit1;
						else
							gb_fit <= p_fit1;
						address <= address + 1;
               end
            default :
               begin
                  gb_fit <= 0;
						address <= 0;
               end
         endcase
   end
endmodule
