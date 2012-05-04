// -*- verilog -*-
//
//  USRP - Universal Software Radio Peripheral
//
//  Copyright (C) 2003 Matt Ettus
//  Copyright (C) 2010 John Brzustowski
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Boston, MA  02110-1301  USA
//

// timing:
//
// if rate == 0
//                   ________          ________          ________          ________          __
// clock:        ___/        \________/        \________/        \________/        \________/  
//                   __________________________________________________________________________
// enable:       ___/                                                                   
//                                    _________________________________________________________
// strobe_out:   ____________________/                                                                   


// if rate == 1  
//                   ________          ________          ________          ________          __
// clock:        ___/        \________/        \________/        \________/        \________/  
//                   __________________________________________________________________________
// enable:       ___/                                                                   
//                                    _________________                   _________________
// strobe_out:   ____________________/                 \_________________/                 \___
//
//
// ... and so on.

// i.e. there is a 1 clock delay between enable and the first assertion of strobe_out 
// then a <rate> clock delay between subsequent assertions of strobe_out

module decim
  (clock,reset,enable,init,data_in,strobe_in,rate,data_out,strobe_out);

   parameter data_width = 16;
   parameter rate_width = 16;
   
   input clock;
   input reset;
   input enable;
   input init;
   input [data_width-1:0] data_in;
   input 		  strobe_in;
   input [rate_width-1:0] rate;

   output reg [data_width-1:0] data_out;
   output reg 		       strobe_out;
      
   reg [rate_width-1:0]   count;
   
   always @(posedge clock)
     if(reset | ~enable | init)
       begin
	  count <= #1 0;
	  strobe_out <= #1 0;
       end
     else if (strobe_in)
       begin
	  if (count == 0)
	    begin
	       count <= #1 rate;
	       strobe_out <= #1 1;
	       data_out <= #1 data_in;
	    end
	  else
	    begin
	       count <= #1 count - 1'b1;
	       strobe_out <= #1 0;
	    end
       end
   
endmodule // simple_decim

