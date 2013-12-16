// -*- verilog -*-
//
//  USRP - Universal Software Radio Peripheral
//
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
// strobe_in:    ___/        \________/        \________/        \________/        \________/  
//                   __________________________________________________________________________
// enable:       ___/                                                                   
//                                    _________________________________________________________
// strobe_out:   ____________________/                                                                   


// if rate == 1  
//                   ________          ________          ________          ________          __
// strobe_in:    ___/        \________/        \________/        \________/        \________/  
//                   __________________________________________________________________________
// enable:       ___/                                                                   
//                                    _________________                   _________________
// strobe_out:   ____________________/                 \_________________/                 \___
//
//
// ... and so on.

// i.e. there is a 1 clock delay between enable and the first assertion of strobe_out 
// then a <rate> clock delay between subsequent assertions of strobe_out

// the decimation rate is grabbed from <rate> when <init> is true.

module decim
  (clock,reset,enable,init,strobe_in,rate,strobe_out);

   parameter rate_width = 16;
   
   input clock;
   input reset;
   input enable;
   input init;
   input strobe_in;
   input [rate_width-1:0] rate;

   output reg 		       strobe_out;
      
   reg [rate_width-1:0]   count;
   reg [rate_width-1:0]   rate_cached;
   
   always @(posedge clock)
     if(reset | ~enable | init)
       begin
	  count <= #1 0;
	  strobe_out <= #1 0;
	  if (init)
	    rate_cached <= #1 rate;
       end
     else if (strobe_in)
       begin
	  if (count == 0)
	    begin
	       count <= #1 rate_cached;
	       strobe_out <= #1 1;
	    end
	  else
	    begin
	       count <= #1 count - 1'b1;
	       strobe_out <= #1 0;
	    end
       end
     else
       strobe_out <= #1 0;
   
endmodule // decim

