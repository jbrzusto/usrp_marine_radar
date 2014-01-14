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

// Clock decimation: pass every n'th (positive) clock pulse, 
//
// timing:
//
// if rate == 0 (no decimation; clock_out is always true)
//                   ________          ________          ________          ________          __
// clock_in :    ___/        \________/        \________/        \________/        \________/  
//                   __________________________________________________________________________
// enable:       ___/                                                                   
//                                    _________________________________________________________
// clock_out:    ____________________/         


// if rate == 1  
//                   ________          ________          ________          ________          __
// clock_in:     ___/        \________/        \________/        \________/        \________/  
//                   __________________________________________________________________________
// enable:       ___/                                                                   
//                                    _________                           _________
// clock_out:    ____________________/         \_________________________/         \___________
//
//
// ... and so on.

// i.e. there is a 1 clock delay between enable and the first assertion of clock_out 
// then a <rate> clock delay between subsequent assertions of clock_out

// the decimation rate is grabbed from <rate> when <init> is true.

module decim
  (reset,enable,init,rate,clock_in,clock_out);

   parameter rate_width = 32;
   
   input clock_in;
   input reset;
   input enable;
   input init;
   input [rate_width-1:0] rate;

   output reg 		       clock_out;
      
   reg [rate_width-1:0]   count;
   reg [rate_width-1:0]   rate_cached;
   
   always @(posedge clock_in)
     if(reset | ~enable | init)
       begin
	  count <= #1 0;
	  clock_out <= #1 0;
	  if (init)
	    rate_cached <= #1 rate;
       end
     else
       begin
	  if (count == 0)
	    begin
	       count <= #1 rate_cached;
	       clock_out <= #1 1;
	    end
	  else
	    begin
	       count <= #1 count - 1'b1;
	       clock_out <= #1 0;
	    end
       end
endmodule // decim

