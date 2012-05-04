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

// multiplex data sources
// data are packed from high to low order in data_in. select chooses which slice to use,
// with select=0 choosing the lowest order data.

// NOTE: the case statement below must contain cases for all possible values of select
// because there seems to be no way to programmatically generate them.  Yikes.
// Or (more likely) I just don't know enough verilog.

module multiplex
  (clock,reset,enable,select,data_in,strobe_in,data_out,strobe_out);

   parameter data_width = 16;
   parameter num_channels = 5;
   parameter select_width = 3; // must be able to hold up to num_channels-1
   
   input clock;
   input reset;
   input enable;
   input [select_width - 1:0] select;
   input [data_width * num_channels - 1:0] data_in;
   input 		  strobe_in;
   
   output reg [data_width-1:0] data_out;
   output reg 		       strobe_out;

   always @(posedge clock)
     if(reset | ~enable)
       begin
	  strobe_out <= #1 0;
       end
     else if (strobe_in)
       begin
	  case (select) // ugly: can't use for loop with genvar??
	    0:
	      data_out <= #1 data_in[data_width-1:0];
	    1:
	      data_out <= #1 data_in[2 * data_width-1:data_width];
	    2:
	      data_out <= #1 data_in[3 * data_width-1: 2 * data_width];
	    3:
	      data_out <= #1 data_in[4 * data_width-1: 3 * data_width];
	    4:
	      data_out <= #1 data_in[5 * data_width-1: 4 * data_width];
	    5:
	      data_out <= #1 data_in[6 * data_width-1: 5 * data_width];
	  endcase
	  strobe_out <= #1 1;
       end
     else
       strobe_out <= #1 0;
   
endmodule // multiplex

