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

// interleave two data sources, if active; if inactive, just copy first data source
// delay: one clock.

module interleave
  (clock,reset,enable,init,data1_in,data2_in,strobe_in,active,data_out,strobe_out);

   parameter data_width = 16;
   
   input clock;
   input reset;
   input enable;
   input init;
   input [data_width-1:0] data1_in;
   input [data_width-1:0] data2_in;
   input 		  strobe_in;
   input 		  active;

   output reg [data_width-1:0] data_out;
   output reg 		       strobe_out;
      
   reg 			       toggle;
   
   always @(posedge clock)
     if(reset | ~enable | init)
       begin
	  toggle <= #1 0;
	  strobe_out <= #1 0;
       end
     else if (strobe_in)
       begin
	  data_out = #1 (active & toggle) ? data2_in : data1_in;
	  toggle = #1 ~toggle;
	  strobe_out <= #1 1;
       end
     else
       strobe_out <= #1 0;
   
endmodule // interleave

