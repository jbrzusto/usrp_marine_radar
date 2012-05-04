// -*- verilog -*-
//
//  USRP - Universal Software Radio Peripheral
//
//  Copyright (C) 2003 Matt Ettus
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

module averaging_decim
  (clock,reset,enable,log2rate,data,strobe_out,data_out);
   parameter rate_width = 16;
   parameter data_width = 12;
   
   input clock;
   input reset;
   input enable;
   input [rate_width-1:0] log2rate;
   input [data_width-1:0] data;
   output reg [data_width-1:0] data_out;
   
   output reg  strobe_out;	
   
   reg [rate_width-1:0]   count;
   reg [rate_width+data_width-1:0] sum;

   always @(posedge clock)
     if(reset | ~enable)
       begin
	  count <= #1 1'b1 << log2rate;
	  strobe_out <= #1 1'b0;
	  sum <= #1 0;
       end
     else if (enable)
       if (count == 1'b1)
	 begin
	    data_out <= #1 (|log2rate) ? (sum >> log2rate) : data;
	    count <= #1 1'b1 << log2rate;
	    strobe_out <= #1 1'b1;
	    sum <= #1 data;
	 end
       else
	 begin
	    count <= #1 count - 1'b1;
	    strobe_out <= #1 1'b0;
	    sum <= #1 sum + data;
	 end
   
endmodule // averaging_decim

