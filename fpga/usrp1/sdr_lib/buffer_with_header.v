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

// Buffer that packs metadata into high-order bits of sample words.
//
// meta[127:0], sample[15:0][n]
// -> {meta[3:0], sample[11:0][0]},
//    {meta[7:4], sample[11:0][1]},
//    ...,
//    {meta[127:124], sample[11:0][31]},
//    {0, 0, 0, 0,    sample[11:0][32]},
//    {0, 0, 0, 0,    sample[11:0][33]},
//    ...,
//    {0, 0, 0, 0,    sample[11:0][n-1]}

module buffer_with_header
  (clock,reset,enable,init,data_in,strobe_in,meta_data,data_out,strobe_out);
   
   parameter data_width = 16;
   parameter data_width_used = 12; // must be less than data_width!
   parameter meta_data_width = 128;
   
`define pack_width (data_width - data_width_used)
`define num_packs meta_data_width / `pack_width
   
   input clock;
   input reset;
   input enable;
   input init;
   input [data_width-1:0]      data_in;
   input 		       strobe_in;
   input [meta_data_width-1:0] meta_data;
   output reg [data_width-1:0] data_out;
   output reg 		       strobe_out;
   
   reg [meta_data_width-1:0]   hold_meta;
   
   always @(posedge clock)
     if(reset | ~enable)
       begin
	  strobe_out <= #1 0;
       end
     else if (init)
       begin
	  hold_meta <= #1 meta_data;
	  strobe_out <= #1 0;
       end
     else if (strobe_in)
       begin
	  data_out <= #1 {hold_meta[`pack_width - 1:0], data_in[data_width_used - 1:0]};
	  hold_meta <= #1 hold_meta >> `pack_width;
	  strobe_out <= #1 1;
       end
     else
       strobe_out <= #1 0;
   
endmodule // buffer_with_header


