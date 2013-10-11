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

// Interface to Cypress FX2 bus
// A packet is 512 Bytes, the fifo has 4096 lines of 18 bits each

`include "../include/fpga_regs_common.v"
`include "../include/fpga_regs_marine_radar.v"

module feed_fifo
  ( // Read/USB side
    input usbclk,
    input bus_reset,
    output [15:0] usbdata,
    input RD,
    output reg have_pkt_rdy,
    input clear_status,
    // Write/DSP side
    input rxclk,
    input reset,  // DSP side reset (used here), do not reset registers
    input rxstrobe,
    input wire init,
    input wire [15:0] data,
    input wire [31:0] num_data,
    output reg rx_overrun,
    output reg fifo_hungry // need more samples? controls upstream pipeline
    );
      
   wire [11:0] 	  rxfifolevel;
   wire 	  rx_full;
   reg [31:0] 	  count_data;
   
   // USB Read Side of FIFO
   always @(negedge usbclk)
     have_pkt_rdy <= (rxfifolevel >= 256);

   // 257 Bug Fix
   reg [8:0] 	  read_count;
   always @(negedge usbclk)
     if(bus_reset)
       read_count <= 0;
     else if(RD)
       read_count <= read_count + 9'b1;
     else
       read_count <= 0;

   // write counter for knowing when to stamp packet serial number.
   // We provide 16-bit serial numbers on USB packets.  This lets us
   // track dropping of USB packets.

   reg [7:0] 	  write_count;
   reg [31:0] 	  serial_no;
   wire [15:0] 	  stamped_data; // data with portion of packet serial number possibly overwriting upper 4 bits

   // stamp serial number onto data word, if appropriate
   // FIXME:  assumes 12-bit samples, 32-bit serial number, serial number occupies first 8 metadata slots in packet

   assign stamped_data = {{(write_count == 0) ? serial_no[ 3: 0] :
			   (write_count == 1) ? serial_no[ 7: 4] :
			   (write_count == 2) ? serial_no[11: 8] :
			   (write_count == 3) ? serial_no[15:12] :
                           (write_count == 4) ? serial_no[19:16] : 
			   (write_count == 5) ? serial_no[23:20] :
			   (write_count == 6) ? serial_no[27:24] :
			   (write_count == 7) ? serial_no[31:28] :
			   data[15:12]},
			  data[11:0]};   
   // FIFO

   wire [1:0] 	  dummy;
   
   fifo_4k_18 rxfifo 
     ( // DSP Write Side
       .data ( {1'b0, 1'b0, stamped_data} ),
       .wrreq (~rx_full & rxstrobe),
       .wrclk ( rxclk ),
       .wrfull ( rx_full ),
       .wrempty ( ),
       .wrusedw ( ),
       // USB Read Side
       .q ( {dummy[0],dummy[1],usbdata} ),
       .rdreq ( RD & ~read_count[8] ), 
//       .rdreq ( RD ),
       .rdclk ( ~usbclk ),
       .rdfull ( ),
       .rdempty ( ),
       .rdusedw ( rxfifolevel ),
       // Async, shared
       .aclr ( reset | bus_reset) );

   // Detect overrun
   
   reg clear_status_dsp, rx_overrun_dsp;
   always @(posedge rxclk)
     clear_status_dsp <= clear_status;

   always @(negedge usbclk)
     rx_overrun <= rx_overrun_dsp;

   always @(posedge rxclk)
     if(reset | bus_reset)
       begin
	  rx_overrun_dsp <= 1'b0;
	  write_count <= #1 8'b0;
	  serial_no <= #1 16'b0;
       end
     else if (~rx_full & rxstrobe)
       begin
	  write_count <= #1 write_count + 1'b1; // we've written one more word to the packet
	  if (&write_count)			// if the last word has been written, bump up
	    serial_no <= #1 serial_no + 1'b1;   // the serial number for the next packet
       end
     else if(rxstrobe & rx_full)
       rx_overrun_dsp <= 1'b1;
     else if(clear_status_dsp)
       rx_overrun_dsp <= 1'b0;


   always @(posedge rxclk)
     if(reset | bus_reset)
       begin
	  fifo_hungry <= #1 0;
	  count_data <= #1 0;
       end
     else if (init)
       begin
	  fifo_hungry <= #1 1;
	  count_data <= #1 num_data;
       end
     else if (~rx_full & rxstrobe) 
       begin
	  count_data = count_data - 1'b1;
	  fifo_hungry = #1 | count_data;
       end
   
endmodule // feed_fifo

