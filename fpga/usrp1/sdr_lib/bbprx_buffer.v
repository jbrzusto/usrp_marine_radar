// -*- verilog -*-
//
//  USRP - Universal Software Radio Peripheral
//
//  Copyright (C) 2003 Matt Ettus
//  Copyright (C) 2010-2012 John Brzustowski
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

`include "../../firmware/include/fpga_regs_common.v"
`include "../../firmware/include/fpga_regs_bbprx.v"

module bbprx_buffer
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
    input wire [15:0] data,
    output reg rx_overrun
    );
      
   wire [11:0] 	  rxfifolevel;
   wire 	  rx_full;
   
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
   
   // FIFO

   wire [1:0] 	  dummy;
   
   fifo_4k_18 rxfifo 
     ( // DSP Write Side
       .data ( {1'b0, 1'b0, data} ),
       .wrreq (~rx_full & rxstrobe),
       .wrclk ( rxclk ),
       .wrfull ( rx_full ),
       .wrempty ( ),
       .wrusedw ( ),
       // USB Read Side
       .q ( {dummy[0],dummy[1],usbdata} ),
       .rdreq ( RD & ~read_count[8] ), 
       .rdclk ( ~usbclk ),
       .rdfull ( ),
       .rdempty ( ),
       .rdusedw ( rxfifolevel ),
       // Async, shared
       .aclr ( reset ) );

   // Detect overrun
   
   reg clear_status_dsp, rx_overrun_dsp;
   always @(posedge rxclk)
     clear_status_dsp <= clear_status;

   always @(negedge usbclk)
     rx_overrun <= rx_overrun_dsp;

   always @(posedge rxclk)
     if(reset)
       rx_overrun_dsp <= 1'b0;
     else if(rxstrobe & rx_full)
       rx_overrun_dsp <= 1'b1;
     else if(clear_status_dsp)
       rx_overrun_dsp <= 1'b0;


endmodule // bbprx_buffer

