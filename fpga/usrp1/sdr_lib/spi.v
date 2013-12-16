// -*- verilog -*-
//
//  Coypright (C) 2013 John Brzustowski
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

// Simple SPI with coarse timing.  Note: no control over how long
// we wait between consecutive writes, but most devices impose some
// limit which must be respected by instantiator.

// value_out is grabbed on assertion of strobe_out;
// when SPI exchange has completed, strobe_in is asserted and
// read value is in value_in.

`include "../../include/fpga_regs_common.v"
`include "../../include/fpga_regs_marine_radar.v"

module spi
  ( input                        clock,  // system clock
    input                        reset,
    input                        strobe_out, // new value is available to be written
    output reg                   strobe_in,// new value is available to be read
    input wire [word_size-1 : 0] value_out, // value to be written
    output reg [word_size-1 : 0] value_in, // value read
    output reg                   MOSI, // data in line to SPI slave
    input wire                    MISO, // data out line from SPI slave
    output reg                   SCLCK, // serial clock line to SPI slave
    output reg                   SYNC);  // chip select line (active low) to SPI slave

   parameter clock_decim_rate = 2; // 0 means none; 1 means every 2nd clock, 2 = every 3rd clock, etc.
                                   // each timing step is one decimated clock, so choose a decimation rate large
                                   // enough to accomodate *all* required minimum time delays.
   
   parameter word_size = 16;       // number of bits in shift register

   // possible values for state
`define SPI_STATE_IDLE       2'd0  // not currently sending 
`define SPI_STATE_WAIT_TSYNC 2'd1  // two uses: SYNC & SCLCK have been set high, waiting before bringing SYNC low 
                                   //       or: a bit has already been sent and SCLCK is high; time to set up data
`define SPI_STATE_WAIT_TDS   2'd2  // SYNC has been set low, data has been set up; waiting before bringing clock low
`define SPI_STATE_WAIT_TDH   2'd3  // data has been set up; clock has been brought low; waiting before letting clock rise
   
   reg [1:0]                     state; // current state (from SPI_STATE_... values)
   
   reg [7:0]                     bit_count; // number of bits left to send; when zero, we're idle; FIXME: implies max word size 255
   reg [word_size-1 : 0]         bit_buff;  // bit buffer; sent from MSB to LSB

   wire                          decimated_clock; // decimated clock signal; used for all SPI timing delays.
   reg                           prev_decimated_clock; // previous decimated clock

   decim clock_decimator (.clock(clock), .reset(reset), .enable(|state), .init(strobe_out), .strobe_in(clock), .rate(clock_decim_rate), .strobe_out(decimated_clock));
   
   always @(posedge clock)
     begin
        if(reset)
          begin
	     bit_count <= #1 1'b0;
             state <= #1 `SPI_STATE_IDLE;
             strobe_in <= #1 1'b0;
             value_in <= #1 1'b0;             
          end
        else
          begin
             prev_decimated_clock <= #1 decimated_clock;
             
             if (state == `SPI_STATE_IDLE)
               begin
                  if (strobe_out)
                    begin
	               bit_buff[word_size-1 : 0] <= #1 value_out[word_size-1 : 0];
                       bit_count <= #1 word_size;
                       SYNC <= #1 1'b1;
                       SCLCK <= #1 1'b1;
                       state <= #1 `SPI_STATE_WAIT_TSYNC;
                       strobe_in <= #1 1'b0;
	            end
               end // if (state == `SPI_STATE_IDLE)
          end
        if (decimated_clock & ~prev_decimated_clock)
          begin
             case (state)
               `SPI_STATE_WAIT_TSYNC:
                 begin
                    SYNC <= #1 1'b0; // bring down sync line (NO-OP after 1st bit of word has been sent)
                    
                    MOSI <= #1 bit_buff[word_size - 1]; // shift out MSB
                    value_in <= #1 {value_in[word_size - 2:0], MISO}; // shift in LSB
                    bit_buff <= #1 {bit_buff[word_size - 2:0], 1'b0};
                    bit_count <= #1 bit_count - 1'b1;
                    
                    state <= #1 `SPI_STATE_WAIT_TDS;
                 end
               `SPI_STATE_WAIT_TDS:
                 begin
                    SCLCK <= #1 1'b0; // bring down clock line
                    state <= #1 `SPI_STATE_WAIT_TDH;
                 end
               `SPI_STATE_WAIT_TDH:
                 begin
                    SCLCK <= #1 1'b1; // take clock line back up
                    if (| bit_count)
                      begin
                         state <= #1 `SPI_STATE_WAIT_TSYNC; // a bit kludgy
                      end
                    else
                      begin
                         MOSI <= #1 1'b0; // pull down MOSI line for lower power consumption between writes
                         state <= #1 `SPI_STATE_IDLE;
                         strobe_in <= #1 1'b1;  // input data value ready
                      end
                 end
             endcase
          end
     end
endmodule // spi
