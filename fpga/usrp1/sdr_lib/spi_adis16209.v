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

// SPI interface to ADIS 16209
// This uses phase=1, polarity=1 flavour of SPI.

// value_out is grabbed on assertion of strobe_out;
// when SPI exchange has completed, strobe_in is asserted and
// read value is in value_in.

`include "../../include/fpga_regs_common.v"
`include "../../include/fpga_regs_marine_radar.v"

module spi_adis16209
  ( input                        clock,  // system clock
    output reg [7:0]             debug, // debug register
    input                        reset,
    input wire                   strobe_out, // new value is available to be written
    output reg                   strobe_in,// new value is available to be read
    input wire [word_size-1 : 0] value_out, // value to be written
    output reg [word_size-1 : 0] value_in, // value read
    output reg                   MOSI, // data in line to SPI slave
    input wire                   MISO, // data out line from SPI slave
    output reg                   SCLCK, // serial clock line to SPI slave
    output reg                   CS);  // chip select line (active low) to SPI slave

   // minimum timing parameters, all in nanoseconds
   
   parameter cs_rest_state = 1; // rest state of CS signal; if 0, time_cs parameter specifies how long
   // to raise before lowering; CS is always active-low.
      
   parameter time_cs = 48.8; // time CS must be held low before SCLCK falling edge
   parameter time_dav = 100; // data valid: time before incoming data is valid
   parameter time_dsu = 24.4; // data set-up: time MOSI must be held before SCLCK rising edge
   parameter time_dhd = 48.8; // data hold: time MOSI must be held after SCLCK rising edge
   parameter time_sfs = 5.0; // time SCLCK must be low before CS rises
   parameter word_size = 16;       // number of bits in shift register

   parameter clock_rate = 64000000;  // rate of clock.

   // possible values for state
   
`define SPI_STATE_IDLE        3'd0  // not currently sending
`define SPI_STATE_WAIT_CS     3'd1  // CS has been pulled low; waiting to pull down SCLCK
`define SPI_STATE_WAIT_DAV    3'd2  // SCLCK has been pulled low; waiting for incoming data to be valid  
`define SPI_STATE_WAIT_DSU    3'd3  // data has been set; waiting before raising SCLCK
`define SPI_STATE_WAIT_DHD    3'd4  // SCLCK has been raised; waiting for data hold time
`define SPI_STATE_WAIT_SFS    3'd5  // all data has been sent; waiting before raising CS

`define CLOCK_SCALE  1.0e-9 * clock_rate * 100; // convert nanoseconds into clocks

   reg [2:0]                     state; // current state (from SPI_STATE_... values)

   reg [7:0]                     bit_count; // number of bits left to send; when zero, we're idle; FIXME: implies max word size 255
   reg [word_size-1 : 0]         value_buff;  // bit buffer; sent from MSB to LSB

   // register for waiting n clock cycles;
   reg [15:0]                    wait_counter;

   always @(posedge clock)
     begin
        if(reset)
          begin
	     bit_count <= #1 1'b0;
             debug[7] <= #1 1'b0;
             
             debug[6:3] <= #1 bit_count[3:0];
             state <= #1 `SPI_STATE_IDLE;
             debug[2:0] <= #1 `SPI_STATE_IDLE;

             strobe_in <= #1 1'b0;
             value_in <= #1 1'b0;
             CS <= #1 1'b1; // CS rests high
             SCLCK <= #1 1'b1; // SCLCK rests high
             MOSI <= #1 1'b0; // MOSI rests low
          end
        else
          begin
             
             case (state)
               `SPI_STATE_IDLE:
                 begin
                    strobe_in <= #1 1'b0;
                    if (strobe_out)
                      begin
                         debug[7] <= #1 1'b1;
	                 value_buff[word_size-1 : 0] <= #1 value_out[word_size-1 : 0];
                         bit_count <= #1 word_size;
                         debug[6:3] <= #1 word_size;
                         CS <= #1 1'b0;
                         wait_counter <= #1 time_cs * `CLOCK_SCALE;
                         state <= #1 `SPI_STATE_WAIT_CS;
                         debug[2:0] <= #1 `SPI_STATE_WAIT_CS;
	              end
                 end // case: `SPI_STATE_IDLE
               
               `SPI_STATE_WAIT_CS:
                 begin
                    if (wait_counter == 0)
                      begin
                         SCLCK <= #1 1'b0;
                         wait_counter <= #1 time_dav * `CLOCK_SCALE;
                         state <= #1 `SPI_STATE_WAIT_DAV;
                         debug[2:0] <= #1 `SPI_STATE_WAIT_DAV;
                      end
                    else
                      wait_counter <= #1 wait_counter - 1'b1;
                 end // case: `SPI_STATE_WAIT_CS

               `SPI_STATE_WAIT_DAV:
                 begin
                    if (wait_counter == 0)
                      begin
                         MOSI <= #1 value_buff[word_size - 1]; // shift out MSB
                         value_in <= #1 {value_in[word_size - 2:0], MISO}; // shift in LSB
                         value_buff <= #1 {value_buff[word_size - 2:0], 1'b0};
                         state <= #1 `SPI_STATE_WAIT_DSU;
                         debug[2:0] <= #1 `SPI_STATE_WAIT_DSU;
                         wait_counter <= #1 time_dsu * `CLOCK_SCALE;
                         debug[6:3] <= #1 bit_count[3:0];
                         bit_count <= #1 bit_count - 1'b1;
                      end // if (wait_counter == 0)
                    else
                      wait_counter <= #1 wait_counter - 1'b1;
                 end // case: `SPI_STATE_WAIT_DAV

               `SPI_STATE_WAIT_DSU:
                 begin
                    if (wait_counter == 0)
                      begin
                         SCLCK <= #1 1'b1; // bring up clock line
                         state <= #1 `SPI_STATE_WAIT_DHD;
                         debug[2:0] <= #1 `SPI_STATE_WAIT_DHD;
                         wait_counter <= #1 time_dhd * `CLOCK_SCALE;
                      end
                    else
                      wait_counter <= #1 wait_counter - 1'b1;
                 end // case: `SPI_STATE_WAIT_DS
               
               `SPI_STATE_WAIT_DHD:
                 begin
                    if (wait_counter == 0)
                      begin
                         if (|bit_count)
                           begin
                              SCLCK <= #1 1'b0; // pull down clock line
                              state <= #1 `SPI_STATE_WAIT_DAV; 
                              wait_counter <= #1 time_dav * `CLOCK_SCALE;
                              debug[2:0] <= #1 `SPI_STATE_WAIT_DAV;
                           end
                         else
                           begin
                              SCLCK <= #1 1'b1;
                              state <= #1 `SPI_STATE_WAIT_SFS;
                              debug[2:0] <= #1 `SPI_STATE_WAIT_SFS;
                              wait_counter <= #1 time_sfs * `CLOCK_SCALE;
                              strobe_in <= #1 1'b1;
                           end // else: !if(|bit_count)
                      end // if (wait_counter == 0)
                    else
                      wait_counter <= #1 wait_counter - 1'b1;
                 end // case: `SPI_STATE_WAIT_DHD

               `SPI_STATE_WAIT_SFS:
                 begin
                    if (wait_counter == 0)
                      begin
                         CS <= #1 1'b1;
                         state <= #1 `SPI_STATE_IDLE;
                       debug[2:0] <= #1 `SPI_STATE_IDLE;
                      end
                    else
                      begin
                         wait_counter <= #1 wait_counter - 1'b1;
                         strobe_in <= #1 1'b0;
                      end
                 end // case `SPI_STATE_WAIT_SFS
             endcase // case (state)
             
          end // else: !if(reset)      
     end // always @ (posedge clock)
endmodule // spi
