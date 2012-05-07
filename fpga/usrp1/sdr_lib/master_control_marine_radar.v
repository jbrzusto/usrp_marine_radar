// -*- verilog -*-
//
//  USRP - Universal Software Radio Peripheral
//
//  Copyright (C) 2003,2005 Matt Ettus
//  Copyright (C) 2007 Corgan Enterprises LLC
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

// Clock, enable, and reset controls for whole system - marine radar
// digitizer.

`include "../include/fpga_regs_common.v"
`include "../include/fpga_regs_marine_radar.v"

module master_control_marine_radar
  ( input master_clk, input usbclk,
    input wire [6:0] serial_addr, input wire [31:0] serial_data, input wire serial_strobe,
    output 	     rx_bus_reset,
    output wire      rx_dsp_reset,
    output wire      enable_rx,
    output wire [15:0] decim_rate,
    output wire       vid_negate,
    output wire [11:0] trig_thresh_excite,
    output wire [11:0] trig_thresh_relax,
    output wire [15:0] trig_latency,
    output wire [15:0] trig_delay,
    output wire [11:0] ARP_thresh_excite,
    output wire [11:0] ARP_thresh_relax,
    output wire [15:0] ARP_latency,
    output wire [11:0] ACP_thresh_excite,
    output wire [11:0] ACP_thresh_relax,
    output wire [15:0] ACP_latency,
    output wire [15:0] n_samples,
    output wire [2:0] marine_radar_mode,
    output wire new_mode,
    output wire [31:0] signal_sources
    );
   
   // FIXME need a separate reset for all control settings 
   // Master Controls assignments
   wire [7:0] 	       master_controls;
   setting_reg #(`FR_MASTER_CTRL) sr_mstr_ctrl(.clock(master_clk),.reset(1'b0),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(master_controls));
   // assign     enable_rx = master_controls[0];
   assign     rx_dsp_reset = master_controls[3];
   // Unused - 4-7

   // Enable
   setting_reg #(`FR_ACTIVE) sr_active(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(enable_rx));
   
   // Decimation rate
   setting_reg #(`FR_DECIM_RATE_MARINE_RADAR) sr_decim(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(decim_rate));

   // Video negate
   setting_reg #(`FR_VID_NEGATE) sr_vid_negate(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(vid_negate));

   // Trigger excitation threshold
   setting_reg #(`FR_TRIG_THRESH_EXCITE) sr_trig_thresh_excite(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(trig_thresh_excite));

   // Trigger relaxation threshold
   setting_reg #(`FR_TRIG_THRESH_RELAX) sr_trig_thresh_relax(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(trig_thresh_relax));
   
   // Trigger latency
   setting_reg #(`FR_TRIG_LATENCY) sr_trig_latency(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(trig_latency));

   // Trigger delay
   setting_reg #(`FR_TRIG_DELAY) sr_trig_delay(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(trig_delay));

   // ARP excitation threshold
   setting_reg #(`FR_ARP_THRESH_EXCITE) sr_ARP_thresh_excite(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(ARP_thresh_excite));

   // ARP relaxation threshold
   setting_reg #(`FR_ARP_THRESH_RELAX) sr_ARP_thresh_relax(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(ARP_thresh_relax));
   
   // ARP latency
   setting_reg #(`FR_ARP_LATENCY) sr_arp_latency(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(ARP_latency));

   // ACP excitation threshold
   setting_reg #(`FR_ACP_THRESH_EXCITE) sr_ACP_thresh_excite(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(ACP_thresh_excite));

   // ACP relaxation threshold
   setting_reg #(`FR_ACP_THRESH_RELAX) sr_ACP_thresh_relax(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(ACP_thresh_relax));
   
   // ACP latency
   setting_reg #(`FR_ACP_LATENCY) sr_acp_latency(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(ACP_latency));

   // Number of samples (per pulse)
   setting_reg #(`FR_N_SAMPLES) sr_n_samples(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(n_samples));
      
   // marine_radar mode
   setting_reg #(`FR_MARINE_RADAR_MODE) sr_marine_radar_mode(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(marine_radar_mode),.changed(new_mode));

   // Number of samples (per pulse)
   setting_reg #(`FR_SIGNAL_SOURCES) sr_signal_sources(.clock(master_clk),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(signal_sources));
      
      
   // Reset syncs for bus (usbclk) side
   // The RX bus side reset isn't used, the TX bus side one may not be needed
   reg 	  tx_reset_bus_sync1, rx_reset_bus_sync1, tx_reset_bus_sync2, rx_reset_bus_sync2;
 	   
   always @(posedge usbclk)
     begin
   	rx_reset_bus_sync1 <= #1 rx_dsp_reset; 
   	rx_reset_bus_sync2 <= #1 rx_reset_bus_sync1;
     end

   assign rx_bus_reset = rx_reset_bus_sync2;

endmodule // master_control_marine_radar
