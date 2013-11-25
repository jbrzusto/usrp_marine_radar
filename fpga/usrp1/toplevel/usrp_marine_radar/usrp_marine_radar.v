// -*- verilog -*-
//
//  USRP - Universal Software Radio Peripheral
//
//  Copyright (C) 2003,2004 Matt Ettus
//
//  Modifications for pulsed marine radar.
//
//  Coypright (C) 2010-2012 John Brzustowski
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

// Top level module for a baseband pulse digitizer using one or two
// LF_RX daughter boards.  

`include "../../include/fpga_regs_common.v"
`include "../../include/fpga_regs_marine_radar.v"

module usrp_marine_radar
(output MYSTERY_SIGNAL,
 input 		   master_clk,
 input 		   SCLK,
 input 		   SDI,
 inout 		   SDO,
 input 		   SEN_FPGA,

 input 		   FX2_1,
 output 	   FX2_2,
 output 	   FX2_3,
 
 input wire [11:0] rx_a_a, // video 
 input wire [11:0] rx_b_a, // trigger
 input wire [11:0] rx_a_b, // heading (with dual LFRX configuration)
 input wire [11:0] rx_b_b, // azimuth (with dual LFRX configuration)
  // USB interface
 input 		   usbclk,
 input wire [2:0]  usbctl,
 output wire [1:0] usbrdy,
 inout [15:0] 	   usbdata, // NB Careful, inout

 // These are the general purpose i/o's that go to the daughterboard slots
 inout wire [15:0] io_rx_a,
 inout wire [15:0] io_rx_b
 );	
   wire [15:0] debugdata,debugctrl;
   assign MYSTERY_SIGNAL = 1'b0;
   
   wire clk64;
   
   wire WR = usbctl[0];
   wire RD = usbctl[1];
   wire OE = usbctl[2];

   wire have_pkt_rdy;
   assign usbrdy[1] = have_pkt_rdy;

   wire   clear_status = FX2_1;
   wire   rx_overrun;
   
   assign FX2_2 = rx_overrun;
      
   wire [15:0] usbdata_out;
   
   wire        clk_decim; // decimated clock
   wire        eventually_enable_rx;
   reg         enable_rx;  // a delayed copy of eventually_enable_rx
   reg [32:0]  enable_rx_delay_counter; // counts down time between assertion of eventually_enable_rx and enable_rx
   
   wire [15:0]  decim_rate;
   wire        vid_negate;
   wire [11:0] trig_thresh_excite;
   wire [11:0] trig_thresh_relax;
   wire [11:0] ACP_thresh_excite;
   wire [11:0] ACP_thresh_relax;
   wire [31:0] ACP_latency;
   wire [11:0] ARP_thresh_excite;
   wire [11:0] ARP_thresh_relax;
   wire [31:0] ARP_latency;
   wire [31:0] trig_delay;
   wire [31:0] trig_latency;
   wire [31:0] n_samples;
   wire [2:0]  marine_radar_mode; // mode: 0 = normal; 1 = raw video signal; 2 = raw trigger signal; 3 = raw ARP signal; 4 = raw ACP signal;
   // 5 = interleave all raw

   // multiplex selection for signal sources:
   wire [31:0] signal_sources;

   // Components for each signal:

   wire [7:0]  VID_source = signal_sources[31:24];
   wire [7:0]  TRG_source = signal_sources[23:16];
   wire [7:0]  ARP_source = signal_sources[15:8];
   wire [7:0]  ACP_source = signal_sources[7:0];

   // each component is a selector from this list:
   
`define SIGNAL_SOURCE_RX_A_A	8'd0
`define SIGNAL_SOURCE_RX_B_A	8'd1
`define SIGNAL_SOURCE_RX_A_B	8'd2
`define SIGNAL_SOURCE_RX_B_B	8'd3
`define SIGNAL_SOURCE_IO_RX_A_0 8'd4
`define SIGNAL_SOURCE_IO_RX_A_1 8'd5
`define SIGNAL_SOURCE_IO_RX_B_0 8'd6
`define SIGNAL_SOURCE_IO_RX_B_1 8'd7

   wire [15:0] n_ACPs_per_sweep;
   wire        use_ACP_for_sweeps;
   
   // strobes asserted for one clk64 tick when each signal is detected:
   wire       trigger_strobe;
   wire       ARP_strobe;
   wire       ACP_strobe;
    
   // there are two major digitization modes: pulsed and continuous
   reg        non_stop;	       // digitize continuously, if true

   reg 	      start_pipeline;  // asserted for one clock when pipeline is to begin (start of pulse or right away)
   wire 	      pipeline_active; // data is moving through the pipeline

   wire        rx_bus_reset;
   wire        rx_dsp_reset;

   wire [7:0] 	 settings;
   
   // Tri-state bus macro
   bustri bustri( .data(usbdata_out),.enabledt(OE),.tridata(usbdata) );

   assign      clk64 = master_clk;

   wire [31:0] 	 n_trigs;			// number of triggers seen since reset
   wire [31:0] 	 n_ACPs;			// number of ACPs seen since reset
   wire [31:0] 	 n_ARPs;			// number of ARPs seen since reset
   wire [31:0] 	 ticks_since_last_ARP;		// number of clock ticks since last ARP pulse
   wire [31:0] 	 ticks_since_last_ACP;		// number of clock ticks since last ACP pulse
   reg [31:0] 	 ACP_count_last_ARP;		// value of ACP counter at last ARP pulse
   reg [31:0] 	 ACP_age_last_ARP;		// ticks since last ACP at last ARP pulse
   reg           ARP_strobe_since_last_ACP;	// has an ARP pulse been seen since the most recent ACP pulse?
   reg [31:0] 	 last_ACP_interval_with_ARP;	// length ACP interval in which last ARP pulse occurred
   reg  	 got_trig_since_last_ACP;	// has a trigger pulse been seen since the most recent ACP pulse?
   reg [31:0] 	 last_ACP_interval_with_trig;	// (length of) most recent ACP interval in which a trigger occurred
   wire [31:0] 	 trig_interval;			// number of clock ticks between most recent consecutive trigger pulses
   wire [31:0] 	 ACP_interval;			// number of clock ticks between most recent consecutive ACP pulses
   wire [31:0] 	 ARP_interval;			// number of clock ticks between most recent consecutive ARP pulses

   reg [15:0] 	 n_ACPs_this_sweep;             // number of ACP since start of most recent sweep (only valid when use_ACP_for_sweeps is true)
   reg [63:0] 	 ticks_at_sweep_start;          // value of clock_ticks at start of most recent sweep (only valid when use_ACP_for_sweeps is true)
   reg [31:0] 	 ticks_this_sweep;		// number of clock ticks since start of most recent sweep (only valid when use_ACP_for_sweeps is true)
   reg           got_first_ACP;                 // have we seen an ACP since starting to digitize? (only valid when use_ACP_for_sweeps is true)
   
   wire        serial_strobe;
   wire [6:0]  serial_addr;
   wire [31:0] serial_data;

   reg [63:0] clock_ticks;  // ticks since device reset

   reg [1:0]  ticks_in_pulse; // ticks since start of pulse (mod 4)
      
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Receive Side

   wire counter = settings[1];

   wire [11:0] VID_signal = rx_a_a[11:0];
   
	       // (VID_source == `SIGNAL_SOURCE_RX_A_A) ? rx_a_a[11:0] :
	       // (VID_source == `SIGNAL_SOURCE_RX_B_A) ? rx_b_a[11:0] :
	       // (VID_source == `SIGNAL_SOURCE_RX_A_B) ? rx_a_b[11:0] :
	       // (VID_source == `SIGNAL_SOURCE_RX_B_B) ? rx_b_b[11:0] :
	       // (VID_source == `SIGNAL_SOURCE_IO_RX_A_0) ? {io_rx_a[0], 11'b0}:
	       // (VID_source == `SIGNAL_SOURCE_IO_RX_A_1) ? {io_rx_a[1], 11'b0}:
	       // (VID_source == `SIGNAL_SOURCE_IO_RX_B_0) ? {io_rx_b[0], 11'b0}:
	       // (VID_source == `SIGNAL_SOURCE_IO_RX_B_1) ? {io_rx_b[1], 11'b0}:
	       // {12'b0};

   wire [11:0] TRG_signal = rx_b_a[11:0];
   
	       // (TRG_source == `SIGNAL_SOURCE_RX_A_A) ? rx_a_a[11:0] :
	       // (TRG_source == `SIGNAL_SOURCE_RX_B_A) ? rx_b_a[11:0] :
	       // (TRG_source == `SIGNAL_SOURCE_RX_A_B) ? rx_a_b[11:0] :
	       // (TRG_source == `SIGNAL_SOURCE_RX_B_B) ? rx_b_b[11:0] :
	       // (TRG_source == `SIGNAL_SOURCE_IO_RX_A_0) ? {io_rx_a[0], 11'b0}:
	       // (TRG_source == `SIGNAL_SOURCE_IO_RX_A_1) ? {io_rx_a[1], 11'b0}:
	       // (TRG_source == `SIGNAL_SOURCE_IO_RX_B_0) ? {io_rx_b[0], 11'b0}:
	       // (TRG_source == `SIGNAL_SOURCE_IO_RX_B_1) ? {io_rx_b[1], 11'b0}:
	       // {12'b0};

   wire [11:0] ARP_signal = rx_a_b[11:0];
   
	       // (ARP_source == `SIGNAL_SOURCE_RX_A_A) ? rx_a_a[11:0] :
	       // (ARP_source == `SIGNAL_SOURCE_RX_B_A) ? rx_b_a[11:0] :
	       // (ARP_source == `SIGNAL_SOURCE_RX_A_B) ? rx_a_b[11:0] :
	       // (ARP_source == `SIGNAL_SOURCE_RX_B_B) ? rx_b_b[11:0] :
	       // (ARP_source == `SIGNAL_SOURCE_IO_RX_A_0) ? {io_rx_a[0], 11'b0}:
	       // (ARP_source == `SIGNAL_SOURCE_IO_RX_A_1) ? {io_rx_a[1], 11'b0}:
	       // (ARP_source == `SIGNAL_SOURCE_IO_RX_B_0) ? {io_rx_b[0], 11'b0}:
	       // (ARP_source == `SIGNAL_SOURCE_IO_RX_B_1) ? {io_rx_b[1], 11'b0}:
	       // {12'b0};

   wire [11:0] ACP_signal = rx_b_b[11:0];
   
	       // (ACP_source == `SIGNAL_SOURCE_RX_A_A) ? rx_a_a[11:0] :
	       // (ACP_source == `SIGNAL_SOURCE_RX_B_A) ? rx_b_a[11:0] :
	       // (ACP_source == `SIGNAL_SOURCE_RX_A_B) ? rx_a_b[11:0] :
	       // (ACP_source == `SIGNAL_SOURCE_RX_B_B) ? rx_b_b[11:0] :
	       // (ACP_source == `SIGNAL_SOURCE_IO_RX_A_0) ? {io_rx_a[0], 11'b0}:
	       // (ACP_source == `SIGNAL_SOURCE_IO_RX_A_1) ? {io_rx_a[1], 11'b0}:
	       // (ACP_source == `SIGNAL_SOURCE_IO_RX_B_0) ? {io_rx_b[0], 11'b0}:
	       // (ACP_source == `SIGNAL_SOURCE_IO_RX_B_1) ? {io_rx_b[1], 11'b0}:
	       // {12'b0};
   
   // output of stage i of pipeline is sample_i, which is ready when strobe_i goes high
   
   wire strobe_1;
   wire strobe_2;
   wire strobe_3;

   wire [15:0] sample_1;
   wire [15:0] sample_2;
   wire [15:0] sample_3;

   wire        new_mode;
   
   always @(posedge clk64)
     if(rx_dsp_reset)
       begin
	  clock_ticks <= #1 64'd0;
	  non_stop <= #1 1'b0;
	  start_pipeline <= #1 1'b0;
	  enable_rx <= #1 1'b0;
	  enable_rx_delay_counter <= #1 32'd12800000; // delay 0.2 seconds before making enable_rx effective

	  ARP_strobe_since_last_ACP <= #1 1'b0;
	  got_trig_since_last_ACP <= #1 1'b0;
	  got_first_ACP <= #1 1'b0;
       end
     else
       begin
	  if (new_mode)
	    non_stop <= #1 | marine_radar_mode;

	  clock_ticks <= #1 clock_ticks + 64'd1;

	  if (enable_rx)
	    begin
	       if ((~non_stop & trigger_strobe & ~pipeline_active) | (non_stop & ~pipeline_active))
		 begin
		    start_pipeline <= #1 1'b1;
		    ticks_in_pulse <= #1 2'b0;
		    got_trig_since_last_ACP <= #1 trigger_strobe;
		 end
	       else
		 begin
		    start_pipeline <= #1 1'b0;
		    if (clk_decim)
		      ticks_in_pulse <= #1 ticks_in_pulse + 2'b1;
		 end
	    end

	  if (eventually_enable_rx)
	    begin
	       if (| enable_rx_delay_counter)
		 enable_rx_delay_counter <= #1 enable_rx_delay_counter - 1;
	       else
		 enable_rx <= #1 1'b1;
	    end
	  else
	    begin
	       enable_rx <= #1 1'b0;
	       enable_rx_delay_counter <= #1 32'd6400000;
	    end // else: !if(eventually_enable_rx)

	  if (ARP_strobe)
	    begin
	       ACP_count_last_ARP <= #1 n_ACPs;
	       ACP_age_last_ARP <= #1 ticks_since_last_ACP;
	       ARP_strobe_since_last_ACP <= #1 1'b1;
	    end
	  if (ACP_strobe)
	    begin
	       if (got_trig_since_last_ACP)
		 begin
		    last_ACP_interval_with_trig <= #1 ACP_interval;
		    got_trig_since_last_ACP <= #1 1'b0;
		 end
	       if (ARP_strobe_since_last_ACP)
		 begin
		    last_ACP_interval_with_ARP <= #1 ACP_interval;
		    ARP_strobe_since_last_ACP <= #1 1'b0;
		 end
	       if (use_ACP_for_sweeps)
		 begin
		    if (! got_first_ACP)
		      begin
			 // we haven't started the first sweep yet, so
			 // call the leading edge of this ACP pulse the start of the sweep
			 n_ACPs_this_sweep <= #1 16'd0;
			 ticks_at_sweep_start <= #1 clock_ticks;
			 got_first_ACP <= #1 1'b1;
		      end
		    else
		      begin
			 n_ACPs_this_sweep = 1'b1 + n_ACPs_this_sweep;   // NB: blocking assign
			 if (n_ACPs_this_sweep == n_ACPs_per_sweep)
			   begin
			      ticks_at_sweep_start <= #1 clock_ticks;
			      n_ACPs_this_sweep <= #1 16'd0;
			   end
		      end // else: !if(! |ticks_at_sweep_start)
		 end // if (use_ACP_for_sweeps)
	    end // if (ACP_strobe)
	  if (use_ACP_for_sweeps)
	    ticks_this_sweep[31:0] <= #1 clock_ticks[31:0] - ticks_at_sweep_start[31:0];
       end // else: !if(rx_dsp_reset)
   
   trigger_gen trigger_gen
     ( .clock(clk64), .reset(rx_dsp_reset), .enable(enable_rx),
       .signal(rx_b_a), .thresh_excite(trig_thresh_excite),
       .thresh_relax(trig_thresh_relax), .delay(trig_delay), .latency(trig_latency),
       .trigger(trigger_strobe), .counter(n_trigs),
       .prev_interval(trig_interval));

   trigger_gen #(.do_smoothing(0)) trigger_gen_ARP  // not really a trigger; we're just counting these pulses
     ( .clock(clk64), .reset(rx_dsp_reset), .enable(enable_rx),
       .signal(ARP_signal), .thresh_excite(ARP_thresh_excite),
       .thresh_relax(ARP_thresh_relax), .delay(0), .latency(ARP_latency),
       .trigger(ARP_strobe), .counter(n_ARPs),
       .age(ticks_since_last_ARP),
       .prev_interval(ARP_interval));

   trigger_gen #(.do_smoothing(0)) trigger_gen_ACP  // not really a trigger; we're just counting these pulses
     ( .clock(clk64), .reset(rx_dsp_reset), .enable(enable_rx),
       .signal(ACP_signal), .thresh_excite(ACP_thresh_excite),
       .thresh_relax(ACP_thresh_relax), .delay(0), .latency(ACP_latency),
       .trigger(ACP_strobe), .counter(n_ACPs),
       .age(ticks_since_last_ACP),
       .prev_interval(ACP_interval));


   // decimate the clock

   decim clock_decim
     ( .clock(clk64), .reset(rx_dsp_reset), .enable(pipeline_active), .init(start_pipeline),
       .strobe_in(1'b1),
       .rate(decim_rate),
       .strobe_out(clk_decim));

   // pipeline for sample handling
   // Stage 1: pick a data source depending on marine_radar_mode

   multiplex  #(.num_channels(6)) stage_1
     ( .clock(clk64), .reset(rx_dsp_reset), .enable(pipeline_active),
       .select (marine_radar_mode),     
       .data_in ({
		  {4'b0, {  // mode 5: interleave raw all, changing every decimated clock
			    (  ticks_in_pulse == 2'd0) ? VID_signal
			    : (ticks_in_pulse == 2'd1) ? TRG_signal
			    : (ticks_in_pulse == 2'd2) ? ARP_signal
			    :                            ACP_signal
			    }
		   },
		  {4'b0, ACP_signal}, // mode 4: raw ACP
		  {4'b0, ARP_signal}, // mode 3: raw ARP
		  {4'b0, TRG_signal}, // mode 2: raw trigger
		  {4'b0, VID_signal}, // mode 1: raw video
		  {4'b0, vid_negate ? 12'h0fff - VID_signal : VID_signal}  // mode 0: pulse-sync'd video, possibly negated
		  }),
       .strobe_in (clk_decim),
       .data_out (sample_1),
       .strobe_out (strobe_1));
   
   // Stage 2: interleave debug information

   interleave stage_2
     ( .clock(clk64), .reset(rx_dsp_reset), .enable(pipeline_active), .init(start_pipeline),
       .data1_in(sample_1), .data2_in({4'b0,clock_ticks[11:0]}),
       .strobe_in(strobe_1),
       .active(counter),
       .data_out(sample_2), 
       .strobe_out(strobe_2));

   // Stage 3: pack with metadata
       
       // If we have at least one bit per 16-bit sample to spare, then
       // we have at least 256 bits to spare per USB packet.
       // In fact, with 12-bit samples, we have 1024 bits per USB packet
       // available, and we'll use some of these for for other meta
       // data later on (tilter feedback, weather station sentences, ...)
       // For now, we're not being very efficient.  Some of these sizes
       // could be reduced (e.g. down to 56 bits for clock_ticks is still 35.7 years' time)
   
   pack_metadata stage_3
     ( .clock(clk64), .reset(rx_dsp_reset), .enable(pipeline_active), .init(start_pipeline),
       .data_in(sample_2),
       .strobe_in(strobe_2),
       .meta_data({
		   clock_ticks[63:0],
		   trig_interval[31:0],
		   ARP_interval[31:0],
		   last_ACP_interval_with_trig[31:0],
		   use_ACP_for_sweeps ? ticks_this_sweep[31:0] : ticks_since_last_ARP[31:0],
		   ticks_since_last_ACP[31:0],
		   use_ACP_for_sweeps ? 32'b0 : ACP_count_last_ARP[31:0],
		   use_ACP_for_sweeps ? 32'b0 : ACP_age_last_ARP[31:0],
		   last_ACP_interval_with_ARP[31:0],
		   n_trigs[31:0],
		   n_ARPs[31:0], 
		   use_ACP_for_sweeps ? {16'b0, n_ACPs_this_sweep[15:0]} : n_ACPs[31:0],
		   32'b0, // space for packet serial number; which will be in first 8 samples of packet
		  }), 
       .data_out(sample_3),
       .strobe_out(strobe_3));

   // Stage 4: feed fifo, count samples (possibly stopping pipeline), stamp USB packets with serial numbers.
   
   feed_fifo stage_4
     ( .usbclk(usbclk),.bus_reset(rx_bus_reset), 
       .usbdata(usbdata_out),.RD(RD),.have_pkt_rdy(have_pkt_rdy),
       .clear_status(clear_status),
       .rxclk(clk64),.reset(rx_dsp_reset),
       .rxstrobe(strobe_3 & pipeline_active),
       .init(start_pipeline),
       .data(sample_3),
       .num_data(n_samples),
       .rx_overrun(rx_overrun),
       .fifo_hungry(pipeline_active));
   	
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Control Functions

   serial_io serial_io
     ( .master_clk(clk64),.serial_clock(SCLK),.serial_data_in(SDI),
       .enable(SEN_FPGA),.reset(1'b0),.serial_data_out(SDO),
       .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe),
       .readback_0(n_ACPs),.readback_1(n_ARPs),.readback_2(32'habcdef01),.readback_3(32'hf0f0931a),
       .readback_4(clock_ticks[31:0]),.readback_5(clock_ticks[63:32]),.readback_6(n_trigs),.readback_7(32'hf0adf0ad)
       );

   wire [15:0] reg_0,reg_1,reg_2,reg_3;
   master_control_marine_radar master_control_marine_radar
     ( .master_clk(clk64),.usbclk(usbclk),
       .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe),
       .rx_bus_reset(rx_bus_reset),
       .rx_dsp_reset(rx_dsp_reset),
       .enable_rx(eventually_enable_rx),
       .decim_rate(decim_rate),
       .vid_negate(vid_negate),
       .trig_thresh_excite(trig_thresh_excite),
       .trig_thresh_relax(trig_thresh_relax),
       .trig_delay(trig_delay),
       .trig_latency(trig_latency),
       .ARP_thresh_excite(ARP_thresh_excite),
       .ARP_thresh_relax(ARP_thresh_relax),
       .ARP_latency(ARP_latency),
       .ACP_thresh_excite(ACP_thresh_excite),
       .ACP_thresh_relax(ACP_thresh_relax),
       .ACP_latency(ACP_latency),
       .n_samples(n_samples),
       .marine_radar_mode(marine_radar_mode),
       .new_mode(new_mode),
       .signal_sources(signal_sources),
       .n_ACPs_per_sweep(n_ACPs_per_sweep),
       .use_ACP_for_sweeps(use_ACP_for_sweeps)
       );
      
   io_pins io_pins
     (.io_0(/* io_tx_a */),.io_1(io_rx_a),.io_2(/* io_tx_b */),.io_3(io_rx_b),
      .reg_0(reg_0),.reg_1(reg_1),.reg_2(reg_2),.reg_3(reg_3),
      .clock(clk64),.rx_reset(rx_dsp_reset),.tx_reset(/* tx_dsp_reset */),
      .serial_addr(serial_addr),.serial_data(serial_data),.serial_strobe(serial_strobe));

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Misc Settings
   setting_reg #(`FR_MODE) sr_misc(.clock(clk64),.reset(rx_dsp_reset),.strobe(serial_strobe),.addr(serial_addr),.in(serial_data),.out(settings));


   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // AUX DIGITAL OUTPUTS
   // reg_1[0] : tilter control serial output line

//    uart #(.CLOCK_DIVIDE(64000000 / (4800 * 4)), // serial I/O rate is 4800 bps
// 	  .TWO_TX_STOP_BITS(0)) // only send 1 stop bit, so each byte of output takes 10 serial clocks
//    tilter_byte_out
//      (.clk(clk64),
//       .rst(rx_dsp_reset),
//       .tx(reg_1[0]),
//       .transmit(tilter_out_byte_ready),
//       .tx_byte(tilter_out_byte),
//       .is_transmitting(tilter_out_active)
//       );
   

endmodule // usrp_marine_radar
