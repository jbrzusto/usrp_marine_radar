/* -*- c++ -*- */
/*
 * Copyright 2003,2004,2006 Free Software Foundation, Inc.
 * 
 * This file is part of GNU Radio
 * 
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.	If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef INCLUDED_FPGA_REGS_MARINE_RADAR_H
#define INCLUDED_FPGA_REGS_MARINE_RADAR_H

// Register numbers 0 to 31 are reserved for use in fpga_regs_common.h.
// Registers 64 to 79 are available for custom FPGA builds.

// We use regs from 64 upward for the baseband radar pulse digitizer

// is channel active?
#define FR_ACTIVE       64      // [0, 1]

// decimation rate
#define	FR_DECIM_RATE_MARINE_RADAR	65	// [1,256]

// should we negate video?
#define FR_VID_NEGATE           66      // [0,1]

// trigger excitation threshold
#define FR_TRIG_THRESH_EXCITE	67	// [0,4095]

// trigger relaxation threshold
#define FR_TRIG_THRESH_RELAX	68	// [0,4095]

// trigger latency
#define FR_TRIG_LATENCY         69	// 32 bits unsigned

// trigger delay
#define FR_TRIG_DELAY           70      // 32 bits unsigned

// ACP excitation threshold
#define FR_ACP_THRESH_EXCITE	71	// [0,4095]

// ACP relaxation threshold
#define FR_ACP_THRESH_RELAX	72	// [0,4095]

// ACP latency
#define FR_ACP_LATENCY		73      // 32 bits unsigned

// ARP excitation threshold
#define FR_ARP_THRESH_EXCITE	74	// [0,4095]

// ARP relaxation threshold
#define FR_ARP_THRESH_RELAX	75	// [0,4095]

// ARP latency
#define FR_ARP_LATENCY		76      // 32 bits unsigned

// number of samples
#define FR_N_SAMPLES            77      // 32 bits unsigned

// mode (normal or one of the raw sampling modes)
#define FR_MARINE_RADAR_MODE    78	// 3 bits

// source for 4 radar signals
#define FR_SIGNAL_SOURCES       79      // 32 bits unsigned

// number of ACP pulses per sweep
#define FR_NUM_ACPS_PER_SWEEP   80      // 16 bits unsigned

// use ACP pulses to delimit sweeps? (rather than using ARP pulse)
#define FR_USE_ACP_FOR_SWEEPS   81      // 1 bit

// Registers 64 to 95 are reserved for user custom FPGA builds.
// The standard USRP software will not touch these.

#define FR_USER_18	82
#define FR_USER_19	83
#define FR_USER_20	84
#define FR_USER_21	85
#define FR_USER_22	86
#define FR_USER_23	87
#define FR_USER_24	88
#define FR_USER_25	89
#define FR_USER_26	90
#define FR_USER_27	91
#define FR_USER_28	92
#define FR_USER_29	93
#define FR_USER_30	94
#define FR_USER_31	95



#endif /* INCLUDED_FPGA_REGS_MARINE_RADAR_H */
