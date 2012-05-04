//
// This file is machine generated from fpga_regs_marine_radar.h
// Do not edit by hand; your edits will be overwritten.
//

// Register numbers 0 to 31 are reserved for use in fpga_regs_common.h.
// Registers 64 to 79 are available for custom FPGA builds.

// We use regs from 64 upward for the baseband radar pulse digitizer

// is channel active?
`define FR_ACTIVE                 7'd64      // [0, 1]

// decimation rate
`define FR_DECIM_RATE_MARINE_RADAR 7'd65	// [1,256]

// should we negate video?
`define FR_VID_NEGATE             7'd66      // [0,1]

// trigger excitation threshold
`define FR_TRIG_THRESH_EXCITE     7'd67	// [0,4095]

// trigger relaxation threshold
`define FR_TRIG_THRESH_RELAX      7'd68	// [0,4095]

// trigger latency
`define FR_TRIG_LATENCY           7'd69	// 32 bits unsigned

// trigger delay
`define FR_TRIG_DELAY             7'd70      // 32 bits unsigned

// ACP excitation threshold
`define FR_ACP_THRESH_EXCITE      7'd71	// [0,4095]

// ACP relaxation threshold
`define FR_ACP_THRESH_RELAX       7'd72	// [0,4095]

// ACP latency
`define FR_ACP_LATENCY            7'd73      // 32 bits unsigned

// ARP excitation threshold
`define FR_ARP_THRESH_EXCITE      7'd74	// [0,4095]

// ARP relaxation threshold
`define FR_ARP_THRESH_RELAX       7'd75	// [0,4095]

// ARP latency
`define FR_ARP_LATENCY            7'd76     // 32 bits unsigned

// number of samples
`define FR_N_SAMPLES              7'd77      // 32 bits unsigned

// mode (normal or one of the raw sampling modes)
`define FR_MARINE_RADAR_MODE      7'd78	// 3 bits

// Registers 64 to 95 are reserved for user custom FPGA builds.
// The standard USRP software will not touch these.

`define FR_USER_15                7'd79
`define FR_USER_16                7'd80
`define FR_USER_17                7'd81
`define FR_USER_18                7'd82
`define FR_USER_19                7'd83
`define FR_USER_20                7'd84
`define FR_USER_21                7'd85
`define FR_USER_22                7'd86
`define FR_USER_23                7'd87
`define FR_USER_24                7'd88
`define FR_USER_25                7'd89
`define FR_USER_26                7'd90
`define FR_USER_27                7'd91
`define FR_USER_28                7'd92
`define FR_USER_29                7'd93
`define FR_USER_30                7'd94
`define FR_USER_31                7'd95



