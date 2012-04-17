/* -*- c++ -*- */
/*
 * Copyright 2003,2006,2008,2009 Free Software Foundation, Inc.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cstdio>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <assert.h>
#include <math.h>
#include "time_stuff.h"
#include <usrp/usrp_bbprx.h>
#include <usrp/usrp_bytesex.h>
#include "fpga_regs_common.h"
#include "fpga_regs_bbprx.h"
#include <boost/program_options.hpp>
#include "usrp_radR_plugin.h"

namespace po = boost::program_options;

#ifdef HAVE_SCHED_H
#include <sched.h>
#endif

#define MAX_N_SAMPLES 16384

static bool test_input (usrp_bbprx_sptr urx, int n_pulses, FILE *fp, bool raw);

int main(int argc, char *argv[])
{
  int			which		   = 0;		// specify which USRP board
  unsigned int		decim		   = 16;	// decimation rate
  float			vid_gain	   = 0;		// video gain
  float			trig_gain	   = 0;		// trigger gain
  float			trig_thresh_excite = 50;	// trigger excitation threshold (50%)
  float			trig_thresh_relax  = 50;	// trigger relaxation threshold (50%)
  unsigned short	trig_latency	   = 0;		// min. clock ticks between consecutive trigger pulses
  unsigned short	trig_delay	   = 0;		// how many clock ticks to wait after trigger before digitizing signal
  int                   ARP_active_low     = 1;         // ARP is active low
  unsigned short	ARP_latency	   = 64000;	// min. clock ticks between consecutive ARP pulses
  int                   ACP_active_low     = 1;         // ACP is active low
  unsigned short	ACP_latency	   = 6400;	// min. clock ticks between consecutive ACP pulses
  unsigned short	n_samples	   = 512;	// set the number of samples to collect per pulse
  int                   n_pulses	   = -1;	// set the number of pulses to collect; -1 means continuous
  bool                  counting	   = false;	// should USRP return fake data from a counter, rather than A/D input
  bool                  vid_negate	   = false;	// is video negated?
  bool                  raw_packets        = false;     // when collecting pulses, return raw usb packets, ignore dropped ones
  unsigned int          bbprx_mode         = 0;         // sampling mode
  std::string		filename	   = "received.dat";
  int			fusb_block_size	   = 0;
  int			fusb_nblocks	   = 0;
  int                   quiet              = false;     // don't output diagnostics to stdout
  po::options_description	cmdconfig("Program options: usrp_rx_bbpulse [options] filename");

  cmdconfig.add_options()
    ("help,h", "produce help message")
    ("which,W", po::value<int>(&which), "select which USRP board")
    ("decim,d", po::value<unsigned int>(&decim), "set fgpa decimation rate (0-65535; default is 16)")
    ("signal-gain,g", po::value<float>(&vid_gain), "set video gain in dB (0-20; default is 0)")
    ("trigger-gain,G", po::value<float>(&trig_gain), "set trigger gain in dB (0-20; default is 0)")
    ("trig-thresh-relax,r", po::value<float>(&trig_thresh_relax), "trigger relaxation threshold (% of max signal; default is 50%)")
    ("trig-thresh-excite,e", po::value<float>(&trig_thresh_excite), "trigger excitation threshold (% of max signal; default is 50%)")
    ("ARP-active-high", "ARP signal is active high, not active low")
    ("ACP-active-high", "ACP signal is active high, not active low")
    ("ARP-latency,A", po::value<unsigned short>(&ARP_latency), "min. clock ticks between consecutive ARPs; default is 0")
    ("ACP-latency,B", po::value<unsigned short>(&ACP_latency), "min. clock ticks between consecutive ACPs; default is 0")
    ("trig-delay,D", po::value<unsigned short>(&trig_delay), "clock ticks to wait after trigger before digitizing signal; default is 0")
    ("trig-latency,L", po::value<unsigned short>(&trig_latency), "min. clock ticks between consecutive triggers; default is 0")
    ("n_samples,n", po::value<unsigned short>(&n_samples), "number of samples to collect per pulse; default is 512; max is 16384")
    ("n_pulses,P", po::value<int>(&n_pulses), "number of pulses to collect; default is continuous")
    ("bbprx_mode,m", po::value<unsigned int>(&bbprx_mode), "sampling mode: 0 (default) = normal; 1 = raw video; 2 = raw trigger; 3 = raw ARP; 4 = raw ACP; 5 = raw ALL interleaved")
    ("fusb_block_size,F", po::value<int>(&fusb_block_size), "set fast usb block size")
    ("fusb_nblocks,N", po::value<int>(&fusb_nblocks), "set fast usb nblocks")
    ("vid_negate,v", "negate video signal (default is no)")
    ("raw_packets,R", "return data from raw USB packets; don't strip metadata; ignore dropped packets")
    ("counting,C", "obtain data from a counter instead of from A/D conversion (for debugging)")
    ("quiet,q", "don't output diagnostics")
#ifdef HAVE_SCHED_SETSCHEDULER
    ("realtime,T", "try to request realtime priority for process")
#endif
    ;

  po::options_description fileconfig("Input file options");
  fileconfig.add_options()
    ("filename", po::value<std::string>(), "input file")
    ;

  po::positional_options_description inputfile;
  inputfile.add("filename", -1);

  po::options_description config;
  config.add(cmdconfig).add(fileconfig);
  
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
	    options(config).positional(inputfile).run(), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << cmdconfig << "\n";
    return 1;
  }

  if (vm.count("counting"))
    counting = true;

  if (vm.count("quiet"))
    quiet = true;

  if (vm.count("vid_negate"))
    vid_negate = true;

  if(vm.count("filename")) {
    filename = vm["filename"].as<std::string>();
  }

  if(vm.count("raw_packets"))
    raw_packets = true;

  if(vm.count("ARP-active-high"))
    ARP_active_low = 0;

  if(vm.count("ACP-active-high"))
    ACP_active_low = 0;
  
#ifdef HAVE_SCHED_SETSCHEDULER
  if(vm.count("realtime")) {
    int policy = SCHED_FIFO;
    int pri = (sched_get_priority_max (policy) - sched_get_priority_min (policy)) / 2;
    int pid = 0;  // this process
    
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = pri;
    int result = sched_setscheduler(pid, policy, &param);
    if (result != 0){
      perror ("sched_setscheduler: failed to set real time priority");
    }
    else
      printf("SCHED_FIFO enabled with priority = %d\n", pri);
  }
#endif

  if (n_samples > MAX_N_SAMPLES)
    perror ("Too many samples requested; max is 16384");

  if (bbprx_mode > BBPRX_MODE_MAX)
    perror ("BBPRX mode too high; max is 5");

  if (!quiet) {
    std::cout << "which:   " << which << std::endl;
    std::cout << "decim:   " << decim << std::endl;
    std::cout << "video gain:    " << vid_gain << std::endl;
    std::cout << "negate video: " << vid_negate << std::endl;
    std::cout << "trigger gain:    " << trig_gain << std::endl;
    std::cout << "trigger excite threshold:    " << trig_thresh_excite << std::endl;
    std::cout << "trigger relax threshold:    " << trig_thresh_relax << std::endl;
    std::cout << "trigger latency:    " << trig_latency << std::endl;
    std::cout << "trigger delay:    " << trig_delay << std::endl;
    std::cout << "ARP is active low: " << (ARP_active_low ? "yes" : "no") << std::endl;
    std::cout << "ACP is active low: " << (ACP_active_low ? "yes" : "no") << std::endl;
    std::cout << "ARP latency:    " << ARP_latency << std::endl;
    std::cout << "ACP latency:    " << ACP_latency << std::endl;
    std::cout << "samples: " << n_samples << std::endl;
    std::cout << "pulses: " << n_pulses << std::endl;
    std::cout << "counting?: " << counting << std::endl;
    std::cout << "sampling mode: " << bbprx_mode << std::endl;
    std::cout << "raw_packets?:" << raw_packets << std::endl;
  }
  FILE *fp = 0;

  fp = fopen (filename.c_str(), "wb");
  if (fp == 0)
    perror (filename.c_str());
      
  int mode = 0;

  if (counting)
    mode |= FPGA_MODE_COUNTING;

  usrp_bbprx_sptr urx = 
    usrp_bbprx::make (which, fusb_block_size, fusb_nblocks);

  if (urx == 0)
    perror ("usrp_bbprx::make");

  if (!urx->set_fpga_mode (mode))
    perror ("urx->set_fpga_mode");

  if (!urx->set_decim_rate ((unsigned int) (decim & 0xffff)))
    perror ("urx->set_decim_rate");

  if (!urx->set_vid_gain (vid_gain))
    perror ("urx->set_vid_gain");

  if (!urx->set_vid_negate (vid_negate))
    perror ("urx->set_vid_negate");

  if (!urx->set_trig_gain (trig_gain))
    perror ("urx->set_trig_gain");

  if (!urx->set_trig_thresh_excite ((unsigned short) (4095.0 * trig_thresh_excite / 100.0)))
    perror ("urx->set_trig_thresh_excite");

  if (!urx->set_trig_thresh_relax ((unsigned short) (4095.0 * trig_thresh_relax / 100.0)))
    perror ("urx->set_trig_thresh_relax");

  if (!urx->set_trig_latency (trig_latency))
    perror ("urx->set_trig_latency");

  if (!urx->set_trig_delay (trig_delay))
    perror ("urx->set_trig_delay");

  if (!urx->set_ARP_thresh_excite (ARP_active_low ? 0 : 1))
    perror ("urx->set_ARP_thresh_excite");

  if (!urx->set_ARP_thresh_relax (ARP_active_low ? 1 : 0))
    perror ("urx->set_ARP_thresh_relax");

  if (!urx->set_ARP_latency (ARP_latency))
    perror ("urx->set_ARP_latency");

  if (!urx->set_ACP_thresh_excite (ACP_active_low ? 0 : 1))
    perror ("urx->set_ACP_thresh_excite");

  if (!urx->set_ACP_thresh_relax (ACP_active_low ? 1 : 0))
    perror ("urx->set_ACP_thresh_relax");

  if (!urx->set_ACP_latency (ACP_latency))
    perror ("urx->set_ACP_latency");

  if (!urx->set_n_samples (n_samples))
    perror ("urx->set_n_samples");

  if (!urx->set_bbprx_mode (bbprx_mode))
    perror ("urx->set_bbprx_mode");

  if (!quiet) 
    std::cout << "block_size:" << urx->block_size() << std::endl;

  if (!urx->set_aux_digital_io ())
    perror ("urx->set_aux_digital_io");

  // start data xfers
  if (!urx->start())
    perror ("urx->start");

  if (!urx->set_active (true))
    perror ("urx->set_active");

  test_input (urx, n_pulses, fp, (bbprx_mode > 0) | raw_packets);

  if (fp)
    fclose (fp);

  if (!urx->stop())
    perror ("urx->stop");

  if (!urx->set_active (false))
    perror ("urx->set_active");

  return 0;
}

static bool
test_input  (usrp_bbprx_sptr urx, int n_pulses, FILE *fp, bool raw)
{
  int		   fd = -1;
  static unsigned short   buf[MAX_N_SAMPLES];

  double	   start_wall_time = get_elapsed_time ();
  double	   start_cpu_time  = get_cpu_usage ();

  pulse_metadata meta;

  unsigned int packets_dropped = 0;
  if (fp)
    fd = fileno (fp);

  bool okay = true;

  for (int j = 0; n_pulses < 0 || j < n_pulses; ++j) {
      
    okay = urx->get_pulse(buf, raw, &meta) && okay;
    if (n_pulses < 0 && !okay) {
      fprintf(stderr, "n_read_errors=%d, n_overruns=%d, n_missing_USB_packets=%d, n_missing_data_packets=%d\n", 
	      urx->n_read_errors, urx->n_overruns, urx->n_missing_USB_packets, urx->n_missing_data_packets);
      urx->n_read_errors = 0;
      urx->n_overruns = 0;
      urx->n_missing_USB_packets = 0;
      urx->n_missing_data_packets = 0;
      okay = true;
    }

    if (write (fd, buf, sizeof(unsigned short) * urx->n_samples()) == -1) {
      perror ("write");
      fd = -1;
      break;
    }
    if (n_pulses < 0 && packets_dropped) {
      fprintf (stderr, "test_input: dropped %d packets\n", packets_dropped);
      packets_dropped = 0;
    }
  }
  if (!okay) {
      fprintf(stderr, "n_read_errors=%d, n_overruns=%d, n_missing_USB_packets=%d, n_missing_data_packets=%d\n", 
	      urx->n_read_errors, urx->n_overruns, urx->n_missing_USB_packets, urx->n_missing_data_packets);
  }
	 
  double stop_wall_time = get_elapsed_time ();
  double stop_cpu_time  = get_cpu_usage ();
  
  double delta_wall = stop_wall_time - start_wall_time;
  double delta_cpu  = stop_cpu_time  - start_cpu_time;
  
  long long nbytes = urx->n_samples() * n_pulses * sizeof(unsigned short);
  printf ("xfered %Ld bytes in %.3g seconds.  %.4g bytes/sec.  cpu time = %.4g\n",
	  nbytes, delta_wall, nbytes / delta_wall, delta_cpu);
  printf ("For last pulse:\nticks=%Lu\ntrig_interval=%u\nARP_interval=%u\nACP_interval=%u\nticks_since_last_ACP=%u\nn_trigs=%u\nn_ARPs=%u\nn_ACPs=%u\nUSB_serial_no=%u\n",
	  meta.clock_ticks,
	  meta.trig_interval,
	  meta.ARP_interval,
	  meta.ACP_interval,
	  meta.ticks_since_last_ACP,
	  meta.n_trigs,
	  meta.n_ARPs,
	  meta.n_ACPs,
	  meta.USB_serial_no);
  return true;
}
