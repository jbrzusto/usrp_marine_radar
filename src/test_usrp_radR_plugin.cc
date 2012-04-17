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
namespace po = boost::program_options;

#include "usrp_radR_plugin.h"

int main(int argc, char *argv[])
{
  int			which		   = 0;		// specify which USRP board
  unsigned int		decim		   = 16;	// decimation rate
  float			vid_gain	   = 0;		// video gain
  float			trig_gain	   = 0;		// trigger gain
  float			trig_thresh_excite = 50;	// trigger excitation threshold (50%)
  float			trig_thresh_relax  = 50;	// trigger relaxation threshold (50%)
  unsigned short	trig_delay	   = 0;		// how many clock ticks to wait after trigger before digitizing signal
  float                 ARP_gain	   = 0;		// ARP gain
  float			ARP_thresh_excite  = 50;	// ARPger excitation threshold (50%)
  float			ARP_thresh_relax   = 50;	// ARPger relaxation threshold (50%)
  float                 ACP_gain	   = 0;		// ACP gain
  float			ACP_thresh_excite  = 50;	// ACPger excitation threshold (50%)
  float			ACP_thresh_relax   = 50;	// ACPger relaxation threshold (50%)
  unsigned short	n_samples	   = 512;	// set the number of samples to collect per pulse
  int                   n_pulses	   = 1024;	// set the number of pulses per sweep to collect
  int                   n_sweeps           = -1;        // set the number of sweeps to collect; default (-1) means continuously
  bool                  counting	   = false;	// should USRP return fake data from a counter, rather than A/D input
  bool                  vid_negate	   = false;	// is video negated?
  unsigned int          bbprx_mode         = 0;         // sampling mode
  int			fusb_block_size	   = 0;
  int			fusb_nblocks	   = 0;
  std::string		filename	   = "received.dat";
  po::options_description	cmdconfig("Program options: test_usrp_radR_plugin [options] filename");

  cmdconfig.add_options()
    ("help,h", "produce help message")
    ("which,W", po::value<int>(&which), "select which USRP board")
    ("decim,d", po::value<unsigned int>(&decim), "set fgpa decimation rate (0-65535; default is 16)")
    ("signal-gain,g", po::value<float>(&vid_gain), "set video gain in dB (0-20; default is 0)")
    ("trigger-gain,G", po::value<float>(&trig_gain), "set trigger gain in dB (0-20; default is 0)")
    ("ARP-gain", po::value<float>(&ARP_gain), "set ARP gain in dB (0-20; default is 0)")
    ("ACP-gain", po::value<float>(&ACP_gain), "set ACP gain in dB (0-20; default is 0)")
    ("trig-thresh-relax,r", po::value<float>(&trig_thresh_relax), "trigger relaxation threshold (% of max signal; default is 50%)")
    ("trig-thresh-excite,e", po::value<float>(&trig_thresh_excite), "trigger excitation threshold (% of max signal; default is 50%)")
    ("ARP-thresh-relax", po::value<float>(&ARP_thresh_relax), "ARP relaxation threshold (% of max signal; default is 50%)")
    ("ARP-thresh-excite", po::value<float>(&ARP_thresh_excite), "ARP excitation threshold (% of max signal; default is 50%)")
    ("ACP-thresh-relax", po::value<float>(&ACP_thresh_relax), "ACP relaxation threshold (% of max signal; default is 50%)")
    ("ACP-thresh-excite", po::value<float>(&ACP_thresh_excite), "ACP excitation threshold (% of max signal; default is 50%)")
    ("trig-delay,D", po::value<unsigned short>(&trig_delay), "clock ticks to wait after trigger before digitizing signal; default is 0")
    ("n_samples,n", po::value<unsigned short>(&n_samples), "number of samples to collect per pulse; default is 512")
    ("n_pulses,P", po::value<int>(&n_pulses), "number of pulses per sweep; default is 1024")
    ("n_sweeps,S", po::value<int>(&n_sweeps), "number of sweeps to collect; default is continuous")
    ("bbprx_mode,m", po::value<unsigned int>(&bbprx_mode), "sampling mode: 0 (default) = normal; 1 = raw video; 2 = raw trigger; 3 = raw ARP; 4 = raw ACP")
    ("fusb_block_size,F", po::value<int>(&fusb_block_size), "set fast usb block size")
    ("fusb_nblocks,N", po::value<int>(&fusb_nblocks), "set fast usb nblocks")
    ("vid_negate,v", "negate video signal (default is no)")
    //    ("counting,C", "obtain data from a counter instead of from A/D conversion (for debugging)")
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

  //  if (vm.count("counting"))
  //    counting = true;

  if (vm.count("vid_negate"))
    vid_negate = true;

  if(vm.count("filename")) {
    filename = vm["filename"].as<std::string>();
  }
  
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

  FILE *fp = 0;

  fp = fopen (filename.c_str(), "wb");
  if (fp == 0)
    perror (filename.c_str());
  
//   int mode = 0;

//   if (counting)
//     mode |= usrp_bbprx::FPGA_MODE_COUNTING;

  usrp_radR_plugin * plg = usrp_radR_plugin::make (fusb_block_size, fusb_nblocks);
  
  if (plg == 0)
    perror ("usrp_radR_plugin::make");

  plg->set_param(PARAM_INDEX_VID_NEGATE, vid_negate);
  plg->set_param(PARAM_INDEX_VID_GAIN, vid_gain);
  plg->set_param(PARAM_INDEX_TRIG_GAIN, vid_gain);
  plg->set_param(PARAM_INDEX_ARP_GAIN, ARP_gain);
  plg->set_param(PARAM_INDEX_ACP_GAIN, ACP_gain);
  plg->set_param(PARAM_INDEX_TRIG_THRESH_RELAX, trig_thresh_relax * 4095.0 / 100);
  plg->set_param(PARAM_INDEX_TRIG_THRESH_EXCITE, trig_thresh_excite * 4095.0 / 100);
  plg->set_param(PARAM_INDEX_ARP_THRESH_RELAX, ARP_thresh_relax * 4095.0 / 100);
  plg->set_param(PARAM_INDEX_ARP_THRESH_EXCITE, ARP_thresh_excite * 4095.0 / 100);
  plg->set_param(PARAM_INDEX_ACP_THRESH_RELAX, ACP_thresh_relax * 4095.0 / 100);
  plg->set_param(PARAM_INDEX_ACP_THRESH_EXCITE, ACP_thresh_excite * 4095.0 / 100);
  plg->set_param(PARAM_INDEX_TRIG_DELAY, trig_delay);
  plg->set_param(PARAM_INDEX_DECIM_RATE, (int) decim);
  plg->set_samples_per_pulse(n_samples);
  plg->set_pulses_max(2 * n_pulses);

  std::cout << "which:   " << which << std::endl;
  std::cout << "decim:   " << plg->get_param(PARAM_INDEX_DECIM_RATE) << std::endl;
  std::cout << "video gain:    " << plg->get_double_param(PARAM_INDEX_VID_GAIN) << std::endl;
  std::cout << "negate video: " << plg->get_param(PARAM_INDEX_VID_NEGATE) << std::endl;
  std::cout << "trigger gain:    " << plg->get_double_param(PARAM_INDEX_TRIG_GAIN) << std::endl;
  std::cout << "trigger excite threshold:    " << plg->get_param(PARAM_INDEX_TRIG_THRESH_EXCITE) << std::endl;
  std::cout << "trigger relax threshold:    " << plg->get_param(PARAM_INDEX_TRIG_THRESH_RELAX) << std::endl;
  std::cout << "trigger delay:    " << plg->get_param(PARAM_INDEX_TRIG_DELAY) << std::endl;
  std::cout << "ARP gain:    " << plg->get_double_param(PARAM_INDEX_ARP_GAIN) << std::endl;
  std::cout << "ARP excite threshold:    " << plg->get_param(PARAM_INDEX_ARP_THRESH_EXCITE) << std::endl;
  std::cout << "ARP relax threshold:    " << plg->get_param(PARAM_INDEX_ARP_THRESH_RELAX) << std::endl;
  std::cout << "ACP gain:    " << plg->get_double_param(PARAM_INDEX_ACP_GAIN) << std::endl;
  std::cout << "ACP excite threshold:    " << plg->get_param(PARAM_INDEX_ACP_THRESH_EXCITE) << std::endl;
  std::cout << "ACP relax threshold:    " << plg->get_param(PARAM_INDEX_ACP_THRESH_RELAX) << std::endl;
  std::cout << "samples: " << n_samples << std::endl;
  std::cout << "pulses: " << n_pulses << std::endl;
  std::cout << "sweeps: " << n_sweeps << std::endl;
  std::cout << "counting?: " << counting << std::endl;
  std::cout << "sampling mode: " << bbprx_mode << std::endl;


  plg->start();

  if (bbprx_mode > 0) {
    pulse_metadata meta;
    unsigned short *buf = new unsigned short[n_samples];
    while (n_pulses != 0) {
      plg->get_raw(bbprx_mode, vid_gain, buf, n_samples, decim, &meta);
      fwrite (buf, sizeof(unsigned short), n_samples, fp);
      std::cout << "==> milliseconds since reset: " << (meta.clock_ticks / 64e6) * 1000.0 << std::endl
		<< "n_trigs: " << meta.n_trigs << std::endl
		<< "n_ARPs: " << meta.n_ARPs << std::endl
		<< "n_ACPs: " << meta.n_ACPs << std::endl
	        << "milliseconds since last ACP: " << meta.ticks_since_last_ACP / 64000.0 * 1000.0 << std::endl; // ticks are 1/62500 sec
      if (n_pulses > 0)
	--n_pulses;
    }
  } else {
    unsigned short *buf = new unsigned short[n_samples * n_pulses];
    sweep_metadata meta;
    while (n_sweeps != 0) {
      plg->get_sweep(buf, false, n_samples, n_pulses, &meta);
      fwrite (buf, sizeof(unsigned short), n_samples * n_pulses, fp);
      std::cout << "==> timestamp: " << meta.timestamp << std::endl
		<< "duration: " << meta.duration << std::endl
		<< "pulses_seen: " << meta.n_actual_pulses << std::endl
		<< "radar_PRF: " << meta.radar_PRF << std::endl
		<< "rx_PRF: " << meta.rx_PRF << std::endl
		<< "range_cell_size: " << meta.range_cell_size << std::endl
	        << "n_ACPs: " << meta.n_ACPs << std::endl;
                   
      if (n_sweeps > 0)
	--n_sweeps;
    }
  }
  fclose (fp);
}
