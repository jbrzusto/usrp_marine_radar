/*  svn $Id: usrp_radR_plugin.cc 159 2011-06-20 12:30:26Z john $

    Copyright 2003,2006,2008,2009 Free Software Foundation, Inc.
    Copyright (C) 2011 John Brzustowski        

    This file is heavily modified from the GNU radio file:

     /usrp/host/apps/test_usrp_standard_rx.cc

    and is part of radR : an R-based platform for acquisition and analysis of radar data

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Read data from a modified Ettus Research USRP-1 board via USB.
    The USRP-1 (http://ettus.com) is an open-source software defined
    radio board.  We have modified the FPGA layout and host-side
    libraries to permit its use as a radar digitizing card.
    
*/

#include "usrp_radR_plugin.h"

usrp_radR_plugin::usrp_radR_plugin() :
  started(false),
  have_scan_data(false),
  upb(0), 
  which(0),
  fusb_block_size(usrp_bbprx::USB_PACKET_SIZE),
  fusb_nblocks(128),
  last_error(usrp_radR_plugin::NO_ERROR), 
  client_data(0)
{

  // Note: threshold parameters here are % of maximum possible value of signal.
  // For analog signals (e.g. trigger), maximum is 4095 (12 bit ADC).
  // For digital signals (e.g. ARP, ACP), maximum is 1.
  // This is the layer of code that translates between threshold in % max to threshold in actual signal level.

  // sensible defaults:

  param[PARAM_DECIM                  ] = 1;  // 32 MSPS; ~ 5 m range cells
  param[PARAM_VID_GAIN               ] = 0;
  param[PARAM_VID_NEGATE             ] = 1;  // video is negated
  param[PARAM_TRIG_GAIN              ] = 0;
  param[PARAM_TRIG_THRESH_EXCITE     ] = 40; // active low, but inverted
  param[PARAM_TRIG_THRESH_RELAX      ] = 20;
  param[PARAM_TRIG_LATENCY           ] = 0;
  param[PARAM_TRIG_DELAY             ] = 64; // 1 microsecond
  param[PARAM_ARP_GAIN               ] = 0;
  param[PARAM_ARP_THRESH_EXCITE      ] = 0; // active low and not inverted
  param[PARAM_ARP_THRESH_RELAX       ] = 100;
  param[PARAM_ARP_LATENCY            ] = 0;
  param[PARAM_ACP_GAIN               ] = 0;
  param[PARAM_ACP_THRESH_EXCITE      ] = 0; // active low and not inverted
  param[PARAM_ACP_THRESH_RELAX       ] = 100;
  param[PARAM_ACP_LATENCY            ] = 0;
  param[PARAM_N_SAMPLES              ] = 1024; // ~ 5km total range
  param[PARAM_N_PULSES               ] = 4096; // pulses in gated scan; also "pulses" per chunk in raw mode
  param[PARAM_COUNTING               ] = 0; // don't interleave debug counter
  param[PARAM_ALL_PULSES             ] = 0; // we want gated data, not all pulses
  param[PARAM_OWN_THREAD             ] = 1; // run data copier in its own thread
  param[PARAM_BBPRX_MODE             ] = 0; // normal mode: some video digitized at each trigger pulse
  param[PARAM_N_BUFS                 ] = 3; // smallest sensible value

  for (int i=0; i < NUM_PARAMS; ++i) 
    param_changed[i] = true;
  
}


usrp_radR_plugin *
usrp_radR_plugin::make()
{
  usrp_radR_plugin *p = new usrp_radR_plugin();
  p->upb = 0;
  p->started = false;
  return(p);
}

bool
usrp_radR_plugin::set_param(int parno, double parval)
{
  if (parno < 0 || parno >= NUM_PARAMS)
    return false;
  if (param[parno] != parval) {
    param[parno] = parval;
    param_changed[parno] = true;
  }
  return true;
}

int
usrp_radR_plugin::send_params ()
{
  if (!upb)
    return NO_ERROR;

  usrp_bbprx_sptr urx = upb->get_urx();
  if (!urx)
    return NO_ERROR;

  for (int i = 0; i < NUM_PARAMS; ++i) {
    if (param_changed[i]) {
      switch(i) {
      case PARAM_DECIM:
	if ( urx->set_decim_rate ((unsigned int) ((unsigned int)(param[i]) & 0xffff)))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_VID_GAIN:
	if ( urx->set_vid_gain (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_VID_NEGATE:
	if ( urx->set_vid_negate (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_TRIG_GAIN:
	if ( urx->set_trig_gain (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_TRIG_THRESH_EXCITE:
	if ( urx->set_trig_thresh_excite ((unsigned short) (4095.0 * param[i] / 100.0)))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_TRIG_THRESH_RELAX:
	if ( urx->set_trig_thresh_relax ((unsigned short) (4095.0 * param[i] / 100.0)))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_TRIG_LATENCY:
	if ( urx->set_trig_latency (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_TRIG_DELAY:
	if ( urx->set_trig_delay (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ARP_GAIN:
	if ( urx->set_ARP_gain (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ARP_THRESH_EXCITE:
	if ( urx->set_ARP_thresh_excite ((unsigned short) (1.0 * param[i] / 100.0)))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ARP_THRESH_RELAX:
	if ( urx->set_ARP_thresh_relax ((unsigned short) (1.0 * param[i] / 100.0)))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ARP_LATENCY:
	if ( urx->set_ARP_latency (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ACP_GAIN:
	if ( urx->set_ACP_gain (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ACP_THRESH_EXCITE:
	if ( urx->set_ACP_thresh_excite ((unsigned short) (1.0 * param[i] / 100.0)))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ACP_THRESH_RELAX:
	if ( urx->set_ACP_thresh_relax ((unsigned short) (1.0 *  param[i] / 100.0)))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_ACP_LATENCY:
	if ( urx->set_ACP_latency (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_N_SAMPLES:
      case PARAM_N_PULSES:
      case PARAM_N_BUFS:
	// handle any change(s) among these in one go
	if (upb->set_bufs(param[PARAM_N_BUFS], param[PARAM_N_PULSES], param[PARAM_N_SAMPLES])) {
	  param_changed[PARAM_N_BUFS] = false;
	  param_changed[PARAM_N_PULSES] = false;
	  param_changed[PARAM_N_SAMPLES] = false;
	} else {
	  return i + ERROR_CANT_SET_PARAM; // report error in setting N_BUFS, even though problem might be due to N_PULSES or N_SAMPLES
	}
	break;

      case PARAM_COUNTING:
	if ( urx->set_fpga_mode (param[i] ? FPGA_MODE_COUNTING : 0))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      case PARAM_BBPRX_MODE:
	if ( urx->set_bbprx_mode (param[i]))
	  param_changed[i] = false;
	else
	  return i + ERROR_CANT_SET_PARAM;
	break;

      default:
	break;
      }
    }
  }
  return NO_ERROR;
}

usrp_radR_plugin::~usrp_radR_plugin()
{
  delete upb;
}

extern "C"
{
  URP
  URP_make() 
  {
    URP p = usrp_radR_plugin::make();
    return p;
  };

  int
  URP_digitizing (URP urp, int msec) 
  {
    unsigned long long np = urp->upb->get_n_total();
    // wait for 10 msec and see whether pulse count has changed
    boost::this_thread::sleep(boost::posix_time::milliseconds(msec));

    bool digitizing = (np != urp->upb->get_n_total());
    printf("URP_digitizing had first np = %Lu.\nDigitizing is %s\n", np, digitizing  ? "ON" : "OFF!");
    if (!digitizing)
      urp->upb->end_getter();
    return digitizing;
  };

  int
  URP_incoming_data (URP urp)
  {
    return urp->upb->incoming_data() ? 1 : 0;
  }

  int
  URP_have_usrp (URP urp)
  {
    return urp->started;
  };

  double
  URP_get_param (URP urp, int which, double bogus)
  {
    if (which < urp->NUM_PARAMS)
      return urp->param[which];
    return bogus;
  };

  void
  URP_get_params (URP urp, double *dest, int max_param)
  {
    memcpy(dest, urp->param, sizeof(double) * max_param);
  };

  int
  URP_set_param (URP urp, int which, double val)
  {
    bool rv = urp->set_param(which, val);
    return rv ? 1 : 0;
  };

  void
  URP_set_pulse_callback (URP urp, void *callback, void *userdata)
  {
    urp->upb->set_pulse_callback ((t_pulse_callback) callback, userdata);
  };

  int
  URP_send_params (URP urp)
  {
    return urp->send_params();
  };

  void *
  URP_get_client_data (URP urp)
  {
    return urp->client_data;
  };

  void
  URP_set_client_data (URP urp, void *data)
  {
    urp->client_data = data;
  };

  int
  URP_started (URP urp)
  {
    return urp->started;
  };

  int
  URP_last_error (URP urp)
  {
    int rc = urp->last_error;
    urp->last_error = urp->NO_ERROR;
    return rc;
  };

  void
  URP_set_last_error (URP urp, int err)
  {
    urp->last_error = err;
  };

  int
  URP_have_scan_data (URP urp)
  {
    return urp->have_scan_data ? 1 : 0;
  };

  void
  URP_set_have_scan_data (URP urp, int have)
  {
    urp->have_scan_data = have;
  };

  int
  URP_get_sweep_nb (URP urp, 
		    unsigned short *dest,
		    unsigned short n_pulses, 
		    unsigned short n_samples, 
		    int gated,
		    double *pulse_angles, 
		    double *pulse_times, 
		    void (*callback)(int, void *)
		    )
  {
    return urp->upb->get_sweep_nb (dest, 
				   gated, 
				   n_pulses,
				   n_samples,
				   0, 
				   pulse_angles,
				   pulse_times,
				   &urp->smeta, 
				   callback, 
				   urp) ? 1 : 0;

  };

  int
  URP_get_sweep (URP urp, 
		    unsigned short *dest,
		    unsigned short n_pulses, 
		    unsigned short n_samples, 
		    int gated,
		    double *pulse_angles, 
		    double *pulse_times
		    )
  {
    return urp->upb->get_sweep (dest, 
				gated, 
				n_pulses,
				n_samples,
				0, 
				pulse_angles,
				pulse_times,
				&urp->smeta) ? 1 : 0;

  };

  double 
  URP_get_timestamp (URP urp)
  {
    return usrp_pulse_buffer::timestamp_to_double(urp->smeta.timestamp);
  };

  double
  URP_get_duration (URP urp)
  {
    return usrp_pulse_buffer::duration_to_double(urp->smeta.duration);
  };

  double
  URP_get_range_cell_size (URP urp)
  {
    return urp->smeta.range_cell_size;
  };

  double
  URP_get_radar_PRF (URP urp)
  {
    return urp->smeta.radar_PRF;
  };

  int 
  URP_start_up (URP urp, const char* fpga_filename, const char* firmware_filename )
  {
    int rv = usrp_radR_plugin::NO_ERROR;

  // build the objects

  if (! urp->upb) {
    usrp_bbprx_sptr urx = 
      usrp_bbprx::make (urp->which, urp->fusb_block_size, urp->fusb_nblocks, std::string(fpga_filename), std::string(firmware_filename));

    if (!urx.get())
      return usrp_radR_plugin::ERROR_CANT_FIND_USRP;

    urx->set_aux_digital_io ();

    urp->upb = usrp_pulse_buffer::make(urx);
  }

  usrp_bbprx_sptr urx = urp->upb->get_urx();

  do {
  // send the current configuration to the usrp
      rv = urp->send_params();
      if (rv != usrp_radR_plugin::NO_ERROR)
	break;

#ifdef HAVE_SCHED_SETSCHEDULER
      int policy = SCHED_FIFO;
      int pri = (sched_get_priority_max (policy) - sched_get_priority_min (policy)) / 2;
      int pid = 0;  // this process
    
      struct sched_param param;
      memset(&param, 0, sizeof(param));
      param.sched_priority = pri;
      sched_setscheduler(pid, policy, &param);
      // FIXME: a warning here?  find out how to let non-root user bump up priority
      //       if (result != 0) {
      // 	rv = usrp_radR_plugin::ERROR_CANT_SET_REALTIME_PRIORITY;
      // 	break;
      //       }
#endif
  
      // start up the data transfers

      if (! urx->start()) {
	rv = usrp_radR_plugin::ERROR_CANT_START_TRANSFERS;
	break;
      }

      // start digitizing at FPGA
      
      if (! urx->set_active (true)) {
	urx->stop();
	rv = usrp_radR_plugin::ERROR_CANT_START_DIGITIZING;
	break;
      }
      // start receiving pulses at host
      urp->upb->begin_getter();

      urp->started = true;
    } while (0);

  return rv;
  };


  int
  URP_shut_down (URP urp)
  {
    int rv = usrp_radR_plugin::NO_ERROR;
    if (urp->started) {
      urp->upb->end_getter();
      usrp_bbprx_sptr urx = urp->upb->get_urx();
      do {
	if (!urx->stop()) {
	  rv = usrp_radR_plugin::ERROR_CANT_STOP_TRANSFERS;
	  break;
	}
	urp->started = false;
	if (!urx->set_active(false)) {
	  rv = usrp_radR_plugin::ERROR_CANT_STOP_DIGITIZING;
	  break;
	}
      } while (0);
    }
    usrp_rescan(); // kludge; but allows us to redetect after user is prompted to turn on USRP
    return rv;
  };

  int URP_destroy (URP urp)
  {
    delete urp;
    return 1;
  };

}

