/* -*- c++ -*- */
/*
 * Copyright 2004,2008,2009 Free Software Foundation, Inc.
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

#include "usrp_marine_radar.h"

#include <stdexcept>
#include <assert.h>
#include <math.h>
#include <ad9862.h>
#include <cstdio>
#include <string.h>

using namespace ad9862;


// ----------------------------------------------------------------
usrp_marine_radar::usrp_marine_radar ()
    n_read_errors(0),
    n_overruns(0),
    n_missing_packets(0),
    n_missing_pulses(0)
    
{
}

usrp_marine_radar::~usrp_marine_radar ()
{
}

static std::vector < std::string > 
usrp_marine_radar::get_attached_devices() {
  uhd::device_addr_t hint;
  hint["type"] = "usrp1";
  uhd::device_addrs_t dev_addrs = uhd::device::find(hint);
  std::vector < std::string > dev_names (dev_addrs.size());
  for (int i=0; i < dev_addrs.size(); ++i)
    dev_names[i] = dev_addrs[i].to_string();
  return dev_names;
}

void 
usrp_marine_radar::open_device (std::string s) {
  usrp = uhd::usrp::multi_usrp::make(s);

  if (usrp->get_mboard_name(0) != "USRP1 (Classic)")
    throw std::runtime_error("usrp_marine_radar: That device is not a USRP1!");
  if (usrp->get_rx_subdev_name(0) != "LF RX")
    throw std::runtime_error("usrp_marine_radar: There is no LF RX daughterboard in slot 0!");
  
  usrp->set_clock_source("internal");

  // get the daughterboard interface so we can change settings needed by marine radar
  dbi = usrp->get_rx_dboard_iface(0);
  
  // set GPIO pins 0, 1 to input (for ARP, ACP); rest are outputs, for now
  dbi->set_gpio_ddr(uhd::usrp::dboard_iface::UNIT_RX, 0xfffc, 0xffff);

  // FIXME: is this needed? deactivate receiver, to start with
  // set_active(false);

  // FIXME: is this necessary?  - needs to be done UHD-style (coerce get_dev into uhd::usrp::usrp1_impl, then call method there)
  // u->db(0)[0]->bypass_adc_buffers(true);

  // FIXME: is this necessary?  easier to just deal with on the FPGA side?  also needs to be done UHD-style
  // usrp_9862_write(u->d_udh, 0, REG_RX_IF, RX_IF_USE_CLKOUT1); // turn off twos-complement mode (by virtue of the

}

int
usrp_marine_radar::get_packet_size() 
{
  return 512;  // FIXME: only for USRP1
}

uint_32_t
usrp_marine_radar::get_clock_rate() 
{
  return 64000000;  // FIXME: only for USRP1
}

double
usrp_marine_radar::get_clock_interval() 
{
  return 1.0 / get_clock_rate();
}


usrp_marine_radar::start ()
{
  rx_stream = usrp->get_rx_stream("sc16"); // signed complex 16-bit samples; but we know they are all real mode!

  // get the USRP starting time; this will be added to USRP clock ticks to 
  // get precise timestamps.  There will be an offset due to the latency of the
  // call to start() etc., but it should be small, and precision is more important
  // than accuracy for our needs. (For better accuracy, see the Ettus GSPDO kit which syncs
  // with UTC via GPS to within 50 ns; i.e. within 4 USRP clock ticks)

  d_time_started = boost::posix_time::microsec_clock::universal_time();

  uhd::stream_cmd_t stream_cmd( uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
  stream_cmd.num_samps = get_packet_size() / sizeof (<std::complex <uint_16_t> >);
  stream_cmd.stream_now = true;
  stream_cmd.time_spec = uhd::time_spec_t();
  usrp->issue_stream_cmd(stream_cmd);
  return true;
}

bool
usrp_marine_radar::stop ()
{
  uhd::stream_cmd_t stream_cmd( uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
  usrp->issue_stream_cmd(stream_cmd);
  return true;
}


bool
usrp_marine_radar::get_pulse (uint_16_t *buf, bool raw, pulse_metadata *meta)
{
  //FIXME: if we're missing some data packets from a pulse, use whatever we have and set the
  //       rest to zero; in some cases, the caller will still get the desired amount of data
  //       from that pulse.

  uhd::rx_metadata_t md;

  bool overrun;
  bool okay = true;
  pulse_metadata hdr;
  bool overflow_message = true;

  
  int nsb = get_packet_size() / sizeof(std::complex < uint_16_t >);
  int n = packets_per_pulse;

  static uint_32_t expected_serial_no = 0;
  uint_32_t serial_no;

  for (int j = 0; j < n; /* nothing */) {
    size_t num_rx_samps = rx_stream->recv((std::complex < uint_16_t> *) &buf[j * nsb], nsb, md, 3.0);

    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
      std::cout << boost::format("Timeout while streaming") << std::endl;
      break;
    }
    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
      ++ n_overruns;
      okay = false;
      if (overflow_message) {
	overflow_message = false;
	std::cerr << boost::format(
				   "Got an overflow indication. Please consider the following:\n"
				   "  Your write medium must sustain a rate of %fMB/s.\n"
				   "  Dropped samples will not be written to the file.\n"
				   "  Please modify this example for your purposes.\n"
				   "  This message will not appear again.\n"
				   ) % (usrp->get_rx_rate()*sizeof(samp_type)/1e6);
      }
      continue;
    }
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
      throw std::runtime_error(str(boost::format(
						 "Unexpected error code 0x%x"
						 ) % md.error_code));
    }

    if (num_rx_samps != nsb) {
      ++ n_read_errors;
      okay = false;
#ifdef DEBUG_PACKETS    
      fprintf (stderr, "test_input: error, ret = %d\n", ret);
      fflush(stderr);
#endif
      continue;
    }
    extract_metadata(&buf[j * nsb], &hdr, meta != NULL);
    serial_no = hdr.USB_serial_no;
#ifdef DEBUG_PACKETS    
    fprintf (stderr, "j=%2d serial_no=%3d  expected=%3d\n", j, serial_no, expected_serial_no);
    fflush(stderr);
#endif
    if (serial_no != expected_serial_no++) {
      n_missing_USB_packets += (serial_no >= expected_serial_no ? 
				serial_no - expected_serial_no :
				((uint_32_t) 0xffffffffu - expected_serial_no) + serial_no + 1);
      okay = false;
      expected_serial_no = serial_no + 1;
      if ((j > 0) & !raw) {
	// we had started receiving a pulse, but dropped a packet for it, so restart
	j = 0;
	continue;
      }
    }

    if (!raw) {
      if (j == 0) {
	if (hdr.clock_ticks == 0LL) {
	  // this is not a header package
	  ++ n_missing_data_packets;
	  okay = false;
#ifdef DEBUG_PACKETS    
	  fprintf (stderr, "missed header\n");
	  fflush(stderr);
#endif
	  continue;
	} else {
	  // good - it's a header
	  ++j;
	  if (meta != NULL)
	    *meta = hdr;
	}
      } else {
	if (hdr.clock_ticks == 0LL) {
	  // good - it's not a header
	  ++j;
	} else {
	  // this is an unexpected header package, so we don't have a complete pulse; restart
	  ++ n_missing_data_packets;
	  okay = false;
	  j = 0;
#ifdef DEBUG_PACKETS    
	  fprintf (stderr, "missed data - restarting\n");
	  fflush(stderr);
#endif
	  continue;
	}
      }
    } else {
      ++j;
    }
  }
  return okay;
}


void 
usrp_marine_radar::extract_metadata(uint_16_t *samples, pulse_metadata *meta, bool remove)
{
  static uint_32_t META_BITS_PER_SAMPLE = 4 + 0;
  static uint_32_t METADATA_MASK = (((1 << META_BITS_PER_SAMPLE) - 1) << (16 - META_BITS_PER_SAMPLE));
  static uint_32_t SAMPLE_MASK =  ((uint_16_t)(~METADATA_MASK));

  // order of fields in metadata is determined by instance "stage_4" of pack_metadata in usrp_marine_radar.v
  
  uint_32_t unpacked = 0;

  uint_32_t i, j;
  for (i = 0, j = 0; i < sizeof(meta->shorts) * 8 / META_BITS_PER_SAMPLE; ++i) {
    unpacked = (unpacked >> META_BITS_PER_SAMPLE) | (samples[i] & METADATA_MASK);
    if (remove)
      samples[i] &= SAMPLE_MASK;
    if ((i & 0x03) == 0x03)
      meta->shorts[j++] = unpacked;
  }
}


// bool
// usrp_marine_radar::set_active (bool active)
// {
//   d_active = active;
//   bool s = disable_rx ();
//   bool ok = _write_fpga_reg (FR_ACTIVE, (int) active);
//   restore_rx (s);
//   return ok;
// }

bool
usrp_marine_radar::set_decim_rate (uint_32_t decim_rate)
{
  d_decim_rate = decim_rate;
  //  bool s = disable_rx ();
  dbi->write_spi(uhd::usrp::dboard_iface::UNIT_RX, spi_config_t::EDGE_RISE, bool ok = _write_fpga_reg (FR_DECIM_RATE_BBPRX, (int) decim_rate);
  //  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_vid_gain (double vid_gain)
{
  d_vid_gain = vid_gain;
  bool s = disable_rx ();
  bool ok = set_pga(0, vid_gain);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_trig_gain (double trig_gain)
{
  d_trig_gain = trig_gain;
  bool s = disable_rx ();
  bool ok = set_pga(1, trig_gain);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ARP_gain (double ARP_gain)
{
  d_ARP_gain = ARP_gain;
  bool s = disable_rx ();
  bool ok = set_pga(2, ARP_gain);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ACP_gain (double ACP_gain)
{
  d_ACP_gain = ACP_gain;
  bool s = disable_rx ();
  bool ok = set_pga(3, ACP_gain);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_vid_negate (bool vid_negate)
{
  d_vid_negate = vid_negate;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_VID_NEGATE, (int) vid_negate);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_trig_thresh_excite (uint_16_t trig_thresh_excite)
{
  d_trig_thresh_excite = trig_thresh_excite;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_TRIG_THRESH_EXCITE, (int) trig_thresh_excite);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_trig_thresh_relax (uint_16_t trig_thresh_relax)
{
  d_trig_thresh_relax = trig_thresh_relax;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_TRIG_THRESH_RELAX, (int) trig_thresh_relax);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ARP_thresh_excite (uint_16_t ARP_thresh_excite)
{
  d_ARP_thresh_excite = ARP_thresh_excite;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_ARP_THRESH_EXCITE, (int) ARP_thresh_excite);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ARP_thresh_relax (uint_16_t ARP_thresh_relax)
{
  d_ARP_thresh_relax = ARP_thresh_relax;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_ARP_THRESH_RELAX, (int) ARP_thresh_relax);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ARP_latency (uint_32_t ARP_latency)
{
  d_ARP_latency = ARP_latency;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_ARP_LATENCY, (int) ARP_latency);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ACP_thresh_excite (uint_16_t ACP_thresh_excite)
{
  d_ACP_thresh_excite = ACP_thresh_excite;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_ACP_THRESH_EXCITE, (int) ACP_thresh_excite);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ACP_thresh_relax (uint_16_t ACP_thresh_relax)
{
  d_ACP_thresh_relax = ACP_thresh_relax;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_ACP_THRESH_RELAX, (int) ACP_thresh_relax);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ACP_latency (uint_32_t ACP_latency)
{
  d_ACP_latency = ACP_latency;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_ACP_LATENCY, (int) ACP_latency);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_trig_latency (uint_32_t trig_latency)
{
  d_trig_latency = trig_latency;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_TRIG_LATENCY, (int) trig_latency);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_trig_delay (uint_32_t trig_delay)
{
  d_trig_delay = trig_delay;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_TRIG_DELAY, (int) trig_delay);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_n_samples (uint_32_t n_samples)
{
  d_n_samples = n_samples;
 // round up to fill USB packets
  packets_per_pulse = (sizeof(uint_16_t) * n_samples + USB_PACKET_SIZE - 1) / USB_PACKET_SIZE;
  samples_in_last_packet = n_samples - (packets_per_pulse - 1) * (USB_PACKET_SIZE / sizeof(uint_16_t));
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_N_SAMPLES, packets_per_pulse * (USB_PACKET_SIZE / sizeof(uint_16_t)) );
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_source_mode (uint_32_t source_mode)
{
  if (source_mode > SOURCE_MODE_MAX)
    throw std::runtime_error("usrp_marine_radar::set_source_mode: invalid mode");

  d_source_mode = source_mode;
  bool s = disable_rx ();
  bool ok = _write_fpga_reg (FR_SOURCE_MODE, (int) source_mode);
  restore_rx (s);
  return ok;
}

bool
usrp_marine_radar::set_ACPs_per_sweep (uint_32_t ACPs_per_sweep)
{
  d_ACPs_per_sweep = ACPs_per_sweep;
  return true;
}


bool
usrp_marine_radar::set_aux_digital_io ()
{
  // we are now guaranteed to have an LFRX daughter board in 
  // slot A, so it is safe to set all aux digital i/o lines
  // there as we please.

  // DO NOT DO THIS WITH OTHER DAUGHTER BOARDS OR YOU MAY FRY THE USRP!

  return _write_fpga_reg (FR_DEBUG_EN,  bmFR_DEBUG_EN_RX_A);
  return _write_fpga_reg (FR_OE_1, 0xfffffffc);
}

  // ACCESSORS
bool 
usrp_marine_radar::active () const
{
  return d_active;
}

uint_32_t 
usrp_marine_radar::decim_rate () const
{
  return d_decim_rate;
}

double
usrp_marine_radar::vid_gain() const
{
  return d_vid_gain;
}

double
usrp_marine_radar::trig_gain() const
{
  return d_trig_gain;
}

double
usrp_marine_radar::ARP_gain() const
{
  return d_ARP_gain;
}

double
usrp_marine_radar::ACP_gain() const
{
  return d_ACP_gain;
}

bool
usrp_marine_radar::vid_negate() const
{
  return d_vid_negate;
}

uint_16_t
usrp_marine_radar::trig_thresh_excite() const
{
  return d_trig_thresh_excite;
}

uint_16_t
usrp_marine_radar::trig_thresh_relax() const
{
  return d_trig_thresh_relax;
}

uint_16_t
usrp_marine_radar::ARP_thresh_excite() const
{
  return d_ARP_thresh_excite;
}

uint_16_t
usrp_marine_radar::ARP_thresh_relax() const
{
  return d_ARP_thresh_relax;
}

uint_32_t
usrp_marine_radar::ARP_latency() const
{
  return d_ARP_latency;
}

uint_16_t
usrp_marine_radar::ACP_thresh_excite() const
{
  return d_ACP_thresh_excite;
}

uint_16_t
usrp_marine_radar::ACP_thresh_relax() const
{
  return d_ACP_thresh_relax;
}

uint_32_t
usrp_marine_radar::ACP_latency() const
{
  return d_ACP_latency;
}

uint_32_t
usrp_marine_radar::trig_latency() const
{
  return d_trig_latency;
}

uint_32_t
usrp_marine_radar::trig_delay() const
{
  return d_trig_delay;
}

uint_32_t
usrp_marine_radar::n_samples() const
{
  return d_n_samples;
}

uint_32_t
usrp_marine_radar::source_mode() const
{
  return d_source_mode;
}

uint_32_t
usrp_marine_radar::ACPs_per_sweep () const
{
  return d_ACPs_per_sweep;
}
  
boost::posix_time::ptime
usrp_marine_radar::time_started() const
{
  return d_time_started;
}
