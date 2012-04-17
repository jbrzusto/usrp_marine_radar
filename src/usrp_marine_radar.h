/* -*- c++ -*- */
/*
 * Copyright 2004,2008,2009 Free Software Foundation, Inc.
 * Copyright 2011 John Brzustowski
 * 
 * This file is part of the usrp_marine_radar package.
 * 
 * usrp_marine_radar is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * usrp_marine_radar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with usrp_marine_radar; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_USRP_MARINE_RADAR_H
#define INCLUDED_USRP_MARINE_RADAR_H

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/usrp/dboard_id.hpp>
#include <uhd/usrp/dboard_iface.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

/**
   digitize baseband radar pulses using an LFRX daughter board.

   The signals are mapped as so:

    video:   RX_A side A 
    trigger: RX_A side B
    ARP:     RX_B side aux I/O pin 0
    ACP:     RX_B side aux I/O pin 1

*/

  enum {
    SOURCE_MODE_NORMAL	    = 0x00,
    SOURCE_MODE_RAW_VIDEO   = 0x01,
    SOURCE_MODE_RAW_TRIGGER = 0x02,
    SOURCE_MODE_RAW_ARP	    = 0x03,
    SOURCE_MODE_RAW_ACP	    = 0x04,
    SOURCE_MODE_RAW_ALL     = 0x05,
    SOURCE_MODE_MAX         = 0x05
  };


/*!
 * \ingroup usrp
 */

struct pulse_metadata
{
  union
  { 
    uint_t_16 shorts[28];			// makes unpacking metadata from sampledata more convenient
    struct {
						// NB: order of fields in metadata must be reverse of instance "stage_4" of pack_metadata in usrp_marine_radar.v

      unsigned int pacet_serial_no;		// serial number of data packet
      unsigned int n_ACPs;			// number of ACPs since last reset
      unsigned int n_ARPs;			// number of ARPs since last reset
      unsigned int n_trigs;			// number of triggers since last reset
      unsigned int ACP_interval_last_ARP;	// ticks in ACP interval that included last ARP pulse (only valid once n_ACPs != ACP_count_last_ARP)
      unsigned int ACP_age_last_ARP;		// ticks since last ACP pulse at last ARP pulse
      unsigned int ACP_count_last_ARP;		// ACP counter at last ARP pulse
      unsigned int ticks_since_last_ACP;	// clock ticks since last ACP pulse
      unsigned int ticks_since_last_ARP;	// clock ticks since last ARP pulse
      unsigned int ACP_interval;		// clock ticks between most recent consecutive ACP pulses that contained a trigger pulse
      unsigned int ARP_interval;		// clock ticks between most recent consecutive ARP pulses
      unsigned int trig_interval;		// clock ticks between most recent consecutive trigger pulses
      unsigned long long clock_ticks;		// number of (64MHz) clock ticks since last reset
    };
  };
  double ACP_clock;				// filled in when length of ACP interval containing this pulse is known.  It is n_ACPs plus a fractional amount
						// estimated based on ticks_since_last_ACP and ACP_interval
};

class usrp_marine_radar
{
protected:
  const static int BYTES_PER_PACKET = 512; // FIXME: generalize to other usrps
  uhd::usrp::multi_usrp::sptr usrp;
  uhd::usrp::dboard_iface dbi;
  uhd::rx_streamer::sptr rx_stream;

  bool				d_active;		// is digitizer active (i.e. are we getting samples from it)
  uint_32_t			d_decim_rate;		// sample decimation rate
  double			d_vid_gain;		// gain for video
  double			d_trig_gain;		// gain for trigger
  double			d_ARP_gain;		// gain for ARP pulse
  double			d_ACP_gain;		// gain for ACP pulse
  bool				d_vid_negate;		// when true, negate video
  uint_16_t			d_trig_thresh_excite;	// trigger excite threshold
  uint_16_t			d_trig_thresh_relax;	// trigger relax threshold
  uint_32_t			d_trig_delay;		// clock ticks to wait after trigger before digitizing signal
  uint_32_t			d_trig_latency;		// minimum clock ticks between consecutive triggers
  uint_16_t			d_ARP_thresh_excite;	// ARP excite threshold
  uint_16_t			d_ARP_thresh_relax;	// ARP relax threshold
  uint_32_t			d_ARP_latency;		// minimum clock ticks between consecutive ARP pulses
  uint_16_t			d_ACP_thresh_excite;	// ACP excite threshold
  uint_16_t			d_ACP_thresh_relax;	// ACP relax threshold
  uint_32_t			d_ACP_latency;		// minimum clock ticks between consecutive ACP pulses
  uint_32_t			d_n_samples;		// number of samples to digitize for each pulse
  uint_32_t			packets_per_pulse;	// number of packets we need to obtain data for one pulse
  uint_32_t			samples_in_last_packet;	// number of desired samples in last data packet for a pulse
  uint_32_t			d_source_mode;		// normal mode, or raw signal from one of the lines
  uint_32_t			d_ACPs_per_sweep;	// number of ACP pulses radar emits per sweep 
  boost::posix_time::ptime	d_time_started;		// time at which digitizing was started (roughly, when 64-bit USRP clock gets reset)

 public:


  usrp_marine_radar ();
  ~usrp_marine_radar ();

  static std::vector < std::string > get_attached_devices();
  void open_device (std::string s);

  int_32_t get_packet_size();
  unsigned int get_clock_rate();
  double get_clock_interval();

  uint_32_t	n_read_errors;		// number of read errors seen
  uint_32_t	n_overruns;		// number of overruns seen since 
  uint_32_t	n_missing_packets;	// number of missing transport packets detected
  uint_32_t	n_missing_pulses;	// number of missing pulses detected


  /*!
   * \brief grab one pulse of data from the USRP.  Return true if no problems were detected,
   *        false if we noticed any overruns or dropped or missing packets.
   *
   * \param buf pointer to a buffer that will receive the samples 
   *            and must contain enough space for d_n_samples samples.
   *
   * \param raw - if false, wait until we find the required number of consecutive
   *        USB packets, with (only) the first containing valid header metadata.
   *        - if true, just fill the data with the first k USB packets we receive,
   *        without regard for missing packets.
   *
   * \param meta - if not NULL, pulse meta data are filled in here and stripped
   *               from the data placed in buf; otherwise, metadata are left
   *               packed in the data.
   */
  bool get_pulse (uint_16_t *buf, bool raw, pulse_metadata *meta);


  /*!
   * \brief extract metadata from the high bits of a pulse
   *
   * \param samples [inout] pointer to a buffer containing samples, as received from the USB port
   *
   * \param meta [out] pointer to a structure to hold metadata
   *
   * \param remove if true, the metadata bits in the original sample buffer are zeroed; if false,
   * no changes are made to the samples buffer
   */
  void extract_metadata(uint_16_t *samples, pulse_metadata *meta, bool remove=true);



  /*!
   * \brief Set whether channel is active
   */
  // FIXME: how do we do this in UHD?  bool set_active (bool active);


  /*!
   * \brief Set decimator rate.  \p For value of rate, each sample in
   * output is average of rate+1 samples at basic clock rate.
   */
  bool set_decim_rate  (uint_32_t decim_rate);


  /*!
   * \brief Set the gain for the video
   */
  bool set_vid_gain (double vid_gain);


  /*!
   * \brief Set the gain for the trigger
   */
  bool set_trig_gain (double trig_gain);


  /*!
   * \brief Set the gain for the ARP
   */
  bool set_ARP_gain (double ARP_gain);


  /*!
   * \brief Set the gain for the ACP
   */
  bool set_ACP_gain (double ACP_gain);


  /*!
   * \brief Set whether the signal should be inverted (i.e. x-> 0xffff-x)
   */
  bool set_vid_negate (bool vid_negate);


  /*!
   * \brief set the trigger excitation threshold
   */
  bool set_trig_thresh_excite (uint_16_t trig_thresh_excite);


  /*!
   * \brief set the trigger relaxation threshold
   */
  bool set_trig_thresh_relax (uint_16_t trig_thresh_relax);


  /*!
   * \brief set the trigger latency
   */
  bool set_trig_latency (uint_32_t trig_latency);


  /*!
   * \brief set the trigger delay
   */
  bool set_trig_delay (uint_32_t trig_delay);


  /*!
   * \brief set the ARP excitation threshold
   */
  bool set_ARP_thresh_excite (uint_16_t ARP_thresh_excite);


  /*!
   * \brief set the ARP relaxation threshold
   */
  bool set_ARP_thresh_relax (uint_16_t ARP_thresh_relax);


  /*!
   * \brief set the ARP latency
   */
  bool set_ARP_latency (uint_32_t ARP_latency);


  /*!
   * \brief set the ACP excitation threshold
   */
  bool set_ACP_thresh_excite (uint_16_t ACP_thresh_excite);


  /*!
   * \brief set the ACP relaxation threshold
   */
  bool set_ACP_thresh_relax (uint_16_t ACP_thresh_relax);


  /*!
   * \brief set the ACP latency
   */
  bool set_ACP_latency (uint_32_t ACP_latency);


  /*!
   * \brief set the number of samples to digitize per pulse; gets rounded up to nearest multiple of 256
   */
  bool set_n_samples (uint_32_t n_samples);


  /*!
   * \brief set the bbprx sampling mode
   */
  bool set_source_mode (uint_32_t source_mode);


  /*!
   * \brief set ACPs per sweep (determined by the radar)
   */
  bool set_ACPs_per_sweep (uint_32_t ACPs_per_sweep);


  /*!
   * \brief set auxilliary digitial I/O registers to be inputs or outputs on 
  * LFRX daughter boards - produces debugging signals.
  * Currently, we set bits 0 and 1 to be inputs (for ACP and ARP, respectively).
  * Bits 31..2 are set to be outputs.
  */
  bool set_aux_digital_io ();


  // ACCESSORS
  bool active () const;
  uint_32_t decim_rate () const;
  double vid_gain() const;
  double trig_gain() const;
  double ARP_gain() const;
  double ACP_gain() const;
  bool vid_negate() const;
  uint_16_t trig_thresh_excite() const;
  uint_16_t trig_thresh_relax() const;
  uint_16_t ARP_thresh_excite() const;
  uint_16_t ARP_thresh_relax() const;
  uint_32_t ARP_latency() const;
  uint_16_t ACP_thresh_excite() const;
  uint_16_t ACP_thresh_relax() const;
  uint_32_t ACP_latency() const;
  uint_32_t trig_delay() const;
  uint_32_t trig_latency() const;
  uint_32_t n_samples() const;
  uint_32_t ACPs_per_sweep() const;
  uint_32_t source_mode() const;
  boost::posix_time::ptime time_started() const;

  // called in base class to derived class order
  bool start ();
  bool stop ();
};

// NEEDED? typedef boost::shared_ptr<usrp_marine_radar> usrp_marine_radar_sptr;

// ----------------------------------------------------------------

#endif /* INCLUDED_USRP_MARINE_RADAR_H */
