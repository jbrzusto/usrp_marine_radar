/* -*- c++ -*- */
/*
 * Copyright 2011 John Brzustowski
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

#ifndef INCLUDED_USRP_PULSE_BUFFER_H
#define INCLUDED_USRP_PULSE_BUFFER_H

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <math.h>
#include "usrp/usrp_bbprx.h"
#include "sweep_buffer.h"

#define VELOCITY_OF_LIGHT 2.99792458E8

typedef void (* got_sweep_function) (int, void *); // callback for completion of sweep

inline unsigned int elapsed_to_from (unsigned int y, unsigned int x) {
  // compute the elapsed interval to y from x; this is the unsigned value that
  // would have to be added to x to get y.  Just meant to get elapsed values of
  // whatever when both counts are unsigned ints.

  return y >= x ? y - x : ~x + y + 1;
};

struct copy_thread_info; // fwd declaration

typedef std::vector<sweep_buffer>::iterator t_all_bufs_iter;
typedef std::list<sweep_buffer *>::iterator t_ordered_bufs_iter;

typedef void (*t_pulse_callback)(t_sample *buf, unsigned short len, pulse_metadata *meta, void *);

class usrp_pulse_buffer {
  
protected:
  usrp_bbprx_sptr		 urx;			/**< pointer to the underlying USRP host */
  unsigned long long		 n_total;		/**< total pulses digitized since reset */ 
  unsigned int                   n_pulses;              /**< number of "pulses" (real or raw) to obtain per buffer */
  unsigned int                   n_samples;             /**< number of samples to obtain per pulse */
  unsigned int                   n_bufs;                /**< number of sweep buffers to maintain */
  bool				 getting;		/**< is there a getter thread running? also used to tell thread to stop */
  bool				 copying;		/**< is there a copy thread running? also used to tell thread to stop */
  boost::mutex			 getting_mutex;		/**< mutex to enforce only one getter thread running at a time */
  boost::mutex			 copying_mutex;		/**< mutex to enforce only one copy thread running at a time */
  boost::condition_variable	 buffer_status_cv;	/**< condition variable for notifying change in buffer status */
  boost::mutex			 buffer_status_mutex;	/**< mutex for buffer_ready condition variable */
  boost::thread			*getter_thread;		/**< the thread getting data from the USRP */
  boost::thread			*copy_thread;		/**< the thread copying data to a client buffer */
  std::vector<sweep_buffer>      bufs;                  /**< the vector of sweep buffers */
  std::list<sweep_buffer*>       bufs_by_age;		/**< list of pointers to sweep buffers, in order from oldest to youngest */
  t_pulse_callback               pulse_callback;        /**< pointer to a function to be called upon receipt of each pulse */
  void *                         callback_user_data;    /**< opaque pointer to user callback data */

  static void getter_thread_fun(usrp_pulse_buffer *upb); // the function run by the getter thread

  static void copy_thread_fun(copy_thread_info *bci); // function run by copying thread or caller thread

  /*!
   * \brief get an EMPTY buffer for saving data into
   * and mark it as FILLING.  If there are no EMPTY buffers,
   * use the oldest FULL buffer.
   */
  sweep_buffer * get_buffer_for_getter();

  /*!
   * \brief get the oldest FULL buffer and mark it as EMPTYING.
   * Blocks waiting for a buffer to fill if none is immediately
   * available.  Throws if no buffer is available.
   */
  sweep_buffer * get_buffer_for_copier();

  /*!
   * \brief indicate a buffer has been used and is now available
   */
  void set_buffer_status(sweep_buffer *buf, sweep_buffer::t_buf_status status);

  /*!
   * \brief set whether or not the getter thread is running
   */
  void set_getting(bool yesno);

  /*!
   * \brief set whether or not the copy thread is running
   */
  void set_copying(bool yesno);

  // private constructor

  usrp_pulse_buffer (usrp_bbprx_sptr urx);

public:

  /* error codes stored by copy_thread_fun */

  static const int OKAY = 0;
  static const int ACP_COUNT_STUCK = 1;
  static const int COPY_INTERRUPTED = 2;

  /* sanity constraints on some parameters; these are generous and will likely be unattainable in many situations. */

  static const unsigned int MAX_N_BUFS    =   128;
  static const unsigned int MAX_N_PULSES  = 16384;  // per buffer
  static const unsigned int MAX_N_SAMPLES = 16384;  // per pulse

  /*!
   * \brief factory method to construct a usrp_pulse_buffer
   */
  static usrp_pulse_buffer * make (usrp_bbprx_sptr urx);

  /*!
   * \brief return the underlying usrp_bbprx device
   */
  usrp_bbprx_sptr get_urx();

  /*!
   * \brief return the total number of pulses digitized
   */
  unsigned long long get_n_total();

  /*!
   * \brief return a flag indicating whether the getter thread is running
   */
  bool is_getting();

  /*!
   * \brief return a flag indicating whether the copier thread is running
   */
  bool is_copying();

  /*!
   * \brief see whether any buffers have data or are acquiring it;
   * serves the higher level "end of data" function
   */
  bool incoming_data();

  /*!
   * \brief set number of buffers and their sizes
   * \param n_sweep_buffers number of ungated full-sweep buffers to maintain (should be at least 3)
   * \param n_pulses number of pulses per full-sweep buffer
   * \param n_samples number of samples per pulse
   */
  bool set_bufs (unsigned int n_bufs, unsigned int n_pulses, unsigned int n_samples);

  /*!
   * \brief set number of sweep buffers
   * \param n_sweep_buffers number of sweep buffers to maintain
   * returns true on success, false otherwise.
   */
  bool set_n_bufs (unsigned int n_bufs);

  /*!
   * \brief set number of samples per pulse
   * \param n_samples number of samples to digitize per pulse or per block in raw modes
   * returns true on success, false otherwise.
   */
  bool set_n_samples (unsigned int n_samples);

  /*!
   * \brief set number of pulses (real or raw) per buffer to obtain
   * \param n_pulses number of pulses per full-sweep buffer
   * Note: in non-raw modes, the buffer will automatically be enlarged to hold
   * any additional pulses obtained in a sweep.
   * returns true on success, false otherwise.
   */
  bool set_n_pulses (unsigned int n_pulses);

  /*!
   * \brief Set the pulse callback function.
   * \param pulse_callback the pulse callback function, or NULL to disable pulse callbacks
   * \param user_data a user data parameter to pass to the callback.
   * 
   */ 
  void set_pulse_callback (t_pulse_callback pulse_callback, void * user_data);

  /*!
   * \brief Start a thread which acquires data into ring buffers
   * 
   * returns true on success, false otherwise
   * 
   */ 
  bool begin_getter ();


  /*!
   * \brief Stop the data getter thread. Also stops the copy thread, if it exists.
   * returns true on success, false otherwise
   * 
   */ 
  bool end_getter ();


  /*! 
  * \brief Get the oldest available sweep of pulse data.  The getter thread must have been started first, by calling begin_getter()
  *
  * \param buf [input] pointer to the destination buffer
  *
  * \param n_pulses the desired (or maximum) number of pulses to obtain.
  *
  * \param gated if true, gate the data, selecting pulses from evenly around the sweep.  n_pulses is then
  * the desired number of pulses.  Pulses are decimated and/or replicated to match the current PRF to the
  * desired number of pulses.  If false, then all pulses are retained, and n_pulses indicates the maximum
  * number returned.  If the sweep ends before n_pulses have been seen, then only the number 
  *
  * \param samples_per_pulse the number of samples to retain for each pulse; if we are digitizing with 
  * fewer samples per pulse (see d_samples_per_pulse), then the additional sample values will be set to zero.
  *
  * \param meta [output] pointer to a buffer of pulse_metadata structures to be filled with the metadata for returned pulses.
  * Can be NULL.
  *
  * \param smeta [output] pointer to a sweep metadata structure.  Can be NULL.
  *
  * returns the number of pulses copied to buf.  Blocks until a full sweep has been transferred; this will require waiting for
  * data to be acquired, if the ring buffer does not already contain a full sweep.
  */
  unsigned int get_sweep (t_sample *buf, bool gated, unsigned int n_pulses, unsigned short samples_per_pulse, struct pulse_metadata *meta, double *pulse_angles, double *pulse_times, struct sweep_metadata *smeta);


  /*! 
  * \brief Get the oldest available sweep of pulse data without blocking.  The getter thread must be running.
  *
  * \param buf [output] pointer to the destination buffer
  *
  * \param samples_per_pulse the number of samples to retain for each pulse; if we are digitizing with 
  * fewer samples per pulse (see d_samples_per_pulse), then the additional sample values will be set to zero.
  *
  * \param n_pulses the desired (and maximum) number of pulses the dest_buf can hold. 
  *
  * \param gated if true, gate the data, selecting pulses from evenly around the sweep.  n_pulses is then
  * the desired number of pulses.  Pulses are decimated and/or replicated to match the current PRF to the
  * desired number of pulses.  If false, then all pulses are retained, and n_pulses indicates the maximum
  * number returned.  If the sweep ends before n_pulses have been seen, then only the number 
  *
  * \param meta [output] pointer to a buffer of pulse_metadata structures to be filled with the metadata for returned pulses.
  * Can be NULL.  If not NULL, must be allocated to hold at least n_pulses elements of type pulse_metadata.
  *
  * \param smeta [output] pointer to a sweep metadata structure.  Can be NULL.
  *
  * \param [input] f pointer to a function to be called after the entire sweep has been copied to buf.  Can be NULL.
  *
  * \param [input] user_data
  *
  * immediately returns true on success, false otherwise.  Additionally, if it returns true,
  * then when the full sweep has been transferred to the supplied buffer, the function f is called with parameter
  * user_data, unless f is NULL.
  *
  */
  bool get_sweep_nb (t_sample *buf, bool gated, unsigned int n_pulses, unsigned short samples_per_pulse, struct pulse_metadata *meta, double *pulse_angles, double *pulse_times, struct sweep_metadata *smeta, got_sweep_function user_fun, void *user_data);


  /*! 
  * \brief convert a duration to a double, in seconds 
  */
  static inline double duration_to_double(t_duration dur) {
    return dur.total_milliseconds() / 1000.0;
  };

  /*! 
  * \brief convert a double in seconds to a duration
  */
  static inline t_duration double_to_duration(double secs) {
    if (secs == sweep_metadata::NOT_A_DATE_TIME)
      return t_duration(boost::date_time::not_a_date_time);
    return boost::posix_time::seconds((long) floor(secs)) + boost::posix_time::milliseconds((long)round(1000 * fmod(secs, 1.0)));
  };

  /*! 
  * \brief convert a double, in seconds past the epoch, to a timestamp
  */
  static inline t_timestamp double_to_timestamp(double secs_AE) {
    if (secs_AE == sweep_metadata::NOT_A_DATE_TIME)
      return t_timestamp(boost::date_time::not_a_date_time);
    return boost::posix_time::from_time_t((time_t)floor(secs_AE)) +
      boost::posix_time::milliseconds((long) round(1000 * fmod(secs_AE, 1.0)));
  };

  /*! 
  * \brief convert a timestamp to a double number of seconds past the epoch
  */
  static inline double timestamp_to_double(t_timestamp ts) {
    static t_timestamp epoch = double_to_timestamp(0.0);
    return (ts - epoch).total_milliseconds() / 1000.0;
  };

  /*!
   * \brief dump debugging info
   */
  void dump();

  /*!
   * \brief destructor
   */
  ~usrp_pulse_buffer();

  friend void getter_thread_fun(usrp_pulse_buffer *upb);
  friend void copy_thread_fun(usrp_pulse_buffer *upb);

};

  /*! 
  * \brief Struct for the copy thread
  */
struct copy_thread_info {
  usrp_pulse_buffer	*upb;
  t_sample		*buf;
  bool			 gated;
  unsigned int		 n_pulses;
  unsigned short	 samples_per_pulse;
  struct pulse_metadata *meta;
  double		*pulse_angles;
  double		*pulse_times;
  struct sweep_metadata *smeta;
  got_sweep_function	 user_fun;
  void			*user_data;
  int			 return_code;
};
// ----------------------------------------------------------------

#endif // INCLUDED_USRP_PULSE_BUFFER_H
