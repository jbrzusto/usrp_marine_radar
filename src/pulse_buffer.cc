#include "usrp_pulse_buffer.h"

#if 0
#define DBG_PRINT(X) fputs(X, stderr);fputc('\n', stderr);fflush(stdout);
#define DBG_PRINTF(...) fprintf(stderr, __VA_ARGS__);fputc('\n', stderr);fflush(stdout);
#else
#define DBG_PRINT(X)
#define DBG_PRINTF(...)
#endif

#include <pthread.h>
#include <signal.h>

static sigset_t signal_mask;

usrp_pulse_buffer::usrp_pulse_buffer (usrp_bbprx_sptr urx)
  : n_total(0),
    getting(false),
    copying(false),
    getter_thread(0),
    copy_thread(0),
    pulse_callback(0),
    callback_user_data(0)
{
  this->urx = urx;

  // mask alarm signals that interfere with 
  sigemptyset (&signal_mask);
  sigaddset (&signal_mask, SIGALRM);
  pthread_sigmask(SIG_BLOCK, &signal_mask, 0);

}

usrp_pulse_buffer *
usrp_pulse_buffer::make (usrp_bbprx_sptr urx)
{
  return new usrp_pulse_buffer(urx);
}

usrp_bbprx_sptr 
usrp_pulse_buffer::get_urx()
{
  return urx;
};

unsigned long long
usrp_pulse_buffer::get_n_total()
{
  return n_total;
};

bool
usrp_pulse_buffer::is_getting()
{
  return getting;
}

bool
usrp_pulse_buffer::is_copying()
{
  return copying;
}

bool
usrp_pulse_buffer::incoming_data()
{
  for (t_all_bufs_iter b = bufs.begin(); b != bufs.end(); ++b)
    if ((*b).status != sweep_buffer::BUF_EMPTY && (*b).status != sweep_buffer::BUF_EMPTYING)
      return true;
  return false;
}

bool
usrp_pulse_buffer::set_bufs (unsigned int n_bufs, unsigned int n_pulses, unsigned int n_samples)
{
  if (n_bufs > MAX_N_BUFS || n_pulses > MAX_N_PULSES || n_samples > MAX_N_SAMPLES)
    return false;

  bool cache_getting = getting;
  if (cache_getting)
    end_getter();

  bufs.resize(n_bufs);
  this->n_bufs = n_bufs;
  bufs_by_age.clear();
  for (t_all_bufs_iter b = bufs.begin(); b != bufs.end(); ++b) {
    (*b).set_size(n_pulses, n_samples);
    (*b).clear();
    bufs_by_age.push_back(&(*b));
  }
  this->n_pulses = n_pulses;
  this->n_samples = n_samples;

  urx->set_n_samples (n_samples);

  if (cache_getting)
    begin_getter();
  return true;
}

bool
usrp_pulse_buffer::set_n_bufs (unsigned int n_bufs)
{
  return set_bufs (n_bufs, this->n_pulses, this->n_samples);
}

bool
usrp_pulse_buffer::set_n_pulses (unsigned int n_pulses)
{
  return set_bufs (this->n_bufs, n_pulses, this->n_samples);
}

bool 
usrp_pulse_buffer::set_n_samples (unsigned int n_samples)
{
  return set_bufs (this->n_bufs, this->n_pulses, n_samples);
}

void
usrp_pulse_buffer::set_pulse_callback (t_pulse_callback pulse_callback, void * callback_user_data)
{
  this->pulse_callback = pulse_callback;
  this->callback_user_data = callback_user_data;
}

bool 
usrp_pulse_buffer::begin_getter ()
{

  boost::unique_lock<boost::mutex> lock(getting_mutex);
  if (getting)
    return false;
    
  DBG_PRINT("about to start getter thread\n");
  delete getter_thread;

  set_bufs (n_bufs, n_pulses, n_samples);

  getter_thread = new boost::thread(boost::bind(usrp_pulse_buffer::getter_thread_fun, this));
  DBG_PRINT("started getter thread\n");

  getting = true;
  return true;
}

bool
usrp_pulse_buffer::end_getter ()
{
  boost::unique_lock<boost::mutex> lock_getting(getting_mutex);

  // if we're copying, stop that first because it reads from the buffer
  // we will destroy below, and also might otherwise hang waiting for the getter thread
  // to obtain more data
  DBG_PRINT("about to stop getter thread\n");

  boost::unique_lock<boost::mutex> lock_copying(copying_mutex);
  if (copying) {
    DBG_PRINT("waiting for copier thread to finish...");
    copying = false; // tell copy thread to stop
    copy_thread->interrupt();
    DBG_PRINT("copy thread interrupted");
    if (!copy_thread->timed_join(boost::posix_time::milliseconds(100)) && copy_thread->get_id() != boost::thread::id()) {
      // wait for it to stop
      DBG_PRINT("copier thread could not be stopped in time\n");
    } else {
      DBG_PRINT("copier thread finished\n");
    }
  }

  if (getting) {
    getting = false; // tell getter thread to stop
    DBG_PRINT("waiting for getter thread to finish...");
    if (!getter_thread->timed_join(boost::posix_time::milliseconds(100)) && getter_thread->get_id() != boost::thread::id()) {
      DBG_PRINT("getter thread could not be stopped in time\n");
    } else {
      DBG_PRINT("getter thread finished\n");
    }
  }

  return true;
}

sweep_buffer *
usrp_pulse_buffer::get_buffer_for_getter()
{
  // called only by getter_thread_fun

  boost::unique_lock<boost::mutex> lock(buffer_status_mutex);
  DBG_PRINTF("Get_buffer_for_getter locked buffer_status_mutex\n");
    
  t_ordered_bufs_iter b;

  for (;;) {
    // first, look for an empty buffer
    for (b = bufs_by_age.begin(); b != bufs_by_age.end(); ++b) {
      if ((*b)->status == sweep_buffer::BUF_EMPTY)
	goto found;
    }

    // otherwise, find the oldest buffer that isn't being emptied
    for (b = bufs_by_age.begin(); b != bufs_by_age.end(); ++b) {
      if ((*b)->status != sweep_buffer::BUF_EMPTYING)
	goto found;
    }

    // there's a bug
    throw std::runtime_error("unable to get sweep buffer for storing data");
  }
 found:
  DBG_PRINTF("Got buffer %p for getter\n",  (*b));
  (*b)->clear();
  (*b)->status = sweep_buffer::BUF_FILLING; // NB: must be done after clear(), which sets it to BUF_EMPTY
  bufs_by_age.splice(bufs_by_age.end(), bufs_by_age, b); // move element pointed to by b to end of list
  DBG_PRINTF("Get_buffer_for_getter unlocking buffer_status_mutex\n");
  return *b;
}

sweep_buffer *
usrp_pulse_buffer::get_buffer_for_copier()
{
  // called only by copy_thread_fun
  // FIXME: include a timeout here?

  boost::unique_lock<boost::mutex> lock(buffer_status_mutex);
  DBG_PRINTF("Get_buffer_for_copier locked buffer_status_mutex\n");
  t_ordered_bufs_iter b;
  for (;;) {
    for (b = bufs_by_age.begin(); b != bufs_by_age.end(); ++b) {
      DBG_PRINTF("Buffer %p has status %d\n", (*b), (*b)->status);
      if ((*b)->status == sweep_buffer::BUF_FULL_HAS_META) {
	(*b)->status = sweep_buffer::BUF_EMPTYING;
	DBG_PRINTF("Got buffer %p for copier\n",  (*b));
	DBG_PRINTF("Get_buffer_for_copier unlocking buffer_status_mutex\n");
	return (*b);
      }
    }
    DBG_PRINTF("Get_buffer_for_copier unlocking buffer_status_mutex and waiting for buffer to complete\n");
    buffer_status_cv.wait(lock);
  } 
  throw std::runtime_error("unable to get sweep buffer with complete data");
}

void
usrp_pulse_buffer::set_buffer_status(sweep_buffer *buf, sweep_buffer::t_buf_status status)
{
  boost::unique_lock<boost::mutex> lock(buffer_status_mutex);
  DBG_PRINTF("set_buffer_status locked buffer_status_mutex\n");
  DBG_PRINTF("set_buffer_status setting %p to %d\n", buf, status);
  buf->status = status;
  DBG_PRINTF("set_buffer_status unlocking buffer_status_mutex\n");
}

void
usrp_pulse_buffer::set_getting(bool yesno)
{
  boost::unique_lock<boost::mutex> lock(getting_mutex);
  getting = yesno;
}

void
usrp_pulse_buffer::set_copying(bool yesno)
{
  boost::unique_lock<boost::mutex> lock(copying_mutex);
  copying = yesno;
}

void
usrp_pulse_buffer::getter_thread_fun(usrp_pulse_buffer *upb)
{
  pthread_sigmask(SIG_BLOCK, &signal_mask, 0);
  DBG_PRINT("Got to getter_thread_fun\n");  
  unsigned int prev_ARP_count = 0;
  unsigned int prev_ACP_count = 0;
  bool pulse_seen = false;
  sweep_buffer *buf = upb->get_buffer_for_getter(); // buffer we are currently reading data into
  sweep_buffer *prev_buf = 0; // buffer holding pulses some of whose ACP intervals still need updating

  upb->n_total = 0;

  while (upb->getting) { // loop for each pulse
    bool raw_mode = upb->urx->bbprx_mode() != BBPRX_MODE_NORMAL;

    // ensure there's room for another pulse
    if (buf->i_next_pulse >= buf->pmeta.size()) {
      // Not enough space in buffer, so re-allocate
      buf->set_size(buf->pmeta.size() + buf->BUFFER_REALLOC_INCREMENT, buf->spp);
    }

    // get the pulse

    upb->urx->get_pulse(buf->curr_sample(), false, buf->curr_pulse());
    ++upb->n_total;

    // call back user function
    if (upb->pulse_callback) 
      upb->pulse_callback(buf->curr_sample(), buf->spp, buf->curr_pulse(), upb->callback_user_data);

    if (raw_mode) {
      if (upb->n_pulses <= ++buf->i_next_pulse) {
	// we've obtained enough "pulses" for this buffer; mark it full and notify a waiting copier thread
	upb->set_buffer_status(buf, sweep_buffer::BUF_FULL_HAS_META);
	upb->buffer_status_cv.notify_one();
	buf = upb->get_buffer_for_getter();
      }
      continue;
    }

    if (! pulse_seen) {
      // we haven't seen a pulse yet, so don't have valid ARP/ACP counts
      prev_ARP_count = buf->curr_pulse()->n_ARPs;
      prev_ACP_count = buf->curr_pulse()->n_ACPs;
      pulse_seen = true;
      ++buf->i_next_pulse;
      continue;
    }
    

    if (prev_ACP_count != buf->curr_pulse()->n_ACPs) {
      // this is a new ACP count, so the just-finished ACP interval
      // might be usable to compute angles for some pulses

      prev_ACP_count = buf->curr_pulse()->n_ACPs;

      // the inter-ACP interval length as a denominator
      double factor = 1.0 / buf->curr_pulse()->ACP_interval;

      // First, check for any remaining ACP clocks to be computed
      // in the previous buffer
      if (prev_buf && prev_buf->status == sweep_buffer::BUF_FULL_NEEDS_META) {
	boost::unique_lock<boost::mutex> lock(upb->buffer_status_mutex);
	DBG_PRINTF("getter_thread_fun locked buffer_status_mutex\n");
	pulse_metadata *meta = &prev_buf->pmeta[0];
	for (unsigned int i = prev_buf->i_pulse_needing_ACP; i < prev_buf->i_next_pulse; ++i)
	  meta[i].ACP_clock = meta[i].n_ACPs + meta[i].ticks_since_last_ACP * factor;
	// now signal that this buffer is complete
	prev_buf->status =  sweep_buffer::BUF_FULL_HAS_META;
	upb->buffer_status_cv.notify_one();
	prev_buf = 0; // no longer need a pointer to previous buffer
	DBG_PRINTF("getter_thread_fun unlocking buffer_status_mutex\n");
      }
      // fill in ACP clock for any pulses in current buffer which need it
      pulse_metadata *meta = &buf->pmeta[0];
      unsigned i = buf->i_pulse_needing_ACP;
      unsigned imax = buf->pmeta.size();
      if (meta[i].n_ACPs != prev_ACP_count) {
	if (i == 0)
	  // the ACP interval in which the last ARP pulse occurred has finished, so 
	  // record its length in the corresponding slot of pulse 0, for later use
	  meta[0].ACP_interval_last_ARP = buf->curr_pulse()->ACP_interval_last_ARP;
	while(i < imax && meta[i].n_ACPs != prev_ACP_count) {
	  meta[i].ACP_clock = meta[i].n_ACPs + meta[i].ticks_since_last_ACP * factor;
	  ++i;
	}
	buf->i_pulse_needing_ACP = i;
      }
    }

    // see whether this is a new sweep
    
    if (prev_ARP_count != buf->curr_pulse()->n_ARPs) {
      // this is a new ARP count, so we've begun a new sweep
      // and have several things to do:
      // 1. compute some metadata for the just-finished sweep
      // 2. get a new sweep buffer and copy this pulse to it
      DBG_PRINTF("Got new ARP count: %d;  prev was %d\n", buf->curr_pulse()->n_ARPs, prev_ARP_count);

      prev_ARP_count = buf->curr_pulse()->n_ARPs;

      usrp_bbprx_sptr urx = upb->urx;
      sweep_metadata *smeta = &buf->smeta;
      pulse_metadata *p0 = &buf->pmeta[0];
      pulse_metadata *pn = buf->curr_pulse();

      smeta->serial_no			= p0->n_ARPs;
      double elapsed_seconds            = (p0->clock_ticks - p0->ticks_since_last_ARP) / (double) usrp_bbprx::USRP_CLOCK_RATE;
      smeta->timestamp			= urx->time_started() + boost::posix_time::hours((long) (elapsed_seconds / 3600)) + boost::posix_time::milliseconds((long) (1e3 * fmod(elapsed_seconds, 3600))) + boost::posix_time::microseconds((long) (1e6 * fmod(elapsed_seconds, 1e-3)));
      double dur_secs			= ((pn->clock_ticks - pn->ticks_since_last_ARP) - (p0->clock_ticks - p0->ticks_since_last_ARP)) / (double) usrp_bbprx::USRP_CLOCK_RATE;
      smeta->duration			= boost::posix_time::microseconds(1e6*dur_secs);
      smeta->samples_per_pulse		= buf->spp;
      smeta->n_pulses			= buf->i_next_pulse; // NB: last pulse belongs to next sweep, so don't add 1
      smeta->n_actual_pulses		= elapsed_to_from (pn->n_trigs, p0->n_trigs);
      smeta->radar_PRF			= smeta->n_actual_pulses / dur_secs;
      smeta->rx_PRF			= smeta->n_pulses / dur_secs;
      smeta->range_cell_size		= (urx->decim_rate() + 1.0) / usrp_bbprx::USRP_CLOCK_RATE * VELOCITY_OF_LIGHT / 2.0;
      smeta->n_ACPs			= elapsed_to_from(pn->ACP_count_last_ARP, p0->ACP_count_last_ARP);
      smeta->ACP_clock_at_ARP           = p0->ACP_count_last_ARP + p0->ACP_age_last_ARP / (double) p0->ACP_interval_last_ARP;
      smeta->n_read_errors		= urx->n_read_errors;
      smeta->n_overruns			= urx->n_overruns;
      smeta->n_missing_USB_packets	= urx->n_missing_USB_packets;
      smeta->n_missing_data_packets	= urx->n_missing_data_packets;
      urx->n_read_errors          = 0;
      urx->n_overruns             = 0; 
      urx->n_missing_USB_packets  = 0;
      urx->n_missing_data_packets = 0;

      // keep track of the fact this buffer is full but needs to have its metadata completed
      prev_buf = buf;
      upb->set_buffer_status (buf, sweep_buffer::BUF_FULL_NEEDS_META);
      buf = upb->get_buffer_for_getter();

      // copy last pulse, which actually belongs to new sweep
      memcpy(buf->curr_sample(), prev_buf->curr_sample(), sizeof(t_sample) * buf->spp);
    } else {
      ++ buf->i_next_pulse;
    }
  } // go back for more pulses
}

#define ROUNDED_DIVISION(X, Y) ({(typeof)X _X = X; (typeof) Y _Y = Y; (_X + _Y/2)/_Y;})
#define INTABS(X) ({(typeof)X _X = X; _X >= 0 ? _X : -_X;})
  
void 
usrp_pulse_buffer::copy_thread_fun (copy_thread_info *cti)
{
  pthread_sigmask(SIG_BLOCK, &signal_mask, 0);
  usrp_pulse_buffer *upb;
  got_sweep_function user_fun;
  void * user_data;
  int return_code;
  try {
    DBG_PRINTF("Got to copy_thread_fun\n");
    upb = cti->upb;
    t_sample *dest_buf = cti->buf;
    bool gated = cti->gated;
    unsigned int dest_pulses = cti->n_pulses;
    unsigned int dest_spp = cti->samples_per_pulse;
    struct pulse_metadata *dest_meta = cti->meta;
    double *pulse_angles = cti->pulse_angles;
    double *pulse_times = cti->pulse_times;
    struct sweep_metadata *dest_smeta = cti->smeta;
    user_fun = cti->user_fun;
    user_data = cti->user_data;
    unsigned int i_src, i_dest;
    return_code = cti->return_code = usrp_pulse_buffer::OKAY;

    // wait for a full sweep buffer with complete metadata
    sweep_buffer *buf;

    buf = upb->get_buffer_for_copier(); // might throw boost::thread_interrupted

    if (upb->copying) {

      pulse_metadata *src_meta = &buf->pmeta[0];
      t_sample *src_buf = &buf->samples[0];
      unsigned int src_spp = buf->spp;
      unsigned int src_pulses = buf->n_pulses();

      int extra = dest_spp - src_spp;
      unsigned int samples_to_copy = std::min(dest_spp, src_spp);

      if (!gated) {
	// easy case: no gating
	unsigned int pulses_to_copy = std::min(dest_pulses, src_pulses);
    
	if (dest_meta)
	  memcpy(dest_meta, src_meta, pulses_to_copy * sizeof(pulse_metadata));

	for (i_src = 0; i_src < pulses_to_copy; ++i_src) {
	  memcpy(dest_buf, src_buf, sizeof(t_sample) * samples_to_copy);
	  dest_buf += samples_to_copy;
	  src_buf += src_spp;
	  if (extra > 0) {
	    memset(dest_buf, 0, extra * sizeof(t_sample));
	    dest_buf += extra;
	  }
	  if (pulse_angles)
	    *pulse_angles++ = buf->pulse_angle(i_src);
	  if (pulse_times)
	    *pulse_times++ = src_meta->ticks_since_last_ARP * usrp_bbprx::USRP_CLOCK_INTERVAL;
	  ++src_meta;
	}
      } else {
	// Gated case is more complicated: each output pulse gets a copy
	// of the input pulse whose angle is closest to the one desired by
	// the gating scheme.  This can require decimation and/or
	// replication.  Assuming the radar velocity is always
	// non-negative clockwise (i.e. wind loading is never sufficient
	// to actually reverse the rotation of the radar, even
	// momentarily), we get a monotonic clock supplied by the
	// (interpolated) ACPs.  We interpolate them by dividing time
	// elapsed since the most recent ACP by the time elapsed between
	// that ACP and the next one, using it as the fractional portion
	// of the ACP count.  Then for each output slot, we calculate the
	// desired ACP count, and find the input pulse with the closest
	// such value.  We can use straight-line search because the
	// function mapping slot to best pulse is monotonic.

	// The getter thread has already back-calculated the ACP clock field for each pulse,
	// so all we need to do is normalize to get actual angle.

	unsigned int best_i_src; // slot of pulse whose ACP clock best matches desired ACP

	double ACP_clock_factor = buf->smeta.n_ACPs / (double) dest_pulses; // factor to convert ACP clock to angle in [0, 1)

	double prev_ACP = - buf->smeta.n_ACPs; // guaranteed out of range
	int prev_i_src = -1; // out of range 

	for (i_src = 0, i_dest = 0; i_dest < dest_pulses; ++i_dest) {
	  double desired_ACP = i_dest * ACP_clock_factor;
	  double curr_ACP = src_meta[i_src].ACP_clock - buf->smeta.ACP_clock_at_ARP;

	  // find the best pulse for slot i_dest
	  for (;;) {
	    if (prev_i_src >= 0 && fabs(prev_ACP - desired_ACP) < fabs(curr_ACP - desired_ACP)) {
	      // previous pulse was better for this slot, so copy it
	      best_i_src = prev_i_src;
	      break;
	    }
	    // current pulse is better; if it is not the last available then advance one
	    // pulse to see if we can do better
	    if (i_src < src_pulses - 1) {
	      prev_i_src = i_src++;
	      prev_ACP = curr_ACP;
	      curr_ACP = src_meta[i_src].ACP_clock - buf->smeta.ACP_clock_at_ARP;  // ACP at next source pulse
	    } else {
	      // we're at the last pulse, so we have to use it
	      best_i_src = i_src;
	      break;
	    }
	  }
	  // copy the pulse we found to have the best ACP-clock match to the current output slot
	  memcpy(dest_buf, & src_buf[best_i_src * src_spp], sizeof(t_sample) * samples_to_copy);
	  dest_buf += samples_to_copy;
	  if (extra > 0) {
	    memset(dest_buf, 0, extra * sizeof(t_sample));
	    dest_buf += extra;
	  }
	  if (dest_meta)
	    *dest_meta++ = src_meta[best_i_src];
	  if (pulse_angles)
	    *pulse_angles++ = buf->pulse_angle(best_i_src);
	  if (pulse_times)
	    *pulse_times++ = src_meta[best_i_src].ticks_since_last_ARP * usrp_bbprx::USRP_CLOCK_INTERVAL;
	  prev_i_src = best_i_src;
	}
      }

      // supply metadata, if requested
      if (dest_smeta) {
	*dest_smeta = buf->smeta;
	dest_smeta->n_pulses = dest_pulses;
	dest_smeta->samples_per_pulse = dest_spp;
      }

    }
    // mark buffer as no longer needed
    upb->set_buffer_status(buf, sweep_buffer::BUF_EMPTY);

  } catch (boost::thread_interrupted) {
    DBG_PRINTF("copy_thread_fun: caught exception\n");
    return_code = usrp_pulse_buffer::COPY_INTERRUPTED;
  }
  upb->set_copying(false);

  if (user_fun) {
    DBG_PRINTF("copy_thread_fun: calling user callback\n");
    (*user_fun)(return_code, user_data);
  }
}

unsigned int 
usrp_pulse_buffer::get_sweep (unsigned short *buf, bool gated, unsigned int n_pulses, unsigned short samples_per_pulse, struct pulse_metadata *meta, double *pulse_angles, double *pulse_times, struct sweep_metadata *smeta)
{
  copy_thread_info cti = {this, buf, gated, n_pulses, samples_per_pulse, meta, pulse_angles, pulse_times, smeta, 0, 0};
  copying = true; // set this, since it's also used to interrupt copying
  copy_thread_fun(&cti); // call and block while waiting

  return smeta->n_pulses;
}

static copy_thread_info cti_static;

bool 
usrp_pulse_buffer::get_sweep_nb (unsigned short *buf, bool gated, unsigned int n_pulses, unsigned short samples_per_pulse, struct pulse_metadata *meta, double *pulse_angles, double *pulse_times, struct sweep_metadata *smeta, got_sweep_function user_fun, void *user_data)
{
  boost::unique_lock<boost::mutex> lock(copying_mutex);
  if (copying)
    return false; // copy thread already running


  cti_static = ((copy_thread_info){this, buf, gated, n_pulses, samples_per_pulse, meta, pulse_angles, pulse_times, smeta, user_fun, user_data});
  copying = true;
  delete copy_thread;
  copy_thread = new boost::thread(&copy_thread_fun, &cti_static); // start a thread to do the copying; will call its callback
  return true; // return immediately
};

void
usrp_pulse_buffer::dump()
{
  std::cout << "Size of buf vector: " << bufs.size() << std::endl;
  for (t_all_bufs_iter b = bufs.begin(); b != bufs.end(); ++b)
    std::cout << &(*b) << ": status=" << (*b).status << "  i_next_pulse=" << (*b).i_next_pulse << std::endl;
  std::cout << "Bufs in order from oldest to youngest: " << std::endl;
  for (t_ordered_bufs_iter b = bufs_by_age.begin(); b != bufs_by_age.end(); ++b)
    std::cout << "Pointer is " << (*b) << std::endl;
};

 
usrp_pulse_buffer::~usrp_pulse_buffer()
{
  // add any code here
}

