#ifdef __cplusplus

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
#include <time.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <usrp/usrp_bbprx.h>
#include <usrp/usrp_bytesex.h>
#include <usrp/usrp_prims.h>  // for usrp_rescan()
#include "fpga_regs_common.h"
#include "fpga_regs_bbprx.h"
#include "usrp_pulse_buffer.h"


#ifdef HAVE_SCHED_H
#include <sched.h>
#endif

#endif // __cplusplus

#define MAX_N_SAMPLES 16384
#define N_USRP_RADR_PLUGIN_PARAMS 23
#define USRP_BITS_PER_SAMPLE 12

#ifdef __cplusplus

typedef void *SEXP;
class usrp_radR_plugin; // forward decl for C++

#else  // __cplusplus

struct usrp_radR_plugin; // forward decl for C

#endif // __cplusplus
typedef struct usrp_radR_plugin *URP;


// Forward declarations of "C" friend functions 

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

URP URP_make ();

int URP_digitizing (URP urp, int msec); // check whether trigger signal count increases over given interval

int URP_incoming_data (URP urp); // check whether data is coming in by looking at buffer status

int URP_have_usrp (URP urp);

double URP_get_param (URP urp, int which, double bogus);

void URP_get_params (URP urp, double *dest, int max_param);

int URP_set_param (URP urp, int which, double value);

void URP_set_pulse_callback (URP urp, void *callback, void *userdata);

int URP_send_params (URP urp);

int URP_start_up (URP urp, const char* fpga_filename, const char* firmware_filename);

int URP_shut_down (URP urp);

int URP_destroy (URP urp);

int URP_started (URP urp);

void * URP_get_client_data (URP urp);

void URP_set_client_data (URP urp, void *data);

int URP_get_sweep_nb (URP urp, unsigned short *dest, unsigned short n_pulses, unsigned short n_samples, int gated, double *pulse_angles, double *pulse_times, void (*callback)(int, void *));

int URP_get_sweep (URP urp, unsigned short *dest, unsigned short n_pulses, unsigned short n_samples, int gated, double *pulse_angles, double *pulse_times);

double URP_get_timestamp (URP me);

double URP_get_duration (URP me);

double URP_get_range_cell_size (URP me);

double URP_get_radar_PRF (URP me);

int URP_last_error (URP me);

void URP_set_last_error (URP me, int err);

int URP_have_scan_data (URP me);

void URP_set_have_scan_data (URP me, int have);

#ifdef __cplusplus
}
#endif // __cplusplus

#ifdef __cplusplus

class usrp_radR_plugin {

protected:
  bool                  started            ;/**< has start_up been called on the plugin? */
  sweep_metadata        smeta              ;/**< sweep metadata for background getter */
  SEXP                  scan_info_vector   ;/**< metadata for each scan; re-used */
  SEXP                  trv                ;/**< return value vector for getter thread */ 
  int                   trv_index          ;/**< index in trv of value to be returned by getter thread */
  SEXP                  *scan_info_rv      ;/**< we store scan_info_vector here when it is ready */
  bool                  have_scan_data     ;/**< have we already copied data for latest scan? */
  usrp_pulse_buffer     *upb               ;/**< underlying USRP pulse buffer */
  int			which		   ;/**< which USRP board */
  int			fusb_block_size	   ;/**< size of fast USB block */
  int			fusb_nblocks	   ;/**< number of fast USB blocks */
  int                   last_error         ;/**< code of most recent error; reset to 0 after call to UBP_last_error() */
  void                  *client_data       ;/**< used by extern "C" functions */

  double param[N_USRP_RADR_PLUGIN_PARAMS]      ;/**< values of digitizing parameters */
  bool param_changed[N_USRP_RADR_PLUGIN_PARAMS] ;/**< have values changed since last transmit? */

  usrp_radR_plugin();

  /*! \brief
   *  set a parameter value
   */
  bool set_param(int parno, double parval);

  /*! \brief
   *  send all changed parameter values to the USRP
   */
  int send_params();

public:

  static usrp_radR_plugin * make();
  
  static const int NUM_PARAMS = N_USRP_RADR_PLUGIN_PARAMS; /**< number of parameters for controlling the digitizer */
  static const int BITS_PER_SAMPLE = USRP_BITS_PER_SAMPLE; /**< samples from the usrp fill the lower 12 bits of a 16-bit word */
  static const int CLOCKS_PER_SECOND = 64000000;      /**< the usrp fpga clock runs at 64MHz */

  static const int PARAM_DECIM			=  0; /**< decimation rate; 0 = every sample, 1 = every 2nd sample, etc. */
  static const int PARAM_VID_GAIN		=  1; /**< video channel gain in db */
  static const int PARAM_VID_NEGATE		=  2; /**< should video channel be negated? */
  static const int PARAM_TRIG_GAIN		=  3; /**< trigger channel gain in db */
  static const int PARAM_TRIG_THRESH_EXCITE	=  4; /**< trigger excitation threshold, in % of max */
  static const int PARAM_TRIG_THRESH_RELAX	=  5; /**< trigger excitation threshold, in % of max  */
  static const int PARAM_TRIG_LATENCY		=  6; /**< trigger latency, in usrp clock ticks */
  static const int PARAM_TRIG_DELAY		=  7; /**< trigger delay, in usrp clock ticks */
  static const int PARAM_ARP_GAIN		=  8; /**< ARP gain in db */
  static const int PARAM_ARP_THRESH_EXCITE	=  9; /**< ARP excitation threshold, in % of max */
  static const int PARAM_ARP_THRESH_RELAX	= 10; /**< ARP excitation threshold, in % of max */
  static const int PARAM_ARP_LATENCY		= 11; /**< ARP latency, in usrp clock ticks */
  static const int PARAM_ACP_GAIN		= 12; /**< ACP gain in db */
  static const int PARAM_ACP_THRESH_EXCITE	= 13; /**< ACP excitation threshold, in % of max */
  static const int PARAM_ACP_THRESH_RELAX	= 14; /**< ACP excitation threshold, in % of max */
  static const int PARAM_ACP_LATENCY		= 15; /**< ACP latency, in usrp clock ticks */
  static const int PARAM_N_SAMPLES		= 16; /**< number of samples per pulse which we're trying to obtain */
  static const int PARAM_N_PULSES		= 17; /**< (max) number of pulses we want per sweep */
  static const int PARAM_COUNTING		= 18; /**< DEBUG: interleave actual samples with low order bits of 64 MHz clock? */
  static const int PARAM_ALL_PULSES		= 19; /**< do we want all pulses, or gated */
  static const int PARAM_OWN_THREAD		= 20; /**< should the getter run in its own thread? */
  static const int PARAM_BBPRX_MODE		= 21; /**< DIAGNOSTICS: which mode do we digitize in */
  static const int PARAM_N_BUFS                 = 22; /**< number of sweep buffers to maintain */

  static const int NO_ERROR                            =  0;
  static const int ERROR_CANT_FIND_USRP                =  1;
  static const int ERROR_CANT_START_TRANSFERS          =  2;
  static const int ERROR_CANT_START_DIGITIZING         =  3;
  static const int ERROR_CANT_STOP_TRANSFERS           =  4;
  static const int ERROR_CANT_STOP_DIGITIZING          =  5;
  static const int ERROR_CANT_SET_REALTIME_PRIORITY    =  6;
  static const int ERROR_CANT_SET_PARAM                =  7; // NB: keep this as the last error message

  ~usrp_radR_plugin();

  // C-callable functions that need access to internals

  friend URP URP_make ();

  friend int URP_digitizing (URP urp, int msec);

  friend int URP_incoming_data (URP urp);

  friend int URP_have_usrp (URP urp);

  friend double URP_get_param (URP urp, int which, double bogus);

  friend void URP_get_params (URP urp, double *dest, int max_param);

  friend int URP_set_param (URP urp, int which, double value);

  friend void URP_set_pulse_callback (URP urp, void *callback, void *userdata);

  friend int URP_send_params (URP urp);

  friend int URP_start_up (URP urp, const char* fpga_filename, const char* firmware_filename);

  friend int URP_shut_down (URP urp);

  friend int URP_destroy (URP urp);

  friend int URP_started (URP urp);

  friend void * URP_get_client_data (URP urp);

  friend void URP_set_client_data (URP urp, void *data);

  friend int URP_get_sweep_nb (URP urp, unsigned short *dest, unsigned short n_pulses, unsigned short n_samples, int gated, double *pulse_angles, double *pulse_times, void (*callback)(int, void *));

  friend int URP_get_sweep (URP urp, unsigned short *dest, unsigned short n_pulses, unsigned short n_samples, int gated, double *pulse_angles, double *pulse_times);

  friend double URP_get_timestamp (URP me);

  friend double URP_get_duration (URP me);

  friend double URP_get_range_cell_size (URP me);

  friend double URP_get_radar_PRF (URP me);

  friend int URP_last_error (URP me);

  friend void URP_set_last_error (URP me, int err);

  friend int URP_have_scan_data (URP me);

  friend void URP_set_have_scan_data (URP me, int have);
};

#else // __cplusplus

#define URP_PARAM_DECIM			 0
#define URP_PARAM_VID_GAIN		 1
#define URP_PARAM_VID_NEGATE		 2
#define URP_PARAM_TRIG_GAIN		 3
#define URP_PARAM_TRIG_THRESH_EXCITE	 4
#define URP_PARAM_TRIG_THRESH_RELAX	 5
#define URP_PARAM_TRIG_LATENCY		 6
#define URP_PARAM_TRIG_DELAY		 7
#define URP_PARAM_ARP_GAIN		 8
#define URP_PARAM_ARP_THRESH_EXCITE	 9
#define URP_PARAM_ARP_THRESH_RELAX	10
#define URP_PARAM_ARP_LATENCY		11
#define URP_PARAM_ACP_GAIN		12
#define URP_PARAM_ACP_THRESH_EXCITE	13
#define URP_PARAM_ACP_THRESH_RELAX	14
#define URP_PARAM_ACP_LATENCY		15
#define URP_PARAM_N_SAMPLES		16
#define URP_PARAM_N_PULSES		17
#define URP_PARAM_COUNTING		18
#define URP_PARAM_ALL_PULSES		19
#define URP_PARAM_OWN_THREAD		20
#define URP_PARAM_BBPRX_MODE		21
#define URP_PARAM_N_BUFS                22

#endif // __cplusplus
