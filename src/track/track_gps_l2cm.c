/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Adel Mamin <adel.mamin@exafore.com>
 *          Pasi Miettinen <pasi.miettinen@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* skip weak attributes for L2C API implementation */
#define TRACK_GPS_L2CM_INTERNAL
#include "track_gps_l2cm.h"
#include "track_api.h"
#include "manage.h"
#include "track.h"
#include "l2c_capb.h"

#include <libswiftnav/constants.h>
#include <libswiftnav/logging.h>
#include <libswiftnav/signal.h>
#include <libswiftnav/track.h>

#include <string.h>
#include <assert.h>
#include "math.h"

#include "settings.h"
#include "signal.h"

/** L2C coherent integration time [ms] */
#define L2C_COHERENT_INTEGRATION_TIME_MS 20

/* Alias detection interval [ms] */
#define L2C_ALIAS_DETECT_INTERVAL_MS     500

/* Number of chips to integrate over in the short cycle interval [chips]
 * The value must be within [0..GPS_L2C_CHIPS_NUM].
 * GPS_L2C_CHIPS_NUM equals to 20ms
 */
#define L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS 300

/* Number of chips to integrate over in the long cycle interval [chips] */
#define L2CM_TRACK_LONG_CYCLE_INTERVAL_CHIPS \
  (GPS_L2C_CHIPS_NUM - L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS)

/* Number of chips to integrate over in the long cycle interval
   at start-up [chips] */
#define L2CM_TRACK_LONG_STARTUP_CYCLE_INTERVAL_CHIPS \
  (GPS_L2C_CHIPS_NUM - 2 * L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS)

#define L2CM_TRACK_SETTING_SECTION "l2cm_track"

/*                        code: nbw zeta  k  carr_to_code
                       carrier:                     nbw zeta  k  fll_aid */
#define LOOP_PARAMS_MED "(20 ms, (1, 0.7, 1, 1200), (13, 0.7, 1, 5))"

/*                          k1,   k2,  lp,  lo */
#define LD_PARAMS          "0.0247, 1.5, 50, 240"
#define LD_PARAMS_DISABLE  "0.02, 1e-6, 1, 1"

#define CN0_EST_LPF_CUTOFF 0.1f

#define INTEG_PERIOD_20_MS 20

static const u8 integration_periods[] = {
  INTEG_PERIOD_20_MS
};

#define INTEG_PERIODS_NUM (sizeof(integration_periods) / \
                           sizeof(integration_periods[0]))

static cn0_est_params_t cn0_est_pre_computed[INTEG_PERIODS_NUM];

static struct loop_params {
  float code_bw, code_zeta, code_k, carr_to_code;
  float carr_bw, carr_zeta, carr_k, carr_fll_aid_gain;
  u8 coherent_ms;
} loop_params_stage;

static struct lock_detect_params {
  float k1, k2;
  u16 lp, lo;
} lock_detect_params;

static float track_cn0_use_thres = 37.0; /* dBHz */
static float track_cn0_drop_thres = 31.0; /* dBHz */

static char loop_params_string[120] = LOOP_PARAMS_MED;
static char lock_detect_params_string[24] = LD_PARAMS;
static bool use_alias_detection = true;

typedef struct {
  aided_tl_state_t tl_state;   /**< Tracking loop filter state. */
  corr_t cs[3];                /**< EPL correlation results in correlation period. */
  cn0_est_state_t cn0_est;     /**< C/N0 Estimator. */
  u8 int_ms;                   /**< Integration length. */
  bool short_cycle;            /**< Set to true when a short 1ms integration is requested. */
  u8 startup;                  /**< An indicator of start-up phase. */
  u8 stage;                    /**< 0 = First-stage. 1 ms integration.
                                    1 = Second-stage. After nav bit sync,
                                    retune loop filters and typically (but
                                    not necessarily) use longer integration. */
  alias_detect_t alias_detect; /**< Alias lock detector. */
  lock_detect_t lock_detect;   /**< Phase-lock detector state. */
} gps_l2cm_tracker_data_t;

static tracker_t gps_l2cm_trackers[NUM_GPS_L2CM_TRACKERS];
static gps_l2cm_tracker_data_t gps_l2cm_tracker_data[NUM_GPS_L2CM_TRACKERS];

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data);
static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data);
static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data);

static bool parse_loop_params(struct setting *s, const char *val);
static bool parse_lock_detect_params(struct setting *s, const char *val);
static void precompute_cn0_est_params(void);

static const tracker_interface_t tracker_interface_gps_l2cm = {
  .code =         CODE_GPS_L2CM,
  .init =         tracker_gps_l2cm_init,
  .disable =      tracker_gps_l2cm_disable,
  .update =       tracker_gps_l2cm_update,
  .trackers =     gps_l2cm_trackers,
  .num_trackers = NUM_GPS_L2CM_TRACKERS
};

static tracker_interface_list_element_t
  tracker_interface_list_element_gps_l2cm = {
    .interface = &tracker_interface_gps_l2cm,
    .next = 0
  };

/** Register L2 CM tracker into the the tracker interface & settings
 *  framework.
 */
void track_gps_l2cm_register(void)
{
  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "loop_params",
                 loop_params_string,
                 TYPE_STRING, parse_loop_params);

  SETTING_NOTIFY(L2CM_TRACK_SETTING_SECTION, "lock_detect_params",
                 lock_detect_params_string,
                 TYPE_STRING, parse_lock_detect_params);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_use",
          track_cn0_use_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "cn0_drop",
          track_cn0_drop_thres, TYPE_FLOAT);

  SETTING(L2CM_TRACK_SETTING_SECTION, "alias_detect",
          use_alias_detection, TYPE_BOOL);

  for (u32 i = 0; i < NUM_GPS_L2CM_TRACKERS; i++) {
    gps_l2cm_trackers[i].active = false;
    gps_l2cm_trackers[i].data = &gps_l2cm_tracker_data[i];
  }

  precompute_cn0_est_params();

  tracker_interface_register(&tracker_interface_list_element_gps_l2cm);
}

/** Do L1C/A to L2 CM handover.
 *
 * The condition for the handover is the availability of bitsync on L1 C/A
 *
 * \param sample_count NAP sample count
 * \param sat L1C/A Satellite ID
 * \param code_phase L1CA code phase [chips]
 * \param carrier_freq The current Doppler frequency for the L1 C/A channel
 * \param cn0 CN0 estimate for the L1 C/A channel
 */
void do_l1ca_to_l2cm_handover(u32 sample_count,
                              u16 sat,
                              float code_phase,
                              double carrier_freq,
                              float cn0_init)
{
  /* compose SID: same SV, but code is L2 CM */
  gnss_signal_t sid = construct_sid(CODE_GPS_L2CM, sat);

  if (!tracking_startup_ready(sid)) {
    return; /* L2C signal from the SV is already in track */
  }

  if (0 == (gps_l2cm_l2c_cap_read() & ((u32)1 << (sat - 1)))) {
    return;
  }

  if ((code_phase < 0) ||
      ((code_phase > 0.5) && (code_phase < (GPS_L1CA_CHIPS_NUM - 0.5)))) {
    log_warn_sid(sid, "Unexpected L1C/A to L2C handover code phase: %f",
                 code_phase);
    return;
  }

  if (code_phase > (GPS_L1CA_CHIPS_NUM - 0.5)) {
    code_phase = GPS_L2C_CHIPS_NUM - (GPS_L1CA_CHIPS_NUM - code_phase);
  }

  /* The best elevation estimation could be retrieved by calling
     tracking_channel_evelation_degrees_get(nap_channel) here.
     However, we assume it is done where tracker_channel_init()
     is called. */

  tracking_startup_params_t startup_params = {
    .sid                = sid,
    .sample_count       = sample_count,
    /* recalculate doppler freq for L2 from L1*/
    .carrier_freq       = carrier_freq * GPS_L2_HZ / GPS_L1_HZ,
    .code_phase         = code_phase,
    .chips_to_correlate = L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS,
    /* get initial cn0 from parent L1 channel */
    .cn0_init           = cn0_init,
    .elevation          = TRACKING_ELEVATION_UNKNOWN
  };

  switch (tracking_startup_request(&startup_params)) {
    case 0:
      log_info_sid(sid, "L2 CM handover done");
      break;

    case 1:
      /* sat is already in fifo, no need to inform */
      break;

    case 2:
      log_warn_sid(sid, "Failed to start L2C tracking");
      break;

    default:
      assert(!"Unknown code returned");
      break;
  }
}

static void tracker_gps_l2cm_init(const tracker_channel_info_t *channel_info,
                                  tracker_common_data_t *common_data,
                                  tracker_data_t *tracker_data)
{
  (void)channel_info;
  gps_l2cm_tracker_data_t *data = tracker_data;

  memset(data, 0, sizeof(gps_l2cm_tracker_data_t));
  tracker_ambiguity_unknown(channel_info->context);

  const struct loop_params *l = &loop_params_stage;

  assert(20 == l->coherent_ms);
  data->int_ms = l->coherent_ms;

  aided_tl_init(&(data->tl_state), 1e3 / data->int_ms,
                common_data->code_phase_rate - GPS_CA_CHIPPING_RATE,
                l->code_bw, l->code_zeta, l->code_k,
                l->carr_to_code,
                common_data->carrier_freq,
                l->carr_bw, l->carr_zeta, l->carr_k,
                l->carr_fll_aid_gain);

  data->short_cycle = true;
  data->startup = 2;

  /* Initialise C/N0 estimator */
  cn0_est_init(&data->cn0_est, 1e3 / data->int_ms, common_data->cn0);

  /* Initialize lock detector */
  lock_detect_init(&data->lock_detect,
                   lock_detect_params.k1, lock_detect_params.k2,
                   lock_detect_params.lp, lock_detect_params.lo);

  /* TODO: Reconfigure alias detection between stages */
  u8 alias_detect_ms = l->coherent_ms;
  alias_detect_init(&data->alias_detect,
                    L2C_ALIAS_DETECT_INTERVAL_MS / alias_detect_ms,
                    (alias_detect_ms - 1) * 1e-3);

  /* L2C bit sync is known once we start tracking it since
     the L2C ranging code length matches the bit length (20ms).
     This is the end of 20ms integration period and the edge
     of a data bit. */
  tracker_bit_sync_set(channel_info->context, 0);
}

static void tracker_gps_l2cm_disable(const tracker_channel_info_t *channel_info,
                                     tracker_common_data_t *common_data,
                                     tracker_data_t *tracker_data)
{
  (void)channel_info;
  (void)common_data;
  (void)tracker_data;
}

/** Handle tracker start-up stage.
 * The total length of coherent integrations is always 20 ms.
 * At start-up, the tracker always uses the following integration scheme:
 * 'short + short + start-up long'.
 * Then it continues with 'short + regular long' cycles.
 * The reason behind it is that when tracker gets started in NAP, we always
 * use the short integration cycle first, which is
 * L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS chips long. When the first NAP
 * interrupt arrives, the L2C tracker gets the first chance to change the
 * integration length. It sets it to be
 * L2CM_TRACK_LONG_STARTUP_CYCLE_INTERVAL_CHIPS chips. However, NAP continues
 * doing integration and still uses the previous integration length, which is
 * L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS chips long. At the next interrupt
 * FW sets the integration length to be L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS.
 *
 * \param channel_info INfo associated with a tracker channel
 * \param common_data Common tracking channel data
 * \param data L2C specific tracking channel data
 */
static void handle_tracker_startup(const tracker_channel_info_t *channel_info,
                                   tracker_common_data_t *common_data,
                                   gps_l2cm_tracker_data_t *data)
{
  corr_t cs[3];

  switch (data->startup) {
  case 2:
    tracker_correlations_read(channel_info->context, data->cs,
                              &common_data->sample_count,
                              &common_data->code_phase_early,
                              &common_data->carrier_phase);
    alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);

    tracker_retune(channel_info->context, common_data->carrier_freq,
                   common_data->code_phase_rate,
                   L2CM_TRACK_LONG_STARTUP_CYCLE_INTERVAL_CHIPS);
    break;

  case 1:
    tracker_correlations_read(channel_info->context, cs,
                              &common_data->sample_count,
                              &common_data->code_phase_early,
                              &common_data->carrier_phase);
    /* Accumulate two short cycle correlations */
    for(int i = 0; i < 3; i++) {
      data->cs[i].I += cs[i].I;
      data->cs[i].Q += cs[i].Q;
    }
    tracker_retune(channel_info->context, common_data->carrier_freq,
                   common_data->code_phase_rate,
                   L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS);
    break;

  default:
    assert(0);
    break;
  }
}

static void tracker_gps_l2cm_update(const tracker_channel_info_t *channel_info,
                                    tracker_common_data_t *common_data,
                                    tracker_data_t *tracker_data)
{
  gps_l2cm_tracker_data_t *data = tracker_data;

  if (data->startup) {
    /* Handle start-up case, when we have 1+1+18 integration cycles */
    handle_tracker_startup(channel_info, common_data, data);
    if (1 == data->startup) {
      /* The start-up phase is over. Continue with the normal
         sequence of short & long cycles. The next cycle
         is going to be long. */
      data->short_cycle = false;
    }
    if (data->startup) {
      data->startup--;
    }
    return;
  }

  /* Read early ([0]), prompt ([1]) and late ([2]) correlations. */
  if (data->short_cycle) {
    /* The throw away data. They are not needed for the short cycle.
       And we also do not want to clobber common_data content
       as it contains valid data for the previous full 20 ms cycle. */
    u32 sample_count;        /* Total num samples channel has tracked for. */
    double code_phase_early; /* Early code phase. */
    double carrier_phase;    /* Carrier phase in NAP register units. */

    tracker_correlations_read(channel_info->context, data->cs,
                              &sample_count,
                              &code_phase_early,
                              &carrier_phase);

    alias_detect_first(&data->alias_detect, data->cs[1].I, data->cs[1].Q);
  } else {
    /* This is the end of the long cycle's correlations. */
    corr_t cs[3];
    tracker_correlations_read(channel_info->context, cs,
                              &common_data->sample_count,
                              &common_data->code_phase_early,
                              &common_data->carrier_phase);
    /* Accumulate short cycle correlations with long ones. */
    for(int i = 0; i < 3; i++) {
      data->cs[i].I += cs[i].I;
      data->cs[i].Q += cs[i].Q;
    }
  }

  /* We're doing long integrations, alternate between short and long
   * cycles. This is because of FPGA pipelining and latency.
   * The loop parameters can only be updated at the end of the second
   * integration interval.
   */
  bool short_cycle = data->short_cycle;

  data->short_cycle = !data->short_cycle;

  if (short_cycle) {
    tracker_retune(channel_info->context, common_data->carrier_freq,
                   common_data->code_phase_rate,
                   L2CM_TRACK_SHORT_CYCLE_INTERVAL_CHIPS);
    return;
  }

  common_data->update_count += data->int_ms;

  common_data->TOW_ms = tracker_tow_update(channel_info->context,
                                           common_data->TOW_ms,
                                           data->int_ms);

  /* Call the bit sync update API to do data decoding */
  tracker_bit_sync_update(channel_info->context, data->int_ms, data->cs[1].I);

  corr_t* cs = data->cs;

  /* Update C/N0 estimate */
  {
    cn0_est_params_t params;
    const cn0_est_params_t *pparams = NULL;

    /* TODO
     * Store a pointer to the cn0_est_params_t in the gps_l1ca_tracker_data_t
     * structure so we don't have to scan through the whole array each time
     */
    for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
      if(data->int_ms == integration_periods[i]) {
        pparams = &cn0_est_pre_computed[i];
        break;
      }
    }

    if(NULL == pparams) {
      cn0_est_compute_params(&params, 1e3f / data->int_ms, CN0_EST_LPF_CUTOFF,
                             1e3f / data->int_ms);
      pparams = &params;
    }

    common_data->cn0 = cn0_est(&data->cn0_est,
                               pparams,
                               (float) cs[1].I/data->int_ms,
                               (float) cs[1].Q/data->int_ms);
  }

  if (common_data->cn0 > track_cn0_drop_thres) {
    common_data->cn0_above_drop_thres_count = common_data->update_count;
  }

  if (common_data->cn0 < track_cn0_use_thres) {
    /* SNR has dropped below threshold, indicate that the carrier phase
     * ambiguity is now unknown as cycle slips are likely. */
    tracker_ambiguity_unknown(channel_info->context);
    /* Update the latest time we were below the threshold. */
    common_data->cn0_below_use_thres_count = common_data->update_count;
  }

  /* Update PLL lock detector */
  bool last_outp = data->lock_detect.outp;
  lock_detect_update(&data->lock_detect, cs[1].I, cs[1].Q, data->int_ms);
  if (data->lock_detect.outo) {
    common_data->ld_opti_locked_count = common_data->update_count;
  }
  if (!data->lock_detect.outp) {
    common_data->ld_pess_unlocked_count = common_data->update_count;
  }

  /* Reset carrier phase ambiguity if there's doubt as to our phase lock */
  if (last_outp && !data->lock_detect.outp) {
    log_info_sid(channel_info->sid, "PLL stress");
    tracker_ambiguity_unknown(channel_info->context);
  }

  /* Run the loop filters. */

  /* Output I/Q correlations using SBP if enabled for this channel */
  tracker_correlations_send(channel_info->context, cs);

  correlation_t cs2[3];
  for (u32 i = 0; i < 3; i++) {
    cs2[i].I = cs[2-i].I;
    cs2[i].Q = cs[2-i].Q;
  }

  aided_tl_update(&data->tl_state, cs2);
  common_data->carrier_freq = data->tl_state.carr_freq;
  common_data->code_phase_rate = data->tl_state.code_freq +
                                 GPS_CA_CHIPPING_RATE;

  /* Attempt alias detection if we have pessimistic phase lock detect OR
     optimistic phase lock detect */
  if (use_alias_detection &&
     (data->lock_detect.outp || data->lock_detect.outo)) {
    s32 I = (cs[1].I - data->alias_detect.first_I) / (data->int_ms - 1);
    s32 Q = (cs[1].Q - data->alias_detect.first_Q) / (data->int_ms - 1);
    float err = alias_detect_second(&data->alias_detect, I, Q);
    if (fabs(err) > (250 / data->int_ms)) {
      if (data->lock_detect.outp) {
        log_warn_sid(channel_info->sid, "False phase lock detected");
      }

      tracker_ambiguity_unknown(channel_info->context);
      /* Indicate that a mode change has occurred. */
      common_data->mode_change_count = common_data->update_count;

      data->tl_state.carr_freq += err;
      data->tl_state.carr_filt.y = data->tl_state.carr_freq;
    }
  }

  tracker_retune(channel_info->context, common_data->carrier_freq,
                 common_data->code_phase_rate,
                 L2CM_TRACK_LONG_CYCLE_INTERVAL_CHIPS);
}

/** Parse a string describing the tracking loop filter parameters into
 *  the loop_params_stage struct.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_loop_params(struct setting *s, const char *val)
{
  /** The string contains loop parameters for one stage */

  struct loop_params loop_params_parse;

  const char *str = val;
  struct loop_params *l = &loop_params_parse;

  unsigned int tmp; /* newlib's sscanf doesn't support hh size modifier */

  if (sscanf(str, "( %u ms , ( %f , %f , %f , %f ) , ( %f , %f , %f , %f ) ) ",
             &tmp,
             &l->code_bw, &l->code_zeta, &l->code_k, &l->carr_to_code,
             &l->carr_bw, &l->carr_zeta, &l->carr_k, &l->carr_fll_aid_gain
             ) < 9) {
    log_error("Ill-formatted tracking loop param string: %20s", str);
    return false;
  }
  l->coherent_ms = tmp;

  if (l->coherent_ms != L2C_COHERENT_INTEGRATION_TIME_MS) {
    log_error("Invalid coherent integration length for L2CM: %" PRIu8,
              l->coherent_ms);
    return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&loop_params_stage, &loop_params_parse, sizeof(loop_params_stage));

  return true;
}

/** Parse a string describing the tracking loop phase lock detector
 *  parameters into the lock_detect_params structs.
 *
 * \param s Settings structure provided to store the input string.
 * \param val The input string to parse.
 * \retval true Success
 * \retval false Failure
 */
static bool parse_lock_detect_params(struct setting *s, const char *val)
{
  struct lock_detect_params p;

  if (sscanf(val, "%f , %f , %" SCNu16 " , %" SCNu16,
             &p.k1, &p.k2, &p.lp, &p.lo) < 4) {
      log_error("Ill-formatted lock detect param string: %20s", val);
      return false;
  }
  /* Successfully parsed the input. Save to memory. */
  strncpy(s->addr, val, s->len);
  if (s->len > 0) {
    char *ptr = (char*) s->addr;
    ptr[s->len - 1] = '\0';
  }
  memcpy(&lock_detect_params, &p, sizeof(lock_detect_params));

  return true;
}

/* Pre-compute C/N0 estimator and filter parameters. The parameters are
 * computed using equivalent of cn0_est_compute_params() function for
 * integration periods and cut-off frequency defined in this file.
 */
static void precompute_cn0_est_params(void)
{
  for(u32 i = 0; i < INTEG_PERIODS_NUM; i++) {
    cn0_est_compute_params(&cn0_est_pre_computed[i],
                           1e3f / integration_periods[i],
                           CN0_EST_LPF_CUTOFF,
                           1e3f / integration_periods[i]);
  }
}
