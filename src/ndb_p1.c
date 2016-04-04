/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Roman Gezikov <rgezikov@exafore.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <string.h>
#include <assert.h>
#include "ndb.h"
#include <libswiftnav/constants.h>
#include "libswiftnav/logging.h"
#include "timing.h"
#include "sbp.h"
#include "sbp_utils.h"

static ephemeris_t ndb_ephemeris[PLATFORM_SIGNAL_COUNT] _CCM;
static ndb_element_metadata_t ndb_ephemeris_md[PLATFORM_SIGNAL_COUNT] _CCM;
static almanac_t ndb_almanac[PLATFORM_SIGNAL_COUNT] _CCM;
static ndb_element_metadata_t ndb_almanac_md[PLATFORM_SIGNAL_COUNT] _CCM;
#define NDB_EPHE_FILE_NAME   "ephe"
#define NDB_ALMA_FILE_NAME   "alma"

static ndb_file_t ndb_ephe_file = { .name = NDB_EPHE_FILE_NAME, .fh = -1,
    .expected_size = sizeof(ndb_file_version)
        + sizeof(ephemeris_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_element_metadata_nv_t) * PLATFORM_SIGNAL_COUNT };

static ndb_file_t ndb_alma_file = { .name = NDB_ALMA_FILE_NAME, .fh = -1,
    .expected_size = sizeof(ndb_file_version)
        + sizeof(almanac_t) * PLATFORM_SIGNAL_COUNT
        + sizeof(ndb_element_metadata_nv_t) * PLATFORM_SIGNAL_COUNT };

static bool take_first_ephe = true;
typedef struct {
  ephemeris_t ephe;
  bool used;
  ndb_timestamp_t received_at;
} ephemeris_candidate_t;

typedef enum {
  EPHE_IDENTICAL,
  EPHE_NEW_CANDIDATE,
  EPHE_NEW_TRUSTED,
  EPHE_CAND_MISMATCH,
} ndb_ephemeris_status_t;

#define EPHE_CAND_LIST_LEN (MAX_CHANNELS*2/3)
#define MAX_EPHE_CANDIDATE_AGE 92 /* seconds */
static ephemeris_candidate_t ephe_candidates[EPHE_CAND_LIST_LEN] _CCM;
static MUTEX_DECL(cand_list_access);

#define EPHEMERIS_MESSAGE_SPACING_cycle        (200 / NV_WRITE_REQ_TIMEOUT)
#define EPHEMERIS_TRANSMIT_EPOCH_SPACING_cycle (15 * 1000 / NV_WRITE_REQ_TIMEOUT)

enum ndb_op_code ndb_p1_init()
{
  u32 i;

  memset(ephe_candidates, 0, sizeof(ephe_candidates));

  memset(ndb_ephemeris, 0, sizeof(ndb_ephemeris));
  for (i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_ephemeris[i].sid = sid_from_global_index(i);
  }

  if (ndb_load_data(&ndb_ephe_file, ndb_ephemeris, ndb_ephemeris_md,
                    sizeof(ephemeris_t), PLATFORM_SIGNAL_COUNT)
      != NDB_ERR_NONE) {
    log_info("No ephemeris file present in flash, create an empty one");
  } else
    log_info("Ephemerides loaded from flash");

  for (i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_ephemeris_md[i].data = &ndb_ephemeris[i];
    ndb_ephemeris_md[i].data_size = sizeof(ephemeris_t);
    ndb_ephemeris_md[i].index = i;
    ndb_ephemeris_md[i].n_elements = PLATFORM_SIGNAL_COUNT;
    ndb_ephemeris_md[i].file = &ndb_ephe_file;
    ndb_ephemeris_md[i].next = NULL;
    ndb_ephemeris_md[i].update_c = 1;
  }
  memset(ndb_almanac, 0, sizeof(ndb_almanac));
  for (u32 i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_almanac[i].sid = sid_from_global_index(i);
  }

  if (ndb_load_data(&ndb_alma_file, ndb_almanac, ndb_almanac_md,
                    sizeof(almanac_t), PLATFORM_SIGNAL_COUNT) != NDB_ERR_NONE) {
    log_info("No almanac file present in flash, create an empty one");
  } else
    log_info("Almanacs loaded from flash");

  for (i = 0; i < PLATFORM_SIGNAL_COUNT; i++) {
    ndb_almanac_md[i].data = &ndb_almanac[i];
    ndb_almanac_md[i].data_size = sizeof(almanac_t);
    ndb_almanac_md[i].index = i;
    ndb_almanac_md[i].n_elements = PLATFORM_SIGNAL_COUNT;
    ndb_almanac_md[i].file = &ndb_alma_file;
    ndb_almanac_md[i].next = NULL;
    ndb_almanac_md[i].update_c = 1;
  }

  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_ephemeris_read(gnss_signal_t sid, ephemeris_t *e)
{
  u16 idx = sid_to_global_index(sid);
  ndb_retrieve(e, &ndb_ephemeris[idx], sizeof(ephemeris_t));
  return NDB_ERR_NONE;
}

ephemeris_t* ndb_ephemeris_get(gnss_signal_t sid)
{
  assert(sid_supported(sid));
  u16 idx = sid_to_global_index(sid);
  return &ndb_ephemeris[idx];
}


enum ndb_op_code ndb_update_cache_ephemeris(ephemeris_t *cached_e,
                                            ndb_update_counter_t *uc)
{
  u16 idx = sid_to_global_index(cached_e->sid);
  ndb_element_metadata_t *md = &ndb_ephemeris_md[idx];

  if (md->update_c == *uc) {
    return NDB_ERR_NONE;
  }

  enum ndb_op_code r = ndb_ephemeris_read(cached_e->sid, cached_e);
  if (NDB_ERR_NONE != r) {
    return r;
  }

  *uc = md->update_c;
  return NDB_ERR_NONE;
}

s16 ndb_ephe_find_candidate(const ephemeris_t *new)
{
  int i;
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    if (ephe_candidates[i].used &&
        sid_is_equal(ephe_candidates[i].ephe.sid, new->sid)) {
      log_info("Candidate found, slot %d", i);
      return i;
    }
  }
  log_info("Candidate not found");
  return -1;
}

void ndb_ephe_try_adding_candidate(const ephemeris_t *new)
{
  int i;
  u32 candidate_age;
  ndb_timestamp_t now  = ndb_get_timestamp();
  for (i = 0; i < EPHE_CAND_LIST_LEN; i++) {
    bool empty = true;
    if(ephe_candidates[i].used) {
      candidate_age = ST2S(now - ephe_candidates[i].received_at);
      empty = candidate_age >  MAX_EPHE_CANDIDATE_AGE;
    }

    if (empty) {
      memcpy(&ephe_candidates[i].ephe, new, sizeof(ephemeris_t));
      ephe_candidates[i].received_at = ndb_get_timestamp();
      ephe_candidates[i].used = true;
      log_info("Added candidate, slot %d", i);
      return;
    }
  }
  log_info("No empty slot");
}

void ndb_ephe_release_candidate(s16 cand_index)
{
  ephe_candidates[cand_index].used = false;
  log_info("Released slot %d", cand_index);
}

bool ndb_check_candidate(const ephemeris_t *new, s16 idx) {
  if(memcmp(&ephe_candidates[idx].ephe, new, sizeof(ephemeris_t)) == 0) {
    ndb_ephe_release_candidate(idx);
    return true;
  }
  ndb_ephe_release_candidate(idx);
  return false;
}

ndb_ephemeris_status_t ndb_is_ephemeris_reliable(const ephemeris_t *new)
{
  char buf[SID_STR_LEN_MAX];
  ndb_ephemeris_status_t r;
  s16 cand_index;
  ephemeris_t existing;
  ndb_ephemeris_read(new->sid, &existing);

  sid_to_string(buf, sizeof(buf), new->sid);

  chMtxLock(&cand_list_access);

  if (!existing.valid || !existing.healthy) {
    if((cand_index = ndb_ephe_find_candidate(new)) == -1) {
      log_info("First reception of ephe for %s", buf);
      if (!take_first_ephe) {
        r = EPHE_NEW_CANDIDATE;
        ndb_ephe_try_adding_candidate(new);
      } else {
        log_info("Taking in without verification");
        r = EPHE_NEW_TRUSTED;
      }
    } else {
      if(ndb_check_candidate(new, cand_index)) {
        r = EPHE_NEW_TRUSTED;
        log_info("Candidate trusted for %s", buf);
      } else {
        r = EPHE_CAND_MISMATCH;
        log_info("Candidate not trusted for %s", buf);
      }
    }
  } else if (memcmp(&existing, new, sizeof(ephemeris_t)) == 0) {
    /* New one is identical to the one in DB, no need to do anything */
    r = EPHE_IDENTICAL;
    log_info("Identical ephe for %s", buf);
  } else if ((cand_index = ndb_ephe_find_candidate(new)) != -1) {
    if((r = ndb_check_candidate(new, cand_index)) == true) {
      r = EPHE_NEW_TRUSTED;
      log_info("Candidate trusted for %s", buf);
    } else {
      r = EPHE_CAND_MISMATCH;
      log_info("Candidate not trusted for %s", buf);
    }
  } else {
    /* New one is not in candidate list yet, try to put it
     * to an empty slot if any available*/
    r = EPHE_NEW_CANDIDATE;
    log_info("Non trusted candidate for %s", buf);
    ndb_ephe_try_adding_candidate(new);
  }

  chMtxUnlock(&cand_list_access);

  return r;
}

enum ndb_op_code ndb_ephemeris_store(ephemeris_t *e, enum ndb_data_source src)
{
  if (!e->valid) {
    return NDB_ERR_BAD_PARAM;
  }

  u16 idx = sid_to_global_index(e->sid);

  switch (ndb_is_ephemeris_reliable(e)) {
    case EPHE_IDENTICAL:
      return NDB_ERR_NONE;
    case EPHE_NEW_TRUSTED:
      return ndb_update(&ndb_ephemeris[idx], e, sizeof(ephemeris_t),
                        src,
                        &ndb_ephemeris_md[idx], 0xff, NDB_IE_VALID);
    case EPHE_NEW_CANDIDATE:
    case EPHE_CAND_MISMATCH:
      return NDB_ERR_UNRELIABLE_DATA;
  }
  return NDB_ERR_ALGORITHM_ERROR;
}

enum ndb_op_code ndb_ephemeris_info(gnss_signal_t sid, u8* v, u8* h,
                                    gps_time_t* toe, u8* fit_interval)
{
  u16 idx = sid_to_global_index(sid);
  ndb_lock();
  *v = ndb_ephemeris[idx].valid;
  *h = ndb_ephemeris[idx].healthy;
  if (NULL != toe) {
    *toe = ndb_ephemeris[idx].toe;
  }
  if (NULL != fit_interval) {
    *fit_interval = ndb_ephemeris[idx].fit_interval;
  }
  ndb_unlock();
  return NDB_ERR_NONE;
}
enum ndb_op_code ndb_almanac_read(gnss_signal_t sid, almanac_t *a)
{
  u16 idx = sid_to_global_index(sid);
  ndb_retrieve(a, &ndb_almanac[idx], sizeof(almanac_t));

  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_almanac_store(almanac_t *a, enum ndb_data_source src)
{
  u16 idx = sid_to_global_index(a->sid);
  return ndb_update(&ndb_almanac[idx], a, sizeof(almanac_t),
                    src,
                    &ndb_almanac_md[idx], 0xff, NDB_IE_VALID);
}

enum ndb_op_code ndb_update_cache_almanac(almanac_t *cached_a,
                                          ndb_update_counter_t *uc)
{
  u16 idx = sid_to_global_index(cached_a->sid);
  ndb_element_metadata_t *md = &ndb_almanac_md[idx];

  if (md->update_c == *uc) {
    return NDB_ERR_NONE;
  }

  enum ndb_op_code r = ndb_almanac_read(cached_a->sid, cached_a);
  if (NDB_ERR_NONE != r) {
    return r;
  }

  *uc = md->update_c;

  return NDB_ERR_NONE;
}

/** The function sends ephemeris if valid
 *  Function called every NV_WRITE_REQ_TIMEOUT ms from NDB thread*/
void ndb_p1_sbp_update()
{
  static u32 count = 0;
  static u32 i = 0;
  static bool tx_en = true; /* initially enable SBP TX */

  if (tx_en) {
    if (!(count % EPHEMERIS_MESSAGE_SPACING_cycle)) {
      /* every 200 ms send eph of a SV */
      ephemeris_t e;
      gps_time_t t = get_current_time();
      gnss_signal_t sid = sid_from_global_index(i);
      ndb_ephemeris_read(sid, &e);
      if (ephemeris_valid(&e, &t)) {
        msg_ephemeris_t msg;
        pack_ephemeris(&e, &msg);
        sbp_send_msg(SBP_MSG_EPHEMERIS, sizeof(msg_ephemeris_t), (u8 *) &msg);
      }
      i++;
      if (i == PLATFORM_SIGNAL_COUNT) {
        /* no eph to send */
        i = 0;
        tx_en = false;
      }
    }
  } else {
    if (!(count % EPHEMERIS_TRANSMIT_EPOCH_SPACING_cycle)) {
      /* every 15 sec enable tx again */
      count = 0;
      tx_en = true;
      return;
    }
  }

  count++;
}
