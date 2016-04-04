/*
 * Copyright (C) 2011-2015 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *          Gareth McMullin <gareth@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <string.h>
#include <libswiftnav/ephemeris.h>
#include <libswiftnav/logging.h>
#include <ch.h>
#include <assert.h>

#include "sbp.h"
#include "sbp_utils.h"
#include "track.h"
#include "timing.h"
#include "ephemeris.h"
#include "signal.h"
#include "ndb.h"

void ephemeris_new(ephemeris_t *e)
{
  assert(sid_supported(e->sid));

  char buf[SID_STR_LEN_MAX];
  sid_to_string(buf, sizeof(buf), e->sid);
  if (!e->valid) {
    log_error("Invalid ephemeris for %s", buf);
    return;
  }

  enum ndb_op_code oc = ndb_ephemeris_store(e, NDB_DS_RECEIVER);
  if (NDB_ERR_NONE != oc) {
    if(NDB_ERR_UNRELIABLE_DATA == oc)
      log_info("Ephemeris for %s is unreliable, not saved", buf);
    else
      log_error("Error storing ephemeris for %s", buf);
  } else {
    log_info("Ephemeris for %s saved", buf);
  }
}

static void ephemeris_msg_callback(u16 sender_id, u8 len, u8 msg[], void* context)
{
  (void)sender_id; (void)context;

  if (len != sizeof(msg_ephemeris_t)) {
    log_warn("Received bad ephemeris from peer");
    return;
  }

  ephemeris_t e;
  unpack_ephemeris((msg_ephemeris_t *)msg, &e);
  if (!sid_supported(e.sid)) {
    log_warn("Ignoring ephemeris for invalid sat");
    return;
  }
  u8 v, h;
  gps_time_t toe;
  ndb_ephemeris_info(e.sid, &v, &h, &toe, NULL);
  if (!v || gpsdifftime(&e.toe, &toe) > 0) {
  /* If local ephemeris is not valid or received one is newer then
   * save the received one. */
    log_info("Saving ephemeris received over SBP v:%d [%d,%d] vs [%d,%d]",
             (int)v, toe.wn, toe.tow, e.toe.wn, e.toe.tow);
    ndb_ephemeris_store(&e, NDB_DS_SBP);
  }
}

void ephemeris_setup(void)
{
  static sbp_msg_callbacks_node_t ephemeris_msg_node;
  sbp_register_cbk(
    SBP_MSG_EPHEMERIS,
    &ephemeris_msg_callback,
    &ephemeris_msg_node
  );
}
