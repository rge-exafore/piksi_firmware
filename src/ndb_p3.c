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
#include <libswiftnav/logging.h>
#include "ndb.h"

#define GPS_L2C_CAPB_FILE_NAME "l2capb"
static u32 gps_l2c_capabilities _CCM;
static ndb_element_metadata_t gps_l2c_capabilities_md _CCM;

static ndb_file_t gps_l2c_capb_file = {
  .name = GPS_L2C_CAPB_FILE_NAME,
  .fh = -1,
  .expected_size = sizeof(ndb_file_version) +
  sizeof(gps_l2c_capabilities) +
  sizeof(ndb_element_metadata_nv_t)
};

enum ndb_op_code ndb_p3_init()
{
  if (ndb_load_data(&gps_l2c_capb_file, &gps_l2c_capabilities,
      &gps_l2c_capabilities_md,
      sizeof(gps_l2c_capabilities), 1) != NDB_ERR_NONE) {
    log_info("No L2C capability file present in flash, create an empty one");

    u32 new_val = 0xffffffff;
    ndb_update(&gps_l2c_capabilities,
            &new_val,
            sizeof(gps_l2c_capabilities),
            NDB_DS_INIT,
            &gps_l2c_capabilities_md, 0xff, NDB_IE_VALID);
    log_info("Save default L2C capability 0x%x", new_val);
  }
  else
    log_info("Loaded last L2C capability from file: 0x%x",
              gps_l2c_capabilities);

  gps_l2c_capabilities_md.data = &gps_l2c_capabilities;
  gps_l2c_capabilities_md.data_size = sizeof(gps_l2c_capabilities);
  gps_l2c_capabilities_md.index = 0;
  gps_l2c_capabilities_md.n_elements = 1;
  gps_l2c_capabilities_md.file = &gps_l2c_capb_file;
  gps_l2c_capabilities_md.next = NULL;
  gps_l2c_capabilities_md.update_c = 1;

  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_gps_l2cm_l2c_cap_read(u32 *l2c_cap)
{
  ndb_retrieve(l2c_cap, &gps_l2c_capabilities, sizeof(gps_l2c_capabilities));
  return NDB_ERR_NONE;
}

enum ndb_op_code ndb_gps_l2cm_l2c_cap_store(u32 *l2c_cap,
                                            enum ndb_data_source src)
{
  return ndb_update(&gps_l2c_capabilities, l2c_cap,
      sizeof(gps_l2c_capabilities), src, &gps_l2c_capabilities_md,
      0xff, NDB_IE_VALID);
}

void ndb_p3_sbp_update()
{
}
