/***************************************************************************//**
 * @file sl_wisun_util.c
 * @brief Wi-SUN tools
 * @version $Id$ $Format:%ci$ ($Format:%h$)
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include "rail_config.h"
#include "sl_wisun_api.h"
#include "sl_wisun_util.h"
#include "sli_wisun_driver.h"
#include "sli_wisun_trace.h"

#define TRACE_GROUP "util"

#define SLI_WISUN_ERROR_CHECK_SET_STATUS(__result, __value)\
do {\
  if (!(__result)){\
    status = __value;\
    goto error_handler;\
  }\
} while(0)

#define SLI_WISUN_ERROR_SET_STATUS(__value)\
do {\
  status = __value;\
  goto error_handler;\
} while(0)

/** Matches a RAIL configuration with Wi-SUN frequency band settings. */
typedef struct sli_wisun_rf_settings {
  uint32_t channel_0_center_frequency;  ///< Center frequency
  uint32_t channel_spacing;             ///< Channel spacing
  uint32_t datarate;                    ///< Data rate
  uint16_t number_of_channels;          ///< Number of channels
  uint8_t reg_domain;                   ///< Regulatory domain
  uint8_t op_class;                     ///< Operating class
  uint16_t op_mode;                     ///< Operating mode
} sli_wisun_rf_settings_t;

static const sli_wisun_rf_settings_t rf_settings_CN_1_1b = {
  .channel_0_center_frequency=470200000, .channel_spacing=200000, .datarate=50000, .number_of_channels=199, .reg_domain=4, .op_class=1, .op_mode=0x1b
};
static const sli_wisun_rf_settings_t rf_settings_EU_1_1a = {
  .channel_0_center_frequency=863100000, .channel_spacing=100000, .datarate=50000, .number_of_channels=69, .reg_domain=3, .op_class=1, .op_mode=0x1a
};
static const sli_wisun_rf_settings_t rf_settings_IN_1_1a = {
  .channel_0_center_frequency=865100000, .channel_spacing=100000, .datarate=50000, .number_of_channels=19, .reg_domain=5, .op_class=1, .op_mode=0x1a
};
static const sli_wisun_rf_settings_t rf_settings_NA_1_1b = {
  .channel_0_center_frequency=902200000, .channel_spacing=200000, .datarate=50000, .number_of_channels=129, .reg_domain=1, .op_class=1, .op_mode=0x1b
};

/** Supported Wi-SUN frequency band settings. */
static const sli_wisun_rf_settings_t *rf_settings[] = {
  &rf_settings_CN_1_1b,
  &rf_settings_EU_1_1a,
  &rf_settings_IN_1_1a,
  &rf_settings_NA_1_1b,
  NULL
};

sl_status_t sl_wisun_util_get_rf_settings(uint8_t *reg_domain, uint8_t *op_class, uint16_t *op_mode)
{
  sl_status_t status = SL_STATUS_OK;
  RAIL_Handle_t rail_handle;
  const RAIL_ChannelConfig_t *channel_config;
  RAIL_Status_t ret;
  uint32_t datarate;
  uint16_t number_of_channels;
  const sli_wisun_rf_settings_t *iter=NULL;
  int index = 0;

  tr_debug("sl_wisun_util_get_rf_settings()");

  rail_handle = sli_wisun_driver_get_rail_handle();
  SLI_WISUN_ERROR_CHECK_SET_STATUS(rail_handle, SL_STATUS_FAIL);

  /* Take first RAIL configuration as reference. */
  channel_config = channelConfigs[0];
  ret = RAIL_ConfigChannels(rail_handle, channel_config, NULL);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret == RAIL_STATUS_NO_ERROR, SL_STATUS_FAIL);

  datarate = RAIL_GetBitRate(rail_handle);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(datarate, SL_STATUS_FAIL);
  number_of_channels = channel_config->configs->channelNumberEnd - channel_config->configs->channelNumberStart + 1;

  /* Iterate through supported Wi-SUN frequency band settings to find
    * the matching RAIL configuration. */
  iter = rf_settings[index];
  while (iter) {
    if ((channel_config->configs->baseFrequency == iter->channel_0_center_frequency) &&
        (channel_config->configs->channelSpacing == iter->channel_spacing) &&
        (datarate == iter->datarate) &&
        (number_of_channels == iter->number_of_channels)) {
      /* Matching Wi-SUN frequency band settings found. */
      *reg_domain = iter->reg_domain;
      *op_class = iter->op_class;
      *op_mode = iter->op_mode;
      tr_debug("Using reg_domain %u, op_class %u, op_mode %x", *reg_domain, *op_class, *op_mode);
      break;
    }
    iter = rf_settings[++index];
  }

  if (!iter) {
    tr_debug("No matching WI-SUN frequency band settings");
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_FAIL);
  }

error_handler:

  return status;
}

sl_status_t sl_wisun_util_connect(const uint8_t * network_name)
{
  sl_status_t status;
  uint8_t reg_domain;
  uint8_t op_class;
  uint16_t op_mode;

  tr_debug("sl_wisun_util_connect()");

  status = sl_wisun_util_get_rf_settings(&reg_domain, &op_class, &op_mode);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(status == SL_STATUS_OK, SL_STATUS_FAIL);

  status = sl_wisun_connect(network_name, reg_domain, op_class, op_mode);

error_handler:

  return status;
}
