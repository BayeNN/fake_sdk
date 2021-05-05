/*
 * Copyright (c) 2018-2019, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sli_wisun_timer.h"
#include "ns_types.h"
#include "sli_wisun_trace.h"
#include "platform/arm_hal_interrupt.h"
#include "fhss_config.h"
#include "fhss_api.h"
#include "ns_trace.h"

#define TRACE_GROUP "FHSSTMR"

#ifndef NUMBER_OF_SIMULTANEOUS_TIMEOUTS
#define NUMBER_OF_SIMULTANEOUS_TIMEOUTS  2
#endif //NUMBER_OF_SIMULTANEOUS_TIMEOUTS

static const fhss_api_t *fhss_active_handle = NULL;

typedef struct {
  void (*fhss_timer_callback)(const fhss_api_t *fhss_api, uint16_t);
  uint32_t start_time;
  uint32_t stop_time;
  uint32_t timer_index;
  bool active;
} fhss_timeout_s;

static fhss_timeout_s fhss_timeout[NUMBER_OF_SIMULTANEOUS_TIMEOUTS];

static uint32_t read_current_time(void)
{
  uint32_t timestamp;

  sli_wisun_timer_get_timestamp(&timestamp);

  return timestamp;
}

static fhss_timeout_s *find_timeout(void (*callback)(const fhss_api_t *api, uint16_t))
{
  for (int i = 0; i < NUMBER_OF_SIMULTANEOUS_TIMEOUTS; i++) {
    if (fhss_timeout[i].fhss_timer_callback == callback) {
      return &fhss_timeout[i];
    }
  }
  return NULL;
}

static fhss_timeout_s *allocate_timeout(void)
{
  for (int i = 0; i < NUMBER_OF_SIMULTANEOUS_TIMEOUTS; i++) {
    if (fhss_timeout[i].fhss_timer_callback == NULL) {
      return &fhss_timeout[i];
    }
  }
  return NULL;
}

static void on_fhss_timeout_handler(uint32_t index)
{
  for (int i = 0; i < NUMBER_OF_SIMULTANEOUS_TIMEOUTS; i++) {
    if ((fhss_timeout[i].active) && (fhss_timeout[i].timer_index == index)) {
      fhss_timeout[i].active = false;
      fhss_timeout[i].fhss_timer_callback(fhss_active_handle, read_current_time() - fhss_timeout[i].stop_time);
      return;
    }
  }
}

static int platform_fhss_timer_start(uint32_t slots, void (*callback)(const fhss_api_t *api, uint16_t), const fhss_api_t *callback_param)
{
  int ret_val = -1;
  sl_status_t status;

  platform_enter_critical();

  fhss_timeout_s *fhss_tim = find_timeout(callback);
  if (!fhss_tim) {
    tr_info("FHSS timer allocated for callback %p", callback);
    fhss_tim = allocate_timeout();
  }

  if (!fhss_tim) {
    platform_exit_critical();
    tr_error("Failed to allocate timeout");
    return ret_val;
  }

  if (fhss_tim->active) {
    sli_wisun_timer_stop(fhss_tim->timer_index);
    fhss_tim->active = false;
  }

  fhss_tim->fhss_timer_callback = callback;
  fhss_tim->start_time = read_current_time();
  fhss_tim->stop_time = fhss_tim->start_time + slots;
  status = sli_wisun_timer_start(&fhss_tim->timer_index, slots, on_fhss_timeout_handler);
  if (status == SL_STATUS_OK) {
    fhss_tim->active = true;
    fhss_active_handle = callback_param;
    ret_val = 0;
  } else {
    tr_error("FHSS[%lu] sli_wisun_timer_start(%lu): %lu", fhss_tim->timer_index, slots, status);
  }

  platform_exit_critical();

  return ret_val;
}

static int platform_fhss_timer_stop(void (*callback)(const fhss_api_t *api, uint16_t), const fhss_api_t *api)
{
  (void)api;
  int ret_val = -1;
  sl_status_t status;

  platform_enter_critical();

  fhss_timeout_s *fhss_tim = find_timeout(callback);
  if (!fhss_tim) {
    tr_error("FHSS: unable to find platform FHSS timer");
    platform_exit_critical();
    return -1;
  }

  status = sli_wisun_timer_stop(fhss_tim->timer_index);
  if( status == SL_STATUS_OK) {
    fhss_tim->active = false;
    ret_val = 0;
  } else {
    tr_error("FHSS[%lu]: sli_wisun_timer_stop(): %lu", fhss_tim->timer_index, status);
  }

  platform_exit_critical();

  return ret_val;
}

static uint32_t platform_fhss_get_remaining_slots(void (*callback)(const fhss_api_t *api, uint16_t), const fhss_api_t *api)
{
  uint32_t remaining_slots;
  (void)api;

  platform_enter_critical();

  fhss_timeout_s *fhss_tim = find_timeout(callback);
  if (!fhss_tim) {
    platform_exit_critical();
    return 0;
  }

  sli_wisun_timer_get_time_left(fhss_tim->timer_index, &remaining_slots);

  platform_exit_critical();

  return remaining_slots;
}

static uint32_t platform_fhss_timestamp_read(const fhss_api_t *api)
{
  (void)api;
  return read_current_time();
}

fhss_timer_t fhss_functions = {
  .fhss_timer_start = platform_fhss_timer_start,
  .fhss_timer_stop = platform_fhss_timer_stop,
  .fhss_get_remaining_slots = platform_fhss_get_remaining_slots,
  .fhss_get_timestamp = platform_fhss_timestamp_read,
  .fhss_resolution_divider = 1
};
