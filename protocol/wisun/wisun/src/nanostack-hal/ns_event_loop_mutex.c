/*
 * Copyright (c) 2018, Arm Limited and affiliates.
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

#include <assert.h>
#include <cmsis_os2.h>
#include "sl_cmsis_os2_common.h"
#include "sli_wisun_trace.h"
#include "eventOS_scheduler.h"
#include "ns_event_loop_mutex.h"

#define TRACE_GROUP "evlm"

static osMutexId_t sli_wisun_event_task_mutex;
static osThreadId_t sli_wisun_event_task_owner = NULL;
static uint32_t owner_count = 0;

void eventOS_scheduler_mutex_wait(void)
{
  assert(osMutexAcquire(sli_wisun_event_task_mutex, osWaitForever) == osOK);

  if (0 == owner_count) {
    sli_wisun_event_task_owner = osThreadGetId();
  }

  owner_count++;
}

void eventOS_scheduler_mutex_release(void)
{
  owner_count--;
  if (0 == owner_count) {
    sli_wisun_event_task_owner = NULL;
  }

  assert(osMutexRelease(sli_wisun_event_task_mutex) == osOK);
}

uint8_t eventOS_scheduler_mutex_is_owner(void)
{
  return (osThreadGetId() == sli_wisun_event_task_owner) ? 1 : 0;
}

void ns_event_loop_mutex_init(void)
{
  const osMutexAttr_t sli_wisun_event_task_mutex_attr = {
    "Wi-SUN Event Task Mutex",
    osMutexRecursive,
    NULL,
    0
  };

  sli_wisun_event_task_mutex = osMutexNew(&sli_wisun_event_task_mutex_attr);
  assert(sli_wisun_event_task_mutex != NULL);
}
