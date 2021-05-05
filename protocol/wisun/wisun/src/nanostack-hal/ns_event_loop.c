/*
 * Copyright (c) 2016-2018, Arm Limited and affiliates.
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
#include "ns_event_loop.h"
#include "sl_wisun_common.h"

#define TRACE_GROUP "evlp"
#define SLI_WISUN_EVENT_LOOP_TASK_PRIORITY (osPriority_t)43
#define SLI_WISUN_EVENT_LOOP_TASK_STACK_SIZE 1536 // in units of CPU_INT32U

#define SLI_WISUN_EVENT_LOOP_TASK_FLAG_NONE          (0)
#define SLI_WISUN_EVENT_LOOP_TASK_FLAG_READY         (1)

static osThreadId_t sli_wisun_event_loop_task_id;
__ALIGNED(8) static uint8_t sli_wisun_event_loop_task_tcb[osThreadCbSize];
__ALIGNED(8) static uint8_t sli_wisun_event_loop_task_stack[(SLI_WISUN_EVENT_LOOP_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];

osEventFlagsId_t sli_wisun_event_loop_task_flags;
__ALIGNED(8) static uint8_t sli_wisun_event_loop_task_flags_cb[osEventFlagsCbSize];

void eventOS_scheduler_signal(void)
{
  assert((osEventFlagsSet(sli_wisun_event_loop_task_flags,
                          SLI_WISUN_EVENT_LOOP_TASK_FLAG_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

void eventOS_scheduler_idle(void)
{
  eventOS_scheduler_mutex_release();

  assert((osEventFlagsWait(sli_wisun_event_loop_task_flags,
                           SLI_WISUN_EVENT_LOOP_TASK_FLAG_READY,
                           osFlagsWaitAny,
                           osWaitForever) & CMSIS_RTOS_ERROR_MASK) == 0);

  eventOS_scheduler_mutex_wait();
}

static void event_loop_thread(void *arg)
{
    (void)arg;
    eventOS_scheduler_mutex_wait();
    eventOS_scheduler_run(); //Does not return
}

// This is used to initialize the lock used by event loop even
// if it is not ran in a separate thread.
void ns_event_loop_init(void)
{
    ns_event_loop_mutex_init();
}

void ns_event_loop_thread_create(void)
{
  const osEventFlagsAttr_t sli_wisun_event_loop_task_flags_attr = {
    "Wi-SUN Event Loop Task Flags",
    0,
    &sli_wisun_event_loop_task_flags_cb[0],
    osEventFlagsCbSize
  };

  sli_wisun_event_loop_task_flags = osEventFlagsNew(&sli_wisun_event_loop_task_flags_attr);
  assert(sli_wisun_event_loop_task_flags != NULL);

  osThreadAttr_t sli_wisun_event_loop_task_attribute = {
    "Wi-SUN Event Loop Task",
    osThreadDetached,
    &sli_wisun_event_loop_task_tcb[0],
    osThreadCbSize,
    &sli_wisun_event_loop_task_stack[0],
    (SLI_WISUN_EVENT_LOOP_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u,
    SLI_WISUN_EVENT_LOOP_TASK_PRIORITY,
    0,
    0
    };

  sli_wisun_event_loop_task_id = osThreadNew(event_loop_thread,
                                             NULL,
                                             &sli_wisun_event_loop_task_attribute);
  assert(sli_wisun_event_loop_task_id != 0);
}

void ns_event_loop_thread_start(void)
{
}
