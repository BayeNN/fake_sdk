/***************************************************************************//**
 * @file sli_timer.c
 * @brief Wi-SUN timer API
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <assert.h>
#include <cmsis_os2.h>
#include "sl_cmsis_os2_common.h"
#include "rail.h"
#include "sli_wisun_trace.h"
#include "sli_wisun_timer.h"
#include "sl_wisun_common.h"
#define TRACE_GROUP "WISUNTMR"
#define SLI_WISUN_TIMER_TASK_PRIORITY (osPriority_t)45
#define SLI_WISUN_TIMER_TASK_STACK_SIZE 500 // in units of CPU_INT32U
#define SLI_WISUN_TIMER_TASK_FLAG_NONE (0)
#define SLI_WISUN_TIMER_TASK_FLAG_ALL  (255)
#define SLI_WISUN_TIMER_MAX_COUNT 4

typedef struct {
  RAIL_MultiTimer_t timer;
  RAIL_Time_t stop_time;
  sli_wisun_timer_cb cb;
  uint8_t reserved;
} sli_wisun_timer_entry;

static osThreadId_t sli_wisun_timer_task_id;
__ALIGNED(8) static uint8_t sli_wisun_timer_task_tcb[osThreadCbSize];
__ALIGNED(8) static uint8_t sli_wisun_timer_task_stack[(SLI_WISUN_TIMER_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];

osEventFlagsId_t sli_wisun_timer_task_flags;
__ALIGNED(8) static uint8_t sli_wisun_timer_task_flags_cb[osEventFlagsCbSize];

static sli_wisun_timer_entry sli_wisun_timer_array[SLI_WISUN_TIMER_MAX_COUNT];
static void sli_on_wisun_timer_timeout(struct RAIL_MultiTimer *tmr,
                                       RAIL_Time_t expectedTimeOfEvent,
                                       void *cbArg)
{
  (void)tmr;
  (void)expectedTimeOfEvent;

  uint32_t index = (uint32_t)cbArg;

  // Signal the timer task
  assert((osEventFlagsSet(sli_wisun_timer_task_flags,
                          (1 << index)) & CMSIS_RTOS_ERROR_MASK) == 0);
}

void sli_wisun_timer_init(void)
{
  // RAIL multimer is enabled by sli_wisun_driver_init()

  const osEventFlagsAttr_t sli_wisun_timer_task_flags_attr = {
    "Wi-SUN Timer Task Flags",
    0,
    &sli_wisun_timer_task_flags_cb[0],
    osEventFlagsCbSize
  };

  sli_wisun_timer_task_flags = osEventFlagsNew(&sli_wisun_timer_task_flags_attr);
  assert(sli_wisun_timer_task_flags != NULL);

  osThreadAttr_t sli_wisun_timer_task_attribute = {
    "Wi-SUN Timer Task",
    osThreadDetached,
    &sli_wisun_timer_task_tcb[0],
    osThreadCbSize,
    &sli_wisun_timer_task_stack[0],
    (SLI_WISUN_TIMER_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u,
    SLI_WISUN_TIMER_TASK_PRIORITY,
    0,
    0
    };

  sli_wisun_timer_task_id = osThreadNew(sli_wisun_timer_task,
                                        NULL,
                                        &sli_wisun_timer_task_attribute);
  assert(sli_wisun_timer_task_id != 0);
}

sl_status_t sli_wisun_timer_start(uint32_t *index, uint32_t timeout, sli_wisun_timer_cb cb)
{
  uint32_t i;
  RAIL_Status_t ret;

  for (i = 0; i < SLI_WISUN_TIMER_MAX_COUNT; ++i) {
    if (!sli_wisun_timer_array[i].reserved) {
      sli_wisun_timer_array[i].cb = cb;
      *index = i;

      ret = RAIL_SetMultiTimer(&sli_wisun_timer_array[i].timer,
                               timeout,
                               RAIL_TIME_DELAY,
                               sli_on_wisun_timer_timeout,
                               (void *)i);
      if (ret == RAIL_STATUS_NO_ERROR) {
        sli_wisun_timer_array[i].reserved = true;
        sli_wisun_timer_array[i].stop_time = RAIL_GetTime() + timeout;

        return SL_STATUS_OK;
      } else {
        tr_error("RAIL_SetMultiTimer: %d", ret);
        return SL_STATUS_FAIL;
      }
    }
  }

  return SL_STATUS_NO_MORE_RESOURCE;
}

sl_status_t sli_wisun_timer_stop(uint32_t index)
{
  if (sli_wisun_timer_array[index].reserved) {
    RAIL_CancelMultiTimer(&sli_wisun_timer_array[index].timer);
    sli_wisun_timer_array[index].reserved = false;
  }

  return SL_STATUS_OK;
}

sl_status_t sli_wisun_timer_get_time_left(uint32_t index, uint32_t *time_left)
{
  if (sli_wisun_timer_array[index].reserved) {
    *time_left = RAIL_GetMultiTimer(&sli_wisun_timer_array[index].timer, RAIL_TIME_DELAY);
  } else {
    *time_left = 0;
  }

  return SL_STATUS_OK;
}

sl_status_t sli_wisun_timer_get_timestamp(uint32_t *timestamp)
{
  *timestamp = RAIL_GetTime();

  return SL_STATUS_OK;
}

void sli_wisun_timer_task(void *argument)
{
  uint32_t flags;
  uint32_t i;
  (void)argument;
  SLI_WISUN_TASK_LOOP {
    // Wait for something to happen
    flags = osEventFlagsWait(sli_wisun_timer_task_flags,
                             SLI_WISUN_TIMER_TASK_FLAG_ALL,
                             osFlagsWaitAny,
                             osWaitForever);
    assert((flags & CMSIS_RTOS_ERROR_MASK) == 0);

    // Execute the timer callbacks
    for (i = 0; i < SLI_WISUN_TIMER_MAX_COUNT; ++i) {
      if (flags & (1 << i)) {
        sli_wisun_timer_array[i].reserved = false;
        sli_wisun_timer_array[i].cb(i);
      }
    }
  }
}
