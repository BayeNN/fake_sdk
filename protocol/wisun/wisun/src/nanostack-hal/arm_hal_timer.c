/***************************************************************************//**
 * @file arm_hal_timer.c
 * @brief Wi-SUN platform timer
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

#include "sli_wisun_timer.h"
#include "sli_wisun_trace.h"
#include "arm_hal_timer.h"
#define TRACE_GROUP "HALTMR"

// Wi-SUN timer index
static uint32_t timer_index;
static bool timer_active;

// Timer callback
static void (*arm_hal_callback)(void);

// The amount of microseconds per timer slot
static const uint32_t platform_timer_slot_in_us = 50;

static void on_platform_timer_cb(uint32_t index)
{
  (void)index;

  // Timer is finished
  timer_active = false;

  // Execute the timer callback
  if (arm_hal_callback) {
    arm_hal_callback();
  }
}

void platform_timer_enable(void)
{
  sli_wisun_timer_init();
}

void platform_timer_set_cb(platform_timer_cb new_fp)
{
  arm_hal_callback = new_fp;
}

void platform_timer_disable(void)
{
  if (timer_active) {
    sli_wisun_timer_stop(timer_index);
    timer_active = false;
  }
}

// This is called from inside platform_enter_critical - IRQs can't happen
void platform_timer_start(uint16_t slots)
{
  sl_status_t ret;
  uint32_t due = platform_timer_slot_in_us * slots;

  if (timer_active) {
    sli_wisun_timer_stop(timer_index);
    timer_active = false;
  }
  ret = sli_wisun_timer_start(&timer_index, due, on_platform_timer_cb);
  if (ret == SL_STATUS_OK) {
    timer_active = true;
  } else {
    tr_error("sli_wisun_timer_start(%lu): %lu", due, ret);
  }
}

// This is called from inside platform_enter_critical - IRQs can't happen
uint16_t platform_timer_get_remaining_slots(void)
{
  sl_status_t ret;
  uint32_t left = 0;

  if (!timer_active) {
    tr_info("platform_timer_get_remaining_slots: timer not active");
    return 0;
  }

  ret = sli_wisun_timer_get_time_left(timer_index, &left);
  if (ret != SL_STATUS_OK) {
    tr_error("sli_wisun_timer_get_time_left(): %lu", ret);
    return 0;
  }

  return (uint16_t)(left / platform_timer_slot_in_us);
}
