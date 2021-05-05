/***************************************************************************//**
 * @file sli_timer.h
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

#ifndef SLI_WISUN_TIMER_H
#define SLI_WISUN_TIMER_H

#include <stdint.h>
#include "sl_status.h"

typedef void (*sli_wisun_timer_cb)(uint32_t index);

void sli_wisun_timer_init();

sl_status_t sli_wisun_timer_start(uint32_t *index, uint32_t timeout, sli_wisun_timer_cb cb);

sl_status_t sli_wisun_timer_stop(uint32_t index);

sl_status_t sli_wisun_timer_get_time_left(uint32_t index, uint32_t *time_left);

sl_status_t sli_wisun_timer_get_timestamp(uint32_t *timestamp);

void sli_wisun_timer_task(void *argument);

#endif  // SLI_WISUN_TIMER_H
