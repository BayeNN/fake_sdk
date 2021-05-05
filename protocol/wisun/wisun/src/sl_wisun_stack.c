/***************************************************************************//**
 * @file sl_wisun_stack.c
 * @brief Wi-SUN stack API
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

#include "em_common.h"
#include "sli_wisun_task.h"
#include "sl_wisun_api.h"
#include "sl_wisun_stack.h"

void sl_wisun_stack_init()
{
  sli_wisun_task_init();
}

void sl_wisun_stack_step()
{
  sl_wisun_evt_t* evt = sli_wisun_task_poll();
  if (evt) {
    sl_wisun_on_event(evt);
  }
}
