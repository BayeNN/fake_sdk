/***************************************************************************//**
 * @file sli_wisun_task.h
 * @brief Wi-SUN task API
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

#ifndef SLI_WISUN_TASK_H
#define SLI_WISUN_TASK_H

#include "sl_wisun_msg_api.h"

void sli_wisun_task_init();

void sli_wisun_task_req(const void *req, const void *req_data, void *cnf, void *cnf_data);

sl_wisun_evt_t* sli_wisun_task_poll();

void sli_wisun_req_handler(void);
void sli_wisun_task(void *argument);
void sli_wisun_event_task(void *argument);
#endif  // SLI_WISUN_TASK_H
