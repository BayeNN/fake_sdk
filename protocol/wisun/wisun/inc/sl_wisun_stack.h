/***************************************************************************//**
 * @file sl_wisun_stack.h
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

#ifndef SL_WISUN_STACK_H
#define SL_WISUN_STACK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************//**
 * Initialize Wi-SUN stack
 *****************************************************************************/
void sl_wisun_stack_init();

/**************************************************************************//**
 * Poll Wi-SUN stack for pending events.
 *
 * If there are none, the call will return immediately. Otherwise, a single
 * event is delivered via the callback handler sl_wisun_on_event().
 *****************************************************************************/
void sl_wisun_stack_step();

#ifdef __cplusplus
}
#endif

#endif  // SL_WISUN_STACK_H
