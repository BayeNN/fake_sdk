/***************************************************************************//**
 * @file sli_wisun_driver.h
 * @brief Wi-SUN driver API
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

#ifndef SLI_WISUN_DRIVER_H
#define SLI_WISUN_DRIVER_H

#include <stdint.h>
#include "rail.h"

/**************************************************************************//**
 * Initialize Wi-SUN driver
 *****************************************************************************/
void sli_wisun_driver_init(void);

/**************************************************************************//**
 * Register Wi-SUN driver to the network subsystem
 *****************************************************************************/
void sli_wisun_driver_register(int8_t *driver_id);

/**************************************************************************//**
 * Configure Wi-SUN PTI state
 *****************************************************************************/
void sli_wisun_driver_set_pti_state(bool enable);

/**************************************************************************//**
 * Set TX power
 *****************************************************************************/
void sli_wisun_driver_set_tx_power(int8_t tx_power);

/**************************************************************************//**
 * Driver main loop
 *****************************************************************************/
void rf_task_loop(void *arg);

/**************************************************************************//**
 * Get current RAIL handle
 *****************************************************************************/
RAIL_Handle_t sli_wisun_driver_get_rail_handle(void);

#endif  // SLI_WISUN_DRIVER_H
