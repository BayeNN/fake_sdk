/***************************************************************************//**
 * @file sli_wisun_nvm.h
 * @brief Wi-SUN NVM API
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

#ifndef SLI_WISUN_NVM_H
#define SLI_WISUN_NVM_H

#include <stdio.h>
#include <stdint.h>
#include "sl_status.h"

/**************************************************************************//**
 * @brief Initialize Wi-SUN credential storage.
 *****************************************************************************/
void sli_wisun_nvm_init();

/**************************************************************************//**
 * @brief Clear Wi-SUN credential storage.
 *****************************************************************************/
void sli_wisun_nvm_clear();

#endif  // SLI_WISUN_NVM_H
