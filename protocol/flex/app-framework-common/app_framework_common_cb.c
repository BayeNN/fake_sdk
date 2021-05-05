/***************************************************************************//**
 * @brief Connect Application Framework callbacks stubs.
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

#include "app_framework_common.h"

#include "hal.h"

WEAK(bool emberAfCommonOkToEnterLowPowerCallback(bool enter_em2,
                                                 uint32_t duration_ms))
{
  (void)enter_em2;
  (void)duration_ms;

  return true;
}
