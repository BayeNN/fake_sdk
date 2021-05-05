/***************************************************************************//**
 * @file sli_wisun_trace.h
 * @brief Wi-SUN trace API
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

#ifndef SLI_WISUN_TRACE_H
#define SLI_WISUN_TRACE_H

// Declaration of TRACE_LEVEL_INFO in Micrium conflicts with the declaration
// in mbed_trace.
#ifdef TRACE_LEVEL_INFO
#undef TRACE_LEVEL_INFO
#endif

#include "mbed-trace/mbed_trace.h"

#endif  // SLI_WISUN_TRACE_H
