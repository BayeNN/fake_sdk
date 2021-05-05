/***************************************************************************//**
 * @file
 * @brief debug print API definition.
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
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

#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#ifdef EMBER_TEST
#include "printf-sim.h"
#define local_printf printf_sim
#else
#include "printf.h"
#define local_printf printf
#endif

#include "debug_print_config.h"

// TODO: add doxygen

#if ((CONNECT_DEBUG_PRINTS_ENABLED == 1) && (CONNECT_DEBUG_STACK_GROUP_ENABLED == 1))
#define connect_stack_debug_print(...) local_printf(__VA_ARGS__)
#else
#define connect_stack_debug_print(...)
#endif

#if ((CONNECT_DEBUG_PRINTS_ENABLED == 1) && (CONNECT_DEBUG_CORE_GROUP_ENABLED == 1))
#define connect_core_debug_print(...) local_printf(__VA_ARGS__)
#else
#define connect_core_debug_print(...)
#endif

#if ((CONNECT_DEBUG_PRINTS_ENABLED == 1) && (CONNECT_DEBUG_APP_GROUP_ENABLED == 1))
#define connect_app_debug_print(...) local_printf(__VA_ARGS__)
#else
#define connect_app_debug_print(...)
#endif

#endif // DEBUG_PRINT_H
