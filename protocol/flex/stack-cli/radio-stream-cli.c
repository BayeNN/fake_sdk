/***************************************************************************//**
 * @brief Radio Stream CLI commands.
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

#include "stack/include/ember.h"
#include "sl_cli.h"
#include "debug_print.h"
#include "radio-stream.h"

void start_tx_stream_command(sl_cli_command_arg_t *arguments)
{
  uint8_t mode = sl_cli_get_argument_uint8(arguments, 0);
  uint16_t channel = sl_cli_get_argument_uint16(arguments, 1);
  EmberStatus status = emberStartTxStream(mode, channel);

  if (status == EMBER_SUCCESS) {
    connect_app_debug_print("OK\n");
  } else {
    connect_app_debug_print("Failed to start Tx Stream %d\n", status);
  }
}

void stop_tx_stream_command(void)
{
  EmberStatus status = emberStopTxStream();

  if (status == EMBER_SUCCESS) {
    connect_app_debug_print("OK\n");
  } else {
    connect_app_debug_print("Failed to stop Tx Stream %d\n", status);
  }
}
