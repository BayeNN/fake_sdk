/***************************************************************************//**
 * @brief Frequency Hopping commands.
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

void stack_set_fh_channel_mask_command(sl_cli_command_arg_t *arguments)
{
  size_t channelMaskLength;
  uint8_t *channelMask = sl_cli_get_argument_hex(arguments, 0, &channelMaskLength);

  EmberStatus status = emberFrequencyHoppingSetChannelMask((uint8_t)channelMaskLength, channelMask);

  if (status != EMBER_SUCCESS) {
    connect_app_debug_print("FH Channel Mask Failed, 0x%x\n", status);
  } else {
    connect_app_debug_print("FH Channel Mask Success\n");
  }
}

void stack_start_fh_server_command(sl_cli_command_arg_t *arguments)
{
  (void)arguments;
  EmberStatus status = emberFrequencyHoppingStartServer();

  if (status != EMBER_SUCCESS) {
    connect_app_debug_print("FH Server Failed, 0x%x\n", status);
  } else {
    connect_app_debug_print("FH Server Success\n");
  }
}

void stack_start_fh_client_command(sl_cli_command_arg_t *arguments)
{
  EmberStatus status;
  EmberNodeId nodeId;
  EmberPanId panId;

  nodeId = sl_cli_get_argument_uint16(arguments, 0);
  panId = sl_cli_get_argument_uint16(arguments, 1);

  status = emberFrequencyHoppingStartClient(nodeId, panId);

  if (status != EMBER_SUCCESS) {
    connect_app_debug_print("FH Client Failed, 0x%x\n", status);
  } else {
    connect_app_debug_print("FH Client Success\n");
  }
}

void stack_stop_fh_command(sl_cli_command_arg_t *arguments)
{
  (void)arguments;
  EmberStatus status = emberFrequencyHoppingStop();

  if (status != EMBER_SUCCESS) {
    connect_app_debug_print("FH Stop Failed, 0x%x\n", status);
  } else {
    connect_app_debug_print("FH Stopped\n");
  }
}
