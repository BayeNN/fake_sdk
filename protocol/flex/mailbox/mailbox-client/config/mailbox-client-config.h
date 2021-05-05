/***************************************************************************//**
 * @brief Connect Mailbox Client component configuration header.
 *
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
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

// <<< Use Configuration Wizard in Context Menu >>>

// <h>Connect Mailbox Client configuration

// <o EMBER_AF_PLUGIN_MAILBOX_CLIENT_MAILBOX_ENDPOINT> The mailbox protocol endpoint <0-15>
// <i> Default: 15
// <i> The endpoint used by the mailbox server/client nodes to exchange mailbox-related messages.
#define EMBER_AF_PLUGIN_MAILBOX_CLIENT_MAILBOX_ENDPOINT         (15)

// <o EMBER_MAILBOX_CLIENT_MESSAGE_TIMEOUT_MS> Handshake timeout in milliseconds<200-1000>
// <i> Default: 250
// <i> The maximum amount of time in milliseconds that the client waits for a response from the mailbox server. In case this plugin is used on a star topology sleepy end device, this parameter should be configured accordingly to the short poll parameter in the poll plugin.
#define EMBER_MAILBOX_CLIENT_MESSAGE_TIMEOUT_MS                 (250)

// </h>

// <<< end of configuration section >>>
