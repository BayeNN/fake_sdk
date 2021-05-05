/***************************************************************************//**
 * @brief Connect Mailbox Server component configuration header.
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

// <h>Connect Mailbox Server configuration

// <o EMBER_AF_PLUGIN_MAILBOX_SERVER_MAILBOX_ENDPOINT> The mailbox protocol endpoint <0-15>
// <i> Default: 15
// <i> The endpoint used by the mailbox server/client nodes to exchange mailbox-related messages.
#define EMBER_AF_PLUGIN_MAILBOX_SERVER_MAILBOX_ENDPOINT         (15)

// <o EMBER_AF_PLUGIN_MAILBOX_SERVER_PACKET_TABLE_SIZE> Maximum number of packets <1-254>
// <i> Default: 25
// <i> The maximum number of packets that can be stored at the mailbox server.
#define EMBER_AF_PLUGIN_MAILBOX_SERVER_PACKET_TABLE_SIZE        (25)

// <o EMBER_AF_PLUGIN_MAILBOX_SERVER_PACKET_TIMEOUT_S> Packet timeout in seconds
// <i> Default: 3600
// <i> The time in seconds after which a packet is dropped if not retrieved by its destination.
#define EMBER_AF_PLUGIN_MAILBOX_SERVER_PACKET_TIMEOUT_S         (3600)

// </h>

// <<< end of configuration section >>>
