/***************************************************************************//**
 * @brief Connect OTA Broadcast Bootloader Client component configuration header
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

// <h>Connect OTA Broadcast Bootloader Client configuration

// <o EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_CLIENT_ENDPOINT> OTA Broadcast Endpoint <0-15>
// <i> Default: 14
// <i> The endpoint used by the ota broadcast bootloader server/client nodes to exchange ota broadcast bootloader-related messages.
#define EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_CLIENT_ENDPOINT                  (14)

// <q EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_CLIENT_SECURITY_ENABLED> Use security
// <i> Default: 1
// <i> If this option is enabled, the client will only accept encrypted image segments and other commands. It will also encrypt all the commands sent to the server.
#define EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_CLIENT_SECURITY_ENABLED          (1)

// <o EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_CLIENT_IMAGE_DOWNLOAD_TIMEOUT_S> Download Timeout in seconds<5-10000>
// <i> Default: 100
// <i> The time in seconds after which the client shall fail an ongoing image download process in case no message is received from the server(s).
#define EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_CLIENT_IMAGE_DOWNLOAD_TIMEOUT_S  (100)

// </h>

// <<< end of configuration section >>>
