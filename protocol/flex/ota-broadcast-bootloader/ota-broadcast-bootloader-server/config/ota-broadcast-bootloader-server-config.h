/***************************************************************************//**
 * @brief Connect OTA Broadcast Bootloader Server component configuration header
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

// <h>Connect OTA Broadcast Bootloader Server configuration

// <o EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_SERVER_ENDPOINT> OTA Broadcast Endpoint <0-15>
// <i> Default: 14
// <i> The endpoint used by the ota broadcast bootloader server/client nodes to exchange ota broadcast bootloader-related messages.
#define EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_SERVER_ENDPOINT            (14)

// <q EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_SERVER_SECURITY_ENABLED> Use security
// <i> Default: 1
// <i> If this option is enabled, the server will encrypt image segments and other related commands. It will also drop all non-encrypted client responses.
#define EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_SERVER_SECURITY_ENABLED     (1)

// <o EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_SERVER_TX_INTERVAL_MS> Transmission interval in milliseconds<25-1000>
// <i> Default: 100
// <i> The ota broadcast bootloader server tranmission interval in milliseconds.
#define EMBER_AF_PLUGIN_OTA_BROADCAST_BOOTLOADER_SERVER_TX_INTERVAL_MS     (100)

// </h>

// <<< end of configuration section >>>
