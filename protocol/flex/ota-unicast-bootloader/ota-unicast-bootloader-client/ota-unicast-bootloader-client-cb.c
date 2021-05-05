/***************************************************************************//**
 * @brief Weakly defined callbacks for ota unicast bootloader clients.
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
#include "ota-unicast-bootloader-client.h"

//------------------------------------------------------------------------------
// Weak callbacks definitions

WEAK(bool emberAfPluginOtaUnicastBootloaderClientNewIncomingImageCallback(
       EmberNodeId serverId,
       uint8_t imageTag,
       uint32_t imageSize,
       uint32_t *startIndex
       ))
{
  (void)serverId;
  (void)imageTag;
  (void)imageSize;
  (void)startIndex;

  return false;
}

WEAK(void emberAfPluginOtaUnicastBootloaderClientIncomingImageSegmentCallback(
       EmberNodeId serverId,
       uint32_t startIndex,
       uint32_t endIndex,
       uint8_t imageTag,
       uint8_t *imageSegment))
{
  (void)serverId;
  (void)startIndex;
  (void)endIndex;
  (void)imageTag;
  (void)imageSegment;
}

WEAK(void emberAfPluginOtaUnicastBootloaderClientImageDownloadCompleteCallback(
       EmberAfOtaUnicastBootloaderStatus status,
       uint8_t imageTag,
       uint32_t imageSize))
{
  (void)status;
  (void)imageTag;
  (void)imageSize;
}

WEAK(bool emberAfPluginOtaUnicastBootloaderClientIncomingRequestBootloadCallback(
       EmberNodeId serverId,
       uint8_t imageTag,
       uint32_t bootloadDelayMs))
{
  (void)serverId;
  (void)imageTag;
  (void)bootloadDelayMs;

  return false;
}
