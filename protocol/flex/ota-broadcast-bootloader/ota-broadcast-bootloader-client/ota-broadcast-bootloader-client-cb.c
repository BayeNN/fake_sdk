/***************************************************************************//**
 * @brief Set of weakly defined callbacks for ota broadcast bootloader client.
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
#include "ota-broadcast-bootloader-client.h"

//------------------------------------------------------------------------------
// Weak callbacks definitions

WEAK(bool emberAfPluginOtaBootloaderClientNewIncomingImageCallback(EmberNodeId serverId,
                                                                   EmberNodeId *alternateServerId,
                                                                   uint8_t imageTag))
{
  (void)serverId;
  (void)alternateServerId;
  (void)imageTag;

  return false;
}

WEAK(void emberAfPluginOtaBootloaderClientIncomingImageSegmentCallback(EmberNodeId serverId,
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

WEAK(void emberAfPluginOtaBootloaderClientImageDownloadCompleteCallback(EmberAfOtaBootloaderStatus status,
                                                                        uint8_t imageTag,
                                                                        uint32_t imageSize))
{
  (void)status;
  (void)imageTag;
  (void)imageSize;
}

WEAK(void emberAfPluginOtaBootloaderClientIncomingRequestStatusCallback(EmberNodeId serverId,
                                                                        uint8_t applicationServerStatus,
                                                                        uint8_t *applicationStatus))
{
  (void)serverId;
  (void)applicationServerStatus;
  (void)applicationStatus;
}

WEAK(bool emberAfPluginOtaBootloaderClientIncomingRequestBootloadCallback(EmberNodeId serverId,
                                                                          uint8_t imageTag,
                                                                          uint32_t bootloadDelayMs,
                                                                          uint8_t *applicationStatus))
{
  (void)serverId;
  (void)imageTag;
  (void)bootloadDelayMs;
  (void)applicationStatus;

  return false;
}
