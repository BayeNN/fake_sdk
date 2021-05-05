/***************************************************************************//**
 * @brief Connect Stack callback stubs.
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

#include "stack/include/ember.h"

//------------------------------------------------------------------------------
// Stack handlers weak definitions.

WEAK(void emberStackStatusHandler(EmberStatus status))
{
  (void)status;
}

WEAK(void emberChildJoinHandler(EmberNodeType nodeType, EmberNodeId nodeId))
{
  (void) nodeType;
  (void) nodeId;
}

WEAK(void emberRadioNeedsCalibratingHandler(void))
{
}

WEAK(void emberStackIsrHandler(void))
{
}

WEAK(bool emberStackIdleHandler(uint32_t *idleTimeMs))
{
  (void)idleTimeMs;

  return true;
}

WEAK(void emberMessageSentHandler(EmberStatus status,
                                  EmberOutgoingMessage *message))
{
  (void)status;
  (void)message;
}

WEAK(void emberIncomingMessageHandler(EmberIncomingMessage *message))
{
  (void)message;
}

WEAK(void emberMacMessageSentHandler(EmberStatus status,
                                     EmberOutgoingMacMessage *message))
{
  (void)status;
  (void)message;
}

WEAK(void emberIncomingMacMessageHandler(EmberIncomingMacMessage *message))
{
  (void)message;
}

WEAK(void emberIncomingBeaconHandler(EmberPanId panId,
                                     EmberMacAddress *source,
                                     int8_t rssi,
                                     bool permitJoining,
                                     uint8_t beaconFieldsLength,
                                     uint8_t *beaconFields,
                                     uint8_t beaconPayloadLength,
                                     uint8_t *beaconPayload))
{
  (void)panId;
  (void)source;
  (void)rssi;
  (void)permitJoining;
  (void)beaconFieldsLength;
  (void)beaconFields;
  (void)beaconPayloadLength;
  (void)beaconPayload;
}

WEAK(void emberActiveScanCompleteHandler(void))
{
}

WEAK(void emberEnergyScanCompleteHandler(int8_t mean,
                                         int8_t min,
                                         int8_t max,
                                         uint16_t variance))
{
  (void)mean;
  (void)min;
  (void)max;
  (void)variance;
}

WEAK(void emberFrequencyHoppingStartClientCompleteHandler(EmberStatus status))
{
  (void)status;
}

WEAK(void emberMarkApplicationBuffersHandler(void))
{
}
