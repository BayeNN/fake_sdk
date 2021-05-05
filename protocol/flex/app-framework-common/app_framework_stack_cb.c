/***************************************************************************//**
 * @brief Connect Application Framework common code.
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

#include "callback_dispatcher.h"
#include "app_framework_callback.h"
#include "sl_component_catalog.h"

// If the CMSIS ICP is present, most of the stack callbacks are implemented
// there.
#if !defined(SL_CATALOG_CONNECT_CMSIS_STACK_IPC_PRESENT)

void emberStackStatusHandler(EmberStatus status)
{
  emberAfStackStatus(status);
  emberAfStackStatusCallback(status);
}

void emberChildJoinHandler(EmberNodeType nodeType, EmberNodeId nodeId)
{
  emberAfChildJoin(nodeType, nodeId);
  emberAfChildJoinCallback(nodeType, nodeId);
}

void emberRadioNeedsCalibratingHandler(void)
{
  emberAfRadioNeedsCalibrating();
  emberAfRadioNeedsCalibratingCallback();
}

void emberMessageSentHandler(EmberStatus status,
                             EmberOutgoingMessage *message)
{
  emberAfMessageSent(status, message);
  emberAfMessageSentCallback(status, message);
}

void emberIncomingMessageHandler(EmberIncomingMessage *message)
{
  emberAfIncomingMessage(message);
  emberAfIncomingMessageCallback(message);
}

void emberMacMessageSentHandler(EmberStatus status,
                                EmberOutgoingMacMessage *message)
{
  emberAfMacMessageSent(status, message);
  emberAfMacMessageSentCallback(status, message);
}

void emberIncomingMacMessageHandler(EmberIncomingMacMessage *message)
{
  emberAfIncomingMacMessage(message);
  emberAfIncomingMacMessageCallback(message);
}

void emberIncomingBeaconHandler(EmberPanId panId,
                                EmberMacAddress *source,
                                int8_t rssi,
                                bool permitJoining,
                                uint8_t beaconFieldsLength,
                                uint8_t *beaconFields,
                                uint8_t beaconPayloadLength,
                                uint8_t *beaconPayload)
{
  emberAfIncomingBeacon(panId,
                        source,
                        rssi,
                        permitJoining,
                        beaconFieldsLength,
                        beaconFields,
                        beaconPayloadLength,
                        beaconPayload);
  emberAfIncomingBeaconCallback(panId,
                                source,
                                rssi,
                                permitJoining,
                                beaconFieldsLength,
                                beaconFields,
                                beaconPayloadLength,
                                beaconPayload);
}

void emberActiveScanCompleteHandler(void)
{
  emberAfActiveScanComplete();
  emberAfActiveScanCompleteCallback();
}

void emberEnergyScanCompleteHandler(int8_t mean,
                                    int8_t min,
                                    int8_t max,
                                    uint16_t variance)
{
  emberAfEnergyScanComplete(mean, min, max, variance);
  emberAfEnergyScanCompleteCallback(mean, min, max, variance);
}

void emberFrequencyHoppingStartClientCompleteHandler(EmberStatus status)
{
  emberAfFrequencyHoppingStartClientComplete(status);
  emberAfFrequencyHoppingStartClientCompleteCallback(status);
}

#endif // SL_CATALOG_CONNECT_CMSIS_STACK_IPC_PRESENT

void emberStackIsrHandler(void)
{
  emberAfStackIsr();
  // We do not expose this to the application.
}

void emberMarkApplicationBuffersHandler(void)
{
  emberAfMarkApplicationBuffers();
  emberAfMarkApplicationBuffersCallback();
}
