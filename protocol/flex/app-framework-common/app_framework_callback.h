/***************************************************************************//**
 * @file
 * @brief Connect Application Framework - application callbacks.
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

#ifndef APP_FRAMEWORK_CALLBACK_H
#define APP_FRAMEWORK_CALLBACK_H

/**
 * @addtogroup app_framework_common
 * @brief Application framework common
 *
 * Declares all the required application framework globals, initializes the Connect stack and
 * dispatches stack callbacks calls as needed to the application components.
 *
 * @{
 *
 * @{
 * @name Callbacks
 */

/** @brief Application Framework Initialization Callback
 *
 * A callback invoked once during the initialization. It is called after the stack and plugins
 * initialization.
 *
 */
void emberAfInitCallback(void);

/** @brief Application Framework Tick Callback
 *
 * A callback invoked in each iteration of the application super loop and
 * can be used to perform periodic functions.  The frequency with which this
 * function is called depends on how quickly the main loop runs.  If the
 * application blocks at any time during the main loop, this function will not
 * be called until execution resumes.
 *
 */
void emberAfTickCallback(void);

/** @brief  Application framework equivalent of ::emberStackStatusHandler
 */
void emberAfStackStatusCallback(EmberStatus status);

/** @brief Application framework equivalent of ::emberIncomingMessageHandler
 */
void emberAfIncomingMessageCallback(EmberIncomingMessage *message);

/** @brief Application framework equivalent of ::emberIncomingMacMessageHandler
 */
void emberAfIncomingMacMessageCallback(EmberIncomingMacMessage *message);

/** @brief Application framework equivalent of ::emberMessageSentHandler
 */
void emberAfMessageSentCallback(EmberStatus status,
                                EmberOutgoingMessage *message);

/** @brief Application framework equivalent of ::emberMacMessageSentHandler
 */
void emberAfMacMessageSentCallback(EmberStatus status,
                                   EmberOutgoingMacMessage *message);

/** @brief Application framework equivalent of ::emberChildJoinHandler
 *  @warning Requires the parent support plugin installed.
 */
void emberAfChildJoinCallback(EmberNodeType nodeType,
                              EmberNodeId nodeId);

/** @brief Application framework equivalent of ::emberActiveScanCompleteHandler
 */
void emberAfActiveScanCompleteCallback(void);

/** @brief Application framework equivalent of ::emberEnergyScanCompleteHandler
 */
void emberAfEnergyScanCompleteCallback(int8_t mean,
                                       int8_t min,
                                       int8_t max,
                                       uint16_t variance);

/** @brief Application framework equivalent of ::emberMarkApplicationBuffersHandler
 */
void emberAfMarkApplicationBuffersCallback(void);

/** @brief Application framework equivalent of ::emberIncomingBeaconHandler
 */
void emberAfIncomingBeaconCallback(EmberPanId panId,
                                   EmberMacAddress *source,
                                   int8_t rssi,
                                   bool permitJoining,
                                   uint8_t beaconFieldsLength,
                                   uint8_t *beaconFields,
                                   uint8_t beaconPayloadLength,
                                   uint8_t *beaconPayload);

/** @brief Application framework equivalent of ::emberFrequencyHoppingStartClientCompleteHandler
 */
void emberAfFrequencyHoppingStartClientCompleteCallback(EmberStatus status);

/** @brief Application framework equivalent of ::emberRadioNeedsCalibratingHandler
 */
void emberAfRadioNeedsCalibratingCallback(void);

/** @brief Application framework equivalent of ::emberStackIdleHandler
 */
bool emberAfStackIdleCallback(uint32_t *idleTimeMs);

/** @brief Application framework Low Power notification Callback
 *
 * A callback invoked when the system is about to go sleeping.
 *
 *  @param[in] enter_em2   \b true if the system is about to sleep or \b false to idle.
 *
 *  @param[in] duration_ms   Duration of the low power period. Time to the next event.
 *
 *  @return \b true if the application allows the system to go to sleep.
 */
bool emberAfCommonOkToEnterLowPowerCallback(bool enter_em2,
                                            uint32_t duration_ms);
/**
 * @}
 *
 * @}
 */

#endif // APP_FRAMEWORK_CALLBACK_H
