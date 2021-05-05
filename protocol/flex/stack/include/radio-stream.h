/***************************************************************************//**
 * @brief Radio stream API
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

#ifndef RADIO_STREAM_H
#define RADIO_STREAM_H

/**
 * @addtogroup radio_stream
 * @brief Connect API managing radio stream for RF testing purpose
 *
 * See radio-stream.h for source code.
 * @{
 */

/**
 * @brief Start a continuous Tx stream to test RF.
 *
 * @param[in] parameters Stream mode. See ::EmberTxStreamParameters.
 * @param[in] channel RF channel.
 * @return EMBER_INVALID_CALL if the stack can not process the request\n
 *         EMBER_BAD_ARGUMENT if the parameters are wrong\n
 *         EMBER_SUCCESS if the stream can be started
 */
EmberStatus emberStartTxStream(EmberTxStreamParameters parameters, uint16_t channel);

/**
 * @brief Stop a RF stream in progress
 *
 * @return EMBER_INVALID_CALL if no stream is in progress\n
 *         EMBER_SUCCESS otherwise
 */
EmberStatus emberStopTxStream(void);

/**
 * @}
 */

 #endif
