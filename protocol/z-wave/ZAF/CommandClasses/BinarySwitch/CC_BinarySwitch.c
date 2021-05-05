/**
 * @file
 * Handler for Command Class Binary Switch.
 * @copyright 2018 Silicon Laboratories Inc.
 */

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#include <CC_BinarySwitch.h>
#include <string.h>
#include "ZAF_tx_mutex.h"

//#define DEBUGPRINT
#include "DebugPrint.h"

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

received_frame_status_t
handleCommandClassBinarySwitch(
  RECEIVE_OPTIONS_TYPE_EX *rxOpt,
  ZW_APPLICATION_TX_BUFFER *pCmd,
  uint8_t cmdLength)
{
  switch (pCmd->ZW_Common.cmd)
  {
    case SWITCH_BINARY_GET:
      if(false == Check_not_legal_response_job(rxOpt))
      {
        ZAF_TRANSPORT_TX_BUFFER  TxBuf;
        ZW_APPLICATION_TX_BUFFER *pTxBuf = &(TxBuf.appTxBuf);
        memset((uint8_t*)pTxBuf, 0, sizeof(ZW_APPLICATION_TX_BUFFER) );

        TRANSMIT_OPTIONS_TYPE_SINGLE_EX *pTxOptionsEx;
        RxToTxOptions(rxOpt, &pTxOptionsEx);
        pTxBuf->ZW_SwitchBinaryReportV2Frame.cmdClass = COMMAND_CLASS_SWITCH_BINARY_V2;
        pTxBuf->ZW_SwitchBinaryReportV2Frame.cmd = SWITCH_BINARY_REPORT_V2;

        /* Get the values from the application */
        pTxBuf->ZW_SwitchBinaryReportV2Frame.currentValue = appBinarySwitchGetCurrentValue(rxOpt->destNode.endpoint);
        pTxBuf->ZW_SwitchBinaryReportV2Frame.targetValue  = appBinarySwitchGetTargetValue(rxOpt->destNode.endpoint);
        pTxBuf->ZW_SwitchBinaryReportV2Frame.duration     = appBinarySwitchGetDuration(rxOpt->destNode.endpoint);

        if(ZAF_ENQUEUE_STATUS_SUCCESS != Transport_SendResponseEP(
            (uint8_t *)pTxBuf,
            sizeof(ZW_SWITCH_BINARY_REPORT_V2_FRAME),
            pTxOptionsEx,
            NULL))
        {
          /*Job failed */
          ;
        }
        return RECEIVED_FRAME_STATUS_SUCCESS;
      }
      return RECEIVED_FRAME_STATUS_FAIL;
      break;

    case SWITCH_BINARY_SET:
      if ((0x63 < pCmd->ZW_SwitchBinarySetV2Frame.targetValue) &&
          (0xFF != pCmd->ZW_SwitchBinarySetV2Frame.targetValue))
      {
        return RECEIVED_FRAME_STATUS_FAIL;
      }

      if (2 >= cmdLength)
      {
        return RECEIVED_FRAME_STATUS_FAIL;
      }

      if (3 == cmdLength) /* version 1 (does not have duration) */
      {
        pCmd->ZW_SwitchBinarySetV2Frame.duration = BIN_SWITCH_DURATION_FACTORY_DEFAULT;
      }

      e_cmd_handler_return_code_t return_code = CommandClassBinarySwitchSupportSet(
          pCmd->ZW_SwitchBinarySetV2Frame.targetValue,
          pCmd->ZW_SwitchBinarySetV2Frame.duration,
          rxOpt->destNode.endpoint);

      /* If handler has finished, call TSE */
      if (E_CMD_HANDLER_RETURN_CODE_FAIL == return_code || E_CMD_HANDLER_RETURN_CODE_HANDLED == return_code)
      {
        // Build up new CC data structure
        void* pBinarySwitchData = CC_BinarySwitch_prepare_zaf_tse_data(rxOpt);
        if (false == ZAF_TSE_Trigger((void *)CC_BinarySwitch_report_stx, pBinarySwitchData, true))
        {
          DPRINTF("%s(): ZAF_TSE_Trigger failed\n", __func__);
        }
      }

      // if command handler fail, return fail exit status
      if (E_CMD_HANDLER_RETURN_CODE_FAIL == return_code)
      {
        return RECEIVED_FRAME_STATUS_FAIL;
      }

      return RECEIVED_FRAME_STATUS_SUCCESS;
      break;
  }
  return RECEIVED_FRAME_STATUS_NO_SUPPORT;
}

e_cmd_handler_return_code_t CommandClassBinarySwitchSupportSet(
    uint8_t target_value,
    uint8_t duration,
    uint8_t endpoint)
{
  if (BIN_SWITCH_DURATION_FACTORY_DEFAULT == duration)
  {
    /* Get factory default duration from application */
    duration = appBinarySwitchGetFactoryDefaultDuration(endpoint);
  }

  /* Tell application to set binary switch values.
   * Only map and pass supported targetValues (0-99 or 0xFF). Unsupported values MUST be ignored.
   */
  if (target_value == 0)
  {
    return appBinarySwitchSet(CMD_CLASS_BIN_OFF, duration, endpoint);
  }
  else if ((target_value <= 99) ||
           (target_value == 0xff))
  {
    /* All valid non-zero values are simply mapped to "ON" */
    return appBinarySwitchSet(CMD_CLASS_BIN_ON, duration, endpoint);
  }
  // nothing to be done, return success.
  return E_CMD_HANDLER_RETURN_CODE_HANDLED;
}

JOB_STATUS CmdClassBinarySwitchReportSendUnsolicited(
  AGI_PROFILE* pProfile,
  uint8_t sourceEndpoint,
  CMD_CLASS_BIN_SW_VAL current_value,
  CMD_CLASS_BIN_SW_VAL target_value,
  uint8_t duration,
  VOID_CALLBACKFUNC(pCbFunc)(TRANSMISSION_RESULT * pTransmissionResult))
{
  CMD_CLASS_GRP cmdGrp = {COMMAND_CLASS_SWITCH_BINARY_V2, SWITCH_BINARY_REPORT_V2};

  ZW_SWITCH_BINARY_REPORT_V2_FRAME report_frame;
  report_frame.currentValue = current_value;
  report_frame.targetValue  = target_value;
  report_frame.duration     = duration;

  uint16_t payload_offset = offsetof(ZW_SWITCH_BINARY_REPORT_V2_FRAME, currentValue);
  uint8_t *p_payload      = ((uint8_t*) &report_frame) + payload_offset;

  return cc_engine_multicast_request(
      pProfile,
      sourceEndpoint,
      &cmdGrp,
      p_payload,
      sizeof(report_frame) - payload_offset,
      true,
      pCbFunc);
}


void CC_BinarySwitch_report_stx(
    TRANSMIT_OPTIONS_TYPE_SINGLE_EX txOptions,
    s_CC_binarySwitch_data_t* pData)
{
  DPRINTF("* %s() *\n"
      "\ttxOpt.src = %d\n"
      "\ttxOpt.options %#02x\n"
      "\ttxOpt.secOptions %#02x\n",
      __func__, txOptions.sourceEndpoint, txOptions.txOptions, txOptions.txSecOptions);

  /* Prepare payload for report */
  ZAF_TRANSPORT_TX_BUFFER  TxBuf;
  ZW_APPLICATION_TX_BUFFER *pTxBuf = &(TxBuf.appTxBuf);
  memset((uint8_t*)pTxBuf, 0, sizeof(ZW_APPLICATION_TX_BUFFER) );

  pTxBuf->ZW_SwitchBinaryReportV2Frame.cmdClass = COMMAND_CLASS_SWITCH_BINARY_V2;
  pTxBuf->ZW_SwitchBinaryReportV2Frame.cmd = SWITCH_BINARY_REPORT_V2;

  /* Get the values from the application */
  pTxBuf->ZW_SwitchBinaryReportV2Frame.currentValue = appBinarySwitchGetCurrentValue(pData->rxOptions.destNode.endpoint);
  pTxBuf->ZW_SwitchBinaryReportV2Frame.targetValue  = appBinarySwitchGetTargetValue(pData->rxOptions.destNode.endpoint);
  pTxBuf->ZW_SwitchBinaryReportV2Frame.duration     = appBinarySwitchGetDuration(pData->rxOptions.destNode.endpoint);

  if (ZAF_ENQUEUE_STATUS_SUCCESS != Transport_SendRequestEP((uint8_t *)pTxBuf,
                                                                sizeof(ZW_SWITCH_BINARY_REPORT_V2_FRAME),
                                                                &txOptions,
                                                                ZAF_TSE_TXCallback))
  {
    //sending request failed
    DPRINTF("%s(): Transport_SendRequestEP() failed. \n", __func__);
  }
}

REGISTER_CC(COMMAND_CLASS_SWITCH_BINARY, SWITCH_BINARY_VERSION_V2, handleCommandClassBinarySwitch);
