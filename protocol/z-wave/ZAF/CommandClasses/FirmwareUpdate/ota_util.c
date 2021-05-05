/**
 * @file
 * This module implements functions used in combination with command class firmware update.
 * @copyright 2018 Silicon Laboratories Inc.
 */

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#include <ota_util.h>
#include <ZW_TransportLayer.h>

#include <CRC.h>

#include <CC_FirmwareUpdate.h>
#include <CC_ManufacturerSpecific.h>

#include <AppTimer.h>
#include <SwTimer.h>

#include <string.h>
#include <ZAF_Common_interface.h>

#ifndef EFR32ZG23
#include<btl_interface.h>
#include<btl_interface_storage.h>
#endif
#include<BootLoader/OTA/bootloader-slot-configuration.h>
#include <em_wdog.h>

#include "stdlib.h"
#include "em_msc.h"
#include <CC_Version.h>
#include <btl_reset_cause_util.h>
#include <nvm3.h>
#include <ZAF_file_ids.h>
#include <ZAF_nvm3_app.h>
#include <SizeOf.h>
#include <board.h>

//#define DEBUGPRINT
#include "DebugPrint.h"


/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/// Possible states of OTA update.
typedef enum _FW_STATE_
{
  FW_STATE_IDLE,         //!< OTA not active
  FW_STATE_READY,        //!< OTA is ready to start
  FW_STATE_AWAIT_REPORT, //!< OTA is in progress and waiting for next MD Report
} FW_STATE;

/// Possible Events of OTA update.
typedef enum _FW_EVENT_
{
  FW_EVENT_REQ_GET_RECEIVED_VALID,  //!< FW_EVENT_REQ_GET_RECEIVED_VALID
  FW_EVENT_REQ_GET_RECEIVED_INVALID,//!< FW_EVENT_REQ_GET_RECEIVED_INVALID
  FW_EVENT_REQ_REPORT_GOT_ACK,      //!< FW_EVENT_REQ_REPORT_GOT_ACK
  FW_EVENT_REQ_REPORT_NO_ACK,       //!< FW_EVENT_REQ_REPORT_NO_ACK
  FW_EVENT_MAX_RETRIES_REACHED,     //!< FW_EVENT_MAX_RETRIES_REACHED
  FW_EVENT_REPORT_RECEIVED_VALID,   //!< FW_EVENT_REPORT_RECEIVED_VALID
  FW_EVENT_REPORT_RECEIVED_INVALID, //!< FW_EVENT_REPORT_RECEIVED_INVALID
  FW_EVENT_REPORT_RECEIVED_BATCH,   //!< FW_EVENT_REPORT_RECEIVED_BATCH
  FW_EVENT_REPORT_RECEIVED_LAST,    //!< FW_EVENT_REPORT_RECEIVED_LAST
} FW_EVENT;

/// Possible actions during OTA update.
typedef void(*fw_action)(void);

/**
 * Single OTA transition.
 * When event happens, OTA makes the action and goes from state to new_state.
 */
typedef struct _OTA_transition_{
  FW_STATE state;     //!< current state
  FW_EVENT event;     //!< Defined event that might happen in the current state
  fw_action action;   //!< Action to make when event happens
  FW_STATE new_state; //!< next state to transition to.
} OTA_transition;


//This version number must be increased if we make changes in the struct SFirmwareUpdateFile
#define FIRMWARE_UPDATE_FILE_VERSION  0x01

//Sizes of SFirmwareUpdateFile and SFirmwareUpdateFile_DEPRECATED_V0 must not be equal for current automatic file migration to work
STATIC_ASSERT(sizeof(SFirmwareUpdateFile) > sizeof(SFirmwareUpdateFile_DEPRECATED_V0), STATIC_ASSERT_FAILED_file_migration_failure);

#define FIRMWARE_UPDATE_REQUEST_TIMEOUTS         10000   /* unit: 1 ms ticks */
#define FIRMWARE_UPDATE_MAX_RETRY 10

#define BOOTLOADER_NO_OF_FLASHPAGES_IN_SECTOR  116U // This is the number of the flash pages in bootloader section

#define APPLICATION_IMAGE_STORAGE_SLOT 0x0

#define ACTIVATION_SUPPORT_MASK_APP        0x80
#define ACTIVATION_SUPPORT_MASK_INITIATOR  0x01
#define ACTIVATION_SUPPORT_ENABLED_MASK    0x81

/// Specifies whether CC FW Update should request multiple MD Reports in FW Update MD Get
/// If enabled, number of reports is calculated based on storage size and max fragment size
#define OTA_MULTI_FRAME_ENABLED 1

#define OTA_CACHE_SIZE 200

/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

/**
 * Internal storage for incoming FW Update MD Reports.
 * Should be big enough to store at least two incoming frames.
 * If not, then frames are written directly to flash and the storage is not used
 */
static uint8_t mdReportsStorage[OTA_CACHE_SIZE];

/**
 * Number of Reports to request in single FW Update MD Get. Minimum is 1.
 * It's value is calculated upon receiving FW Update MD Request Get.
 * mdGetNumberOfReports = sizeof(mdReportsStorage)/(single fragment size)
 */
static uint8_t mdGetNumberOfReports;

/// Actual fragment size calculated upon receiving REQUEST GET.
static uint8_t firmware_update_packetsize;

typedef struct _OTA_UTIL_
{
  CC_FirmwareUpdate_start_callback_t pOtaStart;
  CC_FirmwareUpdate_finish_callback_t pOtaFinish;
  FW_STATE currentState;
  OTA_STATUS finishStatus;
  uint16_t firmwareCrc;
  uint8_t fw_numOfRetries;
  SSwTimer timerFwUpdateFrameGet;
  SSwTimer timerOtaSuccess;
  uint16_t firmwareUpdateReportNumberPrevious;
  uint16_t fw_crcrunning;
  RECEIVE_OPTIONS_TYPE_EX rxOpt;
  bool NVM_valid;
  uint8_t activation_enabled;
  uint8_t requestReport; /// Status to send in FW Update Request Report
  uint8_t statusReport; /// Status to send in FW Update MD Status Report
  uint8_t reportsReceived; /// counter to keep track how many reports is received so far
} OTA_UTIL;

OTA_UTIL myOta = {
    NULL, // pOtaStart
    NULL, // pOtaFinish
    FW_STATE_IDLE,
    OTA_STATUS_DONE,
    0, // firmwareCrc
    0, // fw_numOfRetries
    {{0}}, // timerFwUpdateFrameGet
    {{0}}, // timerOtaSuccess
    0, // firmwareUpdateReportNumberPrevious
    0, // fw_crcrunning
    {0}, // rxOpt
    false,  // NVM_valid
    0, // activation_enabled
    FIRMWARE_UPDATE_MD_REQUEST_REPORT_VALID_COMBINATION_V5, // requestReport
    FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_V5, // statusReport
    0 // reportsReceived
};

#ifndef EFR32ZG23 // OCELOT_HARDCODED
static int pageCnt =0;
static int32_t pageno=1, prevpage=0;
#endif // !EFR32ZG23

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/
static void initOTAState();
static void TimerCancelFwUpdateFrameGet();
static void TimerStartFwUpdateFrameGet();
#ifndef EFR32ZG23 // OCELOT_HARDCODED
static void ZCB_TimerOutFwUpdateFrameGet(SSwTimer* pTimer);
static void ZCB_FinishFwUpdate(TRANSMISSION_RESULT * pTransmissionResult);
static void ZCB_VerifyImage(SSwTimer* pTimer);
#endif // !EFR32ZG23
static void OTA_WriteData(uint32_t offset, uint8_t* pData, uint16_t legth);
static void UpdateStatusSuccess();
static void SendFirmwareUpdateStatusReport();
static bool OtaVerifyImage();

static void handleEvent(uint8_t event);
static void fw_action_send_get();
static void fw_action_send_req_report();
static void fw_action_send_status_report();
static void fw_action_none();
static void fw_action_verify_image();
#ifdef DEBUGPRINT
static char* getStateAsString(FW_STATE state);
static char* getEventAsString(FW_EVENT event);
#endif //DEBUGPRINT
static void resetReceivedReportsData();
static bool useMultiFrames();

/// Transition table with all supported state-event combinations.
static const OTA_transition OTA_transition_table[] = {
  {FW_STATE_IDLE,          FW_EVENT_REQ_GET_RECEIVED_VALID,   fw_action_send_req_report,    FW_STATE_READY},
  {FW_STATE_IDLE,          FW_EVENT_REQ_GET_RECEIVED_INVALID, fw_action_send_req_report,    FW_STATE_IDLE},
  {FW_STATE_READY,         FW_EVENT_REQ_REPORT_GOT_ACK,       fw_action_send_get,           FW_STATE_AWAIT_REPORT},
  {FW_STATE_READY,         FW_EVENT_REQ_REPORT_NO_ACK,        fw_action_none,               FW_STATE_IDLE},
  {FW_STATE_AWAIT_REPORT,  FW_EVENT_REPORT_RECEIVED_INVALID,  fw_action_send_get,           FW_STATE_AWAIT_REPORT},
  {FW_STATE_AWAIT_REPORT,  FW_EVENT_REPORT_RECEIVED_VALID,    fw_action_none,               FW_STATE_AWAIT_REPORT},
  {FW_STATE_AWAIT_REPORT,  FW_EVENT_MAX_RETRIES_REACHED,      fw_action_send_status_report, FW_STATE_IDLE},
  {FW_STATE_AWAIT_REPORT,  FW_EVENT_REPORT_RECEIVED_BATCH,    fw_action_send_get,           FW_STATE_AWAIT_REPORT},
  {FW_STATE_AWAIT_REPORT,  FW_EVENT_REPORT_RECEIVED_LAST,     fw_action_verify_image,       FW_STATE_IDLE},
};


static inline bool ActivationIsEnabled(void)
{
  return (ACTIVATION_SUPPORT_ENABLED_MASK == myOta.activation_enabled);
}

/**
 * Do cleanup after end flash storage write/erase operation.
 *
 * All the MSC_Init() function does is to unlock the flash controller
 * to allow for other entities (such as the NVM3 module) to access the
 * internal flash device after we are done using it.
 *
 * This is required because the bootloader leaves the flash controller
 * LOCKED upon return from all OTA write/erase operations.
 */
#ifndef EFR32ZG23 // OCELOT_HARDCODED
static void OtaFlashWriteEraseDone(void)
{
  MSC_Init();
}
#endif

bool CC_FirmwareUpdate_Init(
    CC_FirmwareUpdate_start_callback_t pOtaStart,
    CC_FirmwareUpdate_finish_callback_t pOtaFinish,
    bool support_activation)
{
#ifndef EFR32ZG23 // OCELOT_HARDCODED
  int32_t retvalue;

  BootloaderStorageSlot_t slot;
  BootloaderInformation_t bloaderInfo;

  myOta.pOtaStart = pOtaStart;
  myOta.pOtaFinish = pOtaFinish;
  myOta.NVM_valid = true;

  pageCnt = 0;
  pageno = 1;
  prevpage = 0;

  mdGetNumberOfReports = 1;

  if (true == support_activation) myOta.activation_enabled |= ACTIVATION_SUPPORT_MASK_APP;
  else myOta.activation_enabled &= ~ACTIVATION_SUPPORT_MASK_APP;

  retvalue = bootloader_init();
  if(retvalue != BOOTLOADER_OK)
  {
    DPRINTF("\r\nBootloader NOT OK! %x", retvalue);
    myOta.NVM_valid = false;
  }
  /* Checking the bootloader validity before proceed, if it is non silabs bootloader then make it non upgradable */
  bootloader_getInfo(&bloaderInfo);
  if(bloaderInfo.type != SL_BOOTLOADER)
  {
     DPRINTF("\r\nNo bootloader is present or non silabs bootloader hence it's not upgradable type =%x",bloaderInfo.type);
     myOta.NVM_valid = false;
  }
  /*Checking this bootloader has storage capablity or not, just in case a wrong bootloader been loaded into the device*/
  if(!(bloaderInfo.capabilities & BOOTLOADER_CAPABILITY_STORAGE))
  {
     DPRINT("\r\nThis bootloader do not have storage capablity hence it can't be used for OTA");
     myOta.NVM_valid = false;
  }

  //Get the bootloader storage slot information
  slot.address = 0;
  slot.length  = 0;
  bootloader_getStorageSlotInfo(APPLICATION_IMAGE_STORAGE_SLOT,&slot);

  DPRINTF("\r\nslot address %d, length %d",slot.address,slot.length);

  Ecode_t errCode = 0;
  uint32_t objectType;
  size_t dataLen = 0;
  nvm3_Handle_t * pFileSystem = ZAF_GetFileSystemHandle();
  errCode = nvm3_getObjectInfo(pFileSystem,
                               ZAF_FILE_ID_CC_FIRMWARE_UPDATE,
                               &objectType,
                               &dataLen);

  if (ECODE_NVM3_OK != errCode)
  {
    DPRINT("\nFile default!");
    SFirmwareUpdateFile file = {0, 0};
    file.fileVersion = FIRMWARE_UPDATE_FILE_VERSION;
    dataLen = ZAF_FILE_SIZE_CC_FIRMWARE_UPDATE;
    errCode = nvm3_writeData(pFileSystem,
                             ZAF_FILE_ID_CC_FIRMWARE_UPDATE,
                             &file,
                             dataLen);
    ASSERT(ECODE_NVM3_OK == errCode);
  }

  uint16_t bootloader_reset_reason;
  if (CC_FirmwareUpdate_IsFirstBoot(&bootloader_reset_reason))
  {
    DPRINT("\nFIRMWARE UPDATE DONE NOW!");

    SFirmwareUpdateFile file;
    errCode = nvm3_readData(pFileSystem,
                            ZAF_FILE_ID_CC_FIRMWARE_UPDATE,
                            &file,
                            dataLen);
    ASSERT(ECODE_NVM3_OK == errCode);

    DPRINTF("\nF INIT: %x", file.activation_was_applied);

    if (sizeof(SFirmwareUpdateFile_DEPRECATED_V0) == dataLen)
    {
      //Do automatic file migration.
      SFirmwareUpdateFile_DEPRECATED_V0 oldFile;
      memcpy((uint8_t *)&oldFile, (uint8_t *)&file, sizeof(oldFile));

      file.activation_was_applied = oldFile.activation_was_applied;
      file.checksum    = oldFile.checksum;
      file.srcNodeID   = oldFile.srcNodeID;
      file.srcEndpoint = oldFile.srcEndpoint;
      file.rxStatus    = oldFile.rxStatus;
      file.securityKey = oldFile.securityKey;
    }

    RECEIVE_OPTIONS_TYPE_EX rxOpt;
    rxOpt.sourceNode.nodeId   = file.srcNodeID;
    rxOpt.sourceNode.endpoint = file.srcEndpoint;
    rxOpt.rxStatus            = file.rxStatus;
    rxOpt.securityKey         = file.securityKey;
    rxOpt.destNode.endpoint   = 0; // Firmware update is part of the root device.

    if (ACTIVATION_SUPPORT_ENABLED_MASK == file.activation_was_applied)
    {
      // TX Activation Status Report including checksum
      DPRINT("\nTX Activation Status Report!");

      uint8_t status;
      if (BOOTLOADER_RESET_REASON_GO == bootloader_reset_reason)
      {
        status = FIRMWARE_UPDATE_ACTIVATION_STATUS_REPORT_FIRMWARE_UPDATE_COMPLETED_SUCCESSFULLY_V5;
      }
      else
      {
        status = FIRMWARE_UPDATE_ACTIVATION_STATUS_REPORT_ERROR_ACTIVATING_THE_FIRMWARE_V5;
      }
      CC_FirmwareUpdate_ActivationStatusReport_tx(&rxOpt, file.checksum, status);
    }
    else
    {
      uint8_t status;
      if (BOOTLOADER_RESET_REASON_GO == bootloader_reset_reason)
      {
        status = FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_V5;
      }
      else
      {
        status = FIRMWARE_UPDATE_MD_STATUS_REPORT_INVALID_FILE_HEADER_INFORMATION_V5;
      }
      // Tx Status Report
      DPRINT("\nTX Status Report!");
      CmdClassFirmwareUpdateMdStatusReport(&rxOpt,
                                           status,
                                           0,
                                           NULL);
    }
  }

  // Register timer for re-sending FW Update MD Get
  //DPRINT("Registering timer for timerFwUpdateFrameGeT\n");
  bool timerRegisterStatus = AppTimerRegister(&myOta.timerFwUpdateFrameGet,
                                              true,
                                              ZCB_TimerOutFwUpdateFrameGet);
  ASSERT(timerRegisterStatus);

  if(AppTimerRegister(&myOta.timerOtaSuccess, false, ZCB_VerifyImage))
  {
    DPRINT("\r\n**Registering timer OK for last report...**");
  }
  else
  {
    DPRINT("\r\n**Registering timer Failed for  the last report**");
  }
  DPRINTF("\r\nInit including bootloader init finished--bootloader init status%x",retvalue);
  return myOta.NVM_valid;
#else
  UNUSED(pOtaStart);
  UNUSED(pOtaFinish);
  UNUSED(support_activation);
  return false;
#endif // !EFR32ZG23
}

/// Write data to the OTA NVM
/// @param offset Starting address in NVM
/// @param pData pointer to data to be written
/// @param length length of data to be written
static void OTA_WriteData(uint32_t offset, uint8_t *pData, uint16_t length)
{
#ifndef EFR32ZG23 //OCELOT_HARDCODED
  int32_t retvalue = BOOTLOADER_OK;

  if (pageCnt < BOOTLOADER_NO_OF_FLASHPAGES_IN_SECTOR)
  {
    pageno = ((offset + length) / FLASH_PAGE_SIZE) + 1;
    if (pageno != prevpage) //address on the same page so no write
    {
      retvalue = bootloader_eraseRawStorage(
          BTL_STORAGE_SLOT_START_ADDRESS + (pageCnt * FLASH_PAGE_SIZE), FLASH_PAGE_SIZE);
      prevpage = pageno;
      if (retvalue != BOOTLOADER_OK)
      {
        DPRINTF("OTA_ERROR_ERASING_FLASH ERROR =0x%x\n", retvalue);
      }
      else
      {
        //DPRINTF("OTA_SUCESS_ERASING_FLASH page=%d\n",pageCnt);
        pageCnt++;
      }
    }
  }
  /*writing gbl image to the flash*/
  if (0 == (length % 4))
  {
    retvalue = bootloader_writeRawStorage(BTL_STORAGE_SLOT_START_ADDRESS + offset, pData, length); //for EFR32ZG Chip
  }
  else
  {
    //last packet make the writing as 4 bytes alligned
    DPRINTF("Writing the last packet length is %d...\n", length);
    retvalue = bootloader_writeRawStorage(BTL_STORAGE_SLOT_START_ADDRESS + offset, pData,
                                          length + (4 - (length % 4)));
  }
  if (retvalue != BOOTLOADER_OK)
  {
    DPRINTF("OTA_ERROR_WRITING_FLASH ERROR =0x%x,length=%d,offset=%d", retvalue, length, offset);
  }
  OtaFlashWriteEraseDone(); // Required after end flash write/erase operation
#else
  UNUSED(offset);
  UNUSED(pData);
  UNUSED(length);
#endif // !EFR32ZG23
}

/*======================== UpdateStatusSuccess ==========================
** Function to set ev state after successful verification of image.
**
** Side effects:
**
**-------------------------------------------------------------------------*/
static void UpdateStatusSuccess()
{

  DPRINT("OTA_SUCCESS_CB");
  if (ActivationIsEnabled())
  {
    myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_WAITING_FOR_ACTIVATION_V5;
  }
  else
  {
    /* send FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_V5 to controller.
       Device reboot itself*/
    myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_V5;
  }
  SendFirmwareUpdateStatusReport();
  myOta.finishStatus = OTA_STATUS_DONE;
}

/*======================== ZCB_VerifyImage ==========================
** Timer callback to start image verification *after* we have ack/routed-ack'ed
** the last fw update frame.
**
** Side effects:
**
**-------------------------------------------------------------------------*/
#ifndef EFR32ZG23 // OCELOT_HARDCODED
static void ZCB_VerifyImage(SSwTimer* pTimer)
{
  UNUSED(pTimer);
  handleEvent(FW_EVENT_REPORT_RECEIVED_LAST);
}
#endif

void
handleCmdClassFirmwareUpdateMdReport( uint16_t crc16Result,
                                      uint16_t firmwareUpdateReportNumber,
                                      uint8_t properties,
                                      uint8_t* pData,
                                      uint8_t fw_actualFrameSize)
{
  // Ignore FW Update MD Report if OTA is not in progress
  // handleEvent() would handle any unexpected events anyway.
  // Purpose of this check is just to speed up the process.
  if( FW_STATE_AWAIT_REPORT != myOta.currentState)
  {
    /*Not correct state.. just stop*/
    DPRINTF("Received MD Report in wrong state %d, expected was %d. Stop.\n",
            myOta.currentState, FW_STATE_AWAIT_REPORT);
    return;
  }
  /*Check checksum*/
  DPRINTF(" (CRC----: 0x%04X)\r\n", crc16Result);
  if (0 != crc16Result)
  {
    DPRINT("CRC invalid\r\n");
    myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_UNABLE_TO_RECEIVE_WITHOUT_CHECKSUM_ERROR_V5;
    return;
  }

  // CRC of the received frame OK, continue.
  myOta.fw_numOfRetries = 0;

  /* Check report number */
  if (firmwareUpdateReportNumber != myOta.firmwareUpdateReportNumberPrevious + 1)
  {
    DPRINT("Report Number invalid\r\n");
    // (firmwareUpdateReportNumber == myOta.firmwareUpdateReportNumberPrevious + 1) do not match.
    // Set Status value and let the timer handle retries
    myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_UNABLE_TO_RECEIVE_V5;
    return;
  }

  /* Right report number, continue */
  if ((firmware_update_packetsize != fw_actualFrameSize) && (!(properties & FIRMWARE_UPDATE_MD_REPORT_PROPERTIES1_LAST_BIT_MASK)))
  {
    DPRINTF("%% 0x%x - 0x%x\r\n", firmware_update_packetsize, fw_actualFrameSize);
    // (firmware_update_packetsize != fw_actualFrameSize) and not last packet - do not match.
    // Set Status value and let the timer handle retries
    myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_UNABLE_TO_RECEIVE_V5;
    return;
  }
  myOta.firmwareUpdateReportNumberPrevious = firmwareUpdateReportNumber;


  myOta.fw_crcrunning = CRC_CheckCrc16(myOta.fw_crcrunning, pData, fw_actualFrameSize);

  uint32_t startAddress;

  if (useMultiFrames())
  {
    // If mdReportsStorage > 1, then we expect multiple frames after single MD Get.
    // If so, write all incoming frames into mdReportsStorage,
    // and write entire content of it into NVM when all of them have been received
    startAddress = myOta.reportsReceived * firmware_update_packetsize;
    memcpy(&mdReportsStorage[startAddress], pData, fw_actualFrameSize);
  }
  else
  {
    // Otherwise, write data directly to flash
    startAddress = ((uint32_t)(firmwareUpdateReportNumber - 1) * firmware_update_packetsize);
    OTA_WriteData(startAddress, pData, fw_actualFrameSize);
  }
  myOta.reportsReceived ++;

  /* Is this the last report ? */
  if (properties & FIRMWARE_UPDATE_MD_REPORT_PROPERTIES1_LAST_BIT_MASK)
  {
    /*check CRC for received dataBuffer*/
    if (myOta.fw_crcrunning != myOta.firmwareCrc)
    {
      DPRINTF("**OTA_FAIL CRC!!** 0x%x - 0x%x\r\n", myOta.fw_crcrunning, myOta.firmwareCrc);
      myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_UNABLE_TO_RECEIVE_WITHOUT_CHECKSUM_ERROR_V5;
      handleEvent(FW_EVENT_REPORT_RECEIVED_INVALID);
      return;
    }
    DPRINT("**OTA_SUCCESS_CRC**");
    if (useMultiFrames())
    {
      // Calculate length: all reports except for last are full size
      // Last report might be shorter
      uint16_t len = (myOta.reportsReceived - 1) * firmware_update_packetsize
          + fw_actualFrameSize;

      // Data is written to flash when mdGetNumberOfReports is received.
      // Save the starting address for writing to flash
      startAddress = ((uint32_t)(firmwareUpdateReportNumber - myOta.reportsReceived) * firmware_update_packetsize);
      OTA_WriteData(startAddress, mdReportsStorage, len);
    }
    // Delay verification of the firmware image
    // so we can transmit the ack or routed ack first
    if(ESWTIMER_STATUS_FAILED == TimerStart(&myOta.timerOtaSuccess, 100))
    {
      DPRINT("OTA_SUCCESS_NOTIMER");
      handleEvent(FW_EVENT_REPORT_RECEIVED_LAST);
    }
  }
  else
  {
    if (myOta.reportsReceived < mdGetNumberOfReports)
    {
      // Waiting for more reports
      handleEvent(FW_EVENT_REPORT_RECEIVED_VALID);
    }
    else
    {
      // If multi frames are used, write to flash now.
      // If not, all data has already been written to flash.
      if (useMultiFrames())
      {
        // Calculate length: number of received * size of one report
        uint16_t len = myOta.reportsReceived * firmware_update_packetsize;

        // Find the starting address for writing to flash
        startAddress = ((uint32_t)(firmwareUpdateReportNumber - myOta.reportsReceived) * firmware_update_packetsize);
        OTA_WriteData(startAddress, mdReportsStorage, len);
      }
      handleEvent(FW_EVENT_REPORT_RECEIVED_BATCH);
    }
  }
}

void handleCmdClassFirmwareUpdateMdReqGet(
  RECEIVE_OPTIONS_TYPE_EX *rxOpt,
  ZW_FIRMWARE_UPDATE_MD_REQUEST_GET_V5_FRAME * pFrame,
  uint8_t cmdLength,
  uint8_t* pStatus)
{
  uint8_t fwTarget = 0;
  uint32_t fragmentSize = 0xFFFFFFFF;

  /* Verify if the FragmentSize and FirmwareTarget fields are part of the command (V3 and onwards) */
  if (sizeof(ZW_FIRMWARE_UPDATE_MD_REQUEST_GET_V3_FRAME) <= cmdLength)
  {
    fwTarget = pFrame->firmwareTarget;
    fragmentSize = 0;
    fragmentSize +=  (((uint32_t)pFrame->fragmentSize1) << 8);
    fragmentSize +=  (((uint32_t)pFrame->fragmentSize2) & 0xff);
  }

  if(0 != fwTarget)
  {
    DPRINT("** External fwExtern is not supported, aborting.**");
    *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_NOT_UPGRADABLE_V5;
    handleEvent(FW_EVENT_REQ_GET_RECEIVED_INVALID);
    return;
  }

  if (sizeof(ZW_FIRMWARE_UPDATE_MD_REQUEST_GET_V4_FRAME) <= cmdLength)
  {
    // Activation bit
    if (pFrame->properties1 & FIRMWARE_UPDATE_MD_REQUEST_GET_PROPERTIES1_ACTIVATION_BIT_MASK_V5)
    {
      myOta.activation_enabled |= ACTIVATION_SUPPORT_MASK_INITIATOR;
    }
    else myOta.activation_enabled &= ~ACTIVATION_SUPPORT_MASK_INITIATOR;
  }

  /* Validate the hardwareVersion field (V5 and onwards) */
  uint8_t hardwareVersion = CC_Version_GetHardwareVersion_handler();
  if ((sizeof(ZW_FIRMWARE_UPDATE_MD_REQUEST_GET_V5_FRAME) <= cmdLength) &&
      (hardwareVersion != pFrame->hardwareVersion))
  {
    /* Invalid hardware version */
    *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_INVALID_HARDWARE_VERSION_V5;
    myOta.requestReport = *pStatus;
    handleEvent(FW_EVENT_REQ_GET_RECEIVED_INVALID);
    return;
  }

  if (pFrame->firmwareTarget >= CC_Version_getNumberOfFirmwareTargets_handler())
  {
    /*wrong target!!*/
    *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_NOT_UPGRADABLE_V5;
    myOta.requestReport = *pStatus;
    handleEvent(FW_EVENT_REQ_GET_RECEIVED_INVALID);
    return;
  }

  uint16_t manufacturerID = 0;
  uint16_t productID      = 0;
  CC_ManufacturerSpecific_ManufacturerSpecificGet_handler(&manufacturerID, &productID);

  uint32_t maxFragmentSize = (uint32_t)handleCommandClassFirmwareUpdateMaxFragmentSize() & 0x0000FFFF;

  if (0xFFFFFFFF == fragmentSize)
  {
    // The Request Get command did not contain a fragment size => Set it to max fragment size.
    fragmentSize = maxFragmentSize;
  }
  else if ((fragmentSize > maxFragmentSize) || (0 == fragmentSize))
  {
    /*
     * The fragment size given in Request Get was too high or zero.
     * Report status of invalid fragment size.
     */
    *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_INVALID_FRAGMENT_SIZE_V5;
    myOta.requestReport = *pStatus;
    handleEvent(FW_EVENT_REQ_GET_RECEIVED_INVALID);
    return;
  }

  uint16_t manufacturerIdIncoming = (((uint16_t)pFrame->manufacturerId1) << 8)
                                    | (uint16_t)pFrame->manufacturerId2;
  uint16_t firmwareIdIncoming = (((uint16_t)pFrame->firmwareId1) << 8)
                                | (uint16_t)pFrame->firmwareId2;
  uint16_t firmwareId = handleFirmWareIdGet(fwTarget);
  if ((manufacturerIdIncoming != manufacturerID) ||
      (firmwareIdIncoming != firmwareId))
  {
    *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_INVALID_COMBINATION_V5;
    myOta.requestReport = *pStatus;
    handleEvent(FW_EVENT_REQ_GET_RECEIVED_INVALID);
    return;
  }

  if (false == myOta.NVM_valid)
  {
    *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_NOT_UPGRADABLE_V5;
    myOta.requestReport = *pStatus;
    handleEvent(FW_EVENT_REQ_GET_RECEIVED_INVALID);
    return;
  }

  uint16_t checksumIncoming = (((uint16_t)pFrame->checksum1) << 8)
                              | (uint16_t)pFrame->checksum2;

  /*Firmware valid.. ask OtaStart to start update*/
  if (NON_NULL(myOta.pOtaStart) &&
      (false == myOta.pOtaStart(handleFirmWareIdGet(fwTarget), checksumIncoming)))
  {
    DPRINT("&");
    *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_REQUIRES_AUTHENTICATION_V5;
    myOta.requestReport = *pStatus;
    handleEvent(FW_EVENT_REQ_GET_RECEIVED_INVALID);
    return;
  }
  initOTAState();
  memcpy( (uint8_t*) &myOta.rxOpt, (uint8_t*)rxOpt, sizeof(RECEIVE_OPTIONS_TYPE_EX));

  // Save activationstatus, checksum and RX options
  Ecode_t errCode = 0;
  SFirmwareUpdateFile file = {
                              .activation_was_applied = myOta.activation_enabled,
                              .fileVersion = FIRMWARE_UPDATE_FILE_VERSION,
                              .checksum = checksumIncoming,
                              .srcNodeID = rxOpt->sourceNode.nodeId,
                              .srcEndpoint = rxOpt->sourceNode.endpoint,
                              .rxStatus = rxOpt->rxStatus,
                              .securityKey = (uint32_t)rxOpt->securityKey
  };
  nvm3_Handle_t * pFileSystem = ZAF_GetFileSystemHandle();
  errCode = nvm3_writeData(pFileSystem, ZAF_FILE_ID_CC_FIRMWARE_UPDATE, &file, ZAF_FILE_SIZE_CC_FIRMWARE_UPDATE);
  ASSERT(ECODE_NVM3_OK == errCode);

  DPRINTF("\nF: %x", file.activation_was_applied);

  myOta.firmwareCrc = checksumIncoming;
  firmware_update_packetsize = (uint8_t)fragmentSize;

  // At this point maxFragmentSize is known => calculate Number of Reports
  // If storage is not big enough for at least 2 reports, don't enable multi data frames
  if (OTA_MULTI_FRAME_ENABLED
      && sizeof(mdReportsStorage) >= (2 * firmware_update_packetsize))
  {
    mdGetNumberOfReports = sizeof(mdReportsStorage) / firmware_update_packetsize;
  }
  DPRINTF("FW Update MD Get - Number Of Reports: %d\n", mdGetNumberOfReports);

  *pStatus = FIRMWARE_UPDATE_MD_REQUEST_REPORT_VALID_COMBINATION_V5;
  myOta.requestReport = *pStatus;

  handleEvent(FW_EVENT_REQ_GET_RECEIVED_VALID);
}

void ZCB_CmdClassFwUpdateMdReqReport(uint8_t txStatus)
{
  if (FIRMWARE_UPDATE_MD_REQUEST_REPORT_VALID_COMBINATION_V5 != myOta.requestReport)
  {
    // If the request get command failed, nothing was initiated. Hence, don't restart.
    return;
  }

  if(txStatus == TRANSMIT_COMPLETE_OK)
  {
    handleEvent(FW_EVENT_REQ_REPORT_GOT_ACK);
  }
  else{
    handleEvent(FW_EVENT_REQ_REPORT_NO_ACK);
  }
}


/// Sets/Resets myOta to initial values
static void initOTAState()
{
  myOta.currentState = FW_STATE_IDLE;
  myOta.fw_crcrunning = 0x1D0F;
  myOta.firmwareUpdateReportNumberPrevious = 0;
  myOta.fw_numOfRetries = 0;
  myOta.firmwareCrc = 0;
  myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_V5;
}


/**
 * Sends FW Update Status Report after OTA is done
 * Possible values defined in SDS13782, CC:007A.05.07.11.006
 * @param status Status to be sent.
 */
static void
SendFirmwareUpdateStatusReport()
{
#ifndef EFR32ZG23 //OCELOT_HARDCODED
  uint8_t waitTime = WAITTIME_FWU_FAIL;
  TimerCancelFwUpdateFrameGet();

  switch (myOta.statusReport)
  {
    case FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_STORED_V5:
      // The image is stored. Report it and wait for user reboot.
      waitTime = 0;
      break;
    case FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_V5:
      // Reboot right away and report afterwards.
      bootloader_rebootAndInstall();
      return;
      break;
    case FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_WAITING_FOR_ACTIVATION_V5:
      // Image is stored. Wait for activation.
      waitTime = 0;
      break;
    default:
      // Do nothing
      break;
  }

  if (JOB_STATUS_SUCCESS !=
      CmdClassFirmwareUpdateMdStatusReport(&myOta.rxOpt,
                                           myOta.statusReport,
                                           waitTime,
                                           ZCB_FinishFwUpdate))
  {
    /*Failed to send frame and we do not get a CB. Inform app we are finish*/
    ZCB_FinishFwUpdate(NULL);
  }
#endif // !EFR32ZG23
}

/// Callback Finish Fw update status to application.
#ifndef EFR32ZG23 // OCELOT_HARDCODED
static void ZCB_FinishFwUpdate(TRANSMISSION_RESULT * pTransmissionResult)
{
  UNUSED(pTransmissionResult);
  if (NULL != myOta.pOtaFinish)
  {
    myOta.pOtaFinish(myOta.finishStatus);
  }

  // Reboot if the firmware update went well and activation is disabled.
  if ((OTA_STATUS_DONE == myOta.finishStatus) &&
      !ActivationIsEnabled())
  {
    DPRINT("Reboot from ZCB_FinishFwUpdate\n");
    DPRINT("Now telling the bootloader new image to boot install\n");
    bootloader_rebootAndInstall();
  }
}
#endif // !EFR32ZG23

/**
 * Uses bootloder functionality to verify downloaded image.
 * @attention Returns false in case of app version downgrade
 * @return true if there exists a valid image that can be installed by the bootloader.
 */
static bool OtaVerifyImage()
{
#ifndef EFR32ZG23 //OCELOT_HARDCODED
  int32_t verifystat;

  static bool watchdogPrev_state = false;

  if (WDOG_IsEnabled())
  {
    watchdogPrev_state = true;
    WDOG_Enable(false);
  }

  verifystat = bootloader_verifyImage(APPLICATION_IMAGE_STORAGE_SLOT, NULL);

  if (BOOTLOADER_OK == verifystat)
  {
    DPRINT("bootloader_verifyImage went OK\n");
    WDOG_Enable(watchdogPrev_state);
    return true;
  }
  else
  {
    DPRINTF("booloader_verifyImage went WRONG!!   ERR: %4x\n", verifystat);
    // Prepare for the next OTA update attempt
    // Rewind the page counters and erase the storage
    pageno=1;
    prevpage=0;
    int32_t retvalue = bootloader_eraseStorageSlot(APPLICATION_IMAGE_STORAGE_SLOT);
    if (BOOTLOADER_OK != retvalue)
    {
      DPRINTF("bootloader_eraseStorageSlot FAILED!!   ERR: %4x\n", retvalue);
    }
    OtaFlashWriteEraseDone(); // Required after end flash write/erase operation
    WDOG_Enable(watchdogPrev_state);
    return false;
  }
#else
  return false;
#endif // !EFR32ZG23
}

/// Cancel timer for retries on Get next firmware update frame.
static void
TimerCancelFwUpdateFrameGet(void)
{
  if (TimerIsActive(&myOta.timerFwUpdateFrameGet))
  {
    TimerStop(&myOta.timerFwUpdateFrameGet);
  }
  myOta.fw_numOfRetries = 0;
}


/// Callback on timeout on Get next firmware update frame.
/// It retry to Send a new Get frame.
/// @param pTimer Fw Update Frame Get timer
#ifndef EFR32ZG23 // OCELOT_HARDCODED
static void ZCB_TimerOutFwUpdateFrameGet(SSwTimer* pTimer)
{
  UNUSED(pTimer);
  DPRINTF("Timer expired. retry %d. time\n", myOta.fw_numOfRetries);

  if (FIRMWARE_UPDATE_MAX_RETRY > ++(myOta.fw_numOfRetries))
  {
    DPRINTF("Send MD GET again, reportNo=%d\n", myOta.firmwareUpdateReportNumberPrevious + 1);
    CmdClassFirmwareUpdateMdGet( &myOta.rxOpt, myOta.firmwareUpdateReportNumberPrevious + 1);
    /*Start/restart timer*/
  }
  else
  {
    DPRINT("+");
    if (FIRMWARE_UPDATE_MD_STATUS_REPORT_SUCCESSFULLY_V5 == myOta.statusReport)
    {
      // If we haven't set status value already, set it now.
      myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_UNABLE_TO_RECEIVE_V5;
    }
    handleEvent(FW_EVENT_MAX_RETRIES_REACHED);
  }
}
#endif // !EFR32ZG23

/// Start or restart timer for retries on Get next firmware update frame.
static void
TimerStartFwUpdateFrameGet(void)
{
  myOta.fw_numOfRetries = 0;
  if (false == TimerIsActive(&myOta.timerFwUpdateFrameGet))
  {
    ESwTimerStatus timerStatus = TimerStart(&myOta.timerFwUpdateFrameGet,
                                            FIRMWARE_UPDATE_REQUEST_TIMEOUTS);

    if (ESWTIMER_STATUS_FAILED == timerStatus)
    {
      /* No timer! we update without a timer for retries */
      DPRINTF("Failed to start timerFwUpdateFrameGet\n");
    }
  }
  else
  {
    TimerRestart(&myOta.timerFwUpdateFrameGet);
  }
}

/*
 * Maximum fragment size definitions.
 */
#define FIRMWARE_UPDATE_MD_REPORT_ENCAPSULATION_LENGTH 6

uint16_t handleCommandClassFirmwareUpdateMaxFragmentSize(void)
{
  uint16_t maxFragmentSize;

  maxFragmentSize = ZAF_getAppHandle()->pNetworkInfo->MaxPayloadSize - FIRMWARE_UPDATE_MD_REPORT_ENCAPSULATION_LENGTH;

  // Align with 32 bit due to the writing to flash.
  maxFragmentSize = maxFragmentSize - (maxFragmentSize % 4);

  return maxFragmentSize;
}

bool CC_FirmwareUpdate_ActivationSet_handler(
    ZW_FIRMWARE_UPDATE_ACTIVATION_SET_V5_FRAME * pFrame,
    uint8_t * pStatus)
{
#ifndef EFR32ZG23 // OCELOT_HARDCODED
  uint16_t firmwareId = handleFirmWareIdGet(pFrame->firmwareTarget);
  uint16_t manufacturerID = 0;
  uint16_t productID      = 0;
  CC_ManufacturerSpecific_ManufacturerSpecificGet_handler(&manufacturerID, &productID);
  uint8_t hardwareVersion = CC_Version_GetHardwareVersion_handler();

  uint16_t manufacturerIdIncoming = (((uint16_t)pFrame->manufacturerId1) << 8)
                                    | (uint16_t)pFrame->manufacturerId2;
  uint16_t firmwareIdIncoming = (((uint16_t)pFrame->firmwareId1) << 8)
                                | (uint16_t)pFrame->firmwareId2;
  uint16_t checksumIncoming = (((uint16_t)pFrame->checksum1) << 8)
                              | (uint16_t)pFrame->checksum2;

  /* Either no checksum value has yet been calculated, or we are an EM4 sleeping device and therefore need to restore the calculated value from file storage */
  if (0 == myOta.firmwareCrc)
  {
    SFirmwareUpdateFile file;
    nvm3_Handle_t * pFileSystem = ZAF_GetFileSystemHandle();
    Ecode_t errCode = nvm3_readData(pFileSystem,
                                    ZAF_FILE_ID_CC_FIRMWARE_UPDATE,
                                    &file,
                                    ZAF_FILE_SIZE_CC_FIRMWARE_UPDATE);
    ASSERT(ECODE_NVM3_OK == errCode);
    myOta.firmwareCrc = file.checksum;
  }

  if ((manufacturerIdIncoming != manufacturerID) ||
      (firmwareIdIncoming != firmwareId) ||
      (checksumIncoming != myOta.firmwareCrc) ||
      (pFrame->hardwareVersion != hardwareVersion))
  {
    *pStatus = FIRMWARE_UPDATE_ACTIVATION_STATUS_REPORT_INVALID_COMBINATION_V5;
    return false;
  }
  DPRINTF("\n manufacturerIdIncoming: %4x", manufacturerIdIncoming);
  DPRINTF("\n manufacturerID: %4x", manufacturerID);
  DPRINTF("\n firmwareIdIncoming: %4x", firmwareIdIncoming);
  DPRINTF("\n firmwareId: %4x", firmwareId);

  bootloader_rebootAndInstall();
  return true;
#else
  UNUSED(pFrame);
  UNUSED(pStatus);
  return false;
#endif // !EFR32ZG23
}


/**
 * Goes through all transitions in @ref OTA_transition_table
 * and finds next action for the current event.
 * Called when @ref OTA_transition.event happens.
 * @param event Event to handle
 */
static
void handleEvent(uint8_t event)
{
  uint8_t len = sizeof_array(OTA_transition_table);
  int i = 0;
  for (; i < len; i++)
  {
    // Check if event exists in transition table
    if(OTA_transition_table[i].event == event)
    {
      DPRINTF("%s[%d]: Event %s\n", __func__,
                                   i,
                                   getEventAsString(event));
      // Check if event is defined for current state
      if (OTA_transition_table[i].state == myOta.currentState)
      {
        // Correct state-event combination found. Do Action and set new state.
        DPRINTF("%s: state %s ---> %s\n", __func__,
                getStateAsString(OTA_transition_table[i].state),
                getStateAsString(OTA_transition_table[i].new_state));
        fw_action fwaction = OTA_transition_table[i].action;
        fwaction();
        myOta.currentState = OTA_transition_table[i].new_state;
        return;
      }
      else
      {
        // State doesn't match. Check if there is another entry
        // with the same event, but another state
        DPRINTF("%s: State %s doesn't match. Continue.\n",
                __func__,
                getStateAsString(OTA_transition_table[i].state));
      }
    }
  }
  // Reached the end of the loop without finding matching state/event pair.
  DPRINTF("%s: Event %s doesn't exist or not expected in the current state %s. Ignore.\n",
           __func__,
           getEventAsString(event),
           getStateAsString(myOta.currentState));
  // Ignore invalid frame and continue.
  // For more strict control, comment out next line.
  // Board_ResetHandler();
}
/// Sends FW Update MD Get
static void fw_action_send_get()
{
  DPRINTF(">> %s\n", __func__);

  resetReceivedReportsData();
  TimerStartFwUpdateFrameGet();
  CmdClassFirmwareUpdateMdGet(&myOta.rxOpt,
                              myOta.firmwareUpdateReportNumberPrevious + 1);
}
/// Handles sending FW Update Request Report.
static void fw_action_send_req_report()
{
  DPRINTF(">> %s, requestReport status: %d\n", __func__, myOta.requestReport);

  if (FIRMWARE_UPDATE_MD_REQUEST_REPORT_VALID_COMBINATION_V5 == myOta.requestReport)
  {
    CC_FirmwareUpdate_InvalidateImage();
    myOta.fw_crcrunning = 0x1D0F;
    myOta.firmwareUpdateReportNumberPrevious = 0;
    TimerCancelFwUpdateFrameGet();
  }
  // Done. Actual Request Report is sent from CC.
}
/// OTA done. Sends FW Update Status Report.
static void fw_action_send_status_report()
{
  DPRINTF(">> %s, send status report[%d] and end.\n", __func__, myOta.statusReport);

  // OTA should be finished after this point. Send Status report.
  // SendFirmwareUpdateStatusReport() will also stop any running timers
  myOta.finishStatus = OTA_STATUS_ABORT;
  SendFirmwareUpdateStatusReport();
  if (FIRMWARE_UPDATE_MD_STATUS_REPORT_UNABLE_TO_RECEIVE_V5 == myOta.statusReport)
  {
    // Device unable to receive new frames.
    // Reboot to avoid situation where device gets stuck for any unpredicted reason.
    Board_ResetHandler();
  }
}
/// Verifies image and sends FW Update Status Report.
static void fw_action_verify_image()
{
  DPRINTF(">> %s \n", __func__);
  resetReceivedReportsData();
  if(OtaVerifyImage())
  {
    UpdateStatusSuccess();
  }
  else
  {
    DPRINT("FIRMWARE_UPDATE_MD_STATUS_REPORT_DOES_NOT_MATCH_THE_FIRMWARE_TARGET_V5");
    myOta.finishStatus = OTA_STATUS_ABORT;
    myOta.statusReport = FIRMWARE_UPDATE_MD_STATUS_REPORT_DOES_NOT_MATCH_THE_FIRMWARE_TARGET_V5;
    SendFirmwareUpdateStatusReport();
  }
}
/// No action needed.
static void fw_action_none()
{
  DPRINTF(">> %s - Nothing to do.\n", __func__);
}

#ifdef DEBUGPRINT
/// Helper function to provide more user friendly debug output
static char* getStateAsString(FW_STATE state)
{
  switch(state)
  {
    case FW_STATE_IDLE:
      return "FW_STATE_IDLE";
    case FW_STATE_READY:
      return "FW_STATE_READY";
    case FW_STATE_AWAIT_REPORT:
      return "FW_STATE_AWAIT_REPORT";
    default:
      return "-STATE UNKNOWN-";
  }
  return NULL;
}
/// Helper function to provide more user friendly debug output
static char* getEventAsString(FW_EVENT event)
{
  switch(event)
  {
    case FW_EVENT_REQ_GET_RECEIVED_VALID:
      return "FW_EVENT_REQ_GET_RECEIVED_VALID";
    case FW_EVENT_REQ_GET_RECEIVED_INVALID:
      return "FW_EVENT_REQ_GET_RECEIVED_INVALID";
    case FW_EVENT_REQ_REPORT_GOT_ACK:
      return "FW_EVENT_REQ_REPORT_GOT_ACK";
    case FW_EVENT_REQ_REPORT_NO_ACK:
      return "FW_EVENT_REQ_REPORT_NO_ACK";
    case FW_EVENT_MAX_RETRIES_REACHED:
      return "FW_EVENT_MAX_RETRIES_REACHED";
    case FW_EVENT_REPORT_RECEIVED_VALID:
      return "FW_EVENT_REPORT_RECEIVED_VALID";
    case FW_EVENT_REPORT_RECEIVED_INVALID:
      return "FW_EVENT_REPORT_RECEIVED_INVALID";
    case FW_EVENT_REPORT_RECEIVED_BATCH:
      return "FW_EVENT_REPORT_RECEIVED_BATCH";
    case FW_EVENT_REPORT_RECEIVED_LAST:
      return "FW_EVENT_REPORT_RECEIVED_LAST";
    default:
      return "-EVENT UNKNOWN-";
  }
  return NULL;
}
#endif //DEBUGPRINT


uint8_t getFWUpdateMDGetNumberOfReports()
{
  // Number of reports to request = Max number of reports - number or reports already received
  // Device should keep receiving MD reports as long as mdReportsStorage is not full.
  // If some received report is invalid, then next MD Get should request as many reports,
  // as it is needed to fill mdReportsStorage completely.

  DPRINTF(">> %s Ask for %d reports. Received so far in this step: %d\n",
          __func__,
          mdGetNumberOfReports - myOta.reportsReceived,
          myOta.reportsReceived);

  return mdGetNumberOfReports - myOta.reportsReceived;
}

/// Called after all reports requested in one GET are received,
/// or entire transfer is complete.
static void resetReceivedReportsData()
{
  DPRINTF(">> %s - reset data\n", __func__);
  myOta.reportsReceived = 0;
  memset(mdReportsStorage, 0x00, sizeof(mdReportsStorage));
}

/// Checks whether multi frames should be used. To use it, Multi frames option
/// must be enabled, and number of reports should be greater than 1
static bool useMultiFrames()
{
  return OTA_MULTI_FRAME_ENABLED && mdGetNumberOfReports;
}
