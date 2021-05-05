/*
 * Copyright (c) 2020 Silicon Laboratories, Inc. http://www.silabs.com
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include "em_device.h"
#include "em_system.h"
#include <assert.h>
#include <cmsis_os2.h>
#include "sl_cmsis_os2_common.h"
#include "ns_types.h"
#include "arm_hal_phy.h"
#include "arm_hal_interrupt.h"
#include "common_functions.h"
#include "sli_wisun_trace.h"
#include "rail.h"
#include "rail_config.h"
#include "pa_conversions_efr32.h"
#include "sli_wisun_driver.h"
#include "sl_wisun_common.h"
#include "pti-config.h"

#define TRACE_GROUP "SLRF"

/* Enable debug printing with SLI_RADIO_DEBUG, override debug printer with SLI_DEBUG_PRINT. */
#ifdef SLI_RADIO_DEBUG
#ifndef SLI_DEBUG_PRINT
#define SLI_DEBUG_PRINT(...) tr_debug(__VA_ARGS__)
#endif
#else
#define SLI_DEBUG_PRINT(...)
#endif

/* RF_TASK_STACK_SIZE defines stack size for the RF PHY worker thread */
#ifndef RF_TASK_STACK_SIZE
#define RF_TASK_STACK_SIZE 500 // in units of CPU_INT32U
#endif

/* RF_QUEUE_SIZE defines queue size for incoming messages */
#ifndef RF_QUEUE_SIZE
#define RF_QUEUE_SIZE 4
#endif

/* RF PHY worker thread priority */
#ifndef RF_TASK_PRIORITY
#define RF_TASK_PRIORITY (osPriority_t)46
#endif

/* The maximum size of a single RAIL packet is 2045 bytes (2 PHR + 2047 PSDU - 4 FCS) */
#define MAC_PACKET_MAX_LENGTH 2045

/* Offsets of prepended data in packet buffer */
#define MAC_PACKET_OFFSET_RSSI 0
#define MAC_PACKET_OFFSET_LQI 1
#define MAC_PACKET_OFFSET_PHR 2
#define MAC_PACKET_OFFSET_PAYLOAD 4

/* Length of prepended RSSI and LQI */
#define MAC_PACKET_INFO_LENGTH 2
/* Length of PHY header */
#define MAC_PACKET_PHR_LENGTH 2
/* Length of FCS field */
#define MAC_PACKET_FCS_LENGTH 4

/* RAIL FIFO sizes */
#define TX_FIFO_SIZE 4092
#define RX_FIFO_SIZE 4096

/* SUN-FSK bitmasks */
#define RF_SUN_FSK_PHR_MODE_SWITCH_MASK 0x0100
#define RF_SUN_FSK_PHR_MODE_SWITCH_SHIFT 8
#define RF_SUN_FSK_PHR_FCS_TYPE_MASK 0x0800
#define RF_SUN_FSK_PHR_FCS_TYPE_SHIFT 11
#define RF_SUN_FSK_PHR_DATA_WHITENING_MASK 0x1000
#define RF_SUN_FSK_PHR_DATA_WHITENING_SHIFT 12
#define RF_SUN_FSK_PHR_DATA_LENGTH_MASK 0x07FF
#define RF_SUN_FSK_PHR_DATA_LENGTH_SHIFT 0

/* SUN-FSK helper macros */
#define RF_SUN_FSK_GET_PHR_DATA_LENGTH(__phr__) ((__RBIT((__phr__) & 0xFF) >> 24) | ((__RBIT((__phr__) >> 8) >> 16) & 0x0700))
#define RF_SUN_FSK_GET_PHR_DATA_WHITENING(__phr__) (((__phr__) & RF_SUN_FSK_PHR_DATA_WHITENING_MASK) >> RF_SUN_FSK_PHR_DATA_WHITENING_SHIFT)
#define RF_SUN_FSK_GET_PHR_FCS_TYPE(__phr__) (((__phr__) & RF_SUN_FSK_PHR_FCS_TYPE_MASK) >> RF_SUN_FSK_PHR_FCS_TYPE_SHIFT)
#define RF_SUN_FSK_GET_PHR_MODE_SWITCH(__phr__) (((__phr__) & RF_SUN_FSK_PHR_MODE_SWITCH_MASK) >> RF_SUN_FSK_PHR_MODE_SWITCH_SHIFT)

#define RF_SUN_FSK_SET_PHR_DATA_LENGTH(__value__) ((__RBIT((__value__) & 0xFF) >> 24) | (__RBIT((__value__) >> 8) >> 16))
#define RF_SUN_FSK_SET_PHR_DATA_WHITENING(__value__) (((__value__) << RF_SUN_FSK_PHR_DATA_WHITENING_SHIFT) & RF_SUN_FSK_PHR_DATA_WHITENING_MASK)
#define RF_SUN_FSK_SET_PHR_FCS_TYPE(__value__) (((__value__) << RF_SUN_FSK_PHR_FCS_TYPE_SHIFT) & RF_SUN_FSK_PHR_FCS_TYPE_MASK)
#define RF_SUN_FSK_SET_PHR_MODE_SWITCH(__value__) (((__value__) << RF_SUN_FSK_PHR_MODE_SWITCH_SHIFT) & RF_SUN_FSK_PHR_MODE_SWITCH_MASK)

/* MAC 15.4 bitmasks */
#define RF_MAC_FC_FRAME_TYPE_MASK 0x0007
#define RF_MAC_FC_FRAME_TYPE_SHIFT 0
#define RF_MAC_FC_SECURITY_ENABLED_MASK 0x0008
#define RF_MAC_FC_SECURITY_ENABLED_SHIFT 3
#define RF_MAC_FC_FRAME_PENDING_MASK 0x0010
#define RF_MAC_FC_FRAME_PENDING_SHIFT 4
#define RF_MAC_FC_ACK_REQUESTED_MASK 0x0020
#define RF_MAC_FC_ACK_REQUESTED_SHIFT 5
#define RF_MAC_FC_PANID_COMPRESSION_MASK 0x0040
#define RF_MAC_FC_PANID_COMPRESSION_SHIFT 6
#define RF_MAC_FC_SEQ_SUPRESSION_MASK 0x0100
#define RF_MAC_FC_SEQ_SUPRESSION_SHIFT 8
#define RF_MAC_FC_IE_PRESENT_MASK 0x0200
#define RF_MAC_FC_IE_PRESENT_SHIFT 9
#define RF_MAC_FC_DEST_ADDR_MODE_MASK 0x0C00
#define RF_MAC_FC_DEST_ADDR_MODE_SHIFT 10
#define RF_MAC_FC_FRAME_VERSION_MASK 0x3000
#define RF_MAC_FC_FRAME_VERSION_SHIFT 12
#define RF_MAC_FC_SRC_ADDR_MODE_MASK 0xC000
#define RF_MAC_FC_SRC_ADDR_MODE_SHIFT 14

/* MAC 15.4 helper macros */
#define RF_MAC_GET_FC_FRAME_TYPE(__fc__) (((__fc__) & RF_MAC_FC_FRAME_TYPE_MASK) >> RF_MAC_FC_FRAME_TYPE_SHIFT)
#define RF_MAC_GET_FC_SECURITY_ENABLED(__fc__) (((__fc__) & RF_MAC_FC_SECURITY_ENABLED_MASK) >> RF_MAC_FC_SECURITY_ENABLED_SHIFT)
#define RF_MAC_GET_FC_FRAME_PENDING(__fc__) (((__fc__) & RF_MAC_FC_FRAME_PENDING_MASK) >> RF_MAC_FC_FRAME_PENDING_SHIFT)
#define RF_MAC_GET_FC_ACK_REQUESTED(__fc__) (((__fc__) & RF_MAC_FC_ACK_REQUESTED_MASK) >> RF_MAC_FC_ACK_REQUESTED_SHIFT)
#define RF_MAC_GET_FC_PANID_COMPRESSION(__fc__) (((__fc__) & RF_MAC_FC_PANID_COMPRESSION_MASK) >> RF_MAC_FC_PANID_COMPRESSION_SHIFT)
#define RF_MAC_GET_FC_SEQ_SUPRESSION(__fc__) (((__fc__) & RF_MAC_FC_SEQ_SUPRESSION_MASK) >> RF_MAC_FC_SEQ_SUPRESSION_SHIFT)
#define RF_MAC_GET_FC_IE_PRESENT(__fc__) (((__fc__) & RF_MAC_FC_IE_PRESENT_MASK) >> RF_MAC_FC_IE_PRESENT_SHIFT)
#define RF_MAC_GET_FC_DEST_ADDR_MODE(__fc__) (((__fc__) & RF_MAC_FC_DEST_ADDR_MODE_MASK) >> RF_MAC_FC_DEST_ADDR_MODE_SHIFT)
#define RF_MAC_GET_FC_FRAME_VERSION(__fc__) (((__fc__) & RF_MAC_FC_FRAME_VERSION_MASK) >> RF_MAC_FC_FRAME_VERSION_SHIFT)
#define RF_MAC_GET_FC_SRC_ADDR_MODE(__fc__) (((__fc__) & RF_MAC_FC_SRC_ADDR_MODE_MASK) >> RF_MAC_FC_SRC_ADDR_MODE_SHIFT)

/* RAIL helper macros */
#define RF_RAIL_STATE(__value__, __state__) ((__value__ & __state__) == __state__)

#define RF_TASK_FLAG_NONE         (0)
#define RF_TASK_FLAG_RX_SUCCESS   (1)
#define RF_TASK_FLAG_RX_FAILURE   (2)
#define RF_TASK_FLAG_TX_SUCCESS   (4)
#define RF_TASK_FLAG_CSMA_TIMEOUT (8)
#define RF_TASK_FLAG_CCA_CLEAR    (16)
#define RF_TASK_FLAG_CCA_BUSY     (32)
#define RF_TASK_FLAG_UNEXPECTED   (64)
#define RF_TASK_FLAG_ALL          (255)

// RAIL events enabled
#define RF_TASK_RAIL_EVENTS (RAIL_EVENT_RX_PACKET_RECEIVED        \
                             | RAIL_EVENT_RX_FRAME_ERROR          \
                             | RAIL_EVENT_RX_FIFO_OVERFLOW        \
                             | RAIL_EVENT_RX_ADDRESS_FILTERED     \
                             | RAIL_EVENT_RX_PACKET_ABORTED       \
                             | RAIL_EVENT_RX_SCHEDULED_RX_MISSED  \
                             | RAIL_EVENT_TX_PACKET_SENT          \
                             | RAIL_EVENT_TX_CHANNEL_CLEAR        \
                             | RAIL_EVENT_TX_CHANNEL_BUSY         \
                             | RAIL_EVENT_TX_UNDERFLOW            \
                             | RAIL_EVENT_TXACK_UNDERFLOW)

// FIXME CALIBRATION

/*  RF driver worker thread definitions */
static osThreadId_t rf_task_id;
__ALIGNED(8) static uint8_t rf_task_tcb[osThreadCbSize];
__ALIGNED(8) static uint8_t rf_task_stack[(RF_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];

static uint32_t rf_task_flags;
osEventFlagsId_t rf_task_flag_group;
__ALIGNED(8) static uint8_t rf_task_flags_cb[osEventFlagsCbSize];

/* Queue for passing messages from interrupt to worker thread */
static volatile uint8_t rx_queue[RF_QUEUE_SIZE][MAC_PACKET_MAX_LENGTH + MAC_PACKET_INFO_LENGTH];
static size_t rx_queue_head = 0;
static size_t rx_queue_tail = 0;

/* RF driver PHY API data */
static phy_device_driver_s rf_device_driver;

/* RF driver PHY API ID */
static int8_t rf_radio_driver_id = -1;

/* RF driver MAC address */
static uint8_t rf_mac_address[8];

#ifdef USE_RAIL_MAC_FILTER
static uint8_t rf_mac_address_reversed[8];

/* RF driver PAN address */
static uint16_t rf_pan_address;

/* RF driver PAN short address */
static uint16_t rf_short_address;
#endif

/* RF driver CSMA-CA timer */
static RAIL_MultiTimer_t rf_csma_timer;

/* Channel configurations */
static const phy_rf_channel_configuration_s rf_phy_24ghz = {2405000000U, 5000000U, 250000U, 16U, M_OQPSK, MODULATION_INDEX_UNDEFINED, false, OFDM_OPTION_1, OFDM_MCS_0};
static const phy_rf_channel_configuration_s rf_phy_subghz = {868300000U, 500000U, 250000U, 11U, M_2FSK, MODULATION_INDEX_UNDEFINED, false, OFDM_OPTION_1, OFDM_MCS_0};

/* RF driver TX power in dBm */
static int8_t rf_tx_power = 20;

/* Channel page configuration */
static const phy_device_channel_page_s rf_phy_channel_pages[] =
{
  { CHANNEL_PAGE_0, &rf_phy_24ghz},
  { CHANNEL_PAGE_2, &rf_phy_subghz},
  { CHANNEL_PAGE_0, NULL}
};

/* Possible RF driver radio states. */
typedef enum {
  RADIO_UNINIT,       /* Radio has not been initialized */
  RADIO_INITING,      /* Radio is being initialized */
  RADIO_IDLE,         /* Radio is idle */
  RADIO_TX_CSMA,      /* Radio is observing CSMA-CA */
  RADIO_TX_CCA,       /* Radio is observing CCA */
  RADIO_TX,           /* Radio is transmitting */
  RADIO_RX,           /* Radio is receiving */
  RADIO_CALIBRATION   /* Radio is being calibrated */
} rf_radio_state_e;

/* RAIL CSMA-CA configuration for CCA without backoff */
static RAIL_CsmaConfig_t rf_rail_csma_config = RAIL_CSMA_CONFIG_SINGLE_CCA;

/* RAIL RX state transition configuration */
static const RAIL_StateTransitions_t rf_rail_rx_state_transit =
{
  .success = RAIL_RF_STATE_RX,
  .error = RAIL_RF_STATE_RX
};

/* RAIL TX state transition configuration */
static const RAIL_StateTransitions_t rf_rail_tx_state_transit =
{
  .success = RAIL_RF_STATE_RX,
  .error = RAIL_RF_STATE_RX
};

/* RAIL state transition timing configuration */
static RAIL_StateTiming_t rf_rail_state_timings =
{
  .idleToRx = 0,
  .txToRx = 0,
  .idleToTx = 0,
  .rxToTx = 0,
  .rxSearchTimeout = 0, /* infinite */
  .txToRxSearchTimeout = 0, /* infinite */
};

#if defined(MBED_CONF_SL_RAIL_PTI) && (MBED_CONF_SL_RAIL_PTI == 1)
/* RAIL PTI configuration */
static const RAIL_PtiConfig_t rf_rail_pti_config =
{
  MBED_CONF_SL_RAIL_PTI_MODE,
  MBED_CONF_SL_RAIL_PTI_BAUDRATE,
  MBED_CONF_SL_RAIL_PTI_DOUT_LOCATION,
  MBED_CONF_SL_RAIL_PTI_DOUT_PORT,
  MBED_CONF_SL_RAIL_PTI_DOUT_PIN,
  MBED_CONF_SL_RAIL_PTI_DCLK_LOCATION,
  MBED_CONF_SL_RAIL_PTI_DCLK_PORT,
  MBED_CONF_SL_RAIL_PTI_DCLK_PIN,
  MBED_CONF_SL_RAIL_PTI_DFRAME_LOCATION,
  MBED_CONF_SL_RAIL_PTI_DFRAME_PORT,
  MBED_CONF_SL_RAIL_PTI_DFRAME_PIN
};
#endif

/* RAIL TX power configuration */
static const RAIL_TxPowerConfig_t rf_rail_tx_power_2p4 =
{
  .mode = RAIL_TX_POWER_MODE_2P4_HP,
  .voltage = 1800,
  .rampTime = 10,
};

static const RAIL_TxPowerConfig_t rf_rail_tx_power_subghz =
{
  .mode = RAIL_TX_POWER_MODE_SUBGIG,
  .voltage = 1800,
  .rampTime = 10,
};

RAIL_DECLARE_TX_POWER_VBAT_CURVES(rf_rail_piecewise_seg, rf_rail_curves_subghz, rf_rail_curves_2p4hp, rf_rail_curves_2p4lp);

static RAIL_TxPowerCurvesConfig_t rf_rail_tx_power_curves;

/* TX handle of the ongoing transmission request */
static volatile uint8_t rf_current_tx_handle = 0;

static volatile bool data_pending = false, last_ack_pending_bit = false;
static RAIL_Handle_t rf_rail_handle = NULL;
static uint8_t txFifo[TX_FIFO_SIZE];
static uint8_t rxFifo[RX_FIFO_SIZE];
static uint16_t rxFifoSize = sizeof(rxFifo);
static RAIL_RxPacketDetails_t rxPacketDetails;

/* Whether platform deep sleep is currently blocked by the RF driver */
static volatile bool rf_platform_sleep_blocked = false;

/* CSMA-CA parameters to be used on the next transmission */
static phy_csma_params_t rf_phy_csma_params;

/* MAC CSMA-CA process status */
static volatile int8_t rf_phy_csma_status;

/* Current channel */
static int16_t rf_channel = -1;

/* Target channel */
static int16_t rf_target_channel = -1;

/* Current RF driver radio state */
static volatile rf_radio_state_e rf_radio_state = RADIO_UNINIT;

/* Current RF PHY configuration */
static phy_rf_channel_configuration_s rf_phy_config;

/* ARM_NWK_HAL prototypes */
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr);
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t new_channel);
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static int8_t rf_tx(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol );

/* Local function prototypes */
static void RAILCb_RfReady(RAIL_Handle_t railHandle);
static void radioEventHandler(RAIL_Handle_t railHandle, RAIL_Events_t events);

static RAIL_Config_t railCfg = { // Must never be const
                                 .eventsCallback = &radioEventHandler,
                                 .protocol = NULL, // For BLE, pointer to a RAIL_BLE_State_t. For IEEE802.15.4 this must be NULL.
                                 .scheduler = NULL, // For MultiProtocol, pointer to a RAIL_SchedConfig_t
                               };

#define SL_ERROR_CHECK_SET_STATUS(__status__, __condition__) \
  do {                                                         \
  if ((__condition__) == 0) {                                \
  __status__ = -1;                                         \
  goto error_handler;                                      \
  }                                                          \
  __status__ = 0;                                            \
  } while (0)

#define SL_ERROR_CHECK(__status__)                           \
  do {                                                         \
  if (((int8_t)__status__) < 0) {                            \
  goto error_handler;                                      \
  }                                                          \
  } while (0)

#define SL_RAIL_ERROR_CHECK(__status__)                      \
  do {                                                         \
  if (((RAIL_Status_t)__status__) != RAIL_STATUS_NO_ERROR) { \
  goto error_handler;                                      \
  }                                                          \
  } while (0)

static int8_t rf_start_csma_timeout();
static int8_t rf_start_cca();
static int8_t rf_start_tx();
static void rf_abort_tx();
static int8_t rf_start_rx();
static int8_t rf_start_idle();
static void rf_deny_platform_sleep();
static void rf_allow_platform_sleep();

static void rf_deny_platform_sleep()
{
  if (!rf_platform_sleep_blocked) {
    /* RX can only happen in EM0/1 */
    rf_platform_sleep_blocked = true;
  }
}

static void rf_allow_platform_sleep()
{
  if (rf_platform_sleep_blocked) {
    rf_platform_sleep_blocked = false;
  }
}

static void rf_handle_rx_success()
{
  //SLI_DEBUG_PRINT("rf_handle_rx_success");

  /* This event can occur in RADIO_TX_CSMA due to CSMA-CA backoff */

  while (rx_queue_tail != rx_queue_head) {
    uint8_t* packet = (uint8_t*) rx_queue[rx_queue_tail];
    uint16_t phr = common_read_16_bit(&packet[MAC_PACKET_OFFSET_PHR]);

    /* The packet received from RAIL is [RSSI][LQI][2-byte PHR][MAC frame].
     *
     * The length extracted from the PHY header contains the 4 FCS bytes that
     * are stripped by RAIL.
     */

    /*
        mbed_tracef(TRACE_LEVEL_DEBUG, "RX", "packet: %s", mbed_trace_array(packet,
            RF_SUN_FSK_GET_PHR_DATA_LENGTH(phr) + MAC_PACKET_INFO_LENGTH + MAC_PACKET_PHR_LENGTH - MAC_PACKET_FCS_LENGTH));
        */

    /* Notify the MAC layer */
    rf_device_driver.phy_rx_cb(&packet[MAC_PACKET_OFFSET_PAYLOAD],
                               RF_SUN_FSK_GET_PHR_DATA_LENGTH(phr) - MAC_PACKET_FCS_LENGTH,
                               packet[MAC_PACKET_OFFSET_LQI],
                               packet[MAC_PACKET_OFFSET_RSSI],
                               rf_radio_driver_id);
    rx_queue_tail = (rx_queue_tail + 1) % RF_QUEUE_SIZE;
  }
}

static void rf_handle_rx_failure()
{
  //SLI_DEBUG_PRINT("rf_handle_rx_failure");

  /* This event can occur in RADIO_TX_CSMA due to CSMA-CA backoff */

  /**
   * Comments in NanostackRfPhys2lp.cpp indicate the driver is responsible
   * for changing the channel in case of an RX failure IF a channel switch
   * has been deferred due to a pending RX.
   *
   * That doesn't make any sense since the driver has already communicated
   * an error to the MAC layer and thus rf_mac_setup->mac_channel points
   * to the previous channel.
   */
}

static void rf_handle_tx_success()
{
  SLI_DEBUG_PRINT("rf_handle_tx_success");

  /* Make sure we are on the right channel */
  rf_start_rx();

  /* Notify the MAC layer */
  rf_device_driver.phy_tx_done_cb(rf_radio_driver_id,
                                  rf_current_tx_handle,
                                  PHY_LINK_TX_SUCCESS,
                                  1,
                                  1);
}

static void rf_handle_csma_timeout()
{
  SLI_DEBUG_PRINT("rf_handle_csma_timeout @ 0x%08lx", RAIL_GetTime());

  /* Check CSMA-CA process state */
  rf_phy_csma_status = rf_device_driver.phy_tx_done_cb(
                         rf_radio_driver_id, rf_current_tx_handle, PHY_LINK_CCA_PREPARE, 0, 0);

  if (rf_phy_csma_status == PHY_TX_NOT_ALLOWED) {
    /* Abort the transmission attempt. No need to notify the MAC layer, it already knows. */
    rf_abort_tx();
  } else if (rf_phy_csma_status == PHY_RESTART_CSMA) {
    /* Start CCA */
    rf_start_cca();
  } else if (rf_phy_csma_status == PHY_TX_ALLOWED) {
    /* Start CCA */
    rf_start_cca();
  }
}

static void rf_handle_cca_clear()
{
  SLI_DEBUG_PRINT("rf_handle_cca_clear");

  if (rf_phy_csma_status == PHY_RESTART_CSMA)
  {
    /* Channel is clear, notify the MAC layer */
    rf_phy_csma_status = rf_device_driver.phy_tx_done_cb(
                           rf_radio_driver_id, rf_current_tx_handle, PHY_LINK_CCA_OK, 0, 0);

    /* Restart CSMA-CA backoff, CSMA parameters have been updated */
    rf_start_csma_timeout();
  }
  else
  {
    /* Channel is clear, proceed with TX immediately */
    rf_start_tx();
  }
}

static void rf_handle_cca_busy()
{
  SLI_DEBUG_PRINT("rf_handle_cca_busy");

  /* Abort the transmission attempt */
  rf_abort_tx();

  /* Channel was busy, notify the MAC layer */
  rf_device_driver.phy_tx_done_cb(
        rf_radio_driver_id, rf_current_tx_handle, PHY_LINK_CCA_FAIL, 1, 0);
}

void rf_task_loop(void *arg)
{
  (void)arg;

  //SLI_DEBUG_PRINT("rf_task_loop: starting (id: %d)", (int)rf_thread_id);
  SLI_WISUN_TASK_LOOP {
    rf_task_flags = osEventFlagsWait(rf_task_flag_group,
                                     RF_TASK_FLAG_ALL,
                                     osFlagsWaitAny,
                                     osWaitForever);
    assert((rf_task_flags & CMSIS_RTOS_ERROR_MASK) == 0);

    platform_enter_critical();

    if (rf_task_flags & RF_TASK_FLAG_TX_SUCCESS) {
      rf_handle_tx_success();
    }

    if (rf_task_flags & RF_TASK_FLAG_RX_SUCCESS) {
      rf_handle_rx_success();
    }

    if (rf_task_flags & RF_TASK_FLAG_RX_FAILURE) {
      rf_handle_rx_failure();
    }

    if (rf_task_flags & RF_TASK_FLAG_CSMA_TIMEOUT) {
      rf_task_flags &= ~RF_TASK_FLAG_CSMA_TIMEOUT;
      rf_handle_csma_timeout();
    }

    if (rf_task_flags & RF_TASK_FLAG_CCA_CLEAR) {
      rf_handle_cca_clear();
    }

    if (rf_task_flags & RF_TASK_FLAG_CCA_BUSY) {
      rf_handle_cca_busy();
    }

    if (rf_task_flags & RF_TASK_FLAG_UNEXPECTED) {
      //SLI_DEBUG_PRINT("rf_task_loop: unexpected RF event");
    }

    platform_exit_critical();
  }
}

/*============ CODE =========*/

static void rf_device_init(void)
{
  RAIL_Status_t ret;
  uint64_t unique_id;
  const uint8_t *unique_id_ptr;
  int i;

  osThreadAttr_t rf_task_attribute = {
    "Wi-SUN RF Task",
    osThreadDetached,
    &rf_task_tcb[0],
    osThreadCbSize,
    &rf_task_stack[0],
    (RF_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u,
    RF_TASK_PRIORITY,
    0,
    0
    };

  const osEventFlagsAttr_t rf_task_flags_attr = {
    "Wi-SUN RF Task Flags",
    0,
    &rf_task_flags_cb[0],
    osEventFlagsCbSize
  };

  rf_rail_handle = RAIL_Init(&railCfg, &RAILCb_RfReady);
  if (!rf_rail_handle)
  {
    /* RAIL initialization failed */
    goto error_handler;
  }

  /* RAIL calibration settings */
  ret = RAIL_ConfigCal(rf_rail_handle, RAIL_CAL_ALL);
  SL_RAIL_ERROR_CHECK(ret);

  // Use the PHY configuration as the default */
  ret = RAIL_ConfigChannels(rf_rail_handle, channelConfigs[0], NULL);
  SL_RAIL_ERROR_CHECK(ret);

  // Enable 802.15.4 acceleration features
  //ret = RAIL_IEEE802154_Init(rf_rail_handle, &config);
  //SL_RAIL_ERROR_CHECK(ret);

  // Initialize RAIL TX power curves
  rf_rail_tx_power_curves.txPower24HpCurves = rf_rail_curves_2p4hp;
  rf_rail_tx_power_curves.txPowerSgCurves = rf_rail_curves_subghz;
  rf_rail_tx_power_curves.txPower24LpCurves = rf_rail_curves_2p4lp;
  rf_rail_tx_power_curves.piecewiseSegments = rf_rail_piecewise_seg;

  // RAIL TX power curves
  ret = RAIL_InitTxPowerCurves(&rf_rail_tx_power_curves);
  SL_RAIL_ERROR_CHECK(ret);

  /* RX state transitions */
  ret = RAIL_SetRxTransitions(rf_rail_handle, &rf_rail_rx_state_transit);
  SL_RAIL_ERROR_CHECK(ret);

  /* TX state transitions */
  ret = RAIL_SetTxTransitions(rf_rail_handle, &rf_rail_tx_state_transit);
  SL_RAIL_ERROR_CHECK(ret);

  // State transition timings
  ret = RAIL_SetStateTiming(rf_rail_handle, &rf_rail_state_timings);
  SL_RAIL_ERROR_CHECK(ret);

  // Fire all events by default
  ret = RAIL_ConfigEvents(rf_rail_handle, RF_TASK_RAIL_EVENTS, RF_TASK_RAIL_EVENTS);
  SL_RAIL_ERROR_CHECK(ret);

  // Setup the transmit buffer
  RAIL_SetTxFifo(rf_rail_handle, txFifo, 0, sizeof(txFifo));

  // Setup the receive buffer
  ret = RAIL_SetRxFifo(rf_rail_handle, rxFifo, &rxFifoSize);
  SL_RAIL_ERROR_CHECK(ret);

  // Use RAIL multimer
  RAIL_ConfigMultiTimer(true);

  /* Get real MAC address */
  unique_id = SYSTEM_GetUnique();
  unique_id_ptr = (const uint8_t *)&unique_id;

  /* MAC is stored MSB first */
  for (i = 0; i < 8; i++) {
      rf_mac_address[7 - i] = unique_id_ptr[i];
  }

  rf_task_flag_group = osEventFlagsNew(&rf_task_flags_attr);
  assert(rf_task_flag_group != NULL);

  rf_task_id = osThreadNew(rf_task_loop,
                           NULL,
                           &rf_task_attribute);
  assert(rf_task_id != 0);

error_handler:

  /* TODO: RAIL cleanup needed? */
  return;
}

/*
 * \brief Function initialises and registers the RF driver.
 *
 * \param none
 *
 * \return rf_radio_driver_id Driver ID given by NET library
 */
static int8_t rf_device_register(void)
{
  //SLI_DEBUG_PRINT("rf_device_register: entry");

  /*Set pointer to MAC address*/
  rf_device_driver.PHY_MAC = rf_mac_address;
  rf_device_driver.driver_description = (char*)"EFR32_154";
  /*Type of RF PHY*/
  rf_device_driver.link_type = PHY_LINK_15_4_SUBGHZ_TYPE;
  /*RF PHY page configuration */
  rf_device_driver.phy_channel_pages = rf_phy_channel_pages;
  /*Maximum size of payload is 127*/
  rf_device_driver.phy_MTU = 2043;
  /*2-byte header in PHY layer (length)*/
  rf_device_driver.phy_header_length = 2;
  /*No tail in PHY layer*/
  rf_device_driver.phy_tail_length = 0;
  /*Set address write function*/
  rf_device_driver.address_write = &rf_address_write;
  /*Set RF extension function*/
  rf_device_driver.extension = &rf_extension;
  /*Set RF state control function*/
  rf_device_driver.state_control = &rf_interface_state_control;
  /*Set transmit function*/
  rf_device_driver.tx = &rf_tx;
  /*Upper layer callbacks init to NULL, get populated by arm_net_phy_register*/
  rf_device_driver.phy_rx_cb = NULL;
  rf_device_driver.phy_tx_done_cb = NULL;
  /*Virtual upper data callback init to NULL*/
  rf_device_driver.arm_net_virtual_rx_cb = NULL;
  rf_device_driver.arm_net_virtual_tx_cb = NULL;

  /*Register device driver*/
  rf_radio_driver_id = arm_net_phy_register(&rf_device_driver);
  SL_ERROR_CHECK(rf_radio_driver_id);

  rx_queue_head = 0;
  rx_queue_tail = 0;

error_handler:

  if (rf_radio_driver_id < 0) {
    SLI_DEBUG_PRINT("rf_device_register failed");

    /* TODO: RAIL cleanup needed? */
  }

  return rf_radio_driver_id;
}

static void rf_device_set_pti_state(bool enable)
{
  RAIL_Status_t ret;

  /**
   * PTI pins have been configured as part of common init, no need
   * to touch them here.
   *
   * There's only one PTI configuration per system so the RAIL handle
   * does not matter.
   */

  if (enable) {
    ret = RAIL_SetPtiProtocol(rf_rail_handle, RAIL_PTI_PROTOCOL_CONFIG);
    SL_RAIL_ERROR_CHECK(ret);
  }

  ret = RAIL_EnablePti(rf_rail_handle, enable);
  SL_RAIL_ERROR_CHECK(ret);

error_handler:

  // Ignore the error
  return;
}

static int rf_device_set_tx_power(int8_t tx_power)
{
  RAIL_Status_t ret;
  RAIL_TxPowerLevel_t powerLevel;
  RAIL_TxPower_t power;
  RAIL_TxPowerConfig_t powerConfig;

  // Cache the requested value
  rf_tx_power = tx_power;

  // RAIL uses deci-dBm units
  power = (RAIL_TxPower_t)rf_tx_power * 10;

  // Read the current PA configuration
  ret = RAIL_GetTxPowerConfig(rf_rail_handle, &powerConfig);
  SL_RAIL_ERROR_CHECK(ret);

  // Convert the value to chip-specific units
  powerLevel = RAIL_ConvertDbmToRaw(rf_rail_handle, powerConfig.mode, power);

  // Attempt to set the requested TX power value
  ret = RAIL_SetTxPower(rf_rail_handle, powerLevel);
  SL_RAIL_ERROR_CHECK(ret);

  tr_debug("RAIL TX power: %d dBm => %u", rf_tx_power, powerLevel);

error_handler:

  if (ret != RAIL_STATUS_NO_ERROR) {
    return -1;
  }

  return 0;
}

void rf_csma_timeout_cb(struct RAIL_MultiTimer *tmr,
                        RAIL_Time_t expectedTimeOfEvent,
                        void *cbArg)
{
  (void)tmr;
  (void)expectedTimeOfEvent;
  (void)cbArg;

  /* CSMA-CA backoff delay has expired, notify the main thread */
  assert((osEventFlagsSet(rf_task_flag_group,
                          RF_TASK_FLAG_CSMA_TIMEOUT) & CMSIS_RTOS_ERROR_MASK) == 0);
}

/* Schedule CSMA-CA */
static int8_t rf_start_csma_timeout()
{
  RAIL_Status_t ret;
  RAIL_Time_t csma_ca;

  //SLI_DEBUG_PRINT("rf_start_csma_timeout");

  /* Make sure we are on the right channel */
  rf_start_rx();

  /* Update radio state information */
  rf_radio_state = RADIO_TX_CSMA;

  /* MBED Implementing the PHY API:
     * "If csma_ca is larger than 65000 us, start the CSMA-CA timer with
     *  a minimum possible timeout value (1 us)".
     *
     * TODO: sometimes backoff_time is in the past?
     */

  csma_ca = rf_phy_csma_params.backoff_time - RAIL_GetTime();
  if (csma_ca > 65000)
  {
    /* Use 0 as timeout, RAIL will trigger an event as soon as possible */
    csma_ca = 0;
  }

  /* Abort ongoing TX CSMA backoff */
  RAIL_CancelMultiTimer(&rf_csma_timer);

  /* Start TX CSMA backoff, this will be completed by rf_csma_timeout_cb callback */
  ret = RAIL_SetMultiTimer(&rf_csma_timer, csma_ca, RAIL_TIME_DELAY, rf_csma_timeout_cb, NULL);
  if (ret != RAIL_STATUS_NO_ERROR) {
    SLI_DEBUG_PRINT("rf_start_csma_timeout: RAIL_SetMultiTimer: %u", ret);

    /* RAIL failure, caller is responsible for the cleanup */
    return -1;
  }

  SLI_DEBUG_PRINT("rf_start_csma_timeout: %lu @ 0x%08lx", csma_ca, RAIL_GetTime());

  return 0;
}

static int8_t rf_start_cca()
{
  RAIL_Status_t ret;
  RAIL_RadioState_t state;

  SLI_DEBUG_PRINT("rf_start_cca");

  /* Update radio state information */
  rf_radio_state = RADIO_TX_CCA;

  /* Current radio state */
  state = RAIL_GetRadioState(rf_rail_handle);

  if (RF_RAIL_STATE(state, RAIL_RF_STATE_RX_ACTIVE)) {
    //SLI_DEBUG_PRINT("RFDEBUG: CCA BUSY due to RX");

    /* Radio is receiving a frame, consider the channel busy */
    assert((osEventFlagsSet(rf_task_flag_group,
                            RF_TASK_FLAG_CCA_BUSY) & CMSIS_RTOS_ERROR_MASK) == 0);

    return 0;
  }

  /* Abort any ongoing radio activity */
  RAIL_Idle(rf_rail_handle, RAIL_IDLE_ABORT, true);

  /* Start CCA process */
  ret = RAIL_StartCcaCsmaTx(rf_rail_handle, rf_target_channel, RAIL_TX_OPTION_CCA_ONLY, &rf_rail_csma_config, NULL);
  if (ret != RAIL_STATUS_NO_ERROR) {
    SLI_DEBUG_PRINT("rf_start_cca: RAIL_StartCcaCsmaTx: %u", ret);

    /* RAIL failure, caller is responsible for the cleanup */
    rf_radio_state = RADIO_IDLE;
    return -1;
  }

  //SLI_DEBUG_PRINT("rf_start_cca: %u -> %u", rf_channel, rf_target_channel);

  /* Update radio state information */
  rf_channel = rf_target_channel;

  return 0;
}

static int8_t rf_start_tx()
{
  RAIL_Status_t ret;

  //SLI_DEBUG_PRINT("rf_start_tx");

  /* Update radio state information */
  rf_radio_state = RADIO_TX;

  /* Abort any ongoing radio activity */
  RAIL_Idle(rf_rail_handle, RAIL_IDLE_ABORT, true);

  ret = RAIL_StartTx(rf_rail_handle, rf_target_channel, RAIL_TX_OPTIONS_DEFAULT, NULL);
  if (ret != RAIL_STATUS_NO_ERROR)
  {
    SLI_DEBUG_PRINT("rf_start_tx: RAIL_StartTx: %u", ret);

    /* RAIL failure, caller is responsible for the cleanup */
    rf_radio_state = RADIO_IDLE;
    return -1;
  }

  //SLI_DEBUG_PRINT("rf_start_tx: %u -> %u", rf_channel, rf_target_channel);

  /* Update radio state information */
  rf_channel = rf_target_channel;

  return 0;
}

static void rf_abort_tx()
{
  //SLI_DEBUG_PRINT("rf_abort_tx");

  /* Abort ongoing TX CSMA backoff */
  RAIL_CancelMultiTimer(&rf_csma_timer);

  /* Ignore already signaled but not yet handled CSMA-CA events */
  assert((osEventFlagsClear(rf_task_flag_group,
                            RF_TASK_FLAG_CSMA_TIMEOUT + RF_TASK_FLAG_CCA_CLEAR + RF_TASK_FLAG_CCA_BUSY) & CMSIS_RTOS_ERROR_MASK) == 0);

  /* Prevent rf_task_loop from handling already signaled CSMA-CA events */
  rf_task_flags &= ~(RF_TASK_FLAG_CSMA_TIMEOUT | RF_TASK_FLAG_CCA_CLEAR | RF_TASK_FLAG_CCA_BUSY);

  /* Kill pending TX frames in the FIFO */
  RAIL_ResetFifo(rf_rail_handle, true, false);

  /* Make sure we are on the right channel */
  rf_start_rx();
}

static int8_t rf_start_rx()
{
  RAIL_Status_t ret;
  RAIL_RadioState_t state;

  //SLI_DEBUG_PRINT("rf_start_rx");

  /* Update radio state information */
  rf_radio_state = RADIO_RX;

  /* Current radio state */
  state = RAIL_GetRadioState(rf_rail_handle);

  if ((RF_RAIL_STATE(state, RAIL_RF_STATE_RX)) &&
      (rf_channel == rf_target_channel)) {
    /* Radio is already receiving on the right channel */
    return 0;
  }

  if ((RF_RAIL_STATE(state, RAIL_RF_STATE_RX_ACTIVE)) &&
      (rf_channel != rf_target_channel)) {
    //SLI_DEBUG_PRINT("RFDEBUG: RX busy");

    /* Radio is receiving a frame, ignore channel change. */
    rf_target_channel = rf_channel;

    /* Return an error to the MAC layer to indicate the channel was not changed. */
    return -1;
  }

  /* Abort any ongoing radio activity */
  RAIL_Idle(rf_rail_handle, RAIL_IDLE_ABORT, true);

  /* Start RX on the requested channel */
  ret = RAIL_StartRx(rf_rail_handle, rf_target_channel, NULL);
  if (ret != RAIL_STATUS_NO_ERROR) {
    SLI_DEBUG_PRINT("rf_start_rx: RAIL_StartRx: %u", ret);

    /* RAIL failure, caller is responsible for the cleanup */
    rf_radio_state = RADIO_IDLE;
    return -1;
  }

  //SLI_DEBUG_PRINT("rf_start_rx: %u -> %u", rf_channel, rf_target_channel);

  /* Update radio state information */
  rf_channel = rf_target_channel;

  return 0;
}

/*
 * \brief Function starts the CCA process before starting data transmission and copies the data to RF TX FIFO.
 *
 * \param data_ptr Pointer to TX data
 * \param data_length Length of the TX data
 * \param tx_handle Handle to transmission
 * \return 0 Success
 * \return -1 Busy
 */
static int8_t rf_tx(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol )
{
  uint16_t value16;
  int8_t status = 0;

  //SLI_DEBUG_PRINT("rf_tx");

  if(data_protocol != PHY_LAYER_PAYLOAD)
  {
    SLI_DEBUG_PRINT("rf_tx: unsupported protocol: %u", data_protocol);
    return -1;
  }

  switch (rf_radio_state) {
    case RADIO_UNINIT:
      //SLI_DEBUG_PRINT("rf_tx: Radio uninit");
      return -1;
    case RADIO_INITING:
      //SLI_DEBUG_PRINT("rf_tx: Radio initing");
      return -1;
    case RADIO_CALIBRATION:
      //SLI_DEBUG_PRINT("rf_tx: Radio calibrating");
      return -1;
    case RADIO_TX_CSMA:
      /* Falls through on purpose */
    case RADIO_TX_CCA:
      /* Falls through on purpose */
    case RADIO_TX:
      SLI_DEBUG_PRINT("rf_tx: Radio in TX mode");
      return -1;
    case RADIO_IDLE:
    case RADIO_RX:

      platform_enter_critical();

      /* Since we set up Nanostack to give us a 2-byte PHY header, we get the two extras byte at the start of data_ptr
       * and need to populate it with the MAC-frame length (including the 4-byte hardware-inserted CRC)
       *
       * Note that the passed data length does NOT include the PHY header length.
       */
      value16 = RF_SUN_FSK_SET_PHR_DATA_LENGTH(data_length + MAC_PACKET_FCS_LENGTH);
      value16 |= RF_SUN_FSK_SET_PHR_DATA_WHITENING(true);
      common_write_16_bit(value16, &data_ptr[0]);

      /* Load the frame into FIFO */
      RAIL_ResetFifo(rf_rail_handle, true, false);
      value16 = RAIL_WriteTxFifo(rf_rail_handle, data_ptr, data_length + MAC_PACKET_PHR_LENGTH, true);
      SL_ERROR_CHECK_SET_STATUS(status, value16 == (data_length + MAC_PACKET_PHR_LENGTH));

      /* TX handle is needed for callbacks */
      rf_current_tx_handle = tx_handle;

      if (!rf_phy_csma_params.cca_enabled)
      {
        /* If CCA is not required, we can just transmit */
        status = rf_start_tx();
        SL_ERROR_CHECK(status);
      }
      else
      {
        /* Start CSMA-CA timeout */
        status = rf_start_csma_timeout();
        SL_ERROR_CHECK(status);
      }

error_handler:

      if (status < 0)
      {
        /* Abort the transmission attempt. The MAC layer is notified via the return code. */
        rf_abort_tx();
      }

      platform_exit_critical();

      return status;
  }

  //Should never get here...
  return -1;
}

static int8_t rf_start_idle()
{
  /* Force radio to idle */
  RAIL_Idle(rf_rail_handle, RAIL_IDLE_FORCE_SHUTDOWN_CLEAR_FLAGS, true);

  /* Update radio state information */
  rf_radio_state = RADIO_IDLE;

  return 0;
}

static int8_t rf_handle_interface_reset()
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must stop any active TX and RX processes and set radio to idle state."
     */

  //SLI_DEBUG_PRINT("PHY_INTERFACE_RESET");

  /* Disable radio */
  rf_start_idle();

  /* Allow deep sleep */
  rf_allow_platform_sleep();

  return 0;
}

static int8_t rf_handle_interface_down()
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must stop any active TX and RX processes and set radio to sleep or idle state."
     */

  //SLI_DEBUG_PRINT("PHY_INTERFACE_DOWN");

  /* Disable radio */
  rf_start_idle();

  /* Allow deep sleep */
  rf_allow_platform_sleep();

  return 0;
}

static int8_t rf_handle_interface_up(uint8_t new_channel)
{
  int8_t status;

  /**
     * MBED Implementing the PHY API:
     * "Driver must wake up the radio and enable receiver on a channel that was given as parameter."
     */

  SLI_DEBUG_PRINT("PHY_INTERFACE_UP: rf_target_channel: %u", new_channel);

  if (rf_radio_state != RADIO_IDLE)
  {
    //SLI_DEBUG_PRINT("PHY_INTERFACE_UP: radio not idle");

    /* RF radio is not idle */
    return -1;
  }

  if (RAIL_IsValidChannel(rf_rail_handle, new_channel) != RAIL_STATUS_NO_ERROR)
  {
    SLI_DEBUG_PRINT("PHY_INTERFACE_UP: channel %u is not valid", new_channel);

    /* The requested channel is not part of the configured channel set */
    return -1;
  }

  /* Set the requested channel as the target */
  rf_target_channel = new_channel;

  /* Start RX on the requested channel */
  status = rf_start_rx();
  if (status < 0)
  {
    SLI_DEBUG_PRINT("PHY_INTERFACE_UP: rf_start_rx failed");

    /* State change failed */
    return -1;
  }

  /* Prevent deep sleep, RX can only happen in EM0/EM1 */
  rf_deny_platform_sleep();

  return 0;
}

static int8_t rf_handle_interface_rx_energy_state(uint8_t new_channel)
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must initialize energy detection on a channel that was given as parameter."
     */

  //SLI_DEBUG_PRINT("PHY_INTERFACE_RX_ENERGY_STATE: new_channel: %u", new_channel);

  if (rf_radio_state != RADIO_IDLE)
  {
    //SLI_DEBUG_PRINT("PHY_INTERFACE_RX_ENERGY_STATE: radio not idle");

    /* RF radio is not idle */
    return -1;
  }

  if (RAIL_IsValidChannel(rf_rail_handle, new_channel) != RAIL_STATUS_NO_ERROR)
  {
    SLI_DEBUG_PRINT("PHY_INTERFACE_RX_ENERGY_STATE: channel %u is not valid", new_channel);

    /* The requested channel is not part of the configured channel set */
    return -1;
  }

  return -1;
}

static int8_t rf_handle_interface_sniffer_state(uint8_t new_channel)
{
  int8_t status;

  /**
     * MBED Implementing the PHY API:
     * "Driver must enable receiver on a channel that was given as parameter.
     *  All filtering must be disabled."
     */

  //SLI_DEBUG_PRINT("PHY_INTERFACE_SNIFFER_STATE: new_channel: %u", new_channel);

  if (rf_radio_state != RADIO_IDLE)
  {
    //SLI_DEBUG_PRINT("PHY_INTERFACE_SNIFFER_STATE: radio not idle");

    /* RF radio is not idle */
    return -1;
  }

  if (RAIL_IsValidChannel(rf_rail_handle, new_channel) != RAIL_STATUS_NO_ERROR)
  {
    SLI_DEBUG_PRINT("PHY_INTERFACE_SNIFFER_STATE: channel %u is not valid", new_channel);

    /* The requested channel is not part of the configured channel set */
    return -1;
  }

  /* Set the requested channel as the target */
  rf_target_channel = new_channel;

  /* Start RX on the requested channel */
  status = rf_start_rx();
  if (status < 0)
  {
    //SLI_DEBUG_PRINT("PHY_INTERFACE_SNIFFER_STATE: rf_start_rx failed");

    /* State change failed */
    return -1;
  }

  /* Prevent deep sleep, RX can only happen in EM0/EM1 */
  rf_deny_platform_sleep();

  return 0;
}

/*
 * \brief Function gives the control of RF states to the MAC layer.
 *
 * \param new_state RF state
 * \param new_channel RF channel
 *
 * \return 0 Success
 */
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t new_channel)
{
  switch (new_state) {
    case PHY_INTERFACE_RESET:
      return rf_handle_interface_reset();
    case PHY_INTERFACE_DOWN:
      return rf_handle_interface_down();
    case PHY_INTERFACE_UP:
      return rf_handle_interface_up(new_channel);
    case PHY_INTERFACE_RX_ENERGY_STATE:
      return rf_handle_interface_rx_energy_state(new_channel);
    case PHY_INTERFACE_SNIFFER_STATE:
      return rf_handle_interface_sniffer_state(new_channel);
  }

  //SLI_DEBUG_PRINT("rf_interface_state_control: unhandled state: %u", new_state);
  return -1;
}

static int8_t rf_handle_extension_ctrl_pending_bit(uint8_t *data_ptr)
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must set frame pending bit of the Ack frames high or low depending
     *  on the value of given (uint8_t *) parameter. If parameter > 0, set frame
     *  pending bit to 1 and otherwise to 0."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_CTRL_PENDING_BIT");

  if (*data_ptr) {
    data_pending = true;
  } else {
    data_pending = false;
  }

  return 0;
}

static int8_t rf_handle_extension_read_last_ack_pending_status(uint8_t *data_ptr)
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must return the state of the frame pending bit used in last transmitted
     *  Ack frame. State (0 or 1) must be written to given (uint8_t *) parameter."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS");

  if (last_ack_pending_bit) {
    *data_ptr = true;
  } else {
    *data_ptr = false;
  }

  return 0;
}

static int8_t rf_handle_extension_read_channel_energy(uint8_t *data_ptr)
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must return the read channel energy. Value must be written in given
     *  (uint8_t *) parameter as an 8-bit integer."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_READ_CHANNEL_ENERGY");

  *data_ptr = 0;

  return -1;
}


static int8_t rf_handle_extension_accept_any_beacon(uint8_t *data_ptr)
{
  (void)data_ptr;

  /**
     * MBED Implementing the PHY API:
     * "Driver must stop filtering received 802.15.4 Beacon frames and accept them
     *  all if the given (uint8_t *) parameter is > 0."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_ACCEPT_ANY_BEACON");

  return -1;
}


static int8_t rf_handle_extension_set_channel(uint8_t *data_ptr)
{
  uint8_t new_channel;

  /**
     * MBED Implementing the PHY API:
     * "Driver must enable receiver on a channel which was given as parameter.
     * If radio is currently receiving or transmitting a frame, channel should
     * be changed after the TX/RX process."
     */

  new_channel = *data_ptr;
  if (RAIL_IsValidChannel(rf_rail_handle, new_channel) != RAIL_STATUS_NO_ERROR)
  {
    SLI_DEBUG_PRINT("PHY_EXTENSION_SET_CHANNEL: channel %u is not valid", new_channel);

    /* The requested channel is not part of the configured channel set */
    return -1;
  }

  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_CHANNEL: %u - > %u @ 0x%08lx", rf_channel, new_channel, RAIL_GetTime());

  /* Set the requested channel as the target */
  rf_target_channel = new_channel;

  if (rf_radio_state == RADIO_TX) {
    /* Radio is transmitting a frame, ignore channel change. */
    rf_target_channel = rf_channel;

    /* Return an error to the MAC layer to indicate the channel was not changed. */
    return -1;
  } else if ((rf_radio_state == RADIO_TX_CCA) && (rf_phy_csma_status == PHY_TX_ALLOWED)) {
    /* If the ongoing CCA indicates clear, TX will be started immediately.
     * TX must use the same channel as CCA, ignore channel change.
     *
     * This condition should only occur when FHSS timer attempts to change
     * the RX channel during TX.
     */
    rf_target_channel = rf_channel;

    /* Return an error to the MAC layer to indicate the channel was not changed. */
    return -1;
  } else if ((rf_radio_state == RADIO_TX_CSMA) || (rf_radio_state == RADIO_TX_CCA)) {
    /**
     * Channel change will be handled inside the ongoing TX operation.
     * Report a success to the MAC layer.
     */
    return 0;
  } else {
    /* Start RX on the requested channel */
    return rf_start_rx();
  }
}

static int8_t rf_handle_extension_read_rx_time(uint8_t *data_ptr)
{
  uint32_t value32;

  /**
     * MBED Implementing the PHY API:
     * "Driver must return the time stamp of last received packet. Time stamp must
     *  be referenced to first byte after SFD field of the received packet."
     *
     * Note that MBED expects the timestamp to be in MSB order.
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_READ_RX_TIME");

  value32 = rxPacketDetails.timeReceived.packetTime;
  common_write_32_bit(value32, data_ptr);

  return 0;
}

static int8_t rf_handle_extension_dynamic_rf_supported(uint8_t *data_ptr)
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must return 1 if it supports extended RF driver implementation.
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_DYNAMIC_RF_SUPPORTED");

  *data_ptr = true;

  return 0;
}

static int8_t rf_handle_extension_get_timestamp(uint8_t *data_ptr)
{
  uint32_t value32;

  /**
     * MBED Implementing the PHY API:
     * "Driver must return 32-bit time stamp by writing it to given (uint8_t *) parameter."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_GET_TIMESTAMP");

  value32 = RAIL_GetTime();
  common_write_32_bit_inverse(value32, data_ptr);

  return 0;
}

static int8_t rf_handle_extension_set_csma_parameters(uint8_t *data_ptr)
{
  /**
     * MBED Implementing the PHY API:
     * "Driver must read phy_csma_params_t type structure behind given (uint8_t *) parameter.
     *  If parameter backoff_time is 0, any on-going transmission must be canceled and radio
     *  set to receive state on the current channel. 32-bit backoff_time and boolean cca_enabled
     *  must be stored because they will be used for next packet transmission."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_CSMA_PARAMETERS");

  /* Take a copy of the CSMA-CA parameters */
  memcpy(&rf_phy_csma_params, data_ptr, sizeof(rf_phy_csma_params));

  if (!rf_phy_csma_params.backoff_time)
  {
    /* The MAC layer will set backoff_time to 0 in order to abort any ongoing TX */
    rf_abort_tx();
  }

  return 0;
}

static int8_t rf_handle_extension_get_symbols_per_second(uint8_t *data_ptr)
{
  uint32_t value32;

  /**
     * MBED Implementing the PHY API:
     * "Driver must return symbol rate as symbols per second. 32-bit value
     *  must be written to given (uint8_t *) parameter."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_GET_SYMBOLS_PER_SECOND");

  value32 =  RAIL_GetSymbolRate(rf_rail_handle);
  common_write_32_bit_inverse(value32, data_ptr);

  return 0;
}

static int8_t rf_handle_extension_set_rf_configuration(uint8_t *data_ptr)
{
  const RAIL_ChannelConfig_t *iter;
  RAIL_Status_t ret;
  uint32_t datarate;
  uint16_t number_of_channels;
  int index = 0;

  /**
     * MBED Implementing the PHY API:
     * "Driver must read phy_rf_channel_configuration_s type structure behind
     *  given (uint8_t *) parameter. Driver must start using given RF configuration
     *  as soon as possible. If radio is currently receiving or transmitting a frame,
     *  configuration should be changed after the TX/RX process."
     */

  SLI_DEBUG_PRINT("PHY_EXTENSION_SET_RF_CONFIGURATION");

  /* Take a copy of the RF PHY configuration */
  memcpy(&rf_phy_config, data_ptr, sizeof(rf_phy_config));

  /* Iterate through available RAIL configurations to find
   * a matching configuration
   */

  iter = channelConfigs[index];
  while (iter) {
    ret = RAIL_ConfigChannels(rf_rail_handle, iter, NULL);
    if (ret == RAIL_STATUS_NO_ERROR) {
      number_of_channels = iter->configs->channelNumberEnd - iter->configs->channelNumberStart + 1;
      datarate = RAIL_GetBitRate(rf_rail_handle);

      if(datarate) {
        if ((rf_phy_config.channel_0_center_frequency == iter->configs->baseFrequency) &&
            (rf_phy_config.channel_spacing == iter->configs->channelSpacing) &&
            (rf_phy_config.datarate == datarate) &&
            (rf_phy_config.number_of_channels == number_of_channels)) {
          SLI_DEBUG_PRINT("Using RAIL configuration index #%u", index);

          /* RAIL configuration matches the requested RF PHY configuration */
          break;
        }
      }
    }
    iter = channelConfigs[++index];
  }

  if (!iter) {
    SLI_DEBUG_PRINT("No matching RAIL configuration");

    /* No matching configuration found */
    return -1;
  }

  /* Configure PA */
  if (rf_phy_config.channel_0_center_frequency >= 2400000000) {
    ret = RAIL_ConfigTxPower(rf_rail_handle, &rf_rail_tx_power_2p4);
  } else {
    ret = RAIL_ConfigTxPower(rf_rail_handle, &rf_rail_tx_power_subghz);
  }

  if (ret != RAIL_STATUS_NO_ERROR) {
    SLI_DEBUG_PRINT("RAIL PA configuration failed: %u", ret);

    return -1;
  }

  /* Configure TX power */
  if (rf_device_set_tx_power(rf_tx_power) < 0) {
    SLI_DEBUG_PRINT("RAIL TX power configuration failed");

    return -1;
  }

  return 0;
}

static int8_t rf_handle_extension_filtering_support(uint8_t *data_ptr)
{
  /**
     * MBED Implementing the PHY API:
     * "If RF driver can support filtering and acking certain MAC frame types,
     *  it can set the corresponding bit in a given (uint8_t *) parameter to 1,
     *  which disables the filtering of this frame type from Nanostack."
     */

  //SLI_DEBUG_PRINT("PHY_EXTENSION_FILTERING_SUPPORT");

  *data_ptr = 0; /* Filtering not supported */

  return 0;
}

static int8_t rf_handle_extension_read_link_status(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_READ_LINK_STATUS");

  return -1;
}

static int8_t rf_handle_extension_convert_signal_info(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_CONVERT_SIGNAL_INFO");

  return -1;
}

static int8_t rf_handle_extension_set_tx_time(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_TX_TIME");

  return -1;
}

static int8_t rf_handle_extension_set_tx_finish_time(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_READ_TX_FINNISH_TIME");

  return -1;
}

static int8_t rf_handle_extension_set_tx_power(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_TX_POWER");

  return 0;
}

static int8_t rf_handle_extension_set_cca_threshold(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_CCA_THRESHOLD");

  return 0;
}

static int8_t rf_handle_extension_set_channel_cca_threshold(uint8_t *data_ptr)
{
  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_CHANNEL_CCA_THRESHOLD");

  rf_rail_csma_config.ccaThreshold = (int8_t)*data_ptr;

  return 0;
}

static int8_t rf_handle_extension_set_802_15_4_mode(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_802_15_4_MODE");

  return 0;
}

static int8_t rf_handle_extension_set_data_whitening(uint8_t *data_ptr)
{
  (void)data_ptr;

  //SLI_DEBUG_PRINT("PHY_EXTENSION_SET_DATA_WHITENING");

  return 0;
}

/*
 * \brief Function controls the ACK pending, channel setting and energy detection.
 *
 * \param extension_type Type of control
 * \param data_ptr Data from NET library
 *
 * \return 0 Success
 */
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
  switch (extension_type)
  {
    case PHY_EXTENSION_CTRL_PENDING_BIT:
      return rf_handle_extension_ctrl_pending_bit(data_ptr);
    case PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS:
      return rf_handle_extension_read_last_ack_pending_status(data_ptr);
    case PHY_EXTENSION_SET_CHANNEL:
      return rf_handle_extension_set_channel(data_ptr);
    case PHY_EXTENSION_READ_CHANNEL_ENERGY:
      return rf_handle_extension_read_channel_energy(data_ptr);
    case PHY_EXTENSION_READ_LINK_STATUS:
      return rf_handle_extension_read_link_status(data_ptr);
    case PHY_EXTENSION_CONVERT_SIGNAL_INFO:
      return rf_handle_extension_convert_signal_info(data_ptr);
    case PHY_EXTENSION_ACCEPT_ANY_BEACON:
      return rf_handle_extension_accept_any_beacon(data_ptr);
    case PHY_EXTENSION_SET_TX_TIME:
      return rf_handle_extension_set_tx_time(data_ptr);
    case PHY_EXTENSION_READ_RX_TIME:
      return rf_handle_extension_read_rx_time(data_ptr);
    case PHY_EXTENSION_READ_TX_FINNISH_TIME:
      return rf_handle_extension_set_tx_finish_time(data_ptr);
    case PHY_EXTENSION_DYNAMIC_RF_SUPPORTED:
      return rf_handle_extension_dynamic_rf_supported(data_ptr);
    case PHY_EXTENSION_GET_TIMESTAMP:
      return rf_handle_extension_get_timestamp(data_ptr);
    case PHY_EXTENSION_SET_CSMA_PARAMETERS:
      return rf_handle_extension_set_csma_parameters(data_ptr);
    case PHY_EXTENSION_GET_SYMBOLS_PER_SECOND:
      return rf_handle_extension_get_symbols_per_second(data_ptr);
    case PHY_EXTENSION_SET_RF_CONFIGURATION:
      return rf_handle_extension_set_rf_configuration(data_ptr);
    case PHY_EXTENSION_FILTERING_SUPPORT:
      return rf_handle_extension_filtering_support(data_ptr);
    case PHY_EXTENSION_SET_TX_POWER:
      return rf_handle_extension_set_tx_power(data_ptr);
    case PHY_EXTENSION_SET_CCA_THRESHOLD:
      return rf_handle_extension_set_cca_threshold(data_ptr);
    case PHY_EXTENSION_SET_CHANNEL_CCA_THRESHOLD:
      return rf_handle_extension_set_channel_cca_threshold(data_ptr);
    case PHY_EXTENSION_SET_802_15_4_MODE:
      return rf_handle_extension_set_802_15_4_mode(data_ptr);
    case PHY_EXTENSION_SET_DATA_WHITENING:
      return rf_handle_extension_set_data_whitening(data_ptr);
  }

  //SLI_DEBUG_PRINT("rf_extension: unhandled extension: %u", extension_type);
  return -1;
}

/*
 * \brief Function sets the addresses to RF address filters.
 *
 * \param address_type Type of address
 * \param address_ptr Pointer to given address
 *
 * \return 0 Success
 */
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{
  int8_t ret_val = 0;
  switch (address_type)
  {
    /*Set 48-bit address*/
    case PHY_MAC_48BIT:
      // 15.4 does not support 48-bit addressing
      ret_val = -1;
      break;
      /*Set 64-bit MAC address*/
    case PHY_MAC_64BIT:
      /* Store MAC in MSB order */
      memcpy(rf_mac_address, address_ptr, 8);
      tr_debug("rf_address_write: PHY_MAC_64BIT: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
          rf_mac_address[0], rf_mac_address[1], rf_mac_address[2], rf_mac_address[3],
          rf_mac_address[4], rf_mac_address[5], rf_mac_address[6], rf_mac_address[7]);
      break;
    default:
      ret_val = -1;
      break;
  }
  return ret_val;
}

/*****************************************************************************/
/*****************************************************************************/

static void rf_if_lock(void)
{
  platform_enter_critical();
}

static void rf_if_unlock(void)
{
  platform_exit_critical();
}

//====================== RAIL-defined callbacks =========================
/**
 * Callback that lets the app know when the radio has finished init
 * and is ready.
 */
static void RAILCb_RfReady(RAIL_Handle_t handle) {
  (void)handle;

  rf_radio_state = RADIO_IDLE;
}

// An empty callback to prevent RAIL from using a built-in RX FIFO
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t railHandle) {
  (void)railHandle;
  return RAIL_STATUS_NO_ERROR;
}

static void rf_handle_event_rx_success()
{
  /* Get RX packet that got signaled */
  RAIL_RxPacketInfo_t rxPacketInfo;
  RAIL_RxPacketHandle_t rxHandle = RAIL_GetRxPacketInfo(rf_rail_handle,
                                                        RAIL_RX_PACKET_HANDLE_NEWEST,
                                                        &rxPacketInfo);

  /* Only process the packet if it had a correct CRC */
  if (rxPacketInfo.packetStatus == RAIL_RX_PACKET_READY_SUCCESS) {
    /* Get RSSI and LQI information about this packet */
    rxPacketDetails.timeReceived.timePosition = RAIL_PACKET_TIME_AT_SYNC_END_USED_TOTAL;
    rxPacketDetails.timeReceived.totalPacketBytes = rxPacketInfo.packetBytes - MAC_PACKET_INFO_LENGTH + MAC_PACKET_FCS_LENGTH;
    RAIL_GetRxPacketDetails(rf_rail_handle, rxHandle, &rxPacketDetails);

    /* Drop this packet if we're out of space */
    if (((rx_queue_head + 1) % RF_QUEUE_SIZE) == rx_queue_tail) {
      RAIL_ReleaseRxPacket(rf_rail_handle, rxHandle);
    }

    /* Point to a free slot in the queue */
    uint8_t *packetBuffer = (uint8_t*)rx_queue[rx_queue_head];

    /* First two bytes are RSSI and LQI, respectively */
    packetBuffer[MAC_PACKET_OFFSET_RSSI] = (uint8_t)rxPacketDetails.rssi;
    packetBuffer[MAC_PACKET_OFFSET_LQI] = (uint8_t)rxPacketDetails.lqi;

    /* Copy packet payload from circular FIFO into contiguous memory */
    RAIL_CopyRxPacket(&packetBuffer[MAC_PACKET_INFO_LENGTH], &rxPacketInfo);

    /* Release RAIL resources early */
    RAIL_ReleaseRxPacket(rf_rail_handle, rxHandle);

    rx_queue_head = (rx_queue_head + 1) % RF_QUEUE_SIZE;
    assert((osEventFlagsSet(rf_task_flag_group,
                            RF_TASK_FLAG_RX_SUCCESS) & CMSIS_RTOS_ERROR_MASK) == 0);
  } else {
    //SLI_DEBUG_PRINT("RAIL_EVENT_RX_PACKET_RECEIVED: %u", rxPacketInfo.packetStatus);
  }
}

static void rf_handle_event_rx_failure()
{
  assert((osEventFlagsSet(rf_task_flag_group,
                          RF_TASK_FLAG_RX_FAILURE) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void rf_handle_event_tx_success()
{
  assert((osEventFlagsSet(rf_task_flag_group,
                          RF_TASK_FLAG_TX_SUCCESS) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void rf_handle_event_cca_clear()
{
  assert((osEventFlagsSet(rf_task_flag_group,
                          RF_TASK_FLAG_CCA_CLEAR) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void rf_handle_event_cca_busy()
{
  assert((osEventFlagsSet(rf_task_flag_group,
                          RF_TASK_FLAG_CCA_BUSY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void rf_handle_event_unexpected()
{
  assert((osEventFlagsSet(rf_task_flag_group,
                          RF_TASK_FLAG_UNEXPECTED) & CMSIS_RTOS_ERROR_MASK) == 0);
}

/**
 * Event handler for RAIL-fired events. Usually gets called from IRQ context.
 * Due to IRQ latency and tailchaining, multiple event flags might be set simultaneously,
 * so we have to check all of them */
static void radioEventHandler(RAIL_Handle_t railHandle,
                              RAIL_Events_t events)
{
  /* RAIL_Events_t is a 64-bit event mask, but a thread only supports 32
     * signal bits. This means we have to convert from a RAIL event mask to
     * our own custom event mask.
     */
  if (railHandle != rf_rail_handle)
    return;

  size_t index = 0;
  do {
    if (events & 1ull) {
      switch (index) {
        case RAIL_EVENT_RX_PACKET_RECEIVED_SHIFT:
          rf_handle_event_rx_success();
          break;
        case RAIL_EVENT_RX_FRAME_ERROR_SHIFT:
          /* Falls through on purpose */
        case RAIL_EVENT_RX_FIFO_OVERFLOW_SHIFT:
          /* Falls through on purpose */
        case RAIL_EVENT_RX_ADDRESS_FILTERED_SHIFT:
          /* Falls through on purpose */
        case RAIL_EVENT_RX_PACKET_ABORTED_SHIFT:
          /* Falls through on purpose */
        case RAIL_EVENT_RX_SCHEDULED_RX_MISSED_SHIFT:
          rf_handle_event_rx_failure();
          break;
        case RAIL_EVENT_TX_PACKET_SENT_SHIFT:
          rf_handle_event_tx_success();
          break;
        case RAIL_EVENT_TX_CHANNEL_CLEAR_SHIFT:
          rf_handle_event_cca_clear();
          break;
        case RAIL_EVENT_TX_CHANNEL_BUSY_SHIFT:
          rf_handle_event_cca_busy();
          break;
        case RAIL_EVENT_TX_UNDERFLOW_SHIFT:
          /* Falls through on purpose */
        case RAIL_EVENT_TXACK_UNDERFLOW_SHIFT:
          /* Falls through on purpose */
        case RAIL_EVENT_CAL_NEEDED_SHIFT:
          rf_handle_event_unexpected();
          break;
        default:
          break;
      }
    }
    events = events >> 1;
    index += 1;
  }
  while (events != 0);
}

void sli_wisun_driver_init(void)
{
  tr_debug("sli_wisun_driver_init()");

  rf_device_init();

  tr_debug("sli_wisun_driver_init() - done");
}

void sli_wisun_driver_register(int8_t *driver_id)
{
  tr_debug("sli_wisun_driver_register()");

  rf_if_lock();

  *driver_id = rf_device_register();

  rf_if_unlock();

  tr_debug("sli_wisun_driver_register() - driver_id: %d", *driver_id);
}

void sli_wisun_driver_set_pti_state(bool enable)
{
  tr_debug("sli_wisun_driver_set_pti_state()");

  rf_device_set_pti_state(enable);

  tr_debug("sli_wisun_driver_set_pti_state() - done");
}

void sli_wisun_driver_set_tx_power(int8_t tx_power)
{
  tr_debug("sli_wisun_driver_set_tx_power()");

  rf_device_set_tx_power(tx_power);

  tr_debug("sli_wisun_driver_set_tx_power() - done");
}

RAIL_Handle_t sli_wisun_driver_get_rail_handle(void)
{
  return rf_rail_handle;
}