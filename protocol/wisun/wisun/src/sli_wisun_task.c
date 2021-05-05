/***************************************************************************//**
 * @file sli_wisun_task.h
 * @brief Wi-SUN task API
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

#include "nsconfig.h"
#include <string.h>
#include "em_common.h"
#include <assert.h>
#include <cmsis_os2.h>
#include "sl_cmsis_os2_common.h"
#include "mac_api.h"
#include "mac_filter_api.h"
#include "sw_mac.h"
#include "sli_wisun_trace.h"
#include "SEGGER_RTT.h"
#include "ns_hal_init.h"
#include "fhss_config.h"
#include "sli_wisun_nvm.h"
#include "ws_management_api.h"
#include "socket_api.h"
#include "sli_wisun_driver.h"
#include "sl_wisun_api.h"
#include "eventOS_event.h"
#include "eventOS_event_timer.h"
#include "eventOS_scheduler.h"
#include "ip6string.h"
#include "NWK_INTERFACE/Include/protocol.h"
#include "RPL/rpl_control.h"
#include "net_rpl.h"
#include "sl_slist.h"
#include "sl_malloc.h"
#include "nwk_stats_api.h"
#include "net_fhss.h"
#include "6LoWPAN/ws/ws_common.h"
#include "Core/include/ns_socket.h"
#include "sl_wisun_version.h"
#include "sli_wisun_task.h"
#include "sl_wisun_common.h"
#include "mbedtls/pk.h"
#include "mbedtls/x509_crt.h"
#include "Common_Protocols/ipv6_constants.h"
#include "6LoWPAN/MAC/mac_helper.h"

#ifdef __ICCARM__
#include <iar_dlmalloc.h>
#else
#include <malloc.h>
#endif

#if defined(SL_COMPONENT_CATALOG_PRESENT)
#include "sl_component_catalog.h"
#endif

#if defined(SL_CATALOG_MICRIUMOS_KERNEL_PRESENT)
#include <kernel/include/os.h>
#endif

#define TRACE_GROUP "wisun"
#define SLI_WISUN_TASK_PRIORITY (osPriority_t)44
#define SLI_WISUN_TASK_STACK_SIZE 500 // in units of CPU_INT32U
#define SLI_WISUN_EVENT_TASK_PRIORITY (osPriority_t)42
#define SLI_WISUN_EVENT_TASK_STACK_SIZE 500 // in units of CPU_INT32U

#define SLI_WISUN_TASK_MAX_API_EVENTS 4
#define SLI_WISUN_TASK_MAX_CERTIFICATE_ENTRIES 4
#define SLI_WISUN_TASK_MAX_SOCKETS 10
#define SLI_WISUN_TASK_MAX_ACCESS_LIST_ENTRIES 10

#define SLI_WISUN_TASK_FLAG_NONE          (0)
#define SLI_WISUN_TASK_FLAG_READY         (1) // Task is ready to process the next request
#define SLI_WISUN_TASK_FLAG_REQ_READY     (2) // A request is ready to be served
#define SLI_WISUN_TASK_FLAG_CNF_READY     (4) // A confirmation is ready to be served
#define SLI_WISUN_TASK_FLAG_IND_READY     (8) // An indication is ready to be served

#define SLI_WISUN_SETTING_FHSS_CHANNEL_MASK_DEFAULT \
  0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,\
  0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
#define SLI_WISUN_SETTING_FHSS_UC_DWELL_INTERVAL_DEFAULT 255
#define SLI_WISUN_SETTING_FHSS_BC_DWELL_INTERVAL_DEFAULT 255
#define SLI_WISUN_SETTING_FHSS_BC_INTERVAL_DEFAULT 1020
#define SLI_WISUN_SETTING_FHSS_CHANNEL_FUNCTION_DEFAULT CHANNEL_FUNCTION_DH1CF
#define SLI_WISUN_SETTING_NETWORK_SIZE_DEFAULT NETWORK_SIZE_SMALL
#define SLI_WISUN_SETTING_PHY_CH0_FREQUENCY_DEFAULT 0
#define SLI_WISUN_SETTING_PHY_NUMBER_OF_CHANNELS_DEFAULT 0
#define SLI_WISUN_SETTING_PHY_CHANNEL_SPACING_DEFAULT 0
#define SLI_WISUN_SETTING_DISCOVERY_DEFAULT 0
#define SLI_WISUN_SETTING_DISCOVERY_IMIN_FAST 5
#define SLI_WISUN_SETTING_DISCOVERY_IMAX_FAST 15
#define SLI_WISUN_SETTING_PAN_TIMEOUT_DEFAULT 0
#define SLI_WISUN_SETTING_SECURITY_DEFAULT 0
#define SLI_WISUN_SETTING_SECURITY_INITIAL_MIN_FAST 2
#define SLI_WISUN_SETTING_SECURITY_INITIAL_MAX_FAST 3
#define SLI_WISUN_SETTING_GTK_MAX_MISMATCH_DEFAULT 0
#define SLI_WISUN_SETTING_GTK_MAX_MISMATCH_CERTIF 1

#define SLI_WISUN_CHANNEL_NONE 65535
#define SLI_WISUN_CHANNEL_MASK_SIZE 8

#define SLI_WISUN_SOCKET_PORT_AUTOMATIC 0
#define SLI_WISUN_SOCKET_BACKLOG 2
#define SLI_WISUN_SOCKET_UDP_SNDBUF_SIZE 2048
#define SLI_WISUN_SOCKET_TCP_SNDBUF_SIZE 2048
#define SLI_WISUN_SOCKET_ICMP_SNDBUF_SIZE 2048
#define SLI_WISUN_SOCKET_UDP_RCVBUF_SIZE 2048
#define SLI_WISUN_SOCKET_TCP_RCVBUF_SIZE 2048
#define SLI_WISUN_SOCKET_ICMP_RCVBUF_SIZE 2048
#define SLI_WISUN_SOCKET_MULTICAST_HOP_LIMIT 255
#define SLI_WISUN_INTERFACE_ID_NONE -1

#define SLI_WISUN_SOCKET_FLAG_NONE    (0)
#define SLI_WISUN_SOCKET_FLAG_POLLING (1 << 0)

#define SLI_WISUN_EVENT_INFO_FLAG_NONE    (0)
#define SLI_WISUN_EVENT_INFO_FLAG_DYNAMIC (1 << 0)

#define SLI_WISUN_CERTIFICATE_FLAG_NONE         (0)
#define SLI_WISUN_CERTIFICATE_FLAG_HAS_KEY      (1 << 0)
#define SLI_WISUN_CERTIFICATE_FLAG_IS_OWNED     (1 << 1)
#define SLI_WISUN_CERTIFICATE_FLAG_KEY_IS_OWNED (1 << 2)

#define SLI_WISUN_ACCESS_LIST_ACTION_DENY 0
#define SLI_WISUN_ACCESS_LIST_ACTION_ALLOW 1

#define SLI_WISUN_ERROR_CHECK(__result)\
do {\
  if (!(__result)){\
    goto error_handler;\
  }\
} while(0)

#define SLI_WISUN_ERROR_CHECK_SET_STATUS(__result, __value)\
do {\
  if (!(__result)){\
    status = __value;\
    goto error_handler;\
  }\
} while(0)

#define SLI_WISUN_ERROR_SET_STATUS(__value)\
do {\
  status = __value;\
  goto error_handler;\
} while(0)

typedef enum {
  SLI_WISUN_TASK_STATE_INITIALIZED, // 0
  SLI_WISUN_TASK_STATE_CONNECTING,  // 1
  SLI_WISUN_TASK_STATE_CONNECTED    // 2
} sli_wisun_task_state_t;

typedef enum {
  SLI_WISUN_APPLICATION_EVENT_STATISTICS = 0
} sli_wisun_application_event_t;

typedef struct {
  uint32_t channel_mask[SLI_WISUN_CHANNEL_MASK_SIZE];
  uint32_t broadcast_interval;
  uint8_t uc_dwell_interval;
  uint8_t bc_dwell_interval;
  uint8_t channel_function;
} sli_wisun_settings_fhss_t;

typedef struct {
  uint32_t ch0_frequency;
  uint16_t number_of_channels;
  uint8_t channel_spacing;
} sli_wisun_settings_phy_t;

typedef struct {
  uint16_t discovery_trickle_imin;
  uint16_t discovery_trickle_imax;
  uint16_t security_initial_min;
  uint16_t security_initial_max;
  uint16_t gtk_max_mismatch;
} sli_wisun_settings_timing_t;

typedef struct {
  sli_wisun_settings_fhss_t fhss;
  sli_wisun_settings_phy_t phy;
  sli_wisun_settings_timing_t timing;
  uint8_t network_size;
} sli_wisun_settings_t;

typedef struct {
  sl_slist_node_t node;
  uint32_t flags;
  arm_certificate_entry_s certificate;
} sli_wisun_certificate_entry_t;

typedef struct {
  nwk_stats_t network;
  ws_statistics_t wisun;
  fhss_statistics_t fhss;
  mac_statistics_t mac;
  phy_rf_statistics_s phy;
} sli_wisun_statistics_t;

typedef struct
{
  sl_slist_node_t node;
  sl_wisun_socket_id_t socket_id;
  uint32_t flags;
  uint16_t sndbuf_limit;
  uint16_t sndbuf_size;
} sli_wisun_socket_t;

typedef struct
{
  uint16_t action;
  uint16_t list_count;
  sl_wisun_mac_address_t list[SLI_WISUN_TASK_MAX_ACCESS_LIST_ENTRIES];
} sli_wisun_access_list_t;

typedef struct {
  void* addr;
  uint16_t length;
} sli_wisun_queue_descriptor_t;

static sli_wisun_statistics_t sli_wisun_statistics;

static osThreadId_t sli_wisun_task_id;
__ALIGNED(8) static uint8_t sli_wisun_task_tcb[osThreadCbSize];
__ALIGNED(8) static uint8_t sli_wisun_task_stack[(SLI_WISUN_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];

static osThreadId_t sli_wisun_event_task_id;
__ALIGNED(8) static uint8_t sli_wisun_event_task_tcb[osThreadCbSize];
__ALIGNED(8) static uint8_t sli_wisun_event_task_stack[(SLI_WISUN_EVENT_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];

osEventFlagsId_t sli_wisun_task_flags;
__ALIGNED(8) static uint8_t sli_wisun_task_flags_cb[osEventFlagsCbSize];

static osMessageQueueId_t sli_wisun_task_api_events_queue;

static osMutexId_t sli_wisun_task_trace_mutex;

static int8_t sli_wisun_driver_id = -1;

static int sli_wisun_interface_id = SLI_WISUN_INTERFACE_ID_NONE;

static const void *sli_wisun_pending_req;
static const void *sli_wisun_pending_req_data;
static void *sli_wisun_pending_cnf;
static void *sli_wisun_pending_cnf_data;

extern fhss_timer_t fhss_functions;

static int8_t sli_wisun_task_event_handler_id;

static const uint32_t SLI_WISUN_CHANNEL_MASK_ALL[SLI_WISUN_CHANNEL_MASK_SIZE] =
{
  0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
  0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

static const sl_wisun_mac_address_t SLI_WISUN_BROADCAST_MAC =
{
  .address =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  }
};

static sli_wisun_settings_t sli_wisun_settings =
{
  .fhss =
  {
    .channel_mask =
    {
      SLI_WISUN_SETTING_FHSS_CHANNEL_MASK_DEFAULT
    },
    .broadcast_interval = SLI_WISUN_SETTING_FHSS_BC_INTERVAL_DEFAULT,
    .uc_dwell_interval = SLI_WISUN_SETTING_FHSS_UC_DWELL_INTERVAL_DEFAULT,
    .bc_dwell_interval = SLI_WISUN_SETTING_FHSS_BC_DWELL_INTERVAL_DEFAULT,
    .channel_function = SLI_WISUN_SETTING_FHSS_CHANNEL_FUNCTION_DEFAULT
  },
  .phy =
  {
    .ch0_frequency = SLI_WISUN_SETTING_PHY_CH0_FREQUENCY_DEFAULT,
    .number_of_channels = SLI_WISUN_SETTING_PHY_NUMBER_OF_CHANNELS_DEFAULT,
    .channel_spacing = SLI_WISUN_SETTING_PHY_CHANNEL_SPACING_DEFAULT
  },
  .timing =
  {
    .discovery_trickle_imin = SLI_WISUN_SETTING_DISCOVERY_DEFAULT,
    .discovery_trickle_imax = SLI_WISUN_SETTING_DISCOVERY_DEFAULT,
    .security_initial_min = SLI_WISUN_SETTING_SECURITY_DEFAULT,
    .security_initial_max = SLI_WISUN_SETTING_SECURITY_DEFAULT,
    .gtk_max_mismatch = SLI_WISUN_SETTING_GTK_MAX_MISMATCH_DEFAULT
  },
  .network_size = SLI_WISUN_SETTING_NETWORK_SIZE_DEFAULT
};

static sli_wisun_task_state_t sli_wisun_task_state;

static sl_slist_node_t *sli_wisun_certificate_entry_list;
static sl_slist_node_t *sli_wisun_trusted_certificate_list;
static sl_slist_node_t *sli_wisun_certificate_list;
static sli_wisun_certificate_entry_t sli_wisun_certificate_entries[SLI_WISUN_TASK_MAX_CERTIFICATE_ENTRIES];

static sl_slist_node_t *sli_wisun_socket_list_free;
static sl_slist_node_t *sli_wisun_socket_list;
static sli_wisun_socket_t sli_wisun_sockets[SLI_WISUN_TASK_MAX_SOCKETS];
static sli_wisun_access_list_t sli_wisun_access_list;

static sl_wisun_mac_address_t sli_wisun_mac_address;

const char * const SLI_WISUN_SOCKET_EVENT_STR[] =
{
  "SOCKET_DATA",
  "SOCKET_CONNECT_DONE",
  "SOCKET_CONNECT_FAIL",
  "SOCKET_CONNECT_AUTH_FAIL",
  "SOCKET_INCOMING_CONNECTION",
  "SOCKET_TX_FAIL",
  "SOCKET_CONNECT_CLOSED",
  "SOCKET_CONNECTION_RESET",
  "SOCKET_NO_ROUTE",
  "SOCKET_TX_DONE",
  "SOCKET_NO_RAM",
  "SOCKET_CONNECTION_PROBLEM"
};

const char * const SLI_WISUN_NWK_STATUS_STR[] =
{
  "ARM_NWK_BOOTSTRAP_READY",
  "ARM_NWK_RPL_INSTANCE_FLOODING_READY",
  "ARM_NWK_SET_DOWN_COMPLETE",
  "ARM_NWK_NWK_SCAN_FAIL",
  "ARM_NWK_IP_ADDRESS_ALLOCATION_FAIL",
  "ARM_NWK_DUPLICATE_ADDRESS_DETECTED",
  "ARM_NWK_AUHTENTICATION_START_FAIL",
  "ARM_NWK_AUHTENTICATION_FAIL",
  "ARM_NWK_NWK_CONNECTION_DOWN",
  "ARM_NWK_NWK_PARENT_POLL_FAIL",
  "ARM_NWK_PHY_CONNECTION_DOWN",
  "ARM_NWK_BOOTSTRAP_STATE_CHANGED"
};

const char * const SLI_WISUN_EVENT_STR[] =
{
  "ARM_LIB_TASKLET_INIT_EVENT",
  "ARM_LIB_NWK_INTERFACE_EVENT",
  "ARM_LIB_SYSTEM_TIMER_EVENT",
  "APPLICATION_EVENT"
};
#if defined(SL_CATALOG_MICRIUMOS_KERNEL_PRESENT)
#if (OS_CFG_TASK_STK_REDZONE_EN == DEF_ENABLED)
static void sli_wisun_stack_overflow(OS_TCB *p_tcb);
#endif
#endif
static void sli_wisun_print_trace(const char *str);
static sl_wisun_evt_t *sli_wisun_task_allocate_event(uint8_t event_id, uint16_t event_length);
static void sli_wisun_task_free_event(sl_wisun_evt_t *event);
static sli_wisun_certificate_entry_t *sli_wisun_task_allocate_certificate_entry();
static void sli_wisun_task_free_certificate_entry(sli_wisun_certificate_entry_t *entry);
static void sli_wisun_task_free_certificate_list(sl_slist_node_t **list);
static void sli_wisun_task_add_certificate_entry(sl_slist_node_t **list, sli_wisun_certificate_entry_t *entry);
static uint8_t *sli_wisun_duplicate_data(uint16_t data_size, const uint8_t *src_data);
static sli_wisun_socket_t *sli_wisun_task_allocate_socket(void);
static void sli_wisun_task_free_socket(sli_wisun_socket_t *socket);
static sli_wisun_socket_t *sli_wisun_task_get_socket(sl_wisun_socket_id_t socket_id);
static int sli_wisun_task_set_default_socket_options(uint8_t protocol, sli_wisun_socket_t *socket);
static int sli_wisun_set_channel_mask(sli_wisun_settings_fhss_t *fhss, const sl_wisun_channel_mask_t *channel_mask);
static int sli_wisun_get_channel_mask_channel(const sli_wisun_settings_fhss_t *fhss);
static sl_status_t sli_wisun_add_access_list_entry(uint8_t action, const sl_wisun_mac_address_t *addr);

static void sli_wisun_task_req_handler(const void *req, const void *req_data, void *cnf, void *cnf_data);
static void sli_wisun_task_req_set_network_size(const sl_wisun_msg_set_network_size_req_t *req, sl_wisun_msg_set_network_size_cnf_t *cnf);
static void sli_wisun_task_req_connect(const sl_wisun_msg_connect_req_t *req, sl_wisun_msg_connect_cnf_t *cnf);
static void sli_wisun_task_req_disconnect(const sl_wisun_msg_disconnect_req_t *req, sl_wisun_msg_disconnect_cnf_t *cnf);
static void sli_wisun_task_req_get_ip_address(const sl_wisun_msg_get_ip_address_req_t *req, sl_wisun_msg_get_ip_address_cnf_t *cnf);
static void sli_wisun_task_req_open_socket(const sl_wisun_msg_open_socket_req_t *req, sl_wisun_msg_open_socket_cnf_t *cnf);
static void sli_wisun_task_req_close_socket(const sl_wisun_msg_close_socket_req_t *req, sl_wisun_msg_close_socket_cnf_t *cnf);
static void sli_wisun_task_req_sendto_on_socket(const sl_wisun_msg_sendto_on_socket_req_t *req, const uint8_t *req_data, sl_wisun_msg_sendto_on_socket_cnf_t *cnf);
static void sli_wisun_task_req_listen_on_socket(const sl_wisun_msg_listen_on_socket_req_t *req, sl_wisun_msg_listen_on_socket_cnf_t *cnf);
static void sli_wisun_task_req_accept_on_socket(const sl_wisun_msg_accept_on_socket_req_t *req, sl_wisun_msg_accept_on_socket_cnf_t *cnf);
static void sli_wisun_task_req_connect_socket(const sl_wisun_msg_connect_socket_req_t *req, sl_wisun_msg_connect_socket_cnf_t *cnf);
static void sli_wisun_task_req_bind_socket(const sl_wisun_msg_bind_socket_req_t *req, sl_wisun_msg_bind_socket_cnf_t *cnf);
static void sli_wisun_task_req_send_on_socket(const sl_wisun_msg_send_on_socket_req_t *req, const uint8_t *req_data, sl_wisun_msg_send_on_socket_cnf_t *cnf);
static void sli_wisun_task_req_receive_on_socket(const sl_wisun_msg_receive_on_socket_req_t *req, sl_wisun_msg_receive_on_socket_cnf_t *cnf, uint8_t *cnf_data);
static void sli_wisun_task_req_set_trusted_certificate(const sl_wisun_msg_set_trusted_certificate_req_t *req, const uint8_t *req_data, sl_wisun_msg_set_trusted_certificate_cnf_t *cnf);
static void sli_wisun_task_req_set_device_certificate(const sl_wisun_msg_set_device_certificate_req_t *req, const uint8_t *req_data, sl_wisun_msg_set_device_certificate_cnf_t *cnf);
static void sli_wisun_task_req_set_device_private_key(const sl_wisun_msg_set_device_private_key_req_t *req, const uint8_t *req_data, sl_wisun_msg_set_device_private_key_cnf_t *cnf);
static void sli_wisun_task_req_get_statistics(const sl_wisun_msg_get_statistics_req_t *req, sl_wisun_msg_get_statistics_cnf_t *cnf);
static void sli_wisun_task_req_set_socket_option(const sl_wisun_msg_set_socket_option_req_t *req, sl_wisun_msg_set_socket_option_cnf_t *cnf);
static void sli_wisun_task_req_set_tx_power(const sl_wisun_msg_set_tx_power_req_t *req, sl_wisun_msg_set_tx_power_cnf_t *cnf);
static void sli_wisun_task_req_set_channel_plan(const sl_wisun_msg_set_channel_plan_req_t *req, sl_wisun_msg_set_channel_plan_cnf_t *cnf);
static void sli_wisun_task_req_set_channel_mask(const sl_wisun_msg_set_channel_mask_req_t *req, sl_wisun_msg_set_channel_mask_cnf_t *cnf);
static void sli_wisun_task_req_allow_mac_address(const sl_wisun_msg_allow_mac_address_req_t *req, sl_wisun_msg_allow_mac_address_cnf_t *cnf);
static void sli_wisun_task_req_deny_mac_address(const sl_wisun_msg_deny_mac_address_req_t *req, sl_wisun_msg_deny_mac_address_cnf_t *cnf);
static void sli_wisun_task_req_get_socket_option(const sl_wisun_msg_get_socket_option_req_t *req, sl_wisun_msg_get_socket_option_cnf_t *cnf);
static void sli_wisun_task_req_get_join_state(const sl_wisun_msg_get_join_state_req_t *req, sl_wisun_msg_get_join_state_cnf_t *cnf);
static void sli_wisun_task_req_clear_credential_cache(const sl_wisun_msg_clear_credential_cache_req_t *req, sl_wisun_msg_clear_credential_cache_cnf_t *cnf);
static void sli_wisun_task_req_get_mac_address(const sl_wisun_msg_get_mac_address_req_t *req, sl_wisun_msg_get_mac_address_cnf_t *cnf);
static void sli_wisun_task_req_set_mac_address(const sl_wisun_msg_set_mac_address_req_t *req, sl_wisun_msg_set_mac_address_cnf_t *cnf);
static void sli_wisun_task_req_default(const sl_wisun_msg_header_t *req, sl_wisun_msg_generic_cnf_t *cnf);

static void sli_wisun_task_nwk_status_handler(arm_event_t *event);
static void sli_wisun_task_application_event_handler(arm_event_t *event);
static void sli_wisun_task_socket_event_handler(void *argument);
static void sli_wisun_task_socket_data_event_handler(socket_callback_t *event);
static void sli_wisun_task_socket_closing_event_handler(socket_callback_t *event);
static void sli_wisun_task_socket_data_sent_event_handler(sl_status_t status, int8_t socket_id);
static void sli_wisun_task_event_handler(arm_event_t *event);
static void sli_wisun_task_ind_connected(sl_status_t status);
static void sli_wisun_task_ind_socket_data(int8_t socket_id, uint16_t socket_data_length);
static void sli_wisun_task_ind_socket_data_available(int8_t socket_id, uint16_t socket_data_length);
static void sli_wisun_task_ind_socket_connected(sl_status_t status, int8_t socket_id);
static void sli_wisun_task_ind_socket_connection_available(int8_t socket_id);
static void sli_wisun_task_ind_socket_closing(int8_t socket_id);
//static void sli_wisun_task_ind_disconnected(void);
static void sli_wisun_task_ind_connection_lost(void);
static void sli_wisun_task_ind_socket_data_sent(sl_status_t status, int8_t socket_id, uint16_t socket_space_left);
//static void sli_wisun_task_ind_error(uint32_t status);
static void sli_wisun_task_ind_join_state_changed(sl_wisun_join_state_t join_state);
static void sli_wisun_task_ind_bootstrap_state_changed(int8_t interface_id, icmp_state_t bootstrap_state);
static void sli_wisun_print_trace(const char *str);
static void sli_wisun_print_statistics(void);
static int sli_wisun_get_dodag_info(int8_t interface_id, rpl_dodag_info_t *dodag_info);
static void sli_wisun_get_phy_statistics(sl_wisun_statistics_phy_t *statistics);
static void sli_wisun_get_mac_statistics(sl_wisun_statistics_mac_t *statistics);
static void sli_wisun_get_fhss_statistics(sl_wisun_statistics_fhss_t *statistics);
static void sli_wisun_get_wisun_statistics(sl_wisun_statistics_wisun_t *statistics);
static void sli_wisun_get_network_statistics(sl_wisun_statistics_network_t *statistics);
static void sli_wisun_trace_mutex_wait(void);
static void sli_wisun_trace_mutex_release(void);
static sl_status_t sli_wisun_socket_set_event_mode(sli_wisun_socket_t *socket, const sl_wisun_socket_option_event_mode_t* event_mode);
static sl_status_t sli_wisun_socket_set_multicast_group(sli_wisun_socket_t *socket, const sl_wisun_socket_option_multicast_group_t* multicast_group);
static sl_status_t sli_wisun_task_set_network_size(int8_t interface_id, uint8_t network_size, const sli_wisun_settings_timing_t *timing);
static sl_status_t sli_wisun_task_set_fhss(int8_t interface_id, const sli_wisun_settings_fhss_t *fhss);
static sl_status_t sli_wisun_set_access_list(int8_t interface_id, const sli_wisun_access_list_t *access_list);
static sl_status_t sli_wisun_get_mac_address(int8_t interface_id, sl_wisun_mac_address_t *address);
static sl_status_t sli_wisun_set_mac_address(int8_t interface_id, const sl_wisun_mac_address_t *address);


// -----------------------------------------------------------------------------
// Functions executed in multiple contexts

#if defined(SL_CATALOG_MICRIUMOS_KERNEL_PRESENT)
#if (OS_CFG_TASK_STK_REDZONE_EN == DEF_ENABLED)
static void sli_wisun_stack_overflow(OS_TCB *p_tcb)
{
  static char debugstr[256];

  sprintf(debugstr, "Stack overflow in task %s, StkSize: %u, StkUsed: %u, StkFree: %u",
    p_tcb->NamePtr, p_tcb->StkSize, p_tcb->StkUsed, p_tcb->StkFree);
  SEGGER_RTT_WriteString(0, debugstr);

  while(1);
}
#endif
#endif

static void sli_wisun_trace_mutex_wait(void)
{
  assert(osMutexAcquire(sli_wisun_task_trace_mutex, osWaitForever) == osOK);
}

static void sli_wisun_trace_mutex_release(void)
{
  assert(osMutexRelease(sli_wisun_task_trace_mutex) == osOK);
}

static void sli_wisun_print_trace(const char *str)
{
  SEGGER_RTT_WriteString(0, str);
}

static void sli_wisun_print_statistics(void)
{
  struct mallinfo malloc_info;

#if defined(__ICCARM__)
  malloc_info = __iar_dlmallinfo();
#elif defined(__GNUC__)
  malloc_info = mallinfo();
#endif
  tr_debug("malloc: arena: %u, uordblks: %u", malloc_info.arena, malloc_info.uordblks);
}

static sl_wisun_evt_t *sli_wisun_task_allocate_event(uint8_t event_id, uint16_t event_length)
{
  sl_wisun_evt_t *evt = NULL;

  if (event_length <= sizeof(sl_wisun_evt_t)) {
    // Attempt to use from the pre-allocated pool
  }

  if (!evt) {
    // Try to allocate dynamically
    evt = (sl_wisun_evt_t *)sl_malloc(event_length);
    if (evt) {
      evt->header.info = SLI_WISUN_EVENT_INFO_FLAG_DYNAMIC;
    }
  }

  if (evt) {
    evt->header.id = event_id;
    evt->header.length = event_length;
  }

  return evt;
}

static void sli_wisun_task_free_event(sl_wisun_evt_t *evt)
{
  if (evt->header.info & SLI_WISUN_EVENT_INFO_FLAG_DYNAMIC) {
    // Dynamically allocated
    sl_free(evt);
  } else {
    // From the pre-allocated pool
  }
}

static sli_wisun_certificate_entry_t *sli_wisun_task_allocate_certificate_entry(void)
{
  sl_slist_node_t *item;
  sli_wisun_certificate_entry_t *entry = NULL;

  item = sl_slist_pop(&sli_wisun_certificate_entry_list);
  if (item) {
    entry = SL_SLIST_ENTRY(item, sli_wisun_certificate_entry_t, node);
    memset(entry, 0, sizeof(sli_wisun_certificate_entry_t));
  }

  return entry;
}

static void sli_wisun_task_free_certificate_entry(sli_wisun_certificate_entry_t *entry)
{
  if (!entry) {
    return;
  }

  if ((entry->certificate.cert) &&
      (entry->flags & SLI_WISUN_CERTIFICATE_FLAG_IS_OWNED)) {
    sl_free((void *)entry->certificate.cert);
  }

  if ((entry->certificate.key) &&
      (entry->flags & SLI_WISUN_CERTIFICATE_FLAG_KEY_IS_OWNED)) {
    sl_free((void *)entry->certificate.key);
  }

  // Push the entry to the free entries list
  sl_slist_push(&sli_wisun_certificate_entry_list, &entry->node);
}

static void sli_wisun_task_free_certificate_list(sl_slist_node_t **list)
{
  sl_slist_node_t *item;
  sli_wisun_certificate_entry_t *entry;

  item = sl_slist_pop(list);
  while (item) {
    entry = SL_SLIST_ENTRY(item, sli_wisun_certificate_entry_t, node);
    sli_wisun_task_free_certificate_entry(entry);
    item = sl_slist_pop(list);
  }
}

static void sli_wisun_task_add_certificate_entry(sl_slist_node_t **list, sli_wisun_certificate_entry_t *entry)
{
  // Device certificate has to be the first
  if (entry->flags & SLI_WISUN_CERTIFICATE_FLAG_HAS_KEY) {
    sl_slist_push(list, &entry->node);
  } else {
    sl_slist_push_back(list, &entry->node);
  }
}

static uint8_t *sli_wisun_duplicate_data(uint16_t data_size, const uint8_t *src_data)
{
  uint8_t *data;

  data = sl_malloc(data_size);
  if (data) {
    memcpy(data, src_data, data_size);
  }

  return data;
}

static sli_wisun_socket_t *sli_wisun_task_allocate_socket(void)
{
  sl_slist_node_t *item;
  sli_wisun_socket_t *socket = NULL;

  item = sl_slist_pop(&sli_wisun_socket_list_free);
  if (item) {
    socket = SL_SLIST_ENTRY(item, sli_wisun_socket_t, node);
    socket->socket_id = SL_WISUN_INVALID_SOCKET_ID;
    socket->flags = 0;
    socket->sndbuf_limit = 0;
    socket->sndbuf_size = 0;
  }

  return socket;
}

static void sli_wisun_task_free_socket(sli_wisun_socket_t *socket)
{
  if (!socket) {
    return;
  }

  // Remove the entry from the active socket list
  sl_slist_remove(&sli_wisun_socket_list, &socket->node);

  // Push the entry to the free socket list
  sl_slist_push(&sli_wisun_socket_list_free, &socket->node);
}

static sli_wisun_socket_t *sli_wisun_task_get_socket(sl_wisun_socket_id_t socket_id)
{
  sli_wisun_socket_t *socket = NULL;

  SL_SLIST_FOR_EACH_ENTRY(sli_wisun_socket_list, socket, sli_wisun_socket_t, node) {
    if (socket->socket_id == socket_id) {
      if (sli_wisun_socket_list != &socket->node) {
        // Make sure the socket is the first on the active list
        sl_slist_remove(&sli_wisun_socket_list, &socket->node);
        sl_slist_push(&sli_wisun_socket_list, &socket->node);
      }
      return socket;
    }
  }

  return NULL;
}

static int sli_wisun_task_set_default_socket_options(uint8_t protocol, sli_wisun_socket_t *socket)
{
  int ret = 0;
  int32_t rcvbufsize;
  int32_t sndbufsize;
  int16_t multicast_hop_limit = SLI_WISUN_SOCKET_MULTICAST_HOP_LIMIT;

  switch (protocol) {
    case SL_WISUN_SOCKET_PROTOCOL_UDP:
      rcvbufsize = SLI_WISUN_SOCKET_UDP_RCVBUF_SIZE;
      sndbufsize = SLI_WISUN_SOCKET_UDP_SNDBUF_SIZE;
      break;
    case SL_WISUN_SOCKET_PROTOCOL_TCP:
      rcvbufsize = SLI_WISUN_SOCKET_TCP_RCVBUF_SIZE;
      sndbufsize = SLI_WISUN_SOCKET_TCP_SNDBUF_SIZE;
      break;
    case SL_WISUN_SOCKET_PROTOCOL_ICMP:
      rcvbufsize = SLI_WISUN_SOCKET_ICMP_RCVBUF_SIZE;
      sndbufsize = SLI_WISUN_SOCKET_ICMP_SNDBUF_SIZE;
      break;
    default:
      tr_error("sli_wisun_task_set_default_socket_options: unsupported protocol type");
      return -1;
  }

  if (rcvbufsize >= 0) {
    ret = socket_setsockopt(socket->socket_id, SOCKET_SOL_SOCKET, SOCKET_SO_RCVBUF, &rcvbufsize, sizeof(rcvbufsize));
    SLI_WISUN_ERROR_CHECK(ret >= 0);
  }

  if (sndbufsize >= 0) {
    ret = socket_setsockopt(socket->socket_id, SOCKET_SOL_SOCKET, SOCKET_SO_SNDBUF, &sndbufsize, sizeof(sndbufsize));
    SLI_WISUN_ERROR_CHECK(ret >= 0);

    // Low-water mark is set equal to the send buffer size to prevent the socket
    // from going over the buffer size limit. Write will return NS_EWOULDBLOCK
    // if over the limit.
    ret = socket_setsockopt(socket->socket_id, SOCKET_SOL_SOCKET, SOCKET_SO_SNDLOWAT, &sndbufsize, sizeof(sndbufsize));
    SLI_WISUN_ERROR_CHECK(ret >= 0);

    // Store send buffer size limit
    socket->sndbuf_limit = sndbufsize;
  }

  if (multicast_hop_limit) {
    ret = socket_setsockopt(socket->socket_id, SOCKET_IPPROTO_IPV6, SOCKET_IPV6_MULTICAST_HOPS, &multicast_hop_limit, sizeof(multicast_hop_limit));
    SLI_WISUN_ERROR_CHECK(ret >= 0);
  }

error_handler:

  return ret;
}

static int sli_wisun_set_channel_mask(sli_wisun_settings_fhss_t *fhss, const sl_wisun_channel_mask_t *channel_mask)
{
  int channel_count = 0;
  int channel = 0;
  int idx;
  int i;

  memset(fhss->channel_mask, 0, sizeof(fhss->channel_mask));

  for (idx = 0; idx < SL_WISUN_CHANNEL_MASK_SIZE; ++idx) {
    for (i = 0; i < 8; ++i, ++channel) {
      if (channel_mask->mask[idx] & (1 << i)) {
        fhss->channel_mask[idx / 4] |= (1 << (channel % 32));
        channel_count++;
      }
    }
  }

  return channel_count;
}

static int sli_wisun_get_channel_mask_channel(const sli_wisun_settings_fhss_t *fhss)
{
  int channel = -1;
  int idx;

  for (idx = 0; idx < SLI_WISUN_CHANNEL_MASK_SIZE; ++idx) {
    if (fhss->channel_mask[idx]) {
      channel = (idx * 32) + SL_CTZ(fhss->channel_mask[idx]);
      break;
    }
  }

  return channel;
}

static sl_status_t sli_wisun_add_access_list_entry(uint8_t action, const sl_wisun_mac_address_t *addr)
{
  int i;

  if (!memcmp(addr->address, SLI_WISUN_BROADCAST_MAC.address, SL_WISUN_MAC_ADDRESS_SIZE)) {
    // In case of an empty list, the action must be reversed
    sli_wisun_access_list.action = !action;
    sli_wisun_access_list.list_count = 0;
    return SL_STATUS_OK;
  }

  // Changing the list type empties the previous entries
  if (action != sli_wisun_access_list.action) {
    sli_wisun_access_list.action = action;
    sli_wisun_access_list.list_count = 0;
  }

  // Ignore duplicate addresses
  for (i = 0; i < sli_wisun_access_list.list_count; ++i) {
    if (!memcmp(addr->address, sli_wisun_access_list.list[i].address, SL_WISUN_MAC_ADDRESS_SIZE)) {
      return SL_STATUS_OK;
    }
  }

  if (sli_wisun_access_list.list_count >= SLI_WISUN_TASK_MAX_ACCESS_LIST_ENTRIES) {
    return SL_STATUS_HAS_OVERFLOWED;
  }

  // Add a new address
  sli_wisun_access_list.list[sli_wisun_access_list.list_count++] = *addr;

  return SL_STATUS_OK;
}

static void sli_wisun_task_ind_connected(sl_status_t status)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_CONNECTED_IND_ID, sizeof(sl_wisun_msg_connected_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.connected.status = status;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void sli_wisun_task_ind_socket_data(int8_t socket_id, uint16_t socket_data_length)
{
  int ret;
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };
  uint16_t data_length;
  ns_address_t remote_address;

  data_length = sizeof(sl_wisun_msg_socket_data_ind_t) + socket_data_length;

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_SOCKET_DATA_IND_ID, data_length);
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.socket_data.status = SL_STATUS_OK;
  evt->evt.socket_data.socket_id = socket_id;

  ret = socket_recvfrom(socket_id, evt->evt.socket_data.data, data_length, 0, &remote_address);
  if (ret < 0) {
    tr_error("sli_wisun_task_ind_socket_data: unable to read from the socket: %d", ret);
    sli_wisun_task_free_event(evt);
    return;
  }

  memcpy(evt->evt.socket_data.remote_address.address, remote_address.address, SL_WISUN_IP_ADDRESS_SIZE);
  evt->evt.socket_data.remote_port = remote_address.identifier;
  evt->evt.socket_data.data_length = ret;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void sli_wisun_task_ind_socket_data_available(int8_t socket_id, uint16_t socket_data_length)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_SOCKET_DATA_AVAILABLE_IND_ID,
                                      sizeof(sl_wisun_msg_socket_data_available_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.socket_data_available.status = SL_STATUS_OK;
  evt->evt.socket_data_available.socket_id = socket_id;
  evt->evt.socket_data_available.data_length = socket_data_length;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void sli_wisun_task_ind_socket_connected(sl_status_t status, int8_t socket_id)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_SOCKET_CONNECTED_IND_ID,
                                      sizeof(sl_wisun_msg_socket_connected_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.socket_connected.status = status;
  evt->evt.socket_connected.socket_id = socket_id;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void sli_wisun_task_ind_socket_connection_available(int8_t socket_id)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_SOCKET_CONNECTION_AVAILABLE_IND_ID,
                                      sizeof(sl_wisun_msg_socket_connection_available_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.socket_connection_available.status = SL_STATUS_OK;
  evt->evt.socket_connection_available.socket_id = socket_id;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void sli_wisun_task_ind_socket_closing(int8_t socket_id)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_SOCKET_CLOSING_IND_ID,
                                      sizeof(sl_wisun_msg_socket_closing_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.socket_closing.status = SL_STATUS_OK;
  evt->evt.socket_closing.socket_id = socket_id;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

#if 0 // not yet used
static void sli_wisun_task_ind_disconnected()
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_DISCONNECTED_IND_ID,
                                      sizeof(sl_wisun_msg_disconnected_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.disconnected.status = SL_STATUS_OK;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}
#endif

static void sli_wisun_task_ind_connection_lost(void)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_CONNECTION_LOST_IND_ID,
                                      sizeof(sl_wisun_msg_connection_lost_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.connection_lost.status = SL_STATUS_OK;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void sli_wisun_task_ind_socket_data_sent(sl_status_t status, int8_t socket_id, uint16_t socket_space_left)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_SOCKET_DATA_SENT_IND_ID,
                                      sizeof(sl_wisun_msg_socket_data_sent_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.socket_data_sent.status = status;
  evt->evt.socket_data_sent.socket_id = socket_id;
  evt->evt.socket_data_sent.socket_space_left = socket_space_left;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

#if 0 // not yet used
static void sli_wisun_task_ind_error(uint32_t status)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_ERROR_IND_ID,
                                      sizeof(sl_wisun_msg_error_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.error.status = status;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}
#endif

static int sli_wisun_get_join_state(int8_t interface_id, sl_wisun_join_state_t *join_state)
{
  ws_stack_info_t info;
  int ret;
 
  ret = ws_stack_info_get(interface_id, &info);
  if (ret < 0) {
    // Interface doesn't exist or is not active.
    return -1;
  }

  switch (info.join_state) {
    case 1:
      *join_state = SL_WISUN_JOIN_STATE_SELECT_PAN;
      break;
    case 2:
      *join_state = SL_WISUN_JOIN_STATE_AUTHENTICATE;
      break;
    case 3:
      *join_state = SL_WISUN_JOIN_STATE_ACQUIRE_PAN_CONFIG;
      break;
    case 4:
      *join_state = SL_WISUN_JOIN_STATE_CONFIGURE_ROUTING;
      break;
    case 5:
      *join_state = SL_WISUN_JOIN_STATE_OPERATIONAL;
      break;
    default:
      // Unknown join state.
      return -1;
  }

  return 0;
}

static void sli_wisun_task_ind_join_state_changed(sl_wisun_join_state_t join_state)
{
  osStatus_t os_status;
  sl_wisun_evt_t *evt;
  sli_wisun_queue_descriptor_t evt_descriptor_put = { NULL, 0 };

  evt = sli_wisun_task_allocate_event(SL_WISUN_MSG_JOIN_STATE_IND_ID,
                                      sizeof(sl_wisun_msg_join_state_ind_t));
  if (!evt) {
    // Unable to allocate an event
    return;
  }

  // Event body
  evt->evt.join_state.status = SL_STATUS_OK;
  evt->evt.join_state.join_state = join_state;

  evt_descriptor_put.addr = (void *)evt;
  evt_descriptor_put.length = evt->header.length;

  os_status = osMessageQueuePut(sli_wisun_task_api_events_queue,
                                (void *)&evt_descriptor_put,
                                0, //TODO: priority ???
                                0);
  if (os_status != osOK) {
    // Unable to queue the event
    sli_wisun_task_free_event(evt);
    return;
  }

  // Notify the indication is ready to be posted
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_IND_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

static void sli_wisun_task_ind_bootstrap_state_changed(int8_t interface_id, icmp_state_t bootstrap_state)
{
  int ret;
  sl_wisun_join_state_t join_state;
  (void)interface_id;
  (void)bootstrap_state;

  ret = sli_wisun_get_join_state(sli_wisun_interface_id, &join_state);
  if (ret < 0) {
    // Interface doesn't exist or join state is unknown.
    return;
  }

  sli_wisun_task_ind_join_state_changed(join_state);
}

static int sli_wisun_get_dodag_info(int8_t interface_id, rpl_dodag_info_t *dodag_info)
{
  bool ret;
  struct rpl_instance *rpl_instance;
  protocol_interface_info_entry_t *interface_info;

  interface_info = protocol_stack_interface_info_get_by_id(interface_id);
  if (!interface_info) {
    // Interface doesn't exist
    return -1;
  }

  if (!interface_info->rpl_domain) {
    // Interface doesn't have an RPL domain
    return -1;
  }

  // Assume the first instance is the one in use
  rpl_instance = rpl_control_enumerate_instances(interface_info->rpl_domain, NULL);
  if (!rpl_instance) {
    // RPL instance doesn't exist
    return -1;
  }

  ret = rpl_control_read_dodag_info(rpl_instance, dodag_info);
  if (!ret) {
    // Unable to read RPL DODAG information
    return -1;
  }

  // Success
  return 0;
}

static void sli_wisun_get_phy_statistics(sl_wisun_statistics_phy_t *statistics)
{
  statistics->crc_fails = sli_wisun_statistics.phy.crc_fails;
  statistics->tx_timeouts = sli_wisun_statistics.phy.tx_timeouts;
  statistics->rx_timeouts = sli_wisun_statistics.phy.rx_timeouts;
}

static void sli_wisun_get_mac_statistics(sl_wisun_statistics_mac_t *statistics)
{
  statistics->tx_queue_size = sli_wisun_statistics.mac.mac_tx_queue_size;
  statistics->tx_queue_peak = sli_wisun_statistics.mac.mac_tx_queue_peak;
  statistics->rx_count = sli_wisun_statistics.mac.mac_rx_count;
  statistics->tx_count = sli_wisun_statistics.mac.mac_tx_count;
  statistics->bc_rx_count = sli_wisun_statistics.mac.mac_bc_rx_count;
  statistics->bc_tx_count = sli_wisun_statistics.mac.mac_bc_tx_count;
  statistics->rx_drop_count = sli_wisun_statistics.mac.mac_rx_drop_count;
  statistics->tx_bytes = sli_wisun_statistics.mac.mac_tx_bytes;
  statistics->rx_bytes = sli_wisun_statistics.mac.mac_rx_bytes;
  statistics->tx_failed_count = sli_wisun_statistics.mac.mac_tx_failed_count;
  statistics->retry_count = sli_wisun_statistics.mac.mac_retry_count;
  statistics->cca_attempts_count = sli_wisun_statistics.mac.mac_cca_attempts_count;
  statistics->failed_cca_count = sli_wisun_statistics.mac.mac_failed_cca_count;
}

static void sli_wisun_get_fhss_statistics(sl_wisun_statistics_fhss_t *statistics)
{
  statistics->drift_compensation = sli_wisun_statistics.fhss.fhss_drift_compensation;
  statistics->hop_count = sli_wisun_statistics.fhss.fhss_hop_count;
  statistics->synch_interval = sli_wisun_statistics.fhss.fhss_synch_interval;
  statistics->prev_avg_synch_fix = sli_wisun_statistics.fhss.fhss_prev_avg_synch_fix;
  statistics->synch_lost = sli_wisun_statistics.fhss.fhss_synch_lost;
  statistics->unknown_neighbor = sli_wisun_statistics.fhss.fhss_unknown_neighbor;
  statistics->channel_retry = sli_wisun_statistics.fhss.fhss_channel_retry;
}

static void sli_wisun_get_wisun_statistics(sl_wisun_statistics_wisun_t *statistics)
{
  statistics->asynch_rx_count = sli_wisun_statistics.wisun.asynch_rx_count;
  statistics->asynch_tx_count = sli_wisun_statistics.wisun.asynch_tx_count;
}

static void sli_wisun_get_network_statistics(sl_wisun_statistics_network_t *statistics)
{
  statistics->ip_rx_count = sli_wisun_statistics.network.ip_rx_count;
  statistics->ip_tx_count = sli_wisun_statistics.network.ip_tx_count;
  statistics->ip_rx_drop = sli_wisun_statistics.network.ip_rx_drop;
  statistics->ip_cksum_error = sli_wisun_statistics.network.ip_cksum_error;
  statistics->ip_tx_bytes = sli_wisun_statistics.network.ip_tx_bytes;
  statistics->ip_rx_bytes = sli_wisun_statistics.network.ip_rx_bytes;
  statistics->ip_routed_up = sli_wisun_statistics.network.ip_routed_up;
  statistics->ip_no_route = sli_wisun_statistics.network.ip_no_route;
  statistics->frag_rx_errors = sli_wisun_statistics.network.frag_rx_errors;
  statistics->frag_tx_errors = sli_wisun_statistics.network.frag_tx_errors;
  statistics->rpl_route_routecost_better_change = sli_wisun_statistics.network.rpl_route_routecost_better_change;
  statistics->ip_routeloop_detect = sli_wisun_statistics.network.ip_routeloop_detect;
  statistics->rpl_memory_overflow = sli_wisun_statistics.network.rpl_memory_overflow;
  statistics->rpl_parent_tx_fail = sli_wisun_statistics.network.rpl_parent_tx_fail;
  statistics->rpl_unknown_instance = sli_wisun_statistics.network.rpl_unknown_instance;
  statistics->rpl_local_repair = sli_wisun_statistics.network.rpl_local_repair;
  statistics->rpl_global_repair = sli_wisun_statistics.network.rpl_global_repair;
  statistics->rpl_malformed_message = sli_wisun_statistics.network.rpl_malformed_message;
  statistics->rpl_time_no_next_hop = sli_wisun_statistics.network.rpl_time_no_next_hop;
  statistics->rpl_total_memory = sli_wisun_statistics.network.rpl_total_memory;
  statistics->buf_alloc = sli_wisun_statistics.network.buf_alloc;
  statistics->buf_headroom_realloc = sli_wisun_statistics.network.buf_headroom_realloc;
  statistics->buf_headroom_shuffle = sli_wisun_statistics.network.buf_headroom_shuffle;
  statistics->buf_headroom_fail = sli_wisun_statistics.network.buf_headroom_fail;
  statistics->etx_1st_parent = sli_wisun_statistics.network.etx_1st_parent;
  statistics->etx_2nd_parent = sli_wisun_statistics.network.etx_2nd_parent;
  statistics->adapt_layer_tx_queue_size = sli_wisun_statistics.network.adapt_layer_tx_queue_size;
  statistics->adapt_layer_tx_queue_peak = sli_wisun_statistics.network.adapt_layer_tx_queue_peak;
}

// -----------------------------------------------------------------------------
// Functions executed in the application task context

void sli_wisun_task_init()
{
  // Initialize trace library
  mbed_trace_init();
  mbed_trace_config_set(TRACE_ACTIVE_LEVEL_ALL | TRACE_CARRIAGE_RETURN);
  mbed_trace_print_function_set(sli_wisun_print_trace);
  mbed_trace_mutex_wait_function_set(sli_wisun_trace_mutex_wait);
  mbed_trace_mutex_release_function_set(sli_wisun_trace_mutex_release);

  const osMutexAttr_t sli_wisun_task_trace_mutex_attr = {
    "Wi-SUN Task Trace Mutex",
    osMutexRecursive,
    NULL,
    0
  };

  sli_wisun_task_trace_mutex = osMutexNew(&sli_wisun_task_trace_mutex_attr);
  assert(sli_wisun_task_trace_mutex != NULL);

  const osEventFlagsAttr_t sli_wisun_task_flags_attr = {
    "Wi-SUN Task Flags",
    0,
    &sli_wisun_task_flags_cb[0],
    osEventFlagsCbSize
  };

  sli_wisun_task_flags = osEventFlagsNew(&sli_wisun_task_flags_attr);
  assert(sli_wisun_task_flags != NULL);

  const osMessageQueueAttr_t sli_wisun_task_api_events_queue_attr = {
    "Wi-SUN Task API Events Queue",
	0,
	NULL,
	0,
	NULL,
	0
  };

  sli_wisun_task_api_events_queue = osMessageQueueNew(SLI_WISUN_TASK_MAX_API_EVENTS,
                                                      (sizeof(sli_wisun_queue_descriptor_t) + 3) & (~3) /* align to 4 bytes */,
                                                      &sli_wisun_task_api_events_queue_attr);
  assert(sli_wisun_task_api_events_queue != NULL);

  osThreadAttr_t sli_wisun_task_attribute = {
    "Wi-SUN Task",
    osThreadDetached,
    &sli_wisun_task_tcb[0],
    osThreadCbSize,
    &sli_wisun_task_stack[0],
    (SLI_WISUN_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u,
    SLI_WISUN_TASK_PRIORITY,
    0,
    0
    };

  sli_wisun_task_id = osThreadNew(sli_wisun_task,
                                  NULL,
                                  &sli_wisun_task_attribute);
  assert(sli_wisun_task_id != 0);

  osThreadAttr_t sli_wisun_event_task_attribute = {
    "Wi-SUN Event Task",
    osThreadDetached,
    &sli_wisun_event_task_tcb[0],
    osThreadCbSize,
    &sli_wisun_event_task_stack[0],
    (SLI_WISUN_EVENT_TASK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u,
    SLI_WISUN_EVENT_TASK_PRIORITY,
    0,
    0
    };

  sli_wisun_event_task_id = osThreadNew(sli_wisun_event_task,
                                        NULL,
                                        &sli_wisun_event_task_attribute);
  assert(sli_wisun_event_task_id != 0);
}

void sli_wisun_task_req(const void *req, const void *req_data, void *cnf, void *cnf_data)
{
  // Wait for the task to be ready
  assert((osEventFlagsWait(sli_wisun_task_flags,
                           SLI_WISUN_TASK_FLAG_READY,
                           osFlagsWaitAny,
                           osWaitForever) & CMSIS_RTOS_ERROR_MASK) == 0);

  sli_wisun_pending_req = req;
  sli_wisun_pending_req_data = req_data;
  sli_wisun_pending_cnf = cnf;
  sli_wisun_pending_cnf_data = cnf_data;

  // Notify the request is ready to be handled
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_REQ_READY) & CMSIS_RTOS_ERROR_MASK) == 0);

  // Wait for the confirmation to be ready
  assert((osEventFlagsWait(sli_wisun_task_flags,
                           SLI_WISUN_TASK_FLAG_CNF_READY,
                           osFlagsWaitAny,
                           osWaitForever) & CMSIS_RTOS_ERROR_MASK) == 0);

  sli_wisun_pending_req = 0;
  sli_wisun_pending_cnf = 0;

  // Ready to serve the next request
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
}

// -----------------------------------------------------------------------------
// Functions executed in the event task context

static void sli_wisun_task_nwk_status_handler(arm_event_t *event)
{
  arm_nwk_interface_status_type_e status = (arm_nwk_interface_status_type_e)event->event_data;
  protocol_interface_info_entry_t *cur = 0;

  tr_debug("sli_wisun_task_nwk_status_handler: %s (0x%04x)",
    SLI_WISUN_NWK_STATUS_STR[status], status);

  switch (status) {
    case ARM_NWK_BOOTSTRAP_READY:
      // Task state
      sli_wisun_task_state = SLI_WISUN_TASK_STATE_CONNECTED;

      // Notify the application
      sli_wisun_task_ind_connected(SL_STATUS_OK);
      break;
    case ARM_NWK_RPL_INSTANCE_FLOODING_READY:
      break;
    case ARM_NWK_SET_DOWN_COMPLETE:
      break;
    case ARM_NWK_NWK_SCAN_FAIL:
      break;
    case ARM_NWK_IP_ADDRESS_ALLOCATION_FAIL:
      break;
    case ARM_NWK_DUPLICATE_ADDRESS_DETECTED:
      break;
    case ARM_NWK_AUHTENTICATION_START_FAIL:
      break;
    case ARM_NWK_AUHTENTICATION_FAIL:
      break;
    case ARM_NWK_NWK_CONNECTION_DOWN:
      // Task state
      sli_wisun_task_state = SLI_WISUN_TASK_STATE_CONNECTING;

      // Notify the application
      sli_wisun_task_ind_connection_lost();
      break;
    case ARM_NWK_NWK_PARENT_POLL_FAIL:
      break;
    case ARM_NWK_PHY_CONNECTION_DOWN:
      break;
    case ARM_NWK_BOOTSTRAP_STATE_CHANGED:
      // Notify the application
      cur = protocol_stack_interface_info_get_by_id(event->event_id);
      if (cur) {
        sli_wisun_task_ind_bootstrap_state_changed(cur->id, cur->nwk_bootstrap_state);
      }
      break;
    default:
      break;
  }
}

static void sli_wisun_task_application_event_handler(arm_event_t *event)
{
    switch (event->event_id) {
      case SLI_WISUN_APPLICATION_EVENT_STATISTICS:
        sli_wisun_print_statistics();
        eventOS_event_timer_request(SLI_WISUN_APPLICATION_EVENT_STATISTICS,
                                    APPLICATION_EVENT, sli_wisun_task_event_handler_id, 30000);
        break;
    }
}

static void sli_wisun_task_event_handler(arm_event_t *event)
{
  arm_library_event_type_e event_type = (arm_library_event_type_e)event->event_type;

  tr_debug("sli_wisun_task_event_handler: %s (0x%04x)",
    SLI_WISUN_EVENT_STR[event_type], event_type);

  switch (event_type) {
    case ARM_LIB_TASKLET_INIT_EVENT:
      break;
    case ARM_LIB_NWK_INTERFACE_EVENT:
      sli_wisun_task_nwk_status_handler(event);
      break;
    case ARM_LIB_SYSTEM_TIMER_EVENT:
      break;
    case APPLICATION_EVENT:
      sli_wisun_task_application_event_handler(event);
      break;
    default:
      break;
  }
}

static void sli_wisun_task_socket_event_handler(void *argument)
{
  socket_callback_t *event = (socket_callback_t *)argument;

  tr_debug("sli_wisun_task_socket_event_handler: %s (0x%02x) on socket %d",
    SLI_WISUN_SOCKET_EVENT_STR[event->event_type >> 4], event->event_type, event->socket_id);

  switch (event->event_type & SOCKET_EVENT_MASK) {
    case SOCKET_DATA:
      sli_wisun_task_socket_data_event_handler(event);
      break;
    case SOCKET_CONNECT_DONE:
      sli_wisun_task_ind_socket_connected(SL_STATUS_OK, event->socket_id);
      break;
    case SOCKET_CONNECT_FAIL:
      sli_wisun_task_ind_socket_connected(SL_STATUS_FAIL, event->socket_id);
      break;
    case SOCKET_INCOMING_CONNECTION:
      sli_wisun_task_ind_socket_connection_available(event->socket_id);
      break;
    case SOCKET_TX_FAIL:
      sli_wisun_task_socket_data_sent_event_handler(SL_STATUS_TRANSMIT, event->socket_id);
      break;
    case SOCKET_CONNECT_CLOSED:
      sli_wisun_task_socket_closing_event_handler(event);
      break;
    case SOCKET_CONNECTION_RESET:
      sli_wisun_task_socket_closing_event_handler(event);
      break;
    case SOCKET_NO_ROUTE:
      sli_wisun_task_socket_data_sent_event_handler(SL_STATUS_TRANSMIT, event->socket_id);
      break;
    case SOCKET_TX_DONE:
      sli_wisun_task_socket_data_sent_event_handler(SL_STATUS_OK, event->socket_id);
      break;
    case SOCKET_NO_RAM:
      sli_wisun_task_socket_data_sent_event_handler(SL_STATUS_TRANSMIT, event->socket_id);
      break;
    case SOCKET_CONNECTION_PROBLEM:
      break;
    default:
      break;
  }
}

static void sli_wisun_task_socket_data_event_handler(socket_callback_t *event)
{
  sli_wisun_socket_t *socket = NULL;

  socket = sli_wisun_task_get_socket(event->socket_id);
  if (!socket) {
    tr_error("sli_wisun_task_socket_data_event_handler: socket structure not found");
    return;
  }

  if (!event->d_len) {
    // Zero-length event is used indicate the peer has closed the TCP socket
    // on its end.
    sli_wisun_task_socket_closing_event_handler(event);
  } else {
    // Notify the application
    if (socket->flags & SLI_WISUN_SOCKET_FLAG_POLLING) {
      sli_wisun_task_ind_socket_data_available(event->socket_id, event->d_len);
    } else {
      sli_wisun_task_ind_socket_data(event->socket_id, event->d_len);
    }
  }
}

static void sli_wisun_task_socket_closing_event_handler(socket_callback_t *event)
{
  // Notify the application
  sli_wisun_task_ind_socket_closing(event->socket_id);
}

static void sli_wisun_task_socket_data_sent_event_handler(sl_status_t status, int8_t socket_id)
{
  socket_t *socket;
  uint16_t socket_space_left = 0;

  socket = socket_pointer_get(socket_id);
  if (!socket) {
    tr_error("sli_wisun_task_socket_data_sent_event_handler: socket structure not found");
    return;
  }

  switch (socket->type) {
    case SOCKET_TYPE_STREAM:
      socket_space_left = SLI_WISUN_SOCKET_TCP_SNDBUF_SIZE - socket->sndq.data_bytes;
      break;
    case SOCKET_TYPE_DGRAM:
      socket_space_left = SLI_WISUN_SOCKET_UDP_SNDBUF_SIZE - socket->sndq.data_bytes;
      break;
    case SOCKET_TYPE_RAW:
      if (socket->inet_pcb->protocol == IPV6_NH_ICMPV6) {
        socket_space_left = SLI_WISUN_SOCKET_ICMP_SNDBUF_SIZE - socket->sndq.data_bytes;
      }
      break;
  }

  // Notify the application
  sli_wisun_task_ind_socket_data_sent(status, socket_id, socket_space_left);
}

// -----------------------------------------------------------------------------
// Functions executed in the event task context

void sli_wisun_event_task(void *argument)
{
  uint32_t flags;
  sl_wisun_evt_t *evt;
  (void)argument;

  SLI_WISUN_TASK_LOOP {
    // Wait for something to happen
    flags = osEventFlagsWait(sli_wisun_task_flags,
                             SLI_WISUN_TASK_FLAG_IND_READY,
                             osFlagsWaitAny,
                             osWaitForever);
    assert((flags & CMSIS_RTOS_ERROR_MASK) == 0);

    // One or more indications are ready to be handled
    if (flags & SLI_WISUN_TASK_FLAG_IND_READY) {
      evt = sli_wisun_task_poll();
      while (evt) {
        sl_wisun_on_event(evt);
        sli_wisun_task_free_event(evt);
        evt = sli_wisun_task_poll();
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Functions executed in the task context

void sli_wisun_task(void *argument)
{
  mac_api_t *mac_api;
  int ret;
  int i;
  (void)argument;

  tr_info("Release version: %d.%d.%d %s",
          SL_WISUN_VERSION_MAJOR, SL_WISUN_VERSION_MINOR,
          SL_WISUN_VERSION_PATCH, SL_WISUN_VERSION_LABEL);

  tr_debug("sli_wisun_task()");

  // Task state
  sli_wisun_task_state = SLI_WISUN_TASK_STATE_INITIALIZED;

  // Initialize certificate lists
  sl_slist_init(&sli_wisun_trusted_certificate_list);
  sl_slist_init(&sli_wisun_certificate_list);
  sl_slist_init(&sli_wisun_certificate_entry_list);
  for (i = 0; i < SLI_WISUN_TASK_MAX_CERTIFICATE_ENTRIES; ++i) {
    sl_slist_push(&sli_wisun_certificate_entry_list, &sli_wisun_certificate_entries[i].node);
  }

  // Initialize socket lists
  sl_slist_init(&sli_wisun_socket_list_free);
  sl_slist_init(&sli_wisun_socket_list);

  for (i = 0; i < SLI_WISUN_TASK_MAX_SOCKETS; ++i) {
    sl_slist_push(&sli_wisun_socket_list_free, &sli_wisun_sockets[i].node);
  }

  // Attempt to load the certificates from manufacturing tokens
  // FIXME

  // Initialize Wi-SUN PHY driver for timer support
  sli_wisun_driver_init();

  // Enable PTI support
  // FIXME this is an application option
  sli_wisun_driver_set_pti_state(true);

  // Initialize NVM */
  sli_wisun_nvm_init();

  // Initialize Nanostack HAL
  ns_hal_init(NULL, 0, NULL, NULL);

  // Initialize Nanostack network subsystem
  ret = net_init_core();
  tr_debug("net_init_core: %d", ret);

  // Register the task as an event OS handler
  sli_wisun_task_event_handler_id = eventOS_event_handler_create(sli_wisun_task_event_handler,
                                                                 ARM_LIB_TASKLET_INIT_EVENT);
  tr_debug("sli_wisun_task_event_handler_id: %d", sli_wisun_task_event_handler_id);

  // Periodic statistics print
  eventOS_event_timer_request(SLI_WISUN_APPLICATION_EVENT_STATISTICS,
                              APPLICATION_EVENT, sli_wisun_task_event_handler_id, 30000);

#if defined(SL_CATALOG_MICRIUMOS_KERNEL_PRESENT)
#if (OS_CFG_TASK_STK_REDZONE_EN == DEF_ENABLED)
  // Initialize application hook for stack overflow
  OS_AppRedzoneHitHookPtr = sli_wisun_stack_overflow;
#endif
#endif

  // Register Wi-SUN PHY driver
  sli_wisun_driver_register(&sli_wisun_driver_id);

  // Initialize Wi-SUN MAC
  mac_description_storage_size_t storage_sizes;
  storage_sizes.device_decription_table_size = 32;
  storage_sizes.key_description_table_size = 4;
  storage_sizes.key_lookup_size = 1;
  storage_sizes.key_usage_size = 3;
  mac_api = ns_sw_mac_create(sli_wisun_driver_id, &storage_sizes);

  // Initialize 6LoWPAN
  sli_wisun_interface_id = arm_nwk_interface_lowpan_init(mac_api, "WiSunInterface");
  tr_debug("arm_nwk_interface_lowpan_init: %d", sli_wisun_interface_id);

  // Store the built-in MAC address
  sli_wisun_get_mac_address(sli_wisun_interface_id, &sli_wisun_mac_address);

  // Certain statistics are enabled here instead of the connection request because
  // they require a pointer to the MAC API upon startup.

  // Initialize PHY statistics
  ret = ns_sw_mac_phy_statistics_start(mac_api, &sli_wisun_statistics.phy);
  tr_info("ns_sw_mac_phy_statistics_start: %d", ret);

  // Initialize MAC statistics
  ret = ns_sw_mac_statistics_start(mac_api, &sli_wisun_statistics.mac);
  tr_info("ns_sw_mac_statistics_start: %d", ret);

  // Ready to serve the next request
  assert((osEventFlagsSet(sli_wisun_task_flags,
                          SLI_WISUN_TASK_FLAG_READY) & CMSIS_RTOS_ERROR_MASK) == 0);

  sli_wisun_req_handler();
}

void sli_wisun_req_handler(void)
{
  uint32_t flags;
  SLI_WISUN_TASK_LOOP { // do
    // Wait for something to happen
    flags = osEventFlagsWait(sli_wisun_task_flags,
                             SLI_WISUN_TASK_FLAG_REQ_READY,
                             osFlagsWaitAny,
                             osWaitForever);
    assert((flags & CMSIS_RTOS_ERROR_MASK) == 0);

    // A request is ready to be handled
    if (flags & SLI_WISUN_TASK_FLAG_REQ_READY) {

      // Handle the pending request
      sli_wisun_task_req_handler(sli_wisun_pending_req, sli_wisun_pending_req_data, sli_wisun_pending_cnf, sli_wisun_pending_cnf_data);

      // Notify the confirmation is ready to be handled
      assert((osEventFlagsSet(sli_wisun_task_flags,
                              SLI_WISUN_TASK_FLAG_CNF_READY) & CMSIS_RTOS_ERROR_MASK) == 0);
    }
  }
}

sl_wisun_evt_t* sli_wisun_task_poll()
{
  sl_wisun_evt_t *evt = NULL;
  sli_wisun_queue_descriptor_t evt_descriptor_get = { NULL, 0 };
  uint8_t msg_prio;

  osMessageQueueGet(sli_wisun_task_api_events_queue,
                    (void *)&evt_descriptor_get,
                    &msg_prio,
                    0);

  evt = (sl_wisun_evt_t *)evt_descriptor_get.addr;

  if (evt) {
    return evt;
  }

  return NULL;
}

static void sli_wisun_task_req_handler(const void *req, const void *req_data, void *cnf, void *cnf_data)
{
  sl_wisun_msg_header_t *hdr = (sl_wisun_msg_header_t *)req;
  switch (hdr->id) {
    case SL_WISUN_MSG_SET_NETWORK_SIZE_REQ_ID:
      sli_wisun_task_req_set_network_size((const sl_wisun_msg_set_network_size_req_t *)req,
                                          (sl_wisun_msg_set_network_size_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_CONNECT_REQ_ID:
      sli_wisun_task_req_connect((const sl_wisun_msg_connect_req_t *)req,
                                 (sl_wisun_msg_connect_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_DISCONNECT_REQ_ID:
      sli_wisun_task_req_disconnect((const sl_wisun_msg_disconnect_req_t *)req,
                                    (sl_wisun_msg_disconnect_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_GET_IP_ADDRESS_REQ_ID:
      sli_wisun_task_req_get_ip_address((const sl_wisun_msg_get_ip_address_req_t *)req,
                                        (sl_wisun_msg_get_ip_address_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_OPEN_SOCKET_REQ_ID:
      sli_wisun_task_req_open_socket((const sl_wisun_msg_open_socket_req_t *)req,
                                     (sl_wisun_msg_open_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_CLOSE_SOCKET_REQ_ID:
      sli_wisun_task_req_close_socket((const sl_wisun_msg_close_socket_req_t *)req,
                                      (sl_wisun_msg_close_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SENDTO_ON_SOCKET_REQ_ID:
      sli_wisun_task_req_sendto_on_socket((const sl_wisun_msg_sendto_on_socket_req_t *)req,
                                          (const uint8_t *)req_data,
                                          (sl_wisun_msg_sendto_on_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_LISTEN_ON_SOCKET_REQ_ID:
      sli_wisun_task_req_listen_on_socket((const sl_wisun_msg_listen_on_socket_req_t *)req,
                                          (sl_wisun_msg_listen_on_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_ACCEPT_ON_SOCKET_REQ_ID:
      sli_wisun_task_req_accept_on_socket((const sl_wisun_msg_accept_on_socket_req_t *)req,
                                          (sl_wisun_msg_accept_on_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_CONNECT_SOCKET_REQ_ID:
      sli_wisun_task_req_connect_socket((const sl_wisun_msg_connect_socket_req_t *)req,
                                        (sl_wisun_msg_connect_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_BIND_SOCKET_REQ_ID:
      sli_wisun_task_req_bind_socket((const sl_wisun_msg_bind_socket_req_t *)req,
                                     (sl_wisun_msg_bind_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SEND_ON_SOCKET_REQ_ID:
      sli_wisun_task_req_send_on_socket((const sl_wisun_msg_send_on_socket_req_t *)req,
                                        (const uint8_t *)req_data,
                                        (sl_wisun_msg_send_on_socket_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_RECEIVE_ON_SOCKET_REQ_ID:
      sli_wisun_task_req_receive_on_socket((const sl_wisun_msg_receive_on_socket_req_t *)req,
                                           (sl_wisun_msg_receive_on_socket_cnf_t *)cnf,
                                           (uint8_t *)cnf_data);
      break;
    case SL_WISUN_MSG_SET_TRUSTED_CERTIFICATE_REQ_ID:
      sli_wisun_task_req_set_trusted_certificate((const sl_wisun_msg_set_trusted_certificate_req_t *)req,
                                                 (const uint8_t *)req_data,
                                                 (sl_wisun_msg_set_trusted_certificate_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SET_DEVICE_CERTIFICATE_REQ_ID:
      sli_wisun_task_req_set_device_certificate((const sl_wisun_msg_set_device_certificate_req_t *)req,
                                                (const uint8_t *)req_data,
                                                (sl_wisun_msg_set_device_certificate_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SET_DEVICE_PRIVATE_KEY_REQ_ID:
      sli_wisun_task_req_set_device_private_key((const sl_wisun_msg_set_device_private_key_req_t *)req,
                                                (const uint8_t *)req_data,
                                                (sl_wisun_msg_set_device_private_key_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_GET_STATISTICS_REQ_ID:
      sli_wisun_task_req_get_statistics((const sl_wisun_msg_get_statistics_req_t *)req,
                                        (sl_wisun_msg_get_statistics_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SET_SOCKET_OPTION_REQ_ID:
      sli_wisun_task_req_set_socket_option((const sl_wisun_msg_set_socket_option_req_t *)req,
                                           (sl_wisun_msg_set_socket_option_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SET_TX_POWER_REQ_ID:
      sli_wisun_task_req_set_tx_power((const sl_wisun_msg_set_tx_power_req_t *)req,
                                      (sl_wisun_msg_set_tx_power_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SET_CHANNEL_PLAN_REQ_ID:
      sli_wisun_task_req_set_channel_plan((const sl_wisun_msg_set_channel_plan_req_t *)req,
                                          (sl_wisun_msg_set_channel_plan_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SET_CHANNEL_MASK_REQ_ID:
      sli_wisun_task_req_set_channel_mask((const sl_wisun_msg_set_channel_mask_req_t *)req,
                                          (sl_wisun_msg_set_channel_mask_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_ALLOW_MAC_ADDRESS_REQ_ID:
      sli_wisun_task_req_allow_mac_address((const sl_wisun_msg_allow_mac_address_req_t *)req,
                                          (sl_wisun_msg_allow_mac_address_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_DENY_MAC_ADDRESS_REQ_ID:
      sli_wisun_task_req_deny_mac_address((const sl_wisun_msg_deny_mac_address_req_t *)req,
                                          (sl_wisun_msg_deny_mac_address_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_GET_SOCKET_OPTION_REQ_ID:
      sli_wisun_task_req_get_socket_option((const sl_wisun_msg_get_socket_option_req_t *)req,
                                           (sl_wisun_msg_get_socket_option_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_GET_JOIN_STATE_REQ_ID:
      sli_wisun_task_req_get_join_state((const sl_wisun_msg_get_join_state_req_t *)req,
                                        (sl_wisun_msg_get_join_state_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_CLEAR_CREDENTIAL_CACHE_REQ_ID:
      sli_wisun_task_req_clear_credential_cache((const sl_wisun_msg_clear_credential_cache_req_t *)req,
                                                (sl_wisun_msg_clear_credential_cache_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_GET_MAC_ADDRESS_REQ_ID:
      sli_wisun_task_req_get_mac_address((const sl_wisun_msg_get_mac_address_req_t *)req,
                                         (sl_wisun_msg_get_mac_address_cnf_t *)cnf);
      break;
    case SL_WISUN_MSG_SET_MAC_ADDRESS_REQ_ID:
      sli_wisun_task_req_set_mac_address((const sl_wisun_msg_set_mac_address_req_t *)req,
                                         (sl_wisun_msg_set_mac_address_cnf_t *)cnf);
      break;
    default:
      sli_wisun_task_req_default(hdr, (sl_wisun_msg_generic_cnf_t *)cnf);
      break;
  }
}

static void sli_wisun_task_req_set_network_size(const sl_wisun_msg_set_network_size_req_t *req,
                                                sl_wisun_msg_set_network_size_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;

  tr_debug("sli_wisun_task_req_set_network_size()");

  switch (req->body.size) {
    case SL_WISUN_NETWORK_SIZE_AUTOMATIC:
      sli_wisun_settings.network_size = NETWORK_SIZE_SMALL;
      break;
    case SL_WISUN_NETWORK_SIZE_SMALL:
      sli_wisun_settings.network_size = NETWORK_SIZE_SMALL;
      break;
    case SL_WISUN_NETWORK_SIZE_MEDIUM:
      sli_wisun_settings.network_size = NETWORK_SIZE_MEDIUM;
      break;
    case SL_WISUN_NETWORK_SIZE_LARGE:
      sli_wisun_settings.network_size = NETWORK_SIZE_LARGE;
      break;
    case SL_WISUN_NETWORK_SIZE_TEST:
      sli_wisun_settings.network_size = NETWORK_SIZE_SMALL;
      break;
    case SL_WISUN_NETWORK_SIZE_CERTIFICATION:
      sli_wisun_settings.network_size = NETWORK_SIZE_SMALL;
      break;
    default:
      tr_error("sli_wisun_task_req_set_network_size: unsupported network size");
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_TYPE);
  }

  if (req->body.size == SL_WISUN_NETWORK_SIZE_TEST) {
    sli_wisun_settings.timing.discovery_trickle_imin = SLI_WISUN_SETTING_DISCOVERY_IMIN_FAST;
    sli_wisun_settings.timing.discovery_trickle_imax = SLI_WISUN_SETTING_DISCOVERY_IMAX_FAST;
    sli_wisun_settings.timing.security_initial_min = SLI_WISUN_SETTING_SECURITY_INITIAL_MIN_FAST;
    sli_wisun_settings.timing.security_initial_max = SLI_WISUN_SETTING_SECURITY_INITIAL_MAX_FAST;
  } else {
    sli_wisun_settings.timing.discovery_trickle_imin = SLI_WISUN_SETTING_DISCOVERY_DEFAULT;
    sli_wisun_settings.timing.discovery_trickle_imax = SLI_WISUN_SETTING_DISCOVERY_DEFAULT;
    sli_wisun_settings.timing.security_initial_min = SLI_WISUN_SETTING_SECURITY_DEFAULT;
    sli_wisun_settings.timing.security_initial_max = SLI_WISUN_SETTING_SECURITY_DEFAULT;
  }

  if (req->body.size == SL_WISUN_NETWORK_SIZE_CERTIFICATION) {
    sli_wisun_settings.timing.gtk_max_mismatch = SLI_WISUN_SETTING_GTK_MAX_MISMATCH_CERTIF;
  } else {
    sli_wisun_settings.timing.gtk_max_mismatch = SLI_WISUN_SETTING_GTK_MAX_MISMATCH_DEFAULT;
  }

  if (sli_wisun_task_state == SLI_WISUN_TASK_STATE_CONNECTED) {
    status = sli_wisun_task_set_network_size(sli_wisun_interface_id,
                                             sli_wisun_settings.network_size,
                                             &sli_wisun_settings.timing);
    SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);
  } else {
    // Network size will taken into use in the next connect
  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_NETWORK_SIZE_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_network_size_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_connect(const sl_wisun_msg_connect_req_t *req,
                                       sl_wisun_msg_connect_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sli_wisun_certificate_entry_t *entry;
  protocol_interface_info_entry_t *interface_info;
  fhss_timer_t *fhss_timer_ptr;
  uint8_t reg_domain;
  uint8_t op_class;
  int ret;

  tr_debug("sli_wisun_task_req_connect()");

  if( sli_wisun_task_state != SLI_WISUN_TASK_STATE_INITIALIZED) {
    // Already connecting or connected
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_NETWORK_UP);
  }

  // At least one trusted certificate exists
  entry = SL_SLIST_ENTRY(sli_wisun_trusted_certificate_list, sli_wisun_certificate_entry_t, node);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(entry, SL_STATUS_FAIL);

  // The device certificate exists
  entry = SL_SLIST_ENTRY(sli_wisun_certificate_list, sli_wisun_certificate_entry_t, node);
  if (!entry || !(entry->flags & SLI_WISUN_CERTIFICATE_FLAG_HAS_KEY)) {
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_FAIL);
  }

  // If application specific channel plan is used, channel plan settings
  // must exist.
  if (req->body.reg_domain == SL_WISUN_REGULATORY_DOMAIN_APP) {
    if (!sli_wisun_settings.phy.ch0_frequency ||
        !sli_wisun_settings.phy.number_of_channels) {
      tr_error("sli_wisun_task_req_connect: invalid channel plan");
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_FAIL);
    }
  }

  reg_domain = req->body.reg_domain;
  if (reg_domain == SL_WISUN_REGULATORY_DOMAIN_APP) {
    // Application specific value is different in Nanostack
    reg_domain = REG_DOMAIN_APP;
  }

  op_class = req->body.op_class;
  if (op_class == SL_WISUN_OPERATING_CLASS_APP) {
    // Application specific value is different in Nanostack
    op_class = OPERATING_MODE_APP;
  }

  // Pointer to the FHSS timer implementation
  fhss_timer_ptr = &fhss_functions;

  // MAC address may have changed
  status = sli_wisun_set_mac_address(sli_wisun_interface_id, &sli_wisun_mac_address);
  SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);

  ret = arm_nwk_interface_configure_6lowpan_bootstrap_set(sli_wisun_interface_id,
                                                          NET_6LOWPAN_ROUTER,
                                                          NET_6LOWPAN_WS);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = ws_management_node_init(sli_wisun_interface_id,
                                reg_domain,
                                (char *)req->body.name,
                                fhss_timer_ptr);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = ws_management_regulatory_domain_set(sli_wisun_interface_id,
                                            reg_domain,
                                            op_class,
                                            req->body.op_mode);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  if (req->body.reg_domain == SL_WISUN_REGULATORY_DOMAIN_APP) {
    ret = ws_management_channel_plan_set(sli_wisun_interface_id,
                                         1, // application specific plan
                                         CHANNEL_FUNCTION_DH1CF, // will be overwritten by FHSS settings
                                         CHANNEL_FUNCTION_DH1CF, // will be overwritten by FHSS settings
                                         sli_wisun_settings.phy.ch0_frequency,
                                         sli_wisun_settings.phy.channel_spacing,
                                         sli_wisun_settings.phy.number_of_channels);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  }

  status = sli_wisun_task_set_network_size(sli_wisun_interface_id,
                                           sli_wisun_settings.network_size,
                                           &sli_wisun_settings.timing);
  SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);

  status = sli_wisun_task_set_fhss(sli_wisun_interface_id,
                                   &sli_wisun_settings.fhss);
  SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);

  status = sli_wisun_set_access_list(sli_wisun_interface_id,
                                     &sli_wisun_access_list);
  SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);

  // Configure trusted certificate chain
  arm_network_trusted_certificates_remove();
  SL_SLIST_FOR_EACH_ENTRY(sli_wisun_trusted_certificate_list, entry, sli_wisun_certificate_entry_t, node) {
    ret = arm_network_trusted_certificate_add(&entry->certificate);
    tr_debug("arm_network_trusted_certificate_add: %d", ret);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  }

  // Configure own certificate chain
  arm_network_own_certificates_remove();
  SL_SLIST_FOR_EACH_ENTRY(sli_wisun_certificate_list, entry, sli_wisun_certificate_entry_t, node) {
    ret = arm_network_own_certificate_add(&entry->certificate);
    tr_debug("arm_network_own_certificate_add: %d", ret);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  }

  // Nanostack expects eventOS to have an active tasklet when arm_nwk_interface_up
  // is called, this tasklet will then receive the ARM_LIB_NWK_INTERFACE_EVENT events
  // related to the state of the connection.
  //
  // In mbed OS this call is not needed because arm_nwk_interface_up is called
  // from an event handler and thus the correct tasklet is active.
  eventOS_scheduler_set_active_tasklet(sli_wisun_task_event_handler_id);

  ret = arm_nwk_interface_up(sli_wisun_interface_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Workaround due to lack of proper FHSS statistics API
  interface_info = protocol_stack_interface_info_get_by_id(sli_wisun_interface_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(interface_info != NULL, SL_STATUS_FAIL);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(interface_info->ws_info != NULL, SL_STATUS_FAIL);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(interface_info->ws_info->fhss_api != NULL, SL_STATUS_FAIL);

  // Initialize FHSS statistics
  ret = ns_fhss_statistics_start(interface_info->ws_info->fhss_api, &sli_wisun_statistics.fhss);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Initialize 6LoWPAN statistics
  protocol_stats_start(&sli_wisun_statistics.network);

  // Initialize Wi-SUN statistics
  ret = ws_statistics_start(sli_wisun_interface_id, &sli_wisun_statistics.wisun);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Task state
  sli_wisun_task_state = SLI_WISUN_TASK_STATE_CONNECTING;

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_CONNECT_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_connect_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_get_ip_address(const sl_wisun_msg_get_ip_address_req_t *req,
                                              sl_wisun_msg_get_ip_address_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  rpl_dodag_info_t dodag_info;

  tr_debug("sli_wisun_task_req_get_ip_address()");

  if (req->body.address_type == SL_WISUN_IP_ADDRESS_TYPE_LINK_LOCAL) {
    ret = arm_net_address_get(sli_wisun_interface_id, ADDR_IPV6_LL, cnf->body.address.address);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  } else if (req->body.address_type == SL_WISUN_IP_ADDRESS_TYPE_GLOBAL) {
    ret = arm_net_address_get(sli_wisun_interface_id, ADDR_IPV6_GP, cnf->body.address.address);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  } else {
    ret = sli_wisun_get_dodag_info(sli_wisun_interface_id, &dodag_info);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

    if (req->body.address_type == SL_WISUN_IP_ADDRESS_TYPE_BORDER_ROUTER) {
      memcpy(cnf->body.address.address, dodag_info.dodag_id, SL_WISUN_IP_ADDRESS_SIZE);
    } else if ((req->body.address_type == SL_WISUN_IP_ADDRESS_TYPE_PRIMARY_PARENT) &&
               (dodag_info.parent_flags & RPL_PRIMARY_PARENT_SET)) {
      memcpy(cnf->body.address.address, dodag_info.primary_parent, SL_WISUN_IP_ADDRESS_SIZE);
    } else if ((req->body.address_type == SL_WISUN_IP_ADDRESS_TYPE_SECONDARY_PARENT) &&
               (dodag_info.parent_flags & RPL_SECONDARY_PARENT_SET)) {
      memcpy(cnf->body.address.address, dodag_info.secondary_parent, SL_WISUN_IP_ADDRESS_SIZE);
    } else {
      // Address not available
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_NOT_FOUND);
    }
  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_GET_IP_ADDRESS_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_get_ip_address_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_open_socket(const sl_wisun_msg_open_socket_req_t *req,
                                           sl_wisun_msg_open_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  uint8_t protocol;
  sli_wisun_socket_t *socket = NULL;

  tr_debug("sli_wisun_task_req_open_socket()");

  switch (req->body.protocol) {
    case SL_WISUN_SOCKET_PROTOCOL_UDP:
      protocol = SOCKET_UDP;
      break;
    case SL_WISUN_SOCKET_PROTOCOL_TCP:
      protocol = SOCKET_TCP;
      break;
    case SL_WISUN_SOCKET_PROTOCOL_ICMP:
      protocol = SOCKET_ICMP;
      break;
    default:
      tr_error("sli_wisun_task_req_open_socket: unsupported protocol type");
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_TYPE);
  }

  socket = sli_wisun_task_allocate_socket();
  SLI_WISUN_ERROR_CHECK_SET_STATUS(socket != NULL, SL_STATUS_FAIL);

  ret = socket_open(protocol, SLI_WISUN_SOCKET_PORT_AUTOMATIC, sli_wisun_task_socket_event_handler);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Store socket ID
  socket->socket_id = ret;

  // Confirmation body
  cnf->body.socket_id = ret;

  ret = sli_wisun_task_set_default_socket_options(req->body.protocol, socket);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Push the socket to the active socket list
  sl_slist_push_back(&sli_wisun_socket_list, &socket->node);

error_handler:

  if ((socket) && (status != SL_STATUS_OK)) {
    if (socket->socket_id != SL_WISUN_INVALID_SOCKET_ID) {
      socket_close(socket->socket_id);
    }
    sli_wisun_task_free_socket(socket);
  }

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_OPEN_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_open_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_close_socket(const sl_wisun_msg_close_socket_req_t *req,
                                            sl_wisun_msg_close_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sli_wisun_socket_t *socket = NULL;
  int ret;

  tr_debug("sli_wisun_task_req_close_socket()");

  socket = sli_wisun_task_get_socket(req->body.socket_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(socket != NULL, SL_STATUS_FAIL);

  ret = socket_close(req->body.socket_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  sli_wisun_task_free_socket(socket);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_CLOSE_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_close_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_sendto_on_socket(const sl_wisun_msg_sendto_on_socket_req_t *req,
                                                const uint8_t *req_data,
                                                sl_wisun_msg_sendto_on_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  ns_address_t remote_address;

  tr_debug("sli_wisun_task_req_sendto_on_socket()");

  remote_address.type = ADDRESS_IPV6;
  memcpy(remote_address.address, req->body.remote_address.address, SL_WISUN_IP_ADDRESS_SIZE);
  remote_address.identifier = req->body.remote_port;

  ret = socket_sendto(req->body.socket_id,
                      &remote_address,
                      req_data,
                      req->body.data_length);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret != NS_EWOULDBLOCK, SL_STATUS_WOULD_OVERFLOW);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SENDTO_ON_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_sendto_on_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_listen_on_socket(const sl_wisun_msg_listen_on_socket_req_t *req,
                                                sl_wisun_msg_listen_on_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;

  tr_debug("sli_wisun_task_req_listen_on_socket()");

  ret = socket_listen(req->body.socket_id, SLI_WISUN_SOCKET_BACKLOG);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_LISTEN_ON_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_listen_on_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_accept_on_socket(const sl_wisun_msg_accept_on_socket_req_t *req,
                                                sl_wisun_msg_accept_on_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  ns_address_t remote_address;
  sli_wisun_socket_t *socket = NULL;

  tr_debug("sli_wisun_task_req_accept_on_socket()");

  socket = sli_wisun_task_allocate_socket();
  SLI_WISUN_ERROR_CHECK_SET_STATUS(socket != NULL, SL_STATUS_FAIL);

  ret = socket_accept(req->body.socket_id, &remote_address, sli_wisun_task_socket_event_handler);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Store socket ID
  socket->socket_id = ret;

  // Confirmation body
  cnf->body.remote_socket_id = ret;
  memcpy(cnf->body.remote_address.address, remote_address.address, SL_WISUN_IP_ADDRESS_SIZE);
  cnf->body.remote_port = remote_address.identifier;

  ret = sli_wisun_task_set_default_socket_options(SL_WISUN_SOCKET_PROTOCOL_TCP, socket);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Push the socket to the active socket list
  sl_slist_push_back(&sli_wisun_socket_list, &socket->node);

error_handler:

  if ((socket) && (status != SL_STATUS_OK)) {
    if (socket->socket_id != SL_WISUN_INVALID_SOCKET_ID) {
      socket_close(socket->socket_id);
    }
    sli_wisun_task_free_socket(socket);
  }

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_ACCEPT_ON_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_accept_on_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_connect_socket(const sl_wisun_msg_connect_socket_req_t *req,
                                              sl_wisun_msg_connect_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  ns_address_t remote_address;

  tr_debug("sli_wisun_task_req_connect_socket()");

  remote_address.type = ADDRESS_IPV6;
  remote_address.identifier = req->body.remote_port;
  memcpy(remote_address.address, req->body.remote_address.address, SL_WISUN_IP_ADDRESS_SIZE);

  ret = socket_connect(req->body.socket_id, &remote_address, true);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_CONNECT_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_connect_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_bind_socket(const sl_wisun_msg_bind_socket_req_t *req,
                                           sl_wisun_msg_bind_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  ns_address_t local_address;

  tr_debug("sli_wisun_task_req_bind_socket()");

  // Because zero is treated similarly in both APIs, the IP address and the port
  // can just be copied.
  local_address.type = ADDRESS_IPV6;
  local_address.identifier = req->body.local_port;
  memcpy(local_address.address, req->body.local_address.address, SL_WISUN_IP_ADDRESS_SIZE);

  ret = socket_bind(req->body.socket_id, &local_address);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_BIND_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_bind_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_send_on_socket(const sl_wisun_msg_send_on_socket_req_t *req,
                                              const uint8_t *req_data,
                                              sl_wisun_msg_send_on_socket_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;

  tr_debug("sli_wisun_task_req_send_on_socket()");

  ret = socket_send(req->body.socket_id,
                    req_data,
                    req->body.data_length);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret != NS_EWOULDBLOCK, SL_STATUS_WOULD_OVERFLOW);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SEND_ON_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_send_on_socket_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_receive_on_socket(const sl_wisun_msg_receive_on_socket_req_t *req,
                                                 sl_wisun_msg_receive_on_socket_cnf_t *cnf,
                                                 uint8_t *cnf_data)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  ns_address_t remote_address;

  tr_debug("sli_wisun_task_req_receive_on_socket()");

  // Make sure confirmation data length is zero in case reading from the socket fails.
  cnf->body.data_length = 0;

  ret = socket_recvfrom(req->body.socket_id,
                        cnf_data,
                        req->body.data_length,
                        0, // flags
                        &remote_address);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Confirmation body
  memcpy(cnf->body.remote_address.address, remote_address.address, SL_WISUN_IP_ADDRESS_SIZE);
  cnf->body.remote_port = remote_address.identifier;
  cnf->body.data_length = ret;

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_RECEIVE_ON_SOCKET_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_receive_on_socket_cnf_t) + cnf->body.data_length;
  cnf->body.status = status;
}

static void sli_wisun_task_req_disconnect(const sl_wisun_msg_disconnect_req_t *req,
                                          sl_wisun_msg_disconnect_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  (void)req;

  tr_debug("sli_wisun_task_req_disconnect()");

  if( sli_wisun_task_state == SLI_WISUN_TASK_STATE_INITIALIZED) {
    // Already disconnected
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_NETWORK_DOWN);
  }

  ret = arm_nwk_interface_down(sli_wisun_interface_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  // Task state
  sli_wisun_task_state = SLI_WISUN_TASK_STATE_INITIALIZED;

  // There are no asynchronous operations to perform during the disconnect,
  // notify the application immediately.
  //sli_wisun_task_ind_disconnected();

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_DISCONNECT_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_disconnect_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_set_trusted_certificate(const sl_wisun_msg_set_trusted_certificate_req_t *req,
                                                       const uint8_t *req_data,
                                                       sl_wisun_msg_set_trusted_certificate_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sli_wisun_certificate_entry_t	*entry = NULL;
  mbedtls_x509_crt cert;

  tr_debug("sli_wisun_task_req_set_trusted_certificate()");

  mbedtls_x509_crt_init(&cert);
  if (mbedtls_x509_crt_parse(&cert, req_data, req->body.certificate_length)) {
    // Certificate is not valid
    mbedtls_x509_crt_free(&cert);
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_PARAMETER);
  }
  mbedtls_x509_crt_free(&cert);

  if (!(req->body.certificate_options & SL_WISUN_CERTIFICATE_OPTION_APPEND)) {
    // Delete the previous certificates
    sli_wisun_task_free_certificate_list(&sli_wisun_trusted_certificate_list);
  }

  if (req->body.certificate_options & SL_WISUN_CERTIFICATE_OPTION_HAS_KEY) {
    // A trusted certificate cannot have a private key
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_PARAMETER);
  }

  entry = sli_wisun_task_allocate_certificate_entry();
  if (!entry) {
    // Unable to allocate a certificate entry
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_NO_MORE_RESOURCE);
  }

  if (req->body.certificate_options & SL_WISUN_CERTIFICATE_OPTION_IS_REF) {
    // Certificate data will remain in scope, no need to copy
    entry->certificate.cert_len = req->body.certificate_length;
    entry->certificate.cert = req_data;
  } else {
    // Take a copy of the certificate data
    entry->certificate.cert_len = req->body.certificate_length;
    entry->certificate.cert = sli_wisun_duplicate_data(req->body.certificate_length, req_data);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(entry->certificate.cert, SL_STATUS_NO_MORE_RESOURCE);

    entry->flags |= SLI_WISUN_CERTIFICATE_FLAG_IS_OWNED;
  }

  // Add the certificate to the list of trusted certificates
  sli_wisun_task_add_certificate_entry(&sli_wisun_trusted_certificate_list, entry);

error_handler:

  if (status != SL_STATUS_OK) {
    sli_wisun_task_free_certificate_entry(entry);
  }

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_TRUSTED_CERTIFICATE_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_trusted_certificate_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_set_device_certificate(const sl_wisun_msg_set_device_certificate_req_t *req,
                                                      const uint8_t *req_data,
                                                      sl_wisun_msg_set_device_certificate_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sli_wisun_certificate_entry_t	*entry = NULL;
  mbedtls_x509_crt cert;

  tr_debug("sli_wisun_task_req_set_device_certificate()");

  mbedtls_x509_crt_init(&cert);
  if (mbedtls_x509_crt_parse(&cert, req_data, req->body.certificate_length)) {
    // Certificate is not valid
    mbedtls_x509_crt_free(&cert);
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_PARAMETER);
  }
  mbedtls_x509_crt_free(&cert);

  if (!(req->body.certificate_options & SL_WISUN_CERTIFICATE_OPTION_APPEND)) {
    // Delete the previous certificates
    sli_wisun_task_free_certificate_list(&sli_wisun_certificate_list);
  }

  if (req->body.certificate_options & SL_WISUN_CERTIFICATE_OPTION_HAS_KEY) {
    entry = SL_SLIST_ENTRY(sli_wisun_certificate_list, sli_wisun_certificate_entry_t, node);

    if (entry && (entry->flags & SLI_WISUN_CERTIFICATE_FLAG_HAS_KEY)) {
      entry = NULL;

      // Device certificate already exists
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_ALREADY_EXISTS);
    }
  }

  entry = sli_wisun_task_allocate_certificate_entry();
  if (!entry) {
    // Unable to allocate a certificate entry
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_NO_MORE_RESOURCE);
  }

  if (req->body.certificate_options & SL_WISUN_CERTIFICATE_OPTION_IS_REF) {
    // Certificate data will remain in scope, no need to copy
    entry->certificate.cert_len = req->body.certificate_length;
    entry->certificate.cert = req_data;
  } else {
    // Take a copy of the certificate data
    entry->certificate.cert_len = req->body.certificate_length;
    entry->certificate.cert = sli_wisun_duplicate_data(req->body.certificate_length, req_data);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(entry->certificate.cert, SL_STATUS_NO_MORE_RESOURCE);

    entry->flags |= SLI_WISUN_CERTIFICATE_FLAG_IS_OWNED;
  }

  if (req->body.certificate_options & SL_WISUN_CERTIFICATE_OPTION_HAS_KEY) {
    entry->flags |= SLI_WISUN_CERTIFICATE_FLAG_HAS_KEY;
  }

  // Add the certificate to the list of certificates
  sli_wisun_task_add_certificate_entry(&sli_wisun_certificate_list, entry);

error_handler:

  if (status != SL_STATUS_OK) {
    sli_wisun_task_free_certificate_entry(entry);
  }

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_DEVICE_CERTIFICATE_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_device_certificate_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_set_device_private_key(const sl_wisun_msg_set_device_private_key_req_t *req,
                                                      const uint8_t *req_data,
                                                      sl_wisun_msg_set_device_private_key_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sli_wisun_certificate_entry_t	*entry = NULL;
  mbedtls_pk_context pk;

  tr_debug("sli_wisun_task_req_set_device_private_key()");

  mbedtls_pk_init(&pk);
  if (mbedtls_pk_parse_key(&pk, req_data, req->body.key_length, NULL, 0)) {
    // Private key is not valid
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_KEY);
  }
  mbedtls_pk_free(&pk);

  // Device certificate is always the first on the certificate list
  entry = SL_SLIST_ENTRY(sli_wisun_certificate_list, sli_wisun_certificate_entry_t, node);

  if (!entry || !(entry->flags & SLI_WISUN_CERTIFICATE_FLAG_HAS_KEY)) {
    // Not a device certificate
    SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_NOT_FOUND);
  }

  if (entry->certificate.key) {
    // Device private key already exists, delete the previous key
    if (entry->flags & SLI_WISUN_CERTIFICATE_FLAG_KEY_IS_OWNED) {
      sl_free((void *)entry->certificate.key);
    }
    entry->certificate.key = NULL;
  }

  if (req->body.key_options & SL_WISUN_PRIVATE_KEY_OPTION_IS_REF) {
    // Private key data will remain in scope, no need to copy
    entry->certificate.key_len = req->body.key_length;
    entry->certificate.key = req_data;
  } else {
    // Take a copy of the private key data
    entry->certificate.key_len = req->body.key_length;
    entry->certificate.key = sli_wisun_duplicate_data(req->body.key_length, req_data);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(entry->certificate.key, SL_STATUS_NO_MORE_RESOURCE);

    entry->flags |= SLI_WISUN_CERTIFICATE_FLAG_KEY_IS_OWNED;
  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_DEVICE_PRIVATE_KEY_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_device_private_key_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_get_statistics(const sl_wisun_msg_get_statistics_req_t *req,
                                              sl_wisun_msg_get_statistics_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;

  tr_debug("sli_wisun_task_req_get_statistics()");

  // The caller guarantees the aligment of the confirmation buffer
  // and message fields themselves are always aligned, thus the
  // warning can be ignored.
  #ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpragmas"
  #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
  #elif defined __ICCARM__
  #pragma diag_suppress=Pa039
  #endif

  switch (req->body.statistics_type) {
    case SL_WISUN_STATISTICS_TYPE_PHY:
      sli_wisun_get_phy_statistics(&cnf->body.statistics.phy);
      break;
    case SL_WISUN_STATISTICS_TYPE_MAC:
      sli_wisun_get_mac_statistics(&cnf->body.statistics.mac);
      break;
    case SL_WISUN_STATISTICS_TYPE_FHSS:
      sli_wisun_get_fhss_statistics(&cnf->body.statistics.fhss);
      break;
    case SL_WISUN_STATISTICS_TYPE_WISUN:
      sli_wisun_get_wisun_statistics(&cnf->body.statistics.wisun);
      break;
    case SL_WISUN_STATISTICS_TYPE_NETWORK:
      sli_wisun_get_network_statistics(&cnf->body.statistics.network);
      break;
    default:
      // Unknown statistics type
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_TYPE);

  // Restore the defaults
  #ifdef __GNUC__
  #pragma GCC diagnostic pop
  #elif defined __ICCARM__
  #pragma diag_default=Pa039
  #endif

  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_GET_STATISTICS_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_get_statistics_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_set_socket_option(const sl_wisun_msg_set_socket_option_req_t *req, sl_wisun_msg_set_socket_option_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sli_wisun_socket_t *socket = NULL;

  tr_debug("sli_wisun_task_req_set_socket_option()");

  socket = sli_wisun_task_get_socket(req->body.socket_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(socket != NULL, SL_STATUS_FAIL);

  // The caller guarantees the aligment of the request buffer
  // and message fields themselves are always aligned, thus the
  // warning can be ignored.
  #ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpragmas"
  #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
  #elif defined __ICCARM__
  #pragma diag_suppress=Pa039
  #endif

  switch (req->body.option) {
    case SL_WISUN_SOCKET_OPTION_EVENT_MODE:
      status = sli_wisun_socket_set_event_mode(socket, &req->body.option_data.event_mode);
      break;
    case SL_WISUN_SOCKET_OPTION_MULTICAST_GROUP:
      status = sli_wisun_socket_set_multicast_group(socket, &req->body.option_data.multicast_group);
      break;
    default:
      tr_error("sli_wisun_task_req_set_socket_option: unsupported socket option");
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_TYPE);
  }

  // Restore the defaults
  #ifdef __GNUC__
  #pragma GCC diagnostic pop
  #elif defined __ICCARM__
  #pragma diag_default=Pa039
  #endif

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_SOCKET_OPTION_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_socket_option_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_set_tx_power(const sl_wisun_msg_set_tx_power_req_t *req,
                                            sl_wisun_msg_set_tx_power_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;

  tr_debug("sli_wisun_task_req_set_tx_power()");

  sli_wisun_driver_set_tx_power(req->body.tx_power);

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_TX_POWER_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_tx_power_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_set_channel_plan(const sl_wisun_msg_set_channel_plan_req_t *req,
                                                sl_wisun_msg_set_channel_plan_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;

  tr_debug("sli_wisun_task_req_set_channel_plan()");

  switch (req->body.channel_spacing) {
    case SL_WISUN_CHANNEL_SPACING_100HZ:
      sli_wisun_settings.phy.channel_spacing = CHANNEL_SPACING_100;
      break;
    case SL_WISUN_CHANNEL_SPACING_200HZ:
      sli_wisun_settings.phy.channel_spacing = CHANNEL_SPACING_200;
      break;
    case SL_WISUN_CHANNEL_SPACING_400HZ:
      sli_wisun_settings.phy.channel_spacing = CHANNEL_SPACING_400;
      break;
    case SL_WISUN_CHANNEL_SPACING_600HZ:
      sli_wisun_settings.phy.channel_spacing = CHANNEL_SPACING_600;
      break;
    default:
      tr_error("sli_wisun_task_req_set_channel_plan: unsupported channel spacing");
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_TYPE);
  }

  sli_wisun_settings.phy.ch0_frequency = req->body.ch0_frequency;
  sli_wisun_settings.phy.number_of_channels = req->body.number_of_channels;

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_CHANNEL_PLAN_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_channel_plan_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_set_channel_mask(const sl_wisun_msg_set_channel_mask_req_t *req,
                                                sl_wisun_msg_set_channel_mask_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;

  tr_debug("sli_wisun_task_req_set_channel_mask()");

  ret = sli_wisun_set_channel_mask(&sli_wisun_settings.fhss, &req->body.channel_mask);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret > 0, SL_STATUS_FAIL);

  if (ret == 1) {
    // Only a single channel is enabled, disable frequency hopping
    sli_wisun_settings.fhss.channel_function = CHANNEL_FUNCTION_FIXED;
  } else {
    sli_wisun_settings.fhss.channel_function = CHANNEL_FUNCTION_DH1CF;
  }

  if (sli_wisun_task_state == SLI_WISUN_TASK_STATE_CONNECTED) {
    status = sli_wisun_task_set_fhss(sli_wisun_interface_id,
                                     &sli_wisun_settings.fhss);
    SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);
  } else {
    // Channel plan will taken into use in the next connect
  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_CHANNEL_MASK_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_channel_mask_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_allow_mac_address(const sl_wisun_msg_allow_mac_address_req_t *req,
                                                 sl_wisun_msg_allow_mac_address_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;

  status = sli_wisun_add_access_list_entry(SLI_WISUN_ACCESS_LIST_ACTION_ALLOW, &req->body.address);
  SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);

  if (sli_wisun_task_state >= SLI_WISUN_TASK_STATE_CONNECTING) {
    status = sli_wisun_set_access_list(sli_wisun_interface_id,
                                     &sli_wisun_access_list);
    SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);
  } else {
    // Access list will taken into use in the next connect
  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_ALLOW_MAC_ADDRESS_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_allow_mac_address_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_deny_mac_address(const sl_wisun_msg_deny_mac_address_req_t *req,
                                                sl_wisun_msg_deny_mac_address_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;

  status = sli_wisun_add_access_list_entry(SLI_WISUN_ACCESS_LIST_ACTION_DENY, &req->body.address);
  SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);

  if (sli_wisun_task_state >= SLI_WISUN_TASK_STATE_CONNECTING) {
    status = sli_wisun_set_access_list(sli_wisun_interface_id,
                                     &sli_wisun_access_list);
    SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);
  } else {
    // Access list will taken into use in the next connect
  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_DENY_MAC_ADDRESS_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_deny_mac_address_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_get_socket_option(const sl_wisun_msg_get_socket_option_req_t *req, sl_wisun_msg_get_socket_option_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sli_wisun_socket_t *socket = NULL;

  tr_debug("sli_wisun_task_req_get_socket_option()");

  socket = sli_wisun_task_get_socket(req->body.socket_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(socket != NULL, SL_STATUS_FAIL);

  // The caller guarantees the aligment of the request buffer
  // and message fields themselves are always aligned, thus the
  // warning can be ignored.
  #ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpragmas"
  #pragma GCC diagnostic ignored "-Waddress-of-packed-member"
  #endif

  switch (req->body.option) {
    case SL_WISUN_SOCKET_OPTION_SEND_BUFFER_LIMIT:
      cnf->body.option_data.send_buffer_limit.limit = socket->sndbuf_limit;
      break;
    default:
      tr_error("sli_wisun_task_req_get_socket_option: unsupported socket option");
      SLI_WISUN_ERROR_SET_STATUS(SL_STATUS_INVALID_TYPE);
  }

  // Restore the defaults
  #ifdef __GNUC__
  #pragma GCC diagnostic pop
  #endif

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_GET_SOCKET_OPTION_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_get_socket_option_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_get_join_state(const sl_wisun_msg_get_join_state_req_t *req, sl_wisun_msg_get_join_state_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  sl_wisun_join_state_t join_state = (sl_wisun_join_state_t)0;
  (void)req;

  tr_debug("sli_wisun_task_req_get_join_state()");

  ret = sli_wisun_get_join_state(sli_wisun_interface_id, &join_state);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_GET_JOIN_STATE_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_get_join_state_cnf_t);
  cnf->body.status = status;
  cnf->body.join_state = join_state;
}

static void sli_wisun_task_req_clear_credential_cache(const sl_wisun_msg_clear_credential_cache_req_t *req, sl_wisun_msg_clear_credential_cache_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  (void)req;

  tr_debug("sli_wisun_task_req_clear_credential_cache()");

  // Remove the credential cache from NVM.
  sli_wisun_nvm_clear();

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_CLEAR_CREDENTIAL_CACHE_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_clear_credential_cache_cnf_t);
  cnf->body.status = status;
}

static void sli_wisun_task_req_get_mac_address(const sl_wisun_msg_get_mac_address_req_t *req,
                                               sl_wisun_msg_get_mac_address_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;
  sl_wisun_mac_address_t address;
  (void)req;

  tr_debug("sli_wisun_task_req_get_mac_address()");

  // The returned MAC address is always the one in use. It may differ from
  // sli_wisun_mac_address if the MAC address has been updated while
  // a connection is ongoing.
  status = sli_wisun_get_mac_address(sli_wisun_interface_id, &address);
  SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_GET_MAC_ADDRESS_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_get_mac_address_cnf_t);
  cnf->body.status = status;
  cnf->body.address = address;
}

static void sli_wisun_task_req_set_mac_address(const sl_wisun_msg_set_mac_address_req_t *req,
                                               sl_wisun_msg_set_mac_address_cnf_t *cnf)
{
  sl_status_t status = SL_STATUS_OK;

  tr_debug("sli_wisun_task_req_set_mac_address()");

  sli_wisun_mac_address = req->body.address;

  if (sli_wisun_task_state == SLI_WISUN_TASK_STATE_INITIALIZED) {
    status = sli_wisun_set_mac_address(sli_wisun_interface_id, &sli_wisun_mac_address);
    SLI_WISUN_ERROR_CHECK(status == SL_STATUS_OK);
  } else {
    // MAC address will taken into use in the next connect
  }

error_handler:

  // Confirmation
  cnf->header.id = SL_WISUN_MSG_SET_MAC_ADDRESS_CNF_ID;
  cnf->header.length = sizeof(sl_wisun_msg_set_mac_address_cnf_t);
  cnf->body.status = status;
}

/**
 * Handles an unknown request and returns a generic confirmation with just
 * a COMMAND_IS_INVALID error status.
 * @param  req  Address of the request in memory
 * @param  cnf  Address of the confirmation in memory
 */
static void sli_wisun_task_req_default(const sl_wisun_msg_header_t *req,
                                       sl_wisun_msg_generic_cnf_t *cnf)
{
  tr_debug("sli_wisun_task_req_default()");

  // Confirmation
  cnf->header.id = req->id;
  cnf->header.length = sizeof(sl_wisun_msg_generic_cnf_t);
  cnf->body.status = SL_STATUS_COMMAND_IS_INVALID;
}

static sl_status_t sli_wisun_socket_set_event_mode(sli_wisun_socket_t *socket, const sl_wisun_socket_option_event_mode_t* event_mode)
{
  switch (event_mode->mode) {
    case SL_WISUN_SOCKET_EVENT_MODE_INDICATION:
      socket->flags &= ~(SLI_WISUN_SOCKET_FLAG_POLLING);
      break;
    case SL_WISUN_SOCKET_EVENT_MODE_POLLING:
      socket->flags |= SLI_WISUN_SOCKET_FLAG_POLLING;
      break;
    default:
      // Unknown mode
      return SL_STATUS_INVALID_TYPE;
  }

  return SL_STATUS_OK;
}

static sl_status_t sli_wisun_socket_set_multicast_group(sli_wisun_socket_t *socket, const sl_wisun_socket_option_multicast_group_t* multicast_group)
{
  ns_ipv6_mreq_t opt_value;
  uint8_t opt_name;
  int ret;

  tr_debug("sli_wisun_socket_set_multicast_group: %s (%lu)", tr_array(multicast_group->address.address, 16), multicast_group->action);

  if( sli_wisun_task_state == SLI_WISUN_TASK_STATE_INITIALIZED) {
    // Multicast requires an existing interface
    return SL_STATUS_NETWORK_DOWN;
  }

  switch (multicast_group->action) {
    case SL_WISUN_MULTICAST_GROUP_ACTION_JOIN:
      opt_name = SOCKET_IPV6_JOIN_GROUP;
      break;
    case SL_WISUN_SOCKET_EVENT_MODE_POLLING:
      opt_name = SOCKET_IPV6_LEAVE_GROUP;
      break;
    default:
      // Unknown action
      return SL_STATUS_INVALID_TYPE;
  }

  // Fill in the option value
  memcpy(opt_value.ipv6mr_multiaddr, multicast_group->address.address, SL_WISUN_IP_ADDRESS_SIZE);
  opt_value.ipv6mr_interface = sli_wisun_interface_id;

  ret = socket_setsockopt(socket->socket_id, SOCKET_IPPROTO_IPV6, opt_name, &opt_value, sizeof(opt_value));
  if(ret < 0)
  {
    tr_error("sli_wisun_socket_set_multicast_group: socket_setsockopt(%u) failed: %d", opt_name, ret);
    return SL_STATUS_FAIL;
  }

  return SL_STATUS_OK;
}

static sl_status_t sli_wisun_task_set_network_size(int8_t interface_id, uint8_t network_size, const sli_wisun_settings_timing_t *timing)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  uint16_t current_disc_trickle_imin;
  uint16_t current_disc_trickle_imax;
  uint8_t current_disc_trickle_k;
  uint16_t current_pan_timeout;
  uint16_t current_initial_key_imin;
  uint16_t current_initial_key_imax;
  uint16_t current_initial_key_min;
  uint16_t current_initial_key_max;
  uint16_t current_revocat_lifetime_reduct;
  uint16_t current_gtk_new_act_time;
  uint8_t current_gtk_new_install_req;
  uint16_t current_gtk_max_mismatch;

  // Force default timing parameters by first switching to NETWORK_SIZE_CERTIFICATE
  // and then to the desired network size. Otherwise the custom parameters may not
  // reset because Nanostack checks first whether the network size changed or not.
  ret = ws_management_network_size_set(interface_id,
                                       NETWORK_SIZE_CERTIFICATE);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = ws_management_network_size_set(interface_id,
                                       network_size);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = ws_management_timing_parameters_get(interface_id,
                                            &current_disc_trickle_imin,
                                            &current_disc_trickle_imax,
                                            &current_disc_trickle_k,
                                            &current_pan_timeout);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  if (timing->discovery_trickle_imin != SLI_WISUN_SETTING_DISCOVERY_DEFAULT) {
    current_disc_trickle_imin = timing->discovery_trickle_imin;
  }
  if (timing->discovery_trickle_imax != SLI_WISUN_SETTING_DISCOVERY_DEFAULT) {
    current_disc_trickle_imax = timing->discovery_trickle_imax;
  }

  // Custom timing parameters have to be set after the network size,
  // otherwise they will be overridden.
  ret = ws_management_timing_parameters_set(interface_id,
                                            current_disc_trickle_imin,
                                            current_disc_trickle_imax,
                                            current_disc_trickle_k,
                                            current_pan_timeout);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = ws_management_sec_prot_parameters_get(interface_id,
                                              &current_initial_key_imin,
                                              &current_initial_key_imax,
                                              &current_initial_key_min,
                                              &current_initial_key_max);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  if (timing->security_initial_min != SLI_WISUN_SETTING_SECURITY_DEFAULT) {
    current_initial_key_min = timing->security_initial_min;
  }
  if (timing->security_initial_max != SLI_WISUN_SETTING_SECURITY_DEFAULT) {
    current_initial_key_max = timing->security_initial_max;
  }

  // Custom security parameters have to be set after the network size,
  // otherwise they will be overridden.
  ret = ws_management_sec_prot_parameters_set(interface_id,
                                              current_initial_key_imin,
                                              current_initial_key_imax,
                                              current_initial_key_min,
                                              current_initial_key_max);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = ws_management_sec_timer_parameters_get(interface_id,
                                               &current_revocat_lifetime_reduct,
                                               &current_gtk_new_act_time,
                                               &current_gtk_new_install_req,
                                               &current_gtk_max_mismatch);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  if (timing->gtk_max_mismatch != SLI_WISUN_SETTING_GTK_MAX_MISMATCH_DEFAULT) {
    current_gtk_max_mismatch = timing->gtk_max_mismatch;
  }

  ret = ws_management_sec_timer_parameters_set(interface_id,
                                               current_revocat_lifetime_reduct,
                                               current_gtk_new_act_time,
                                               current_gtk_new_install_req,
                                               current_gtk_max_mismatch);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  return status;
}

static sl_status_t sli_wisun_task_set_fhss(int8_t interface_id, const sli_wisun_settings_fhss_t *fhss)
{
  sl_status_t status = SL_STATUS_OK;
  uint16_t channel = SLI_WISUN_CHANNEL_NONE;
  int ret;

  if (fhss->channel_function == CHANNEL_FUNCTION_FIXED) {
    channel = sli_wisun_get_channel_mask_channel(fhss);
  }

  ret = ws_management_fhss_unicast_channel_function_configure(interface_id,
                                                              fhss->channel_function,
                                                              channel,
                                                              fhss->uc_dwell_interval);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = ws_management_fhss_broadcast_channel_function_configure(interface_id,
                                                                fhss->channel_function,
                                                                channel,
                                                                fhss->bc_dwell_interval,
                                                                fhss->broadcast_interval);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  if (fhss->channel_function == CHANNEL_FUNCTION_FIXED) {
    // Ignore application channel mask if using a single channel,
    // no need to advertise exclusions.
    ret = ws_management_channel_mask_set(interface_id,
                                         (uint32_t *)SLI_WISUN_CHANNEL_MASK_ALL);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  } else {
    ret = ws_management_channel_mask_set(interface_id,
                                         (uint32_t *)fhss->channel_mask);
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  }

error_handler:

  return status;
}

static sl_status_t sli_wisun_set_access_list(int8_t interface_id, const sli_wisun_access_list_t *access_list)
{
  sl_status_t status = SL_STATUS_OK;
  int ret;
  int i;

  if (access_list->action == SLI_WISUN_ACCESS_LIST_ACTION_ALLOW) {
    tr_debug("mac_filter_start: MAC_FILTER_BLOCKED");
    ret = mac_filter_start(interface_id, MAC_FILTER_BLOCKED);
  } else {
    tr_debug("mac_filter_start: MAC_FILTER_ALLOWED");
    ret = mac_filter_start(interface_id, MAC_FILTER_ALLOWED);
  }
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  ret = mac_filter_clear(interface_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  for (i = 0; i < access_list->list_count; ++i) {
    if (access_list->action == SLI_WISUN_ACCESS_LIST_ACTION_ALLOW) {
      tr_debug("mac_filter_add_long: MAC_FILTER_ALLOWED: %s",
               tr_array(access_list->list[i].address, SL_WISUN_MAC_ADDRESS_SIZE));
      ret = mac_filter_add_long(interface_id, (uint8_t *)access_list->list[i].address, MAC_FILTER_ALLOWED);
    } else {
      tr_debug("mac_filter_add_long: MAC_FILTER_BLOCKED: %s",
               tr_array(access_list->list[i].address, SL_WISUN_MAC_ADDRESS_SIZE));
      ret = mac_filter_add_long(interface_id, (uint8_t *)access_list->list[i].address, MAC_FILTER_BLOCKED);
    }
    SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);
  }

error_handler:

  return status;
}

static sl_status_t sli_wisun_get_mac_address(int8_t interface_id, sl_wisun_mac_address_t *address)
{
  sl_status_t status = SL_STATUS_OK;
  link_layer_address_s params;
  int ret;

  ret = arm_nwk_mac_address_read(interface_id, &params);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

  memcpy(address->address, params.mac_long, SL_WISUN_MAC_ADDRESS_SIZE);

error_handler:

  return status;
}

static sl_status_t sli_wisun_set_mac_address(int8_t interface_id, const sl_wisun_mac_address_t *address)
{
  sl_status_t status = SL_STATUS_OK;
  protocol_interface_info_entry_t *interface_info;
  int ret;

  // There is no network interface API for setting the MAC address,
  // MAC helper API is used instead.
  interface_info = protocol_stack_interface_info_get_by_id(interface_id);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(interface_info != NULL, SL_STATUS_FAIL);

  ret = mac_helper_mac64_set(interface_info, address->address);
  SLI_WISUN_ERROR_CHECK_SET_STATUS(ret >= 0, SL_STATUS_FAIL);

error_handler:

  return status;
}
