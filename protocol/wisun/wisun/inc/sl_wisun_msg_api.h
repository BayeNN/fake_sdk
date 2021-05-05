/***************************************************************************//**
 * @file sl_wisun_msg_api.h
 * @brief Wi-SUN Message API
 * @version $Id: 6c0dca4dcb32c10888bb56670514d582cf0140e5 $ $Format:%ci$ ($Format:%h$)
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

#ifndef SL_WISUN_MSG_API_H
#define SL_WISUN_MSG_API_H

#include "sl_wisun_types.h"

/**************************************************************************//**
 * @addtogroup SL_WISUN_MSG_API Wi-SUN Message API
 * @{
 *****************************************************************************/

/// Wi-SUN Message API request IDs
typedef enum {
  SL_WISUN_MSG_SET_NETWORK_SIZE_REQ_ID        = 0x01,
  SL_WISUN_MSG_CONNECT_REQ_ID                 = 0x02,
  SL_WISUN_MSG_GET_IP_ADDRESS_REQ_ID          = 0x03,
  SL_WISUN_MSG_OPEN_SOCKET_REQ_ID             = 0x04,
  SL_WISUN_MSG_CLOSE_SOCKET_REQ_ID            = 0x05,
  SL_WISUN_MSG_SENDTO_ON_SOCKET_REQ_ID        = 0x06,
  SL_WISUN_MSG_LISTEN_ON_SOCKET_REQ_ID        = 0x07,
  SL_WISUN_MSG_ACCEPT_ON_SOCKET_REQ_ID        = 0x08,
  SL_WISUN_MSG_CONNECT_SOCKET_REQ_ID          = 0x09,
  SL_WISUN_MSG_BIND_SOCKET_REQ_ID             = 0x0A,
  SL_WISUN_MSG_SEND_ON_SOCKET_REQ_ID          = 0x0B,
  SL_WISUN_MSG_RECEIVE_ON_SOCKET_REQ_ID       = 0x0C,
  SL_WISUN_MSG_DISCONNECT_REQ_ID              = 0x0D,
  SL_WISUN_MSG_SET_TRUSTED_CERTIFICATE_REQ_ID = 0x0E,
  SL_WISUN_MSG_SET_DEVICE_CERTIFICATE_REQ_ID  = 0x0F,
  SL_WISUN_MSG_SET_DEVICE_PRIVATE_KEY_REQ_ID  = 0x10,
  SL_WISUN_MSG_GET_STATISTICS_REQ_ID          = 0x11,
  SL_WISUN_MSG_SET_SOCKET_OPTION_REQ_ID       = 0x12,
  SL_WISUN_MSG_SET_TX_POWER_REQ_ID            = 0x13,
  SL_WISUN_MSG_SET_CHANNEL_PLAN_REQ_ID        = 0x14,
  SL_WISUN_MSG_SET_CHANNEL_MASK_REQ_ID        = 0x15,
  SL_WISUN_MSG_ALLOW_MAC_ADDRESS_REQ_ID       = 0x16,
  SL_WISUN_MSG_DENY_MAC_ADDRESS_REQ_ID        = 0x17,
  SL_WISUN_MSG_GET_SOCKET_OPTION_REQ_ID       = 0x18,
  SL_WISUN_MSG_GET_JOIN_STATE_REQ_ID          = 0x19,
  SL_WISUN_MSG_CLEAR_CREDENTIAL_CACHE_REQ_ID  = 0x1A,
  SL_WISUN_MSG_GET_MAC_ADDRESS_REQ_ID         = 0x1B,
  SL_WISUN_MSG_SET_MAC_ADDRESS_REQ_ID         = 0x1C
} sl_wisun_msg_req_id_t;

/// Wi-SUN Message API confirmation IDs
typedef enum {
  SL_WISUN_MSG_SET_NETWORK_SIZE_CNF_ID        = 0x01,
  SL_WISUN_MSG_CONNECT_CNF_ID                 = 0x02,
  SL_WISUN_MSG_GET_IP_ADDRESS_CNF_ID          = 0x03,
  SL_WISUN_MSG_OPEN_SOCKET_CNF_ID             = 0x04,
  SL_WISUN_MSG_CLOSE_SOCKET_CNF_ID            = 0x05,
  SL_WISUN_MSG_SENDTO_ON_SOCKET_CNF_ID        = 0x06,
  SL_WISUN_MSG_LISTEN_ON_SOCKET_CNF_ID        = 0x07,
  SL_WISUN_MSG_ACCEPT_ON_SOCKET_CNF_ID        = 0x08,
  SL_WISUN_MSG_CONNECT_SOCKET_CNF_ID          = 0x09,
  SL_WISUN_MSG_BIND_SOCKET_CNF_ID             = 0x0A,
  SL_WISUN_MSG_SEND_ON_SOCKET_CNF_ID          = 0x0B,
  SL_WISUN_MSG_RECEIVE_ON_SOCKET_CNF_ID       = 0x0C,
  SL_WISUN_MSG_DISCONNECT_CNF_ID              = 0x0D,
  SL_WISUN_MSG_SET_TRUSTED_CERTIFICATE_CNF_ID = 0x0E,
  SL_WISUN_MSG_SET_DEVICE_CERTIFICATE_CNF_ID  = 0x0F,
  SL_WISUN_MSG_SET_DEVICE_PRIVATE_KEY_CNF_ID  = 0x10,
  SL_WISUN_MSG_GET_STATISTICS_CNF_ID          = 0x11,
  SL_WISUN_MSG_SET_SOCKET_OPTION_CNF_ID       = 0x12,
  SL_WISUN_MSG_SET_TX_POWER_CNF_ID            = 0x13,
  SL_WISUN_MSG_SET_CHANNEL_PLAN_CNF_ID        = 0x14,
  SL_WISUN_MSG_SET_CHANNEL_MASK_CNF_ID        = 0x15,
  SL_WISUN_MSG_ALLOW_MAC_ADDRESS_CNF_ID       = 0x16,
  SL_WISUN_MSG_DENY_MAC_ADDRESS_CNF_ID        = 0x17,
  SL_WISUN_MSG_GET_SOCKET_OPTION_CNF_ID       = 0x18,
  SL_WISUN_MSG_GET_JOIN_STATE_CNF_ID          = 0x19,
  SL_WISUN_MSG_CLEAR_CREDENTIAL_CACHE_CNF_ID  = 0x1A,
  SL_WISUN_MSG_GET_MAC_ADDRESS_CNF_ID         = 0x1B,
  SL_WISUN_MSG_SET_MAC_ADDRESS_CNF_ID         = 0x1C
} sl_wisun_msg_cnf_id_t;

/// Wi-SUN Message API indication IDs
typedef enum {
  SL_WISUN_MSG_CONNECTED_IND_ID                   = 0x81,
  SL_WISUN_MSG_SOCKET_DATA_IND_ID                 = 0x82,
  SL_WISUN_MSG_SOCKET_DATA_AVAILABLE_IND_ID       = 0x83,
  SL_WISUN_MSG_SOCKET_CONNECTED_IND_ID            = 0x84,
  SL_WISUN_MSG_SOCKET_CONNECTION_AVAILABLE_IND_ID = 0x85,
  SL_WISUN_MSG_SOCKET_CLOSING_IND_ID              = 0x86,
  SL_WISUN_MSG_DISCONNECTED_IND_ID                = 0x87,
  SL_WISUN_MSG_CONNECTION_LOST_IND_ID             = 0x88,
  SL_WISUN_MSG_SOCKET_DATA_SENT_IND_ID            = 0x89,
  SL_WISUN_MSG_ERROR_IND_ID                       = 0x8A,
  SL_WISUN_MSG_JOIN_STATE_IND_ID                  = 0x8B 
} sl_wisun_msg_ind_id_t;

/// Wi-SUN Message API common header
SL_PACK_START(1)
typedef struct {
  /// Total length of the message in bytes, this field included
  uint16_t length;
  /// ID (request, confirmation, indication) of the message
  uint8_t id;
  /// Processing metadata for the message
  uint8_t info;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_header_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_GENERIC
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_generic_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_generic_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_generic_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_NETWORK_SIZE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Network size
  uint8_t size;
  /// Reserved, set to zero
  uint8_t reserved[3];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_network_size_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_network_size_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_network_size_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_network_size_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_network_size_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_network_size_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_CONNECT
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Name of the network as a NULL terminated string
  uint8_t name[SL_WISUN_NETWORK_NAME_SIZE];
  /// Regulatory domain of the network
  uint8_t reg_domain;
  /// Operational class of the network
  uint8_t op_class;
  /// Operational mode of the network
  uint16_t op_mode;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_connect_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_connect_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_GET_IP_ADDRESS
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t address_type;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_ip_address_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_get_ip_address_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_ip_address_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  sl_wisun_ip_address_t address;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_ip_address_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_get_ip_address_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_ip_address_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_OPEN_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t protocol;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_open_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_open_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_open_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  uint32_t socket_id;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_open_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_open_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_open_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_CLOSE_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_close_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_close_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_close_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_close_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_close_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_close_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SENDTO_ON_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
  sl_wisun_ip_address_t remote_address;
  uint16_t remote_port;
  uint16_t data_length;
  uint8_t data[];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_sendto_on_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_sendto_on_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_sendto_on_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_sendto_on_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_sendto_on_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_sendto_on_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_LISTEN_ON_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_listen_on_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_listen_on_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_listen_on_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_listen_on_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_listen_on_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_listen_on_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_ACCEPT_ON_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_accept_on_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_accept_on_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_accept_on_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  uint32_t remote_socket_id;
  sl_wisun_ip_address_t remote_address;
  uint16_t remote_port;
  /// Reserved, set to zero
  uint16_t reserved;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_accept_on_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_accept_on_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_accept_on_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_CONNECT_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
  sl_wisun_ip_address_t remote_address;
  uint16_t remote_port;
  /// Reserved, set to zero
  uint16_t reserved;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_connect_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_connect_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connect_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_BIND_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
  sl_wisun_ip_address_t local_address;
  uint16_t local_port;
  /// Reserved, set to zero
  uint16_t reserved;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_bind_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_bind_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_bind_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_bind_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_bind_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_bind_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SEND_ON_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
  uint16_t data_length;
  /// Reserved, set to zero
  uint16_t reserved;
  uint8_t data[];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_send_on_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_send_on_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_send_on_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_send_on_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_send_on_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_send_on_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_RECEIVE_ON_SOCKET
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
  uint16_t data_length;
  /// Reserved, set to zero
  uint16_t reserved;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_receive_on_socket_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_receive_on_socket_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_receive_on_socket_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  sl_wisun_ip_address_t remote_address;
  uint16_t remote_port;
  uint16_t data_length;
  uint8_t data[];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_receive_on_socket_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_receive_on_socket_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_receive_on_socket_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_DISCONNECT
 ******************************************************************************/


SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_disconnect_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_disconnect_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_disconnect_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_disconnect_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_TRUSTED_CERTIFICATE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint16_t certificate_options;
  uint16_t certificate_length;
  uint8_t certificate[];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_trusted_certificate_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_trusted_certificate_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_trusted_certificate_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_trusted_certificate_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_trusted_certificate_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_trusted_certificate_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_DEVICE_CERTIFICATE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint16_t certificate_options;
  uint16_t certificate_length;
  uint8_t certificate[];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_certificate_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_device_certificate_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_certificate_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_certificate_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_device_certificate_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_certificate_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_DEVICE_PRIVATE_KEY
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint16_t key_options;
  uint16_t key_length;
  uint8_t key[];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_private_key_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_device_private_key_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_private_key_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_private_key_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_device_private_key_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_device_private_key_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_GET_STATISTICS
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t statistics_type;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_statistics_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_get_statistics_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_statistics_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  sl_wisun_statistics_t statistics;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_statistics_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_get_statistics_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_statistics_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_SOCKET_OPTION
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
  uint32_t option;
  sl_wisun_socket_option_data_t option_data;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_socket_option_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_socket_option_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_socket_option_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_socket_option_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_socket_option_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_socket_option_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_TX_POWER
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// TX power
  int8_t tx_power;
  /// Reserved, set to zero
  uint8_t reserved[3];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_tx_power_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_tx_power_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_tx_power_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_tx_power_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_tx_power_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_tx_power_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_CHANNEL_PLAN
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Central frequency of the first channel in kHz
  uint32_t ch0_frequency;
  /// Number of channels
  uint16_t number_of_channels;
  /// Channel spacing
  uint8_t channel_spacing;
  /// Reserved, set to zero
  uint8_t reserved;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_plan_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_channel_plan_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_plan_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_plan_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_channel_plan_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_plan_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_CHANNEL_MASK
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Channel mask
  sl_wisun_channel_mask_t channel_mask;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_mask_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_channel_mask_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_mask_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_mask_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_channel_mask_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_channel_mask_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_ALLOW_MAC_ADDRESS
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// MAC address
  sl_wisun_mac_address_t address;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_allow_mac_address_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_allow_mac_address_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_allow_mac_address_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_allow_mac_address_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_allow_mac_address_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_allow_mac_address_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_DENY_MAC_ADDRESS
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// MAC address
  sl_wisun_mac_address_t address;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_deny_mac_address_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_deny_mac_address_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_deny_mac_address_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_deny_mac_address_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_deny_mac_address_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_deny_mac_address_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_GET_SOCKET_OPTION
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t socket_id;
  uint32_t option;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_socket_option_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_get_socket_option_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_socket_option_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  sl_wisun_socket_option_data_t option_data;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_socket_option_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_get_socket_option_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_socket_option_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_GET_JOIN_STATE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_join_state_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  uint32_t join_state;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_join_state_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_get_join_state_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_join_state_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_CLEAR_CREDENTIAL_CACHE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_clear_credential_cache_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_clear_credential_cache_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_clear_credential_cache_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_clear_credential_cache_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_GET_MAC_ADDRESS
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_mac_address_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
  /// MAC address
  sl_wisun_mac_address_t address;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_mac_address_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_get_mac_address_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_get_mac_address_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SET_MAC_ADDRESS
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// MAC address
  sl_wisun_mac_address_t address;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_mac_address_req_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Request message body
  sl_wisun_msg_set_mac_address_req_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_mac_address_req_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Status of the request
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_mac_address_cnf_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Confirmation message body
  sl_wisun_msg_set_mac_address_cnf_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_set_mac_address_cnf_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_CONNECTED
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connected_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_connected_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connected_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SOCKET_DATA
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Status of the indication
  uint32_t status;
  /// ID of the socket
  uint32_t socket_id;
  /// IP address of the sender
  sl_wisun_ip_address_t remote_address;
  /// Port number of the sender
  uint16_t remote_port;
  /// Amount of received data
  uint16_t data_length;
  /// Received data
  uint8_t data[];
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_data_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_socket_data_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_data_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SOCKET_DATA_AVAILABLE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Status of the indication
  uint32_t status;
  /// ID of the socket
  uint32_t socket_id;
  /// Amount of data that can be read
  uint16_t data_length;
  /// Reserved, set to zero
  uint16_t reserved;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_data_available_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_socket_data_available_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_data_available_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SOCKET_CONNECTED
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  uint32_t socket_id;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_connected_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_socket_connected_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_connected_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SOCKET_CONNECTION_AVAILABLE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Status of the indication
  uint32_t status;
  /// ID of the socket
  uint32_t socket_id;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_connection_available_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_socket_connection_available_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_connection_available_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SOCKET_CLOSING
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Status of the indication
  uint32_t status;
  /// ID of the socket
  uint32_t socket_id;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_closing_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_socket_closing_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_closing_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_DISCONNECTED
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_disconnected_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_disconnected_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_disconnected_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_CONNECTION_LOST
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connection_lost_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_connection_lost_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_connection_lost_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_SOCKET_DATA_SENT
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Status of the indication
  uint32_t status;
  /// ID of the socket
  uint32_t socket_id;
  /// Amount of free space in the transmission buffer
  uint32_t socket_space_left;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_data_sent_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_socket_data_sent_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_socket_data_sent_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_ERROR
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  /// Status of the indication
  uint32_t status;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_error_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_error_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_error_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_MSG_JOIN_STATE
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  uint32_t status;
  uint32_t join_state;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_join_state_ind_body_t;
SL_PACK_END()

SL_PACK_START(1)
typedef struct {
  /// Common message header
  sl_wisun_msg_header_t header;
  /// Indication message body
  sl_wisun_msg_join_state_ind_body_t body;
} SL_ATTRIBUTE_PACKED sl_wisun_msg_join_state_ind_t;
SL_PACK_END()

/******************************************************************************
 * SL_WISUN_EVT
 ******************************************************************************/

SL_PACK_START(1)
typedef struct {
  sl_wisun_msg_header_t header;
  union {
    /// Connection request has been completed
    sl_wisun_msg_connected_ind_body_t connected;
    /// Data received on a socket
    sl_wisun_msg_socket_data_ind_body_t socket_data;
    /// Data is available on a socket to be read
    sl_wisun_msg_socket_data_available_ind_body_t socket_data_available;
    /// Connect socket request has been completed
    sl_wisun_msg_socket_connected_ind_body_t socket_connected;
    /// Socket has a remote peer connection waiting
    sl_wisun_msg_socket_connection_available_ind_body_t socket_connection_available;
    /// Socket has been closed
    sl_wisun_msg_socket_closing_ind_body_t socket_closing;
    /// Disconnection request has been completed
    sl_wisun_msg_disconnected_ind_body_t disconnected;
    /// Connection has been temporarily lost
    sl_wisun_msg_connection_lost_ind_body_t connection_lost;
    /// Some socket data has been sent
    sl_wisun_msg_socket_data_sent_ind_body_t socket_data_sent;
    /// Error
    sl_wisun_msg_error_ind_body_t error;
    /// Join state changed
    sl_wisun_msg_join_state_ind_body_t join_state;
  } evt;
} SL_ATTRIBUTE_PACKED sl_wisun_evt_t;
SL_PACK_END()

/** @} (end addtogroup SL_WISUN_MSG_API) */

#endif  // SL_WISUN_MSG_API_H
