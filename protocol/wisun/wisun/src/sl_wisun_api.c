/***************************************************************************//**
 * @file sl_wisun_api.c
 * @brief Wi-SUN API
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

#include <em_common.h>
#include <string.h>
#include "sli_wisun_task.h"
#include "sl_wisun_api.h"

SL_WEAK void sl_wisun_on_event(sl_wisun_evt_t *evt)
{
  (void)evt;
}

sl_status_t sl_wisun_set_network_size(sl_wisun_network_size_t size)
{
  SL_ALIGN(4) sl_wisun_msg_set_network_size_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_network_size_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_NETWORK_SIZE_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_network_size_req_t);
  req.body.size = size;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_connect(const uint8_t *name,
                             sl_wisun_regulatory_domain_t reg_domain,
                             sl_wisun_operating_class_t op_class,
                             sl_wisun_operating_mode_t op_mode)
{
  SL_ALIGN(4) sl_wisun_msg_connect_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_connect_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_CONNECT_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_connect_req_t);
  strncpy((char *)req.body.name, (char *)name, SL_WISUN_NETWORK_NAME_SIZE);
  req.body.reg_domain = reg_domain;
  req.body.op_class = op_class;
  req.body.op_mode = op_mode;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_get_ip_address(sl_wisun_ip_address_type_t address_type,
                                    sl_wisun_ip_address_t *address)
{
  SL_ALIGN(4) sl_wisun_msg_get_ip_address_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_get_ip_address_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_GET_IP_ADDRESS_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_get_ip_address_req_t);
  req.body.address_type = address_type;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  if (cnf.body.status == SL_STATUS_OK) {
    *address = cnf.body.address;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_open_socket(sl_wisun_socket_protocol_t protocol,
                                 sl_wisun_socket_id_t *socket_id)
{
  SL_ALIGN(4) sl_wisun_msg_open_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_open_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_OPEN_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_open_socket_req_t);
  req.body.protocol = protocol;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  if (cnf.body.status == SL_STATUS_OK) {
    *socket_id = cnf.body.socket_id;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_close_socket(sl_wisun_socket_id_t socket_id)
{
  SL_ALIGN(4) sl_wisun_msg_close_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_close_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_CLOSE_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_close_socket_req_t);
  req.body.socket_id = socket_id;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_sendto_on_socket(sl_wisun_socket_id_t socket_id,
                                      const sl_wisun_ip_address_t *remote_address,
                                      uint16_t remote_port,
                                      uint16_t data_length,
                                      const uint8_t *data)
{
  SL_ALIGN(4) sl_wisun_msg_sendto_on_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_sendto_on_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SENDTO_ON_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_sendto_on_socket_req_t) + data_length;
  req.body.socket_id = socket_id;
  req.body.remote_address = *remote_address;
  req.body.remote_port = remote_port;
  req.body.data_length = data_length;

  sli_wisun_task_req(&req, data, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_listen_on_socket(sl_wisun_socket_id_t socket_id)
{
  SL_ALIGN(4) sl_wisun_msg_listen_on_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_listen_on_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_LISTEN_ON_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_listen_on_socket_req_t);
  req.body.socket_id = socket_id;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_accept_on_socket(sl_wisun_socket_id_t socket_id,
                                      sl_wisun_socket_id_t *remote_socket_id,
                                      sl_wisun_ip_address_t *remote_address,
                                      uint16_t *remote_port)
{
  SL_ALIGN(4) sl_wisun_msg_accept_on_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_accept_on_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_ACCEPT_ON_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_accept_on_socket_req_t);
  req.body.socket_id = socket_id;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  if (cnf.body.status == SL_STATUS_OK) {
    *remote_socket_id = cnf.body.remote_socket_id;
    *remote_address = cnf.body.remote_address;
    *remote_port = cnf.body.remote_port;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_connect_socket(sl_wisun_socket_id_t socket_id,
                                    const sl_wisun_ip_address_t *remote_address,
                                    uint16_t remote_port)
{
  SL_ALIGN(4) sl_wisun_msg_connect_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_connect_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_CONNECT_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_connect_socket_req_t);
  req.body.socket_id = socket_id;
  req.body.remote_address = *remote_address;
  req.body.remote_port = remote_port;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_bind_socket(sl_wisun_socket_id_t socket_id,
                                 const sl_wisun_ip_address_t *local_address,
                                 uint16_t local_port)
{
  SL_ALIGN(4) sl_wisun_msg_bind_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_bind_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_BIND_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_bind_socket_req_t);
  req.body.socket_id = socket_id;
  req.body.local_address = *local_address;
  req.body.local_port = local_port;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_send_on_socket(sl_wisun_socket_id_t socket_id,
                                    uint16_t data_length,
                                    const uint8_t *data)
{
  SL_ALIGN(4) sl_wisun_msg_send_on_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_send_on_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SEND_ON_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_send_on_socket_req_t) + data_length;
  req.body.socket_id = socket_id;
  req.body.data_length = data_length;

  sli_wisun_task_req(&req, data, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_receive_on_socket(sl_wisun_socket_id_t socket_id,
                                       sl_wisun_ip_address_t *remote_address,
                                       uint16_t *remote_port,
                                       uint16_t *data_length,
                                       uint8_t *data)
{
  SL_ALIGN(4) sl_wisun_msg_receive_on_socket_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_receive_on_socket_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_RECEIVE_ON_SOCKET_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_receive_on_socket_req_t);
  req.body.socket_id = socket_id;
  req.body.data_length = *data_length;

  sli_wisun_task_req(&req, data, &cnf, data);

  if (cnf.body.status == SL_STATUS_OK) {
    *remote_address = cnf.body.remote_address;
    *remote_port = cnf.body.remote_port;
    *data_length = cnf.body.data_length;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_disconnect()
{
  SL_ALIGN(4) sl_wisun_msg_disconnect_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_disconnect_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_DISCONNECT_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_disconnect_req_t);

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_trusted_certificate(uint16_t certificate_options,
                                             uint16_t certificate_length,
                                             const uint8_t *certificate)
{
  SL_ALIGN(4) sl_wisun_msg_set_trusted_certificate_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_trusted_certificate_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_TRUSTED_CERTIFICATE_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_trusted_certificate_req_t) + certificate_length;
  req.body.certificate_options = certificate_options;
  req.body.certificate_length = certificate_length;

  sli_wisun_task_req(&req, certificate, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_device_certificate(uint16_t certificate_options,
                                            uint16_t certificate_length,
                                            const uint8_t *certificate)
{
  SL_ALIGN(4) sl_wisun_msg_set_device_certificate_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_device_certificate_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_DEVICE_CERTIFICATE_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_device_certificate_req_t) + certificate_length;
  req.body.certificate_options = certificate_options;
  req.body.certificate_length = certificate_length;

  sli_wisun_task_req(&req, certificate, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_device_private_key(uint16_t key_options,
                                            uint16_t key_length,
                                            const uint8_t *key)
{
  SL_ALIGN(4) sl_wisun_msg_set_device_private_key_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_device_private_key_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_DEVICE_PRIVATE_KEY_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_device_private_key_req_t) + key_length;
  req.body.key_options = key_options;
  req.body.key_length = key_length;

  sli_wisun_task_req(&req, key, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_get_statistics(sl_wisun_statistics_type_t statistics_type,
                                    sl_wisun_statistics_t *statistics)
{
  SL_ALIGN(4) sl_wisun_msg_get_statistics_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_get_statistics_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_GET_STATISTICS_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_get_statistics_req_t);
  req.body.statistics_type = statistics_type;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  if (cnf.body.status == SL_STATUS_OK) {
    *statistics = cnf.body.statistics;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_socket_option(sl_wisun_socket_id_t socket_id,
                                       sl_wisun_socket_option_t option,
                                       const sl_wisun_socket_option_data_t *option_data)
{
  SL_ALIGN(4) sl_wisun_msg_set_socket_option_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_socket_option_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_SOCKET_OPTION_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_socket_option_req_t);
  req.body.socket_id = socket_id;
  req.body.option = option;
  req.body.option_data = *option_data;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_tx_power(int8_t tx_power)
{
  SL_ALIGN(4) sl_wisun_msg_set_tx_power_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_tx_power_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_TX_POWER_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_tx_power_req_t);
  req.body.tx_power = tx_power;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_channel_plan(uint32_t ch0_frequency,
                                      uint16_t number_of_channels,
                                      sl_wisun_channel_spacing_t channel_spacing)
{
  SL_ALIGN(4) sl_wisun_msg_set_channel_plan_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_channel_plan_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_CHANNEL_PLAN_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_channel_plan_req_t);
  req.body.ch0_frequency = ch0_frequency;
  req.body.number_of_channels = number_of_channels;
  req.body.channel_spacing = channel_spacing;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_channel_mask(const sl_wisun_channel_mask_t *channel_mask)
{
  SL_ALIGN(4) sl_wisun_msg_set_channel_mask_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_channel_mask_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_CHANNEL_MASK_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_channel_mask_req_t);
  req.body.channel_mask = *channel_mask;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_allow_mac_address(const sl_wisun_mac_address_t *address)
{
  SL_ALIGN(4) sl_wisun_msg_allow_mac_address_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_allow_mac_address_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_ALLOW_MAC_ADDRESS_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_allow_mac_address_req_t);
  req.body.address = *address;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_deny_mac_address(const sl_wisun_mac_address_t *address)
{
  SL_ALIGN(4) sl_wisun_msg_deny_mac_address_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_deny_mac_address_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_DENY_MAC_ADDRESS_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_deny_mac_address_req_t);
  req.body.address = *address;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_get_socket_option(sl_wisun_socket_id_t socket_id,
                                       sl_wisun_socket_option_t option,
                                       sl_wisun_socket_option_data_t *option_data)
{
  SL_ALIGN(4) sl_wisun_msg_get_socket_option_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_get_socket_option_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_GET_SOCKET_OPTION_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_get_socket_option_req_t);
  req.body.socket_id = socket_id;
  req.body.option = option;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  if (cnf.body.status == SL_STATUS_OK) {
    *option_data = cnf.body.option_data;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_get_join_state(sl_wisun_join_state_t *join_state)
{
  SL_ALIGN(4) sl_wisun_msg_get_join_state_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_get_join_state_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_GET_JOIN_STATE_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_get_join_state_req_t);

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  if (cnf.body.status == SL_STATUS_OK) {
    *join_state = (sl_wisun_join_state_t)cnf.body.join_state;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_clear_credential_cache()
{
  SL_ALIGN(4) sl_wisun_msg_clear_credential_cache_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_clear_credential_cache_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_CLEAR_CREDENTIAL_CACHE_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_clear_credential_cache_req_t);

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_get_mac_address(sl_wisun_mac_address_t *address)
{
  SL_ALIGN(4) sl_wisun_msg_get_mac_address_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_get_mac_address_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_GET_MAC_ADDRESS_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_get_mac_address_req_t);

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  if (cnf.body.status == SL_STATUS_OK) {
    *address = cnf.body.address;
  }

  return (sl_status_t)cnf.body.status;
}

sl_status_t sl_wisun_set_mac_address(const sl_wisun_mac_address_t *address)
{
  SL_ALIGN(4) sl_wisun_msg_set_mac_address_req_t req SL_ATTRIBUTE_ALIGN(4);
  SL_ALIGN(4) sl_wisun_msg_set_mac_address_cnf_t cnf SL_ATTRIBUTE_ALIGN(4);

  req.header.id = SL_WISUN_MSG_SET_MAC_ADDRESS_REQ_ID;
  req.header.length = sizeof(sl_wisun_msg_set_mac_address_req_t);
  req.body.address = *address;

  sli_wisun_task_req(&req, NULL, &cnf, NULL);

  return (sl_status_t)cnf.body.status;
}
