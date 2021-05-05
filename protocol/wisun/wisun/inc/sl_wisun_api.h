/***************************************************************************//**
 * @file sl_wisun_api.h
 * @brief Wi-SUN API
 * @version $Id: 08b8da5293a710b7052176a4abd1b7a538415c03 $ $Format:%ci$ ($Format:%h$)
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

#ifndef SL_WISUN_API_H
#define SL_WISUN_API_H

#include "sl_wisun_types.h"
#include "sl_wisun_msg_api.h"
#include "sl_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************//**
 * @addtogroup SL_WISUN_API Wi-SUN API
 * @{
 *****************************************************************************/

/**************************************************************************//**
 * @brief Callback handler for a single event.
 * @details The default implementation discards all events. The application can
 *          override this function to customize event handling.
 * @param evt The event to be handled
 *****************************************************************************/
void sl_wisun_on_event(sl_wisun_evt_t *evt);

/**************************************************************************//**
 * @brief Set the size of the Wi-SUN network.
 * @details The size is used set various stack parameters, such as timing
 *          parameters to optimize device behaviour in regards to node count.
 *          The device will function with any setting but may exhibit non-optimal
 *          behaviour. Setting the size too large may cause slow connection speeds
 *          and increased latency. Conversely, a value too small may cause increased
 *          network traffic.
 * @param[in] size Size of the network
 *    <br/><b>SL_WISUN_NETWORK_SIZE_AUTOMATIC</b>: network size is managed automatically
 *    <br/><b>SL_WISUN_NETWORK_SIZE_SMALL</b>: less than 100 nodes
 *    <br/><b>SL_WISUN_NETWORK_SIZE_MEDIUM</b>: 100 to 800 nodes
 *    <br/><b>SL_WISUN_NETWORK_SIZE_LARGE</b>: 800 to 1500 nodes
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_network_size(sl_wisun_network_size_t size);

/**************************************************************************//**
 * @brief Connect to a Wi-SUN network.
 * @param[in] name Name of the Wi-SUN network as a zero-terminated string
 * @param[in] reg_domain Regulatory domain of the Wi-SUN network
 * @param[in] op_class Operational class of the Wi-SUN network
 * @param[in] op_mode Operational mode of the Wi-SUN network
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_connect(const uint8_t *name,
                             sl_wisun_regulatory_domain_t reg_domain,
                             sl_wisun_operating_class_t op_class,
                             sl_wisun_operating_mode_t op_mode);

/**************************************************************************//**
 * @brief Read an IP address.
 * @param[in] address_type Type of the IP address to read
 *    <br/><b>SL_WISUN_IP_ADDRESS_TYPE_LINK_LOCAL</b>: Link-local address of the device
 *    <br/><b>SL_WISUN_IP_ADDRESS_TYPE_GLOBAL</b>: Global unicast address of the device
 *    <br/><b>SL_WISUN_IP_ADDRESS_TYPE_BORDER_ROUTER</b>: Global unicast address of the border router
 *    <br/><b>SL_WISUN_IP_ADDRESS_TYPE_PRIMARY_PARENT</b>: Link-local address of the primary parent
 *    <br/><b>SL_WISUN_IP_ADDRESS_TYPE_SECONDARY_PARENT</b>: Link-local address of the secondary parent
 * @param[out] address IP address read
 * @return SL_STATUS_OK if successful, an error code otherwise.
 *****************************************************************************/
sl_status_t sl_wisun_get_ip_address(sl_wisun_ip_address_type_t address_type,
                                    sl_wisun_ip_address_t *address);

/**************************************************************************//**
 * @brief Open a socket.
 * @param[in] protocol Protocol type of the socket
 * @param[out] socket_id ID of the opened socket
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_open_socket(sl_wisun_socket_protocol_t protocol,
                                 sl_wisun_socket_id_t *socket_id);

/**************************************************************************//**
 * @brief Close a socket.
 * @param[in] socket_id ID of the socket
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_close_socket(sl_wisun_socket_id_t socket_id);

/**************************************************************************//**
 * @brief Write data to an unconnected socket.
 * @details This function is used to transmit data to a remote peer and can
 *          only be used on an unconnected UDP/ICMP socket.
 * @param[in] socket_id ID of the socket
 * @param[in] remote_address IP address of the remote peer
 * @param[in] remote_port Port number of the remote peer
 * @param[in] data_length Amount of data write
 * @param[in] data Pointer to the data
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_sendto_on_socket(sl_wisun_socket_id_t socket_id,
                                      const sl_wisun_ip_address_t *remote_address,
                                      uint16_t remote_port,
                                      uint16_t data_length,
                                      const uint8_t *data);

/**************************************************************************//**
 * @brief Set a TCP socket to listening state.
 * @details Setting a socket to listening state allows it to acts as a server
 *          socket, i.e. receive connection requests from clients. Connection
 *          requests are indicated as SL_WISUN_MSG_SOCKET_CONNECTION_AVAILABLE
 *          indications. This function can only be used on an unconnected
 *          TCP socket.
 * @param[in] socket_id ID of the socket
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_listen_on_socket(sl_wisun_socket_id_t socket_id);

/**************************************************************************//**
 * @brief Accept a pending connection request on a TCP socket.
 * @details Accepts the pending connection request from a remote peer and
 *          creates a new connected TCP socket for the connection.
 *          To decline a connection request, the request must be accepted
 *          and then closed using sl_wisun_close_socket. This function can
 *          only be called on a TCP socket on listening state.
 * @param[in] socket_id ID of the socket on listening state
 * @param[out] remote_socket_id ID of the new connected socket
 * @param[out] remote_address IP address of the remote peer
 * @param[out] remote_port Port number of the remote peer
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_accept_on_socket(sl_wisun_socket_id_t socket_id,
                                      sl_wisun_socket_id_t *remote_socket_id,
                                      sl_wisun_ip_address_t *remote_address,
                                      uint16_t *remote_port);

/**************************************************************************//**
 * @brief Connect a socket to a remote peer.
 * @details Connecting a socket is mandatory for TCP client sockets.
 *          It can also be used on other types of sockets. A connected
 *          socket can only receive and transmit data with the designated
 *          peer. This function can only be used on an unconnected
 *          TCP/UDP socket.
 * @param[in] socket_id ID of the socket
 * @param[in] remote_address IP address of the remote peer
 * @param[in] remote_port Port number of the remote peer
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_connect_socket(sl_wisun_socket_id_t socket_id,
                                    const sl_wisun_ip_address_t *remote_address,
                                    uint16_t remote_port);

/**************************************************************************//**
 * @brief Bind a socket to specific local address and/or port.
 * @details Binding the local address causes the socket to only accept
 *          traffic sent to the specified address. Transmitted packets
 *          will use the designated address as the source address.
 *          If not bind, the socket will accept data sent to any valid
 *          device IP address. Binding the local port sets the source
 *          port for transmitted packets. If not bind, the stack will
 *          select a port automatically. This function can only be used
 *          on an unconnected TCP/UDP socket.
 * @param[in] socket_id ID of the socket
 * @param[in] local_address Local IP address to use on the socket. NULL if not bind.
 * @param[in] local_port Local port number to use on the socket. Zero if not bind.
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_bind_socket(sl_wisun_socket_id_t socket_id,
                                 const sl_wisun_ip_address_t *local_address,
                                 uint16_t local_port);

/**************************************************************************//**
 * @brief Write data to a connected socket.
 * @details This function is used to transmit data to a remote peer on
 *          a connected socket.
 * @param[in] socket_id ID of the socket
 * @param[in] data_length Amount of data write
 * @param[in] data Pointer to the data
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_send_on_socket(sl_wisun_socket_id_t socket_id,
                                    uint16_t data_length,
                                    const uint8_t *data);

/**************************************************************************//**
 * @brief Read data from a socket.
 * @details Read buffered data from a socket. When reading data from
 *          a UDP/ICMP socket, the packet must be read in whole. Any
 *          data left unread is discarded after this call. TCP sockets
 *          allow reading only a part of the buffered data.
 * @param[in] socket_id ID of the socket
 * @param[out] remote_address IP address of the remote peer
 * @param[out] remote_port Port number of the remote peer
 * @param[in,out] data_length Amount of data to read on input, amount of data read on output
 * @param[in] data Pointer to where the read data is stored
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_receive_on_socket(sl_wisun_socket_id_t socket_id,
                                       sl_wisun_ip_address_t *remote_address,
                                       uint16_t *remote_port,
                                       uint16_t *data_length,
                                       uint8_t *data);

/**************************************************************************//**
 * @brief Disconnect from the Wi-SUN network.
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_disconnect();

/**************************************************************************//**
 * @brief Set a trusted certificate used to verify the authentication server certificate.
 * @param[in] certificate_options Options for the certificate
 *   <br/><b>SL_WISUN_CERTIFICATE_OPTION_APPEND</b>: Append the certificate to the list of trusted certificates
 *                                                   instead of replacing the previous entries
 *   <br/><b>SL_WISUN_CERTIFICATE_OPTION_IS_REF</b>: The application guarantees the certificate data will remain
 *                                                   in scope and can thus be referenced instead of being copied
 * @param[in] certificate_length Size of the certificate data
 * @param[in] certificate Pointer to the certificate data
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_trusted_certificate(uint16_t certificate_options,
                                             uint16_t certificate_length,
                                             const uint8_t *certificate);

/**************************************************************************//**
 * @brief Set the device certificate used to authenticate to the authentication server.
 * @param[in] certificate_options Options for the certificate.
 *   <br/><b>SL_WISUN_CERTIFICATE_OPTION_APPEND</b>: Append the certificate to the list of trusted certificates
 *                                                   instead of replacing the previous entries
 *   <br/><b>SL_WISUN_CERTIFICATE_OPTION_IS_REF</b>: The application guarantees the certificate data will remain
 *                                                   in scope and can thus be referenced instead of being copied
 *   <br/><b>SL_WISUN_CERTIFICATE_OPTION_HAS_KEY</b>: The certificate has a private key
 * @param[in] certificate_length Size of the certificate data
 * @param[in] certificate Pointer to the certificate data
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_device_certificate(uint16_t certificate_options,
                                            uint16_t certificate_length,
                                            const uint8_t *certificate);

/**************************************************************************//**
 * @brief Set the private key of the device certificate.
 * @param[in] key_options Options for the private key
 *   <br/><b>SL_WISUN_PRIVATE_KEY_OPTION_IS_REF</b>: The application guarantees the private key data will remain
 *                                                   in scope and can thus be referenced instead of being copied
 * @param[in] key_length Size of the private key data
 * @param[in] key Pointer to the private key data
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_device_private_key(uint16_t key_options,
                                            uint16_t key_length,
                                            const uint8_t *key);

/**************************************************************************//**
 * @brief Read a set of statistics.
 * @param[in] statistics_type Type of statistics to read
 *   <br/><b>SL_WISUN_STATISTICS_TYPE_PHY</b>: PHY/RF statistics
 *   <br/><b>SL_WISUN_STATISTICS_TYPE_MAC</b>: MAC statistics
 *   <br/><b>SL_WISUN_STATISTICS_TYPE_FHSS</b>: Frequency hopping statistics
 *   <br/><b>SL_WISUN_STATISTICS_TYPE_WISUN</b>: Wi-SUN statistics
 *   <br/><b>SL_WISUN_STATISTICS_TYPE_NETWORK</b>: 6LoWPAN/IP stack statistics
 * @param[out] statistics Set of statistics read
 * @return SL_STATUS_OK if successful, an error code otherwise.
 *****************************************************************************/
sl_status_t sl_wisun_get_statistics(sl_wisun_statistics_type_t statistics_type,
                                    sl_wisun_statistics_t *statistics);

/**************************************************************************//**
 * @brief Set a socket option.
 * @param[in] socket_id ID of the socket
 * @param[in] option Socket option to set
 *   <br/><b>SL_WISUN_SOCKET_OPTION_EVENT_MODE</b>: Event mode
 *   <br/><b>SL_WISUN_SOCKET_OPTION_MULTICAST_GROUP</b>: Multicast group
 * @param[in] option_data Socket option specific data
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_socket_option(sl_wisun_socket_id_t socket_id,
                                       sl_wisun_socket_option_t option,
                                       const sl_wisun_socket_option_data_t *option_data);

/**************************************************************************//**
 * @brief Set TX power.
 * @param[in] tx_power TX power in dBm
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_tx_power(int8_t tx_power);

/**************************************************************************//**
 * @brief Set a channel plan.
 * @param[in] ch0_frequency Frequency of the first channel in kHz
 * @param[in] number_of_channels Number of channels
 * @param[in] channel_spacing Spacing between the channels
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_channel_plan(uint32_t ch0_frequency,
                                      uint16_t number_of_channels,
                                      sl_wisun_channel_spacing_t channel_spacing);

/**************************************************************************//**
 * @brief Set a mask of operating channels.
 * @param[in] channel_mask Mask of operating channels
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_channel_mask(const sl_wisun_channel_mask_t *channel_mask);

/**************************************************************************//**
 * @brief Add a MAC address to the list of allowed addresses.
 * @details When the first address is added to the list, the list of denied
 *          addresses is cleared and the device will start preventing communication
 *          with any device whose MAC address does not match any of addresses on
 *          the list. By default all MAC addresses are allowed.
 * @param[in] address MAC address
 *   <br/><b>broadcast address</b>: allow all MAC addresses
 *   <br/><b>unicast address</b>: allow the given MAC address
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_allow_mac_address(const sl_wisun_mac_address_t *address);

/**************************************************************************//**
 * @brief Add a MAC address to the list of denied addresses.
 * @details When the first address is added to the list, the list of allowed
 *          addresses is cleared and the device will start preventing communication
 *          with any device whose MAC address matches any of the addresses on the list.
 *          By default all MAC addresses are allowed.
 * @param[in] address MAC address
 *   <br/><b>broadcast address</b>: deny all MAC addresses
 *   <br/><b>unicast address</b>: deny the given MAC address
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_deny_mac_address(const sl_wisun_mac_address_t *address);

/**************************************************************************//**
 * @brief Get a socket option.
 * @param[in] socket_id ID of the socket
 * @param[in] option Socket option to get
 *   <br/><b>SL_WISUN_SOCKET_OPTION_SEND_BUFFER_LIMIT</b>: Send buffer limit
 * @param[out] option_data Socket option specific data
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_get_socket_option(sl_wisun_socket_id_t socket_id,
                                       sl_wisun_socket_option_t option,
                                       sl_wisun_socket_option_data_t *option_data);

/**************************************************************************//**
 * @brief Get current Wi-SUN join state.
 * @param[out] join_state Wi-SUN join state
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_get_join_state(sl_wisun_join_state_t *join_state);

/**************************************************************************//**
 * @brief Clear the credential cache.
 * @details This function is intended for test purposes. Note that clearing
 *          the credential cache may prevent the node from reconnecting to
 *          the same parent until the corresponding cache entry has expired
 *          on the parent.
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_clear_credential_cache();

/**************************************************************************//**
 * @brief Get the current MAC address in use.
 * @param[out] address MAC address
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_get_mac_address(sl_wisun_mac_address_t *address);

/**************************************************************************//**
 * @brief Set the MAC address to be used.
 * @details This function is used to set the MAC address that will be taken
 *          into use in the next connection. By default the device will use
 *          the built-in unique MAC address. The address is reset to the
 *          built-in value upon power up.
 * @param[in] address MAC address
 * @return SL_STATUS_OK if successful, an error code otherwise
 *****************************************************************************/
sl_status_t sl_wisun_set_mac_address(const sl_wisun_mac_address_t *address);

/** @} (end addtogroup SL_WISUN_API) */

#ifdef __cplusplus
}
#endif

#endif  // SL_WISUN_API_H
