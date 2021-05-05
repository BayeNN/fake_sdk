/*
 * Copyright (c) 2015-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef SLIP_H
#define SLIP_H

#include "arm_hal_phy.h"
#include "eventOS_event.h"
#include "net_interface.h"
#include "mbed_config.h"

#define SLIP_NR_BUFFERS 8
#define SLIP_MTU 1500
#define SLIP_TX_RX_MAX_BUFLEN SLIP_MTU

typedef struct SlipBuffer_s {
    uint8_t buf[SLIP_TX_RX_MAX_BUFLEN];
    size_t length;
} SlipBuffer;

typedef struct CircularBuffer_s {
    SlipBuffer *_pool[SLIP_NR_BUFFERS];
    uint32_t _head;
    uint32_t _tail;
    bool _full;
} CircularBuffer;

#ifdef MBED_CONF_RTOS_PRESENT
#ifdef MBED_CONF_APP_LED1
#define LED1  MBED_CONF_APP_LED1
#endif
#ifdef MBED_CONF_APP_SERIAL_TX
#define SERIAL_TX  MBED_CONF_APP_SERIAL_TX
#endif
#ifdef MBED_CONF_APP_SERIAL_RX
#define SERIAL_RX  MBED_CONF_APP_SERIAL_RX
#endif
#ifdef MBED_CONF_APP_SERIAL_CTS
#define SERIAL_CTS  MBED_CONF_APP_SERIAL_CTS
#endif
#ifdef MBED_CONF_APP_SERIAL_RTS
#define SERIAL_RTS  MBED_CONF_APP_SERIAL_RTS
#endif
#endif

typedef enum {
    SLIP_RX_STATE_SYNCSEARCH,
    SLIP_RX_STATE_SYNCED,
    SLIP_RX_STATE_ESCAPED
} slip_rx_state_t;

typedef enum {
    SLIP_TX_STATE_START,
    SLIP_TX_STATE_RUNNING,
    SLIP_TX_STATE_ESCAPING,
    SLIP_TX_STATE_END
} slip_tx_state_t;

typedef enum {
    TX_Interrupt,
    RX_Interrupt
} slip_IRQ_type;

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD
#define SLIP_PACKET_RX 5

typedef struct SlipMACDriver_s {
    SlipBuffer *pCurSlipRxBuffer;
    CircularBuffer RxSlipBufferFreeList;
    CircularBuffer RxSlipBufferToRxFuncList;
    SlipBuffer rx_buffer[8];
    SlipBuffer tx_buffer[8];
    slip_rx_state_t slip_rx_state;
    uint8_t slip_mac[6];
    SlipBuffer *pCurSlipTxBuffer;
    CircularBuffer TxSlipBufferFreeList;
    CircularBuffer TxSlipBufferToTxFuncList;
    phy_device_driver_s slip_phy_driver;
    int8_t net_slip_id;
} SlipMACDriver;

int8_t Slip_Init(uint8_t *mac);
int8_t slip_if_tx(uint8_t *buf, uint16_t len, uint8_t tx_id, data_protocol_e data_flow);
void buffer_handover();

bool CircularBuffer_peek(CircularBuffer *buf, SlipBuffer *data);


#endif /* SLIP_H */