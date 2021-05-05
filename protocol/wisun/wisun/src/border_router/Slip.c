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

#include <cmsis_os2.h>
#include <em_core.h>
#include <string.h>
#include "Slip.h"
#include "sl_cmsis_os2_common.h"

#include "arm_hal_interrupt.h"
#include "sli_wisun_trace.h"
#include "SEGGER_RTT.h"

#include "uartdrv.h"
#include "sl_uartdrv_instances.h"

// Interrupt routines for UART rx/tx interrupts
void SLI_SLIP_USART_RX_IRQHandler(void);
void process_rx_byte(uint8_t character);
void slip_if_rx(const SlipBuffer *rx_buf);
#ifdef MBED_CONF_RTOS_PRESENT
void slip_if_interrupt_handler(slip_IRQ_type type);
void SLIP_IRQ_Thread_Create(void);
#endif
static void slip_if_lock(void);
static void slip_if_unlock(void);
bool sli_slip_try_transmit(void);
static Ecode_t UARTDRV_Abort_receive_timeout(UARTDRV_Handle_t handle);
static Ecode_t GetTailBuffer(UARTDRV_Buffer_FifoQueue_t *queue,
                             UARTDRV_Buffer_t **buffer);
static Ecode_t DequeueBuffer(UARTDRV_Buffer_FifoQueue_t *queue,
                             UARTDRV_Buffer_t **buffer);
static void DisableReceiver(UARTDRV_Handle_t handle);


uint32_t CircularBuffer_size(CircularBuffer *buf);
void CircularBuffer_reset(CircularBuffer *buf);
bool CircularBuffer_full(CircularBuffer *buf);
bool CircularBuffer_empty(CircularBuffer *buf);
bool CircularBuffer_pop(CircularBuffer *buf, SlipBuffer **data);
void CircularBuffer_push(CircularBuffer *buf, SlipBuffer *data);






static osThreadId_t slip_thread_id;
// Statically allocate size for worker thread
__ALIGNED(8) static uint8_t slip_thread_stk[(512 * sizeof(void *)) & 0xFFFFFFF8u];
__ALIGNED(8) static uint8_t slip_thread_tcb[osThreadCbSize];
static osThreadAttr_t slip_thread_attr = {0, 0, 0, 0, 0, 0, osPriorityNone, 0, 0};
static SlipMACDriver _slipmacdriver;
static phy_device_driver_s *drv;

static uint8_t sli_slip_rx_buf[1500] = { 0 };
static uint8_t sli_slip_tx_buf[1500] = { 0 };

#define SIG_SL_RX       0x0001U
#define TRACE_GROUP  "slip"

#if SL_UARTDRV_USART_VCOM_PERIPHERAL_NO == 0
#define SLI_SLIP_USART_RX_IRQHandler   USART0_RX_IRQHandler
#define SLI_SLIP_USART_RX_IRQn         USART0_RX_IRQn
#elif SL_SLIP_VCOM_PERIPHERAL_NO == 1
#define SLI_SLIP_USART_RX_IRQHandler   USART1_RX_IRQHandler
#define SLI_SLIP_USART_RX_IRQn         USART1_RX_IRQn
#elif SL_UARTDRV_USART_VCOM_PERIPHERAL_NO == 2
#define SLI_SLIP_USART_RX_IRQHandler   USART2_RX_IRQHandler
#define SLI_SLIP_USART_RX_IRQn         USART2_RX_IRQn
#elif SL_UARTDRV_USART_VCOM_PERIPHERAL_NO == 3
#define SLI_SLIP_USART_RX_IRQHandler   USART3_RX_IRQHandler
#define SLI_SLIP_USART_RX_IRQn         USART3_RX_IRQn
#else
#error "Unsupported UART port!"
#endif


int8_t slip_if_tx(uint8_t *buf, uint16_t len, uint8_t tx_id, data_protocol_e data_flow)
{
    bool bufValid;
    (void) data_flow;
    SlipBuffer *pTxBuf;

    if (len > SLIP_TX_RX_MAX_BUFLEN) {
        return 0;
    }

    CORE_CRITICAL_SECTION(
        bufValid = CircularBuffer_pop(&_slipmacdriver.TxSlipBufferFreeList, &pTxBuf);
    )

    //TODO: No more free TX buffers??
    if (!bufValid) {
        tr_error("Ran out of TX Buffers.");
        return 0;
    }

    memcpy(pTxBuf->buf, buf, len);
    pTxBuf->length = len;

    CORE_CRITICAL_SECTION(
        CircularBuffer_push(&_slipmacdriver.TxSlipBufferToTxFuncList, pTxBuf);
    )

    sli_slip_try_transmit();

    // success callback
    if( drv->phy_tx_done_cb ){
        drv->phy_tx_done_cb(_slipmacdriver.net_slip_id, tx_id, PHY_LINK_TX_SUCCESS, 0, 0);
    }

    return 0;
}

static void sli_slip_comm_transmit_complete_cb(UARTDRV_Handle_t handle,
                                               Ecode_t transferStatus,
                                               uint8_t *data,
                                               UARTDRV_Count_t transferCount)
{
  (void)handle;
  (void)data;
  (void)transferCount;
  (void)transferStatus;

  // Call public callback API
  // Transmit of current buffer finished
  CircularBuffer_push(&_slipmacdriver.TxSlipBufferFreeList, _slipmacdriver.pCurSlipTxBuffer);
  _slipmacdriver.pCurSlipTxBuffer = NULL;

  if (!CircularBuffer_empty(&_slipmacdriver.TxSlipBufferToTxFuncList)) {
      sli_slip_try_transmit();
  }
}

void sli_slip_transmit_tx_buff(SlipBuffer *slip_buf)
{
    size_t cnt;
    uint8_t byte;
    size_t tx_byte_on_buffer = 0;

    // Start with an END charracter
    sli_slip_tx_buf[tx_byte_on_buffer] = SLIP_END;
    tx_byte_on_buffer++;

    for (cnt = 0; cnt < slip_buf->length; cnt++) {
        byte = slip_buf->buf[cnt];

        if (byte == SLIP_END) {
            // Escape end character in the middle of the packet
            sli_slip_tx_buf[tx_byte_on_buffer] = SLIP_ESC;
            tx_byte_on_buffer++;
            sli_slip_tx_buf[tx_byte_on_buffer] = SLIP_ESC_END;
            tx_byte_on_buffer++;
        } else if (byte == SLIP_ESC) {
            // Escape escape character in the middle of the packet
            sli_slip_tx_buf[tx_byte_on_buffer] = SLIP_ESC;
            tx_byte_on_buffer++;
            sli_slip_tx_buf[tx_byte_on_buffer] = SLIP_ESC_ESC;
            tx_byte_on_buffer++;
        } else {
            // Write normal character
            sli_slip_tx_buf[tx_byte_on_buffer] = byte;
            tx_byte_on_buffer++;
        }
    }

    // End of packet
    sli_slip_tx_buf[tx_byte_on_buffer] = SLIP_END;
    tx_byte_on_buffer++;

  // Transmit data using a non-blocking transmit function
  UARTDRV_Transmit(sl_uartdrv_usart_vcom_handle,
                   sli_slip_tx_buf,
                   tx_byte_on_buffer,
                   sli_slip_comm_transmit_complete_cb);
}

bool sli_slip_try_transmit(void)
{
    if (!_slipmacdriver.pCurSlipTxBuffer) {
        if (!CircularBuffer_pop(&_slipmacdriver.TxSlipBufferToTxFuncList, &_slipmacdriver.pCurSlipTxBuffer)) {
            // No buffer = error
            return false;
        }
    } else {
        // Previous transmission not finished. Tx will start when previous one is completed.
        return false;
    }

    sli_slip_transmit_tx_buff(_slipmacdriver.pCurSlipTxBuffer);

    return true;
}

void process_rx_byte(uint8_t character)
{
    if (character == SLIP_END) {
        if (_slipmacdriver.pCurSlipRxBuffer && _slipmacdriver.pCurSlipRxBuffer->length > 0) {

            CircularBuffer_push(&_slipmacdriver.RxSlipBufferToRxFuncList, _slipmacdriver.pCurSlipRxBuffer);
            _slipmacdriver.pCurSlipRxBuffer = NULL;
            osThreadFlagsSet(slip_thread_id, SIG_SL_RX);
        }
        _slipmacdriver.slip_rx_state = SLIP_RX_STATE_SYNCED;
        return;
    }

    if (_slipmacdriver.slip_rx_state == SLIP_RX_STATE_SYNCSEARCH) {
        return;
    }

    if (character == SLIP_ESC) {
        _slipmacdriver.slip_rx_state = SLIP_RX_STATE_ESCAPED;
        return;
    }

    if (_slipmacdriver.slip_rx_state == SLIP_RX_STATE_ESCAPED) {
        switch (character) {
            case SLIP_ESC_END:
                character = SLIP_END;
                break;
            case SLIP_ESC_ESC:
                character = SLIP_ESC;
                break;
            default:
                break;
        }
        _slipmacdriver.slip_rx_state = SLIP_RX_STATE_SYNCED;
    }

    // Reached the point we have a data byte to store. Find buffer now.
    if (!_slipmacdriver.pCurSlipRxBuffer) {
        if (!CircularBuffer_pop(&_slipmacdriver.RxSlipBufferFreeList, &_slipmacdriver.pCurSlipRxBuffer)) {
            _slipmacdriver.slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
            return;
        }
    }

    if (_slipmacdriver.pCurSlipRxBuffer->length < sizeof(_slipmacdriver.pCurSlipRxBuffer->buf)) {
        _slipmacdriver.pCurSlipRxBuffer->buf[_slipmacdriver.pCurSlipRxBuffer->length++] = character;
    } else {
        // Buffer overrun - abandon frame
        _slipmacdriver.slip_rx_state = SLIP_RX_STATE_SYNCSEARCH;
        _slipmacdriver.pCurSlipRxBuffer->length = 0;
    }
}

static void sli_slip_comm_receive_complete_cb(UARTDRV_Handle_t handle,
                                              Ecode_t transferStatus,
                                              uint8_t *data,
                                              UARTDRV_Count_t transferCount)
{
    UARTDRV_Count_t cnt;
    (void) transferStatus;
    (void) handle;

    // TODO error checking

    for (cnt = 0; cnt < transferCount; cnt++) {
        process_rx_byte(data[cnt]);
    }
}

void sli_slip_comm_receive(void)
{
  Ecode_t ec;
  (void)ec;

  CORE_SetNvicRamTableHandler(SLI_SLIP_USART_RX_IRQn, (void *) SLI_SLIP_USART_RX_IRQHandler);

  // Clear pending RX interrupt flag in NVIC
  NVIC_ClearPendingIRQ(SLI_SLIP_USART_RX_IRQn);
  NVIC_EnableIRQ(SLI_SLIP_USART_RX_IRQn);

  // Setup RX timeout to 255 bit-time
  sl_uartdrv_usart_vcom_handle->peripheral.uart->TIMECMP1 = \
    USART_TIMECMP1_TSTOP_RXACT
    | USART_TIMECMP1_TSTART_RXEOF
    | (0xff << _USART_TIMECMP1_TCMPVAL_SHIFT);

  // Clear any USART interrupt flags
  USART_IntClear(sl_uartdrv_usart_vcom_handle->peripheral.uart,
                 _USART_IF_MASK);
  USART_IntEnable(sl_uartdrv_usart_vcom_handle->peripheral.uart,
                  USART_IF_TXIDLE | USART_IF_TCMP1);

  ec = UARTDRV_Receive(sl_uartdrv_usart_vcom_handle,
                       sli_slip_rx_buf,
                       sizeof(sli_slip_rx_buf),
                       sli_slip_comm_receive_complete_cb);
}

void SLI_SLIP_USART_RX_IRQHandler(void)
{
  // RX timeout, stop transfer and handle what we got in buffer
  if (sl_uartdrv_usart_vcom_handle->peripheral.uart->IF & USART_IF_TCMP1) {
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_ATOMIC();
    // stop the timer
    sl_uartdrv_usart_vcom_handle->peripheral.uart->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
    sl_uartdrv_usart_vcom_handle->peripheral.uart->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;
    // clear timer interrupt
    USART_IntClear(sl_uartdrv_usart_vcom_handle->peripheral.uart, USART_IF_TCMP1);
    // abort receive operation
    UARTDRV_Abort_receive_timeout(sl_uartdrv_usart_vcom_handle);
    sli_slip_comm_receive();
    CORE_EXIT_ATOMIC();
  }
}

int8_t Slip_Init(uint8_t *mac)
{
    memset(&_slipmacdriver, 0, sizeof(SlipMACDriver));

    _slipmacdriver.pCurSlipRxBuffer = NULL;
    _slipmacdriver.pCurSlipTxBuffer = NULL;
    _slipmacdriver.slip_rx_state = SLIP_RX_STATE_SYNCED;

    if (mac != NULL) {
        // Assign user submitted MAC value
        for (uint8_t i = 0; i < sizeof(_slipmacdriver.slip_mac); ++i) {
            _slipmacdriver.slip_mac[i] = mac[i];
        }
    } else {
        // Generate pseudo value for MAC
        for (uint8_t i = 0; i < sizeof(_slipmacdriver.slip_mac); ++i) {
            _slipmacdriver.slip_mac[i] = i + 2;
        }
    }

    //Build driver data structure
    memset(&_slipmacdriver.slip_phy_driver, 0, sizeof(phy_device_driver_s));

    _slipmacdriver.slip_phy_driver.PHY_MAC = _slipmacdriver.slip_mac;
    _slipmacdriver.slip_phy_driver.link_type = PHY_LINK_SLIP;
    _slipmacdriver.slip_phy_driver.data_request_layer = IPV6_DATAGRAMS_DATA_FLOW;
    _slipmacdriver.slip_phy_driver.driver_description = (char *)"SLIP";
    _slipmacdriver.slip_phy_driver.tx = slip_if_tx;
    drv = &_slipmacdriver.slip_phy_driver;

    // define and bring up the interface
    _slipmacdriver.net_slip_id = arm_net_phy_register(&_slipmacdriver.slip_phy_driver);

    // init rx state machine
    tr_debug("SLIP driver id: %d\r\n", _slipmacdriver.net_slip_id);

    for (int i = 0; i < SLIP_NR_BUFFERS; i++) {
        CircularBuffer_push(&_slipmacdriver.TxSlipBufferFreeList, &_slipmacdriver.tx_buffer[i]);
    }

    for (int i = 0; i < SLIP_NR_BUFFERS; i++) {
        CircularBuffer_push(&_slipmacdriver.RxSlipBufferFreeList, &_slipmacdriver.rx_buffer[i]);
    }

    SLIP_IRQ_Thread_Create();

    sli_slip_comm_receive();

    return _slipmacdriver.net_slip_id;
}

void slip_if_rx(const SlipBuffer *rx_buf)
{
    if (_slipmacdriver.slip_phy_driver.phy_rx_cb) {
        _slipmacdriver.slip_phy_driver.phy_rx_cb(rx_buf->buf, rx_buf->length, 0x80, 0, _slipmacdriver.net_slip_id);
    }
}

void buffer_handover()
{
    for (;;) {
        SlipBuffer *rx_buf = NULL;

        CORE_CRITICAL_SECTION(
            CircularBuffer_pop(&_slipmacdriver.RxSlipBufferToRxFuncList, &rx_buf);
        )

        if (!rx_buf) {
            break;
        }

        slip_if_rx(rx_buf);

        CORE_CRITICAL_SECTION(
            rx_buf->length = 0;
            CircularBuffer_push(&_slipmacdriver.RxSlipBufferFreeList, rx_buf);
        )
    }
}

static void slip_if_lock(void)
{
    platform_enter_critical();
}

static void slip_if_unlock(void)
{
    platform_exit_critical();
}

static __NO_RETURN void SLIP_IRQ_Thread(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t event = osThreadFlagsWait(SIG_SL_RX, osFlagsWaitAny, osWaitForever);

        slip_if_lock();
        if (event & SIG_SL_RX) {
            buffer_handover();
        }
        slip_if_unlock();
    }
}

void SLIP_IRQ_Thread_Create(void)
{
    slip_thread_attr.stack_mem  = &slip_thread_stk[0];
    slip_thread_attr.cb_mem  = &slip_thread_tcb;
    slip_thread_attr.stack_size = sizeof(slip_thread_stk);
    slip_thread_attr.cb_size = sizeof(slip_thread_tcb);
    slip_thread_id = osThreadNew(SLIP_IRQ_Thread, NULL, &slip_thread_attr);
}

/** Push the transaction to the buffer. This overwrites the buffer if it's
 *  full
 *
 * @param data Data to be pushed to the buffer
 */
void CircularBuffer_push(CircularBuffer *buf, SlipBuffer *data)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_ATOMIC();
    if (CircularBuffer_full(buf)) {
        buf->_tail++;
        if (buf->_tail == SLIP_NR_BUFFERS) {
            buf->_tail = 0;
        }
    }
    buf->_pool[buf->_head++] = data;
    if (buf->_head == SLIP_NR_BUFFERS) {
        buf->_head = 0;
    }
    if (buf->_head == buf->_tail) {
        buf->_full = true;
    }
    CORE_EXIT_ATOMIC();
}

/** Pop the transaction from the buffer
 *
 * @param data Data to be popped from the buffer
 * @return True if the buffer is not empty and data contains a transaction, false otherwise
 */
bool CircularBuffer_pop(CircularBuffer *buf, SlipBuffer **data)
{
    CORE_DECLARE_IRQ_STATE;
    bool data_popped = false;
    CORE_ENTER_ATOMIC();
    if (!CircularBuffer_empty(buf)) {
        *data = buf->_pool[buf->_tail++];
        if (buf->_tail == SLIP_NR_BUFFERS) {
            buf->_tail = 0;
        }
        buf->_full = false;
        data_popped = true;
    }
    CORE_EXIT_ATOMIC();
    return data_popped;
}

/** Check if the buffer is empty
 *
 * @return True if the buffer is empty, false if not
 */
bool CircularBuffer_empty(CircularBuffer *buf)
{
    bool is_empty;

    CORE_CRITICAL_SECTION(
        is_empty = ((buf->_head == buf->_tail) && !buf->_full);
        )

    return is_empty;
}

/** Check if the buffer is full
 *
 * @return True if the buffer is full, false if not
 */
bool CircularBuffer_full(CircularBuffer *buf)
{
    return buf->_full;
}

/** Reset the buffer
 *
 */
void CircularBuffer_reset(CircularBuffer *buf)
{
    CORE_CRITICAL_SECTION(
        buf->_head = 0;
        buf->_tail = 0;
        buf->_full = false;
    )
}

/** Get the number of elements currently stored in the circular_buffer */
uint32_t CircularBuffer_size(CircularBuffer *buf)
{
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_ATOMIC();
    uint32_t elements;
    if (!buf->_full) {
        if (buf->_head < buf->_tail) {
            elements = SLIP_NR_BUFFERS + buf->_head - buf->_tail;
        } else {
            elements = buf->_head - buf->_tail;
        }
    } else {
        elements = SLIP_NR_BUFFERS;
    }
    CORE_EXIT_ATOMIC();
    return elements;
}

// TODO: this should be provided by UARTDRV
static Ecode_t UARTDRV_Abort_receive_timeout(UARTDRV_Handle_t handle)
{
  UARTDRV_Buffer_t *rxBuffer;
  Ecode_t status;
  CORE_DECLARE_IRQ_STATE;

  if (handle == NULL) {
    return ECODE_EMDRV_UARTDRV_ILLEGAL_HANDLE;
  }

  CORE_ENTER_ATOMIC();
  if (handle->rxQueue->used == 0) {
    CORE_EXIT_ATOMIC();
    return ECODE_EMDRV_UARTDRV_IDLE;
  }

  // Stop the current transfer
  DMADRV_StopTransfer(handle->rxDmaCh);
  handle->rxDmaActive = false;
  // Update the transfer status of the active transfer
  status = GetTailBuffer(handle->rxQueue, &rxBuffer);
  // If an abort was in progress when DMA completed, the ISR could be deferred
  // until after the critical section. In this case, the buffers no longer
  // exist, even though the DMA complete callback was called.
  if (status == ECODE_EMDRV_UARTDRV_QUEUE_EMPTY) {
    return ECODE_EMDRV_UARTDRV_QUEUE_EMPTY;
  }
  EFM_ASSERT(rxBuffer != NULL);
  DMADRV_TransferRemainingCount(handle->rxDmaCh,
                                (int*)&rxBuffer->itemsRemaining);
  rxBuffer->transferStatus = ECODE_EMDRV_UARTDRV_ABORTED;

  // Dequeue all transfers and call callback
  while (handle->rxQueue->used > 0) {
    DequeueBuffer(handle->rxQueue, &rxBuffer);
    // Call the callback
    if (rxBuffer->callback != NULL) {
      if (rxBuffer->callback != NULL) {
        rxBuffer->callback(handle,
                           ECODE_EMDRV_UARTDRV_OK,
                           rxBuffer->data,
                           rxBuffer->transferCount - rxBuffer->itemsRemaining);
      }
    }
  }

  // Disable the receiver
  if (handle->fcType != uartdrvFlowControlHwUart) {
    DisableReceiver(handle);
  }
  CORE_EXIT_ATOMIC();

  return ECODE_EMDRV_UARTDRV_OK;
}

static Ecode_t GetTailBuffer(UARTDRV_Buffer_FifoQueue_t *queue,
                             UARTDRV_Buffer_t **buffer)
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_ATOMIC();
  if (queue->used == 0) {
    *buffer = NULL;
    CORE_EXIT_ATOMIC();
    return ECODE_EMDRV_UARTDRV_QUEUE_EMPTY;
  }
  *buffer = &queue->fifo[queue->tail];

  CORE_EXIT_ATOMIC();
  return ECODE_EMDRV_UARTDRV_OK;
}

static Ecode_t DequeueBuffer(UARTDRV_Buffer_FifoQueue_t *queue,
                             UARTDRV_Buffer_t **buffer)
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_ATOMIC();
  if (queue->used == 0) {
    *buffer = NULL;
    CORE_EXIT_ATOMIC();
    return ECODE_EMDRV_UARTDRV_QUEUE_EMPTY;
  }
  *buffer = &queue->fifo[queue->tail];
  queue->tail = (queue->tail + 1) % queue->size;
  queue->used--;
  CORE_EXIT_ATOMIC();

  return ECODE_EMDRV_UARTDRV_OK;
}
static void DisableReceiver(UARTDRV_Handle_t handle)
{
#if (defined(LEUART_COUNT) && (LEUART_COUNT > 0) && !defined(_SILICON_LABS_32B_SERIES_2)) \
  || (defined(EUART_COUNT) && (EUART_COUNT > 0) )
  if (handle->type == uartdrvUartTypeUart)
#endif
  {
    // Disable Rx route
#if defined(USART_ROUTEPEN_RXPEN)
    handle->peripheral.uart->ROUTEPEN &= ~USART_ROUTEPEN_RXPEN;
#elif defined(USART_ROUTE_RXPEN)
    handle->peripheral.uart->ROUTE &= ~USART_ROUTE_RXPEN;
#elif defined(GPIO_USART_ROUTEEN_RXPEN)
    GPIO->USARTROUTE_CLR[handle->uartNum].ROUTEEN = GPIO_USART_ROUTEEN_RXPEN;
#endif
    // Disable Rx
    handle->peripheral.uart->CMD = USART_CMD_RXDIS;
  }
#if defined(LEUART_COUNT) && (LEUART_COUNT > 0) && !defined(_SILICON_LABS_32B_SERIES_2)
  else if (handle->type == uartdrvUartTypeLeuart) {
    // Wait for prevous register writes to sync
    while ((handle->peripheral.leuart->SYNCBUSY & LEUART_SYNCBUSY_CMD) != 0U) {
    }

    // Disable Rx route
#if defined(LEUART_ROUTEPEN_RXPEN)
    handle->peripheral.leuart->ROUTEPEN &= ~LEUART_ROUTEPEN_RXPEN;
#else
    handle->peripheral.leuart->ROUTE &= ~LEUART_ROUTE_RXPEN;
#endif
    // Disable Rx
    handle->peripheral.leuart->CMD = LEUART_CMD_RXDIS;
  }
#elif defined(EUART_COUNT) && (EUART_COUNT > 0)
  else if (handle->type == uartdrvUartTypeEuart) {
    if (EUSART_StatusGet(handle->peripheral.euart) &  EUSART_STATUS_TXENS) {
      EUSART_Enable(handle->peripheral.euart, eusartEnableTx);
    } else {
      EUSART_Enable(handle->peripheral.euart, eusartDisable);
    }
  }
#endif
}