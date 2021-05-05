/*
 *  Copyright (c) 2017, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for UART communication
 *   using the iostream APIs
 *
 */

#include <stddef.h>
#include <string.h>
#include "openthread-system.h"
#include "utils/code_utils.h"
#include "utils/uart.h"

#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT

#ifdef SL_CATALOG_KERNEL_PRESENT
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "sl_iostream_usart_vcom_config.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include "sl_iostream_init_usart_instances.h"

#define RECEIVE_BUFFER_SIZE 128

static uint8_t              sReceiveBuffer[RECEIVE_BUFFER_SIZE];
static const uint8_t *      sTransmitBuffer = NULL;
static volatile uint16_t    sTransmitLength = 0;

#ifdef SL_CATALOG_KERNEL_PRESENT

static void gpioSerialWakeupCallback(uint8_t pin)
{
  if (pin == SL_IOSTREAM_USART_VCOM_RX_PIN) {
      otSysEventSignalPending();
  }
}
#endif // SL_CATALOG_KERNEL_PRESENT

static void processReceive(void)
{
    sl_status_t status;
    size_t bytes_read = 0;
    memset(sReceiveBuffer, 0, RECEIVE_BUFFER_SIZE);

#ifdef SL_CATALOG_KERNEL_PRESENT
    // Set Read API to non-blocking mode
    sl_iostream_uart_set_read_block((sl_iostream_uart_t *)sl_iostream_uart_vcom_handle, false);
#endif // SL_CATALOG_KERNEL_PRESENT
 
    status = sl_iostream_read(sl_iostream_vcom_handle, &sReceiveBuffer, sizeof(sReceiveBuffer), &bytes_read);

    if (status == SL_STATUS_OK) {
        otPlatUartReceived(sReceiveBuffer, bytes_read);
    }
}

static void processTransmit(void)
{
    if (sTransmitBuffer != NULL && sTransmitLength == 0)
    {
        sTransmitBuffer = NULL;
        otPlatUartSendDone();
    }
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatUartEnable(void)
{
#ifdef SL_CATALOG_KERNEL_PRESENT
    GPIOINT_Init();
    GPIOINT_CallbackRegister(SL_IOSTREAM_USART_VCOM_RX_PIN, gpioSerialWakeupCallback);
    GPIO_IntConfig(SL_IOSTREAM_USART_VCOM_RX_PORT,
                   SL_IOSTREAM_USART_VCOM_RX_PIN,
                   false, true, true);
#endif
    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError error = OT_ERROR_NONE;
    sl_status_t status;

    otEXPECT_ACTION(sTransmitBuffer == NULL, error = OT_ERROR_BUSY);

    sTransmitBuffer = aBuf;
    sTransmitLength = aBufLength;

    status = sl_iostream_write(sl_iostream_vcom_handle, (uint8_t *)sTransmitBuffer, sTransmitLength);
    if (status == SL_STATUS_OK) {
        sTransmitLength = 0;
    }
    otSysEventSignalPending();
exit:
    return error;
}

void efr32UartProcess(void)
{
    processReceive();
    processTransmit();
}
