/***************************************************************************//**
 * @file
 * @brief MTD application logic.
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
// Define module name for Power Manager debuging feature.
#define CURRENT_MODULE_NAME    "OPENTHREAD_SAMPLE_APP"

#include <string.h>
#include <assert.h>

#include <openthread/cli.h>
#include <openthread/dataset_ftd.h>
#include <openthread/instance.h>
#include <openthread/message.h>
#include <openthread/thread.h>
#include <openthread/udp.h>
#include <openthread/platform/logging.h>
#include <common/logging.hpp>

#include "sl_button.h"
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"
#include "sl_led.h"
#include "sl_simple_led_instances.h"

#include "sl_component_catalog.h"
#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
#include "sl_power_manager.h"
#endif

// Constants
#define MULTICAST_ADDR "ff03::1"
#define MULTICAST_PORT 123
#define RECV_PORT 234
#define SLEEPY_POLL_PERIOD_MS 2000
#define MTD_MESSAGE "mtd button"
#define FTD_MESSAGE "ftd button"

// Forward declarations
otInstance *otGetInstance(void);
void mtdReceiveCallback(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo);
extern void otSysEventSignalPending(void);

// Variables
static otUdpSocket         sMtdSocket;
static bool                sButtonPressed                 = false;
static bool                sRxOnIdleButtonPressed         = false;
static bool                sAllowSleep                    = false;

void sleepyInit(void)
{
    otCliOutputFormat("sleepy-demo-mtd started\r\n");

    otLinkModeConfig config;
    otLinkSetPollPeriod(otGetInstance(), SLEEPY_POLL_PERIOD_MS);

    config.mRxOnWhenIdle = true;
    config.mDeviceType   = 0;
    config.mNetworkData  = 0;
    otThreadSetLinkMode(otGetInstance(), config);
}

/*
 * Callback from sl_ot_is_ok_to_sleep to check if it is ok to go to sleep.
 */
bool efr32AllowSleepCallback(void)
{
    return sAllowSleep;
}

/*
 * Override default network settings, such as panid, so the devices can join a network
 */
void setNetworkConfiguration(void)
{
    static char          aNetworkName[] = "SleepyEFR32";
    otOperationalDataset aDataset;

    memset(&aDataset, 0, sizeof(otOperationalDataset));

    /*
     * Fields that can be configured in otOperationDataset to override defaults:
     *     Network Name, Mesh Local Prefix, Extended PAN ID, PAN ID, Delay Timer,
     *     Channel, Channel Mask Page 0, Network Master Key, PSKc, Security Policy
     */
    aDataset.mActiveTimestamp                      = 1;
    aDataset.mComponents.mIsActiveTimestampPresent = true;

    /* Set Channel to 15 */
    aDataset.mChannel                      = 15;
    aDataset.mComponents.mIsChannelPresent = true;

    /* Set Pan ID to 2222 */
    aDataset.mPanId                      = (otPanId)0x2222;
    aDataset.mComponents.mIsPanIdPresent = true;

    /* Set Extended Pan ID to C0DE1AB5C0DE1AB5 */
    uint8_t extPanId[OT_EXT_PAN_ID_SIZE] = {0xC0, 0xDE, 0x1A, 0xB5, 0xC0, 0xDE, 0x1A, 0xB5};
    memcpy(aDataset.mExtendedPanId.m8, extPanId, sizeof(aDataset.mExtendedPanId));
    aDataset.mComponents.mIsExtendedPanIdPresent = true;

    /* Set master key to 1234C0DE1AB51234C0DE1AB51234C0DE */
    uint8_t key[OT_MASTER_KEY_SIZE] = {0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5, 0x12, 0x34, 0xC0, 0xDE, 0x1A, 0xB5, 0x12, 0x34, 0xC0, 0xDE};
    memcpy(aDataset.mMasterKey.m8, key, sizeof(aDataset.mMasterKey));
    aDataset.mComponents.mIsMasterKeyPresent = true;

    /* Set Network Name to SleepyEFR32 */
    size_t length = strlen(aNetworkName);
    assert(length <= OT_NETWORK_NAME_MAX_SIZE);
    memcpy(aDataset.mNetworkName.m8, aNetworkName, length);
    aDataset.mComponents.mIsNetworkNamePresent = true;

    otDatasetSetActive(otGetInstance(), &aDataset);
}

void initUdp(void)
{
    otError    error;
    otSockAddr bindAddr;

    // Initialize bindAddr
    memset(&bindAddr, 0, sizeof(bindAddr));
    bindAddr.mPort = RECV_PORT;

    error = otUdpOpen(otGetInstance(), &sMtdSocket, mtdReceiveCallback, NULL);

    if (error != OT_ERROR_NONE)
    {
        return;
    }

    error = otUdpBind(otGetInstance(), &sMtdSocket, &bindAddr);
    if (error != OT_ERROR_NONE)
    {
        otUdpClose(otGetInstance(), &sMtdSocket);
        return;
    }
}

void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (&sl_button_btn0 == handle) {
      sRxOnIdleButtonPressed = true;
    } else if (&sl_button_btn1 == handle) {
      sButtonPressed = true;
    }
    otSysEventSignalPending();
  }
}

#ifdef SL_CATALOG_KERNEL_PRESENT
#define applicationTick sl_ot_rtos_application_tick
#endif

void applicationTick(void)
{
    otError          error = 0;
    otMessageInfo    messageInfo;
    otMessage *      message = NULL;
    const char *     payload = MTD_MESSAGE;
    otLinkModeConfig config;

    if (sRxOnIdleButtonPressed == true)
    {
        sRxOnIdleButtonPressed = false;
        sAllowSleep            = !sAllowSleep;
        config.mRxOnWhenIdle   = !sAllowSleep;
        config.mDeviceType     = 0;
        config.mNetworkData    = 0;
        otThreadSetLinkMode(otGetInstance(), config);

#if (defined(SL_CATALOG_KERNEL_PRESENT) && defined(SL_CATALOG_POWER_MANAGER_PRESENT))
        if (sAllowSleep) {
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
        } else {
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
        }
#endif
    }

    if (sButtonPressed == true)
    {
        sButtonPressed = false;

        memset(&messageInfo, 0, sizeof(messageInfo));
        otIp6AddressFromString(MULTICAST_ADDR, &messageInfo.mPeerAddr);
        messageInfo.mPeerPort = MULTICAST_PORT;

        message = otUdpNewMessage(otGetInstance(), NULL);

        if (message != NULL)
        {
            error = otMessageAppend(message, payload, (uint16_t)strlen(payload));

            if (error == OT_ERROR_NONE)
            {
                error = otUdpSend(otGetInstance(), &sMtdSocket, message, &messageInfo);

                if (error == OT_ERROR_NONE)
                {
                    return;
                }
            }
        }

        if (message != NULL)
        {
            otMessageFree(message);
        }
    }
}

void mtdReceiveCallback(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    OT_UNUSED_VARIABLE(aContext);
    OT_UNUSED_VARIABLE(aMessage);
    OT_UNUSED_VARIABLE(aMessageInfo);
    uint8_t buf[64];
    int     length;

    length      = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);
    buf[length] = '\0';

    if (strncmp((char *)buf, FTD_MESSAGE, sizeof(FTD_MESSAGE)) != 0)
    {
        return;
    }

    sl_led_toggle(&sl_led_led0);
    otCliOutputFormat("Message Received: %s\r\n", buf);
}
