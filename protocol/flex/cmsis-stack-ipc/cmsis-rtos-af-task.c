/***************************************************************************//**
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
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

#include PLATFORM_HEADER
#include "cmsis-rtos-ipc-config.h"

#include "stack/include/ember.h"
#include "cmsis-rtos-support.h"
#include "app_framework_common.h"

//------------------------------------------------------------------------------
// Forward and external declarations.

static void appFrameworkTaskYield(void);

extern EmberTaskId emAppTask;
extern const EmberEventData emAppEvents[];

extern osEventFlagsId_t emAfPluginCmsisRtosFlags;

//------------------------------------------------------------------------------
// Internal APIs.

void emAfPluginCmsisRtosAppFrameworkTask(void *p_arg)
{
  (void)p_arg;

  connect_app_framework_init();

  while (true) {
    connect_app_framework_tick();

    // Process incoming callback commands from the vNCP.
    if (!emAfPluginCmsisRtosProcessIncomingCallbackCommand()) {
      // Yield the Application Framework task if no callback message needs to be
      // processed.
      appFrameworkTaskYield();
    }
  }
}

void emAfPluginCmsisRtosWakeUpAppFrameworkTask(void)
{
  assert((osEventFlagsSet(emAfPluginCmsisRtosFlags,
                          FLAG_STACK_CALLBACK_PENDING) & CMSIS_RTOS_ERROR_MASK) == 0);
}

//------------------------------------------------------------------------------
// Static functions.

static void appFrameworkTaskYield(void)
{
  uint32_t idleTimeMs = emberMsToNextEvent(emAppEvents,
                                           EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_YIELD_TIMEOUT_MS);

  if (idleTimeMs > 0) {
    uint32_t yieldTimeTicks = (osKernelGetTickFreq() * idleTimeMs) / 1000;

    osEventFlagsWait(emAfPluginCmsisRtosFlags,
                     FLAG_STACK_CALLBACK_PENDING,
                     osFlagsWaitAny,
                     yieldTimeTicks);
  }
}
