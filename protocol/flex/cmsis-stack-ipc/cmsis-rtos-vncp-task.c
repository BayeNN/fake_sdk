/***************************************************************************//**
 * @brief CMSIS RTOS vncp code.
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
// Define module name for Power Manager debuging feature.
#define CURRENT_MODULE_NAME    "FLEX_RTOS_VNCP_TASK"

#include PLATFORM_HEADER
#include "cmsis-rtos-ipc-config.h"
#include "sl_component_catalog.h"

#include "stack/include/api-rename.h"
#include "stack/include/ember.h"
#include "stack/include/api-rename-undef.h"

#include "hal.h"

#include "app_framework_common.h"

#include "cmsis-rtos-support.h"

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
  #include "sl_power_manager.h"
#endif // SL_CATALOG_POWER_MANAGER_PRESENT

// Some large value still manageable by the OS and the BSP tick code (in case
// sleep is enabled).
#define MAX_VNCP_YIELD_TIME_MS (1000000)

//------------------------------------------------------------------------------
// Forward declarations and external declarations

static void connectStackTaskYield(void);

extern osEventFlagsId_t emAfPluginCmsisRtosFlags;

//------------------------------------------------------------------------------
// Internal APIs

// The stack task main loop.
void emAfPluginCmsisRtosStackTask(void *p_arg)
{
  (void)p_arg;

  connect_stack_init();

  while (true) {
    connect_stack_tick();

    // Process IPC commands from application tasks.
    emAfPluginCmsisRtosProcessIncomingApiCommand();
    // Yield the Connect stack task if possible.
    connectStackTaskYield();
  }
}

// This can be called from ISR.
void emAfPluginCmsisRtosWakeUpConnectStackTask(void)
{
  assert((osEventFlagsSet(emAfPluginCmsisRtosFlags,
                          FLAG_STACK_ACTION_PENDING) & CMSIS_RTOS_ERROR_MASK) == 0);
}

//------------------------------------------------------------------------------
// Static functions

static void connectStackTaskYield(void)
{
  uint16_t currentStackTasks;
  uint32_t idleTimeMs = emApiStackIdleTimeMs(&currentStackTasks);

  if (idleTimeMs > 0) {
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    bool stackTaskDeepSleepAllowed =
      ((currentStackTasks & EMBER_HIGH_PRIORITY_TASKS) == 0);
#endif // SL_CATALOG_POWER_MANAGER_PRESENT

    if (idleTimeMs > MAX_VNCP_YIELD_TIME_MS) {
      idleTimeMs = MAX_VNCP_YIELD_TIME_MS;
    }

    uint32_t yieldTimeTicks = (osKernelGetTickFreq() * idleTimeMs) / 1000;

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    if (!stackTaskDeepSleepAllowed) {
      sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
    }
#endif // SL_CATALOG_POWER_MANAGER_PRESENT

    // Pend on a stack action.
    osEventFlagsWait(emAfPluginCmsisRtosFlags,
                     FLAG_STACK_ACTION_PENDING,
                     osFlagsWaitAny,
                     yieldTimeTicks);

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    if (!stackTaskDeepSleepAllowed) {
      sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
    }
#endif // SL_CATALOG_POWER_MANAGER_PRESENT
  }
}
