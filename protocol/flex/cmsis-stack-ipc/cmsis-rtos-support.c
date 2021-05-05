/***************************************************************************//**
 * @brief CMSIS RTOS support code.
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
#include "sl_component_catalog.h"

#include "cmsis-rtos-ipc-config.h"
#include "cmsis-rtos-support.h"
#include "sl_cmsis_os2_common.h"

#include "stack/include/api-rename.h"
#include "stack/include/ember.h"

//------------------------------------------------------------------------------
// Tasks variables and defines

static osThreadId_t connectStackId;
static uint8_t connectStackCb[osThreadCbSize];
__ALIGNED(8) static uint8_t connectTaskStack[(EMBER_AF_PLUGIN_CMSIS_RTOS_CONNECT_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];

static osThreadId_t appFrameworkId;
static uint8_t appFrameworkStackCb[osThreadCbSize];
__ALIGNED(8) static uint8_t appFrameworkStack[(EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u];

static osMutexId_t bufferSystemMutex;

//------------------------------------------------------------------------------
// Forward and external declarations.

#if defined(SL_CATALOG_MICRIUMOS_KERNEL_PRESENT)
extern void App_OS_SetAllHooks(void);
#endif

//------------------------------------------------------------------------------
// Public APIs.

void emberAfPluginCmsisRtosIpcInit(void)
{
#if defined(SL_CATALOG_MICRIUMOS_KERNEL_PRESENT)
  App_OS_SetAllHooks();
#endif

  emAfPluginCmsisRtosInitTasks();
}

void emberAfPluginCmsisRtosAcquireBufferSystemMutex(void)
{
  assert(osMutexAcquire(bufferSystemMutex,
                        osWaitForever) == osOK);
}

void emberAfPluginCmsisRtosReleaseBufferSystemMutex(void)
{
  assert(osMutexRelease(bufferSystemMutex) == osOK);
}

//------------------------------------------------------------------------------
// Internal APIs

osThreadId_t emAfPluginCmsisRtosGetStackTcb(void)
{
  return connectStackId;
}

void emAfPluginCmsisRtosInitTasks(void)
{
  // Create Connect task.
  osThreadAttr_t connectStackattribute = {
    "Connect Stask",
    osThreadDetached,
    &connectStackCb[0],
    osThreadCbSize,
    &connectTaskStack[0],
    (EMBER_AF_PLUGIN_CMSIS_RTOS_CONNECT_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u,
    EMBER_AF_PLUGIN_CMSIS_RTOS_CONNECT_STACK_PRIO,
    0,
    0
  };

  connectStackId = osThreadNew(emAfPluginCmsisRtosStackTask,
                               NULL,
                               &connectStackattribute);
  assert(connectStackId != 0);

  bufferSystemMutex = osMutexNew(NULL);
  assert(bufferSystemMutex != NULL);

  emAfPluginCmsisRtosIpcInit();

  // Create App Framework task.
  osThreadAttr_t appFrameWorkattribute = {
    "App Framework",
    osThreadDetached,
    &appFrameworkStackCb[0],
    osThreadCbSize,
    &appFrameworkStack[0],
    (EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_STACK_SIZE * sizeof(void *)) & 0xFFFFFFF8u,
    EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_PRIO,
    0,
    0
  };

  appFrameworkId = osThreadNew(emAfPluginCmsisRtosAppFrameworkTask,
                               NULL,
                               &appFrameWorkattribute);
  assert(appFrameworkId != 0);
}

//------------------------------------------------------------------------------
// Implemented callbacks

void emberAfPluginCmsisRtosStackIsr(void)
{
  emAfPluginCmsisRtosWakeUpConnectStackTask();
}

void emAcquireBufferSystemMutexHandler(void)
{
  emberAfPluginCmsisRtosAcquireBufferSystemMutex();
}

void emReleaseBufferSystemMutexHandler(void)
{
  emberAfPluginCmsisRtosReleaseBufferSystemMutex();
}

void emAfPluginCmsisStackIsrHandler(void)
{
  emAfPluginCmsisRtosWakeUpConnectStackTask();
}

// This should not fire when running within an OS.
bool emAfPluginCmsisStackIdleHandler(uint32_t *idleTimeMs)
{
  (void)idleTimeMs;

  assert(0);
  return false;
}

void halPendSvIsr(void)
{
  PendSV_Handler();
}

void halSysTickIsr(void)
{
  SysTick_Handler();
}
