/***************************************************************************//**
 * @brief Connect Application Framework common code.
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

#include "callback_dispatcher.h"
#include "app_framework_callback.h"
#include "hal.h"

#include "sl_component_catalog.h"

#if defined(SL_CATALOG_CONNECT_CMSIS_STACK_IPC_PRESENT)
extern EmberStatus emApiInit(void);
extern void emApiTick(void);
#define stack_init emApiInit
#define stack_tick emApiTick
#else
#define stack_init emberInit
#define stack_tick emberTick
#endif

EmberTaskId emAppTask;
extern const EmberEventData emAppEvents[];
extern void(*emAppEventsHandlerPtrTable[])(void);
extern const uint8_t emAfEventTableOffset;
extern uint8_t emAfEventTableHandleIndex;

void connect_standard_phy_2_4g(void)
{
  assert(emberPhyConfigInit(EMBER_STANDARD_PHY_2_4GHZ) == EMBER_SUCCESS);
}

void connect_stack_init(void)
{
  EmberStatus status;

  emberTaskEnableIdling(true);

  // Initialize the radio and the stack.  If this fails, we have to assert
  // because something is wrong.
  status = stack_init();
  assert(status == EMBER_SUCCESS);
}

void connect_app_framework_init(void)
{
  // Init and register the application events.
  emAppTask = emberTaskInit(emAppEvents);

  // Call the init callback of plugins that subscribed to it.
  emberAfInit();
  // Call the application init callback.
  emberAfInitCallback();
}

EmberStatus emberAfAllocateEvent(EmberEventControl **control, void (*handler)(void))
{
  if (emAppEvents[emAfEventTableHandleIndex + emAfEventTableOffset].control != NULL) {
    *control = emAppEvents[emAfEventTableHandleIndex + emAfEventTableOffset].control;
    emAppEventsHandlerPtrTable[emAfEventTableHandleIndex] = handler;
    emAfEventTableHandleIndex++;
    return EMBER_SUCCESS;
  } else {
    // table is full
    return EMBER_TABLE_FULL;
  }
}

void connect_stack_tick(void)
{
  // Pet the watchdog.
  halResetWatchdog();
  // Call the stack tick API.
  stack_tick();
}

void connect_app_framework_tick(void)
{
  // Pet the watchdog.
  halResetWatchdog();
  // Call the application tick callback.
  emberAfTickCallback();
  // Call the tick callback of plugins that subscribed to it.
  emberAfTick();
  // Run application events.
  emberRunTask(emAppTask);
}
