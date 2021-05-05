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

#ifndef APP_FRAMEWORK_COMMON_H
#define APP_FRAMEWORK_COMMON_H

#include "stack/include/ember.h"

void connect_stack_init(void);
void connect_app_framework_init(void);
void connect_sleep_init(void);

void connect_stack_tick(void);
void connect_app_framework_tick(void);
void connect_sleep_tick(void);

bool connect_is_ok_to_sleep(void);

void connect_standard_phy_2_4g(void);

/**
 * @addtogroup app_framework_common
 * @{
 */

/**
 *
 * @brief Allocate a new event to the app event table.
 *
 *  @param[out] control   The EmberEventControl to allocate
 *
 *  @param[in] handler   Pointer to the handler function associated to the event
 *
 *  @return   An ::EmberStatus value of:
 *  - ::EMBER_SUCCESS if the event was successfully allocated.
 *  - ::EMBER_TABLE_FULL if no more event could be allocated.
 *  @sa emberAfAllocateEvent()
 */
EmberStatus emberAfAllocateEvent(EmberEventControl **control, void (*handler)(void));
/**
 * @}
 */

#endif // APP_FRAMEWORK_COMMON_H
