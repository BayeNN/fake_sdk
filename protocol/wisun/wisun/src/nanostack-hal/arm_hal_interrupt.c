/*
 * Copyright (c) 2016-2018, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <assert.h>
#include <cmsis_os2.h>
#include "sl_cmsis_os2_common.h"
#include "arm_hal_interrupt.h"

static osMutexId_t sli_wisun_platform_mutex;
static uint8_t sys_irq_disable_counter;

void platform_critical_init(void)
{
  const osMutexAttr_t sli_wisun_platform_mutex_attr = {
    "Wi-SUN Platform Mutex",
    osMutexRecursive,
    NULL,
    0
  };

  sli_wisun_platform_mutex = osMutexNew(&sli_wisun_platform_mutex_attr);
  assert(sli_wisun_platform_mutex != NULL);
}

void platform_enter_critical(void)
{
  assert(osMutexAcquire(sli_wisun_platform_mutex, osWaitForever) == osOK);
  sys_irq_disable_counter++;
}

void platform_exit_critical(void)
{
  --sys_irq_disable_counter;
  assert(osMutexRelease(sli_wisun_platform_mutex) == osOK);
}
