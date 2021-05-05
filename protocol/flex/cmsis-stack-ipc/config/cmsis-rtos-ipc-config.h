/***************************************************************************//**
 * @brief Connect CMSIS IPC component configuration header.
 *
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
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

// <<< Use Configuration Wizard in Context Menu >>>

// <h>Connect CMSIS IPC configuration

// <q EMBER_AF_PLUGIN_CMSIS_RTOS_CPU_USAGE> CPU usage tracking
// <i> Default: 0
// <i> If this option is enabled, the OS will keep track of CPU usage required from uc/Probe.
#define EMBER_AF_PLUGIN_CMSIS_RTOS_CPU_USAGE                      (0)

// <o EMBER_AF_PLUGIN_CMSIS_RTOS_CONNECT_STACK_PRIO> Connect task priority <2-55>
// <i> Default: 39
// <i> The priority of the Connect task using the CMSIS order (55 is highest priority)
#define EMBER_AF_PLUGIN_CMSIS_RTOS_CONNECT_STACK_PRIO             (39)

// <o EMBER_AF_PLUGIN_CMSIS_RTOS_CONNECT_STACK_SIZE> Connect Task call stack size <250-5000>
// <i> Default: 1000
// <i> The size in 32-bit words of the Connect task call stack.
#define EMBER_AF_PLUGIN_CMSIS_RTOS_CONNECT_STACK_SIZE             (1000)

// <o EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_PRIO> Application Framework task priority <2-55>
// <i> Default: 38
// <i> The priority of the Application Framework task using the CMSIS order (55 is highest priority)
#define EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_PRIO             (38)

// <o EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_STACK_SIZE> Application Framework Task call stack size <250-5000>
// <i> Default: 500
// <i> The size in 32-bit words of the Application Framework task call stack.
#define EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_STACK_SIZE       (500)

// <o EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_YIELD_TIMEOUT_MS> Application Framework Task yield timeout
// <i> Default: 1000000
// <i> The Application Framework Task yield timeout in milliseconds. This is the most the application task shall yield. Upon timeout, the task will check again if yielding is possible.
#define EMBER_AF_PLUGIN_CMSIS_RTOS_APP_FRAMEWORK_YIELD_TIMEOUT_MS (1000000)

// <o EMBER_AF_PLUGIN_CMSIS_RTOS_MAX_CALLBACK_QUEUE_SIZE> Max callback queue size <5-20>
// <i> Default: 10
// <i> The maximum number of simultaneous callback messages from the stack task to the application tasks.
#define EMBER_AF_PLUGIN_CMSIS_RTOS_MAX_CALLBACK_QUEUE_SIZE        (10)

// </h>

// <<< end of configuration section >>>
