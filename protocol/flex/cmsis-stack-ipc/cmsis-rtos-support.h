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

#ifndef _CMSIS_RTOS_SUPPORT_H_
#define _CMSIS_RTOS_SUPPORT_H_

#include <cmsis_os2.h>
#include "cmsis-rtos-support-gen.h"

#define FLAG_STACK_ACTION_PENDING                       0x01
#define FLAG_STACK_CALLBACK_PENDING                     0x02
#define FLAG_IPC_COMMAND_PENDING                        0x04
#define FLAG_IPC_RESPONSE_PENDING                       0x08

#define CMSIS_RTOS_ERROR_MASK                           0x80000000

//------------------------------------------------------------------------------
// Public APIs

void emberAfPluginCmsisRtosIpcInit(void);

void emberAfPluginCmsisRtosAcquireBufferSystemMutex(void);

void emberAfPluginCmsisRtosReleaseBufferSystemMutex(void);

//------------------------------------------------------------------------------
// Internal APIs - generic OS

void emAfPluginCmsisRtosInitTasks(void);

void emAfPluginCmsisRtosStackTask(void *p_arg);

void emAfPluginCmsisRtosAppFrameworkTask(void *p_arg);

bool emAfPluginCmsisRtosIsCurrentTaskStackTask(void);

osThreadId_t emAfPluginCmsisRtosGetStackTcb(void);

void emAfPluginCmsisRtosWakeUpConnectStackTask(void);

void emAfPluginCmsisRtosWakeUpAppFrameworkTask(void);

//------------------------------------------------------------------------------
// Internal APIs - IPC

void emAfPluginCmsisRtosIpcInit(void);

void emAfPluginCmsisRtosSendBlockingCommand(uint16_t identifier,
                                            const char *format,
                                            ...);

void emAfPluginCmsisRtosSendResponse(uint16_t identifier,
                                     const char *format,
                                     ...);

void emAfPluginCmsisRtosSendCallbackCommand(uint16_t identifier,
                                            const char *format,
                                            ...);

void emAfPluginCmsisRtosProcessIncomingApiCommand(void);

void emAfPluginCmsisRtosHandleIncomingApiCommand(uint16_t commandId);

bool emAfPluginCmsisRtosProcessIncomingCallbackCommand(void);

void emAfPluginCmsisRtosHandleIncomingCallbackCommand(uint16_t commandId,
                                                      uint8_t *callbackParams);

void emAfPluginCmsisRtosAcquireCommandMutex(void);

void emAfPluginCmsisRtosReleaseCommandMutex(void);

void emAfPluginCmsisRtosFetchApiParams(PGM_P format, ...);

void emAfPluginCmsisRtosFetchCallbackParams(uint8_t *callbackParams,
                                            PGM_P format,
                                            ...);

#endif // _CMSIS_RTOS_SUPPORT_H_
