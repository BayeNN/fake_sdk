/***************************************************************************//**
 * @file
 * @brief Provides definitions required to support CLI in both SoC and RCP
 * builds
 *
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 *
 * https://www.silabs.com/about-us/legal/master-software-license-agreement
 *
 * This software is distributed to you in Source Code format and is governed by
 * the sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <openthread/cli.h>

extern otCliCommand sl_ot_custom_commands[];
extern const size_t sl_ot_custom_commands_count;

/**
 * Write formatted string to the Diag output buffer
 *
 * @param[in]  aFmt   A pointer to the format string.
 * @param[in]  ...    A matching list of arguments.
 *
 */
void otPlatDiagOutputFormat(const char *aFmt, ...);

/**
 * Write error code to the Diag output buffer
 *
 * If the @p aError is `OT_ERROR_PENDING` nothing will be outputted.
 *
 * @param[in]  aError Error code value.
 *
 */
void otPlatDiagAppendResult(otError aError);

/**
 * Print all commands in @p commands
 *
 * @param[in]  commands         list of commands
 * @param[in]  commandCount     number of commands in @p commands
 *
 */
void printCommands(otCliCommand commands[], size_t commandCount);

/**
 * Call the corresponding handler for a command
 *
 * This function will look through @p commands to find a @ref otCliCommand that
 * matches @p argv[0]. If found, the handler function for the command will be
 * called with the remaining args passed to it.
 *
 * @param[in]  context          a context
 * @param[in]  argc             number of args
 * @param[in]  argv             list of args
 * @param[in]  commands         list of commands
 * @param[in]  commandCount     number of commands in @p commands
 *
 * @retval false if @p argv[0] is not a command in @p commands
 * @retval true if @p argv[0] is found in @p commands
 *
 */
bool handleCommand(void *context, uint8_t argc, char *argv[], otCliCommand commands[], size_t commandCount);

#if OPENTHREAD_RADIO
#define SL_OT_PRINTF(...) otPlatDiagOutputFormat(__VA_ARGS__)
#define SL_OT_APPEND_RESULT(error) otPlatDiagAppendResult(error)
#else
#define SL_OT_PRINTF(...) otCliOutputFormat(__VA_ARGS__)
#define SL_OT_APPEND_RESULT(error) otCliAppendResult(error)
#endif
