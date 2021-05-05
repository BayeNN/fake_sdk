/*
 *  Copyright (c) 2021, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for the diagnostics.
 *
 */

#include OPENTHREAD_PROJECT_CORE_CONFIG_FILE

#include <stdbool.h>
#include <stdio.h>
#include <sys/time.h>

#include "sl_ot_custom_cli.h"
#include "common/code_utils.hpp"
#include <openthread/cli.h>
#include <openthread/config.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/radio.h>
#include "platform-efr32.h"

#if OPENTHREAD_CONFIG_DIAG_ENABLE

/**
 * Diagnostics mode variables.
 *
 */
static bool sDiagMode = false;

void otPlatDiagModeSet(bool aMode)
{
    sDiagMode = aMode;
}

bool otPlatDiagModeGet()
{
    return sDiagMode;
}

void otPlatDiagChannelSet(uint8_t aChannel)
{
    OT_UNUSED_VARIABLE(aChannel);
}

void otPlatDiagTxPowerSet(int8_t aTxPower)
{
    OT_UNUSED_VARIABLE(aTxPower);
}

void otPlatDiagRadioReceived(otInstance *aInstance, otRadioFrame *aFrame, otError aError)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aFrame);
    OT_UNUSED_VARIABLE(aError);
}

void otPlatDiagAlarmCallback(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
}

#if OPENTHREAD_RADIO

static char *diagOutput = NULL;
static size_t diagOutputCount = 0;
static size_t diagOutputMaxLen = 0;

void otPlatDiagOutputFormat(const char *aFmt, ...)
{
    if (!diagOutput || (diagOutputCount >= diagOutputMaxLen))
    {
        return;
    }

    va_list args;
    va_start(args, aFmt);
    diagOutputCount += vsnprintf(&diagOutput[diagOutputCount], diagOutputMaxLen, aFmt, args);
    va_end(args);
}

void otPlatDiagAppendResult(otError aError)
{
    switch (aError)
    {
    case OT_ERROR_NONE:
        SL_OT_PRINTF("Done");
        break;

    case OT_ERROR_PENDING:
        break;

    default:
        SL_OT_PRINTF("Error %d: %s", aError, otThreadErrorToString(aError));
    }
}

otError otPlatDiagProcess(otInstance *aInstance,
                          uint8_t     aArgsLength,
                          char *      aArgs[],
                          char *      aOutput,
                          size_t      aOutputMaxLen)
{
    otError error = OT_ERROR_INVALID_COMMAND;
    bool commandFound = false;
    VerifyOrExit(aArgsLength != 0);

    diagOutput = aOutput;
    diagOutputMaxLen = aOutputMaxLen;

    commandFound = handleCommand(aInstance, aArgsLength, aArgs, sl_ot_custom_commands, sl_ot_custom_commands_count);

    if (commandFound)
    {
        error = OT_ERROR_NONE;
    }

exit:
    diagOutput = NULL;
    diagOutputCount = 0;
    diagOutputMaxLen = 0;
    return error;
}
#endif

#endif // #if OPENTHREAD_CONFIG_DIAG_ENABLE