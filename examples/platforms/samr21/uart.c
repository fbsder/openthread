/*
 *  Copyright (c) 2016, The OpenThread Authors.
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
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include <stddef.h>

#include <openthread-types.h>
#include <common/code_utils.hpp>
#include <platform/uart.h>
#include "platform-samr21.h"

enum
{
    kPlatformClock = 48000000,
    kBaudRate = 115200,
    kReceiveBufferSize = 128,
};

extern void UART0IntHandler(void);

static void processReceive(void);
static void processTransmit(void);

static const uint8_t *sTransmitBuffer = NULL;
static uint16_t sTransmitLength = 0;

typedef struct RecvBuffer
{
    // The data buffer
    uint8_t mBuffer[kReceiveBufferSize];
    // The offset of the first item written to the list.
    uint16_t mHead;
    // The offset of the next item to be written to the list.
    uint16_t mTail;
} RecvBuffer;

static RecvBuffer sReceive;

ThreadError otPlatUartEnable(void)
{
    uint32_t div;

    sReceive.mHead = 0;
    sReceive.mTail = 0;


    return kThreadError_None;
}

ThreadError otPlatUartDisable(void)
{
    return kThreadError_None;
}

ThreadError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(sTransmitBuffer == NULL, error = kThreadError_Busy);

    sTransmitBuffer = aBuf;
    sTransmitLength = aBufLength;

exit:
    return error;
}

void processReceive(void)
{
    // Copy tail to prevent multiple reads
    uint16_t tail = sReceive.mTail;

    // If the data wraps around, process the first part
    if (sReceive.mHead > tail)
    {
        otPlatUartReceived(sReceive.mBuffer + sReceive.mHead, kReceiveBufferSize - sReceive.mHead);

        // Reset the buffer mHead back to zero.
        sReceive.mHead = 0;
    }

    // For any data remaining, process it
    if (sReceive.mHead != tail)
    {
        otPlatUartReceived(sReceive.mBuffer + sReceive.mHead, tail - sReceive.mHead);

        // Set mHead to the local tail we have cached
        sReceive.mHead = tail;
    }
}

void processTransmit(void)
{
    VerifyOrExit(sTransmitBuffer != NULL, ;);

    for (; sTransmitLength > 0; sTransmitLength--)
    {

    }

    sTransmitBuffer = NULL;
    otPlatUartSendDone();

exit:
    return;
}

void samr21UartProcess(void)
{
    processReceive();
    processTransmit();
}

void UART0IntHandler(void)
{
    uint32_t mis;
    uint8_t byte;

}
