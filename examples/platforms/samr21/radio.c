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
 *   This file implements the OpenThread platform abstraction for radio communication.
 *
 */

#include <openthread-types.h>
#include <openthread.h>
#include <openthread-config.h>

#include <common/code_utils.hpp>
#include <platform/platform.h>
#include <common/logging.hpp>
#include <platform/radio.h>
#include <platform/diag.h>
#include "platform-samr21.h"

enum
{
    IEEE802154_MIN_LENGTH = 5,
    IEEE802154_MAX_LENGTH = 127,
    IEEE802154_ACK_LENGTH = 5,
    IEEE802154_FRAME_TYPE_MASK = 0x7,
    IEEE802154_FRAME_TYPE_ACK = 0x2,
    IEEE802154_FRAME_PENDING = 1 << 4,
    IEEE802154_ACK_REQUEST = 1 << 5,
    IEEE802154_DSN_OFFSET = 2,
};

enum
{
    samr21_RSSI_OFFSET = 73,
    samr21_CRC_BIT_MASK = 0x80,
    samr21_LQI_BIT_MASK = 0x7f,
};

static RadioPacket sTransmitFrame;
static RadioPacket sReceiveFrame;
static ThreadError sTransmitError;
static ThreadError sReceiveError;

static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];
static uint8_t sChannel = 0;

static PhyState sState = kStateDisabled;
static bool sIsReceiverEnabled = false;

void enableReceiver(void)
{

}

void disableReceiver(void)
{

}

void setChannel(uint8_t channel)
{

}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{

}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{

}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *address)
{

}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t address)
{

}

void samr21RadioInit(void)
{
    sTransmitFrame.mLength = 0;
    sTransmitFrame.mPsdu = sTransmitPsdu;
    sReceiveFrame.mLength = 0;
    sReceiveFrame.mPsdu = sReceivePsdu;

    otLogInfoPlat("Initialized", NULL);
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;
    return (sState != kStateDisabled) ? true : false;
}

ThreadError otPlatRadioEnable(otInstance *aInstance)
{
    if (!otPlatRadioIsEnabled(aInstance))
    {
        otLogDebgPlat("State=kStateSleep", NULL);
        sState = kStateSleep;
    }

    return kThreadError_None;
}

ThreadError otPlatRadioDisable(otInstance *aInstance)
{
    if (otPlatRadioIsEnabled(aInstance))
    {
        otLogDebgPlat("State=kStateDisabled", NULL);
        sState = kStateDisabled;
    }

    return kThreadError_None;
}

ThreadError otPlatRadioSleep(otInstance *aInstance)
{
    ThreadError error = kThreadError_InvalidState;
    (void)aInstance;

    if (sState == kStateSleep || sState == kStateReceive)
    {
        otLogDebgPlat("State=kStateSleep", NULL);
        error = kThreadError_None;
        sState = kStateSleep;
        disableReceiver();
    }

    return error;
}

ThreadError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    ThreadError error = kThreadError_InvalidState;
    (void)aInstance;

    if (sState != kStateDisabled)
    {
        otLogDebgPlat("State=kStateReceive", NULL);

        error = kThreadError_None;
        sState = kStateReceive;
        setChannel(aChannel);
        sReceiveFrame.mChannel = aChannel;
        enableReceiver();
    }

    return error;
}

ThreadError otPlatRadioTransmit(otInstance *aInstance, RadioPacket *aPacket)
{
	return kThreadError_None;
}

RadioPacket *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    (void)aInstance;
    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    (void)aInstance;
    return 0;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    (void)aInstance;
    return kRadioCapsNone;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void)aInstance;

    return false;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;

    otLogInfoPlat("PromiscuousMode=%d", aEnable ? 1 : 0);

}

void readFrame(void)
{
    uint8_t length;
    uint8_t crcCorr;
    int i;

    VerifyOrExit(sState == kStateReceive || sState == kStateTransmit, ;);

    // read length
exit:
    return;
}

void samr21RadioProcess(otInstance *aInstance)
{
    readFrame();

    if ((sState == kStateReceive && sReceiveFrame.mLength > 0) ||
        (sState == kStateTransmit && sReceiveFrame.mLength > IEEE802154_ACK_LENGTH))
    {
#if OPENTHREAD_ENABLE_DIAG

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioReceiveDone(aInstance, &sReceiveFrame, sReceiveError);
        }
        else
#endif
    }

    if (sState == kStateTransmit)
    {
        if (sTransmitError != kThreadError_None || (sTransmitFrame.mPsdu[0] & IEEE802154_ACK_REQUEST) == 0)
        {
            if (sTransmitError != kThreadError_None)
            {
                otLogDebgPlat("Transmit failed ErrorCode=%d", sTransmitError);
            }

            sState = kStateReceive;

#if OPENTHREAD_ENABLE_DIAG

            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, false, sTransmitError);
            }
            else
#endif
            {
                otPlatRadioTransmitDone(aInstance, &sTransmitFrame, false, sTransmitError);
            }
        }
        else if (sReceiveFrame.mLength == IEEE802154_ACK_LENGTH &&
                 (sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_ACK &&
                 (sReceiveFrame.mPsdu[IEEE802154_DSN_OFFSET] == sTransmitFrame.mPsdu[IEEE802154_DSN_OFFSET]))
        {
            sState = kStateReceive;

#if OPENTHREAD_ENABLE_DIAG

            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, (sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_PENDING) != 0,
                                            sTransmitError);
            }
            else
#endif
            {
                otPlatRadioTransmitDone(aInstance, &sTransmitFrame, (sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_PENDING) != 0,
                                        sTransmitError);
            }
        }
    }

    sReceiveFrame.mLength = 0;
}

void RFCoreRxTxIntHandler(void)
{

}

void RFCoreErrIntHandler(void)
{

}

uint32_t getSrcMatchEntriesEnableStatus(bool aShort)
{
    uint32_t status = 0;

    return status;
}

int8_t findSrcMatchShortEntry(const uint16_t aShortAddress)
{
	return 0;
}

int8_t findSrcMatchExtEntry(const uint8_t *aExtAddress)
{
	return 0;
}

void setSrcMatchEntryEnableStatus(bool aShort, uint8_t aEntry, bool aEnable)
{

}

int8_t findSrcMatchAvailEntry(bool aShort)
{
	return 0;
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{

}

ThreadError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
	return kThreadError_None;
}

ThreadError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
	return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
	return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
	return kThreadError_None;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{

}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{

}

ThreadError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    (void)aInstance;
    (void)aScanChannel;
    (void)aScanDuration;
    return kThreadError_NotImplemented;
}
