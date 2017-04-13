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
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <openthread/openthread.h>
#include <openthread-core-config.h>
#include <openthread-config.h>
#include <openthread/types.h>

#include <common/code_utils.hpp>
#include <common/logging.hpp>
#include <openthread/platform/alarm.h>
#include <openthread/platform/radio.h>
#include <openthread/platform/diag.h>

#include "platform-kw41z.h"


enum {LOW = 0, HIGH = 1};

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

static RadioPacket sTransmitFrame;
static RadioPacket sReceiveFrame;
static ThreadError sTransmitError;

static uint8_t sTransmitPsdu[IEEE802154_MAX_LENGTH];
static uint8_t sReceivePsdu[IEEE802154_MAX_LENGTH];
static uint8_t cmd_buf[12];

static uint8_t mac_addr[8] = {
    0xFC, 0xC2, 0x3D,       /* factory eui , see http://standards-oui.ieee.org/oui/oui.txt */
    0x00,                   /* board type */
    ((HARDCODED_NODE_ID >> 24) & 0xff), ((HARDCODED_NODE_ID >> 16) & 0xff),
    ((HARDCODED_NODE_ID >>  8) & 0xff), (HARDCODED_NODE_ID        & 0xff),
};

static PhyState sState = kStateDisabled;

static inline void set_reset(uint32_t value)
{

}

static void enableReceiver(void)
{

}

static void disableReceiver(void)
{

}

static void setChannel(uint8_t channel)
{

}

/**
 * Get the factory-assigned IEEE EUI-64 for this interface.
 *
 * @param[in]  aInstance   The OpenThread instance structure.
 * @param[out] aIeeeEui64  A pointer to where the factory-assigned IEEE EUI-64 will be placed.
 *
 */
void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    (void)aInstance;

    memcpy(aIeeeEui64, mac_addr, sizeof(mac_addr) / sizeof(mac_addr[0]));
}

/**
 * Set the PAN ID for address filtering.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 * @param[in] aPanId     The IEEE 802.15.4 PAN ID.
 *
 */
void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
    (void)aInstance;

    uint8_t *p = (uint8_t *)&panid;

}

/**
 * Set the Extended Address for address filtering.
 *
 * @param[in] aInstance         The OpenThread instance structure.
 * @param[in] aExtendedAddress  A pointer to the IEEE 802.15.4 Extended Address.
 *
 */
void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *address)
{
    (void)aInstance;

    for (int i = 0; i < 8; i++)
    {

    }
}

/**
 * Set the Short Address for address filtering.
 *
 * @param[in] aInstance      The OpenThread instance structure.
 * @param[in] aShortAddress  The IEEE 802.15.4 Short Address.
 *
 */
void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t address)
{
    (void)aInstance;

    uint8_t *p = (uint8_t *)&address;

}

void kw41zRadioInit(void)
{


    otLogInfoPlat("Initialized", NULL);
}

/**
 * Enable the radio.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @retval ::kThreadError_None     Successfully enabled.
 * @retval ::kThreadError_Failure  The radio could not be enabled.
 */
ThreadError otPlatRadioEnable(otInstance *aInstance)
{

    return kThreadError_None;
}

/**
 * Disable the radio.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @retval ::kThreadError_None  Successfully transitioned to Disabled.
 */
ThreadError otPlatRadioDisable(otInstance *aInstance)
{

    return kThreadError_None;
}

/**
 * Check whether radio is enabled or not.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @retval ::true   radio is enabled.
 * @retval ::false  radio is disabled.
 */
bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void) aInstance;
    return (sState != kStateDisabled) ? true : false;
}

/**
 * Transition the radio from Receive to Sleep.
 * Turn off the radio.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @retval ::kThreadError_None         Successfully transitioned to Sleep.
 * @retval ::kThreadError_Busy         The radio was transmitting
 * @retval ::kThreadError_InvalidState The radio was disabled
 */
ThreadError otPlatRadioSleep(otInstance *aInstance)
{
    return error;
}

/**
 * Transitioning the radio from Sleep to Receive.
 * Turn on the radio.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 * @param[in]  aChannel   The channel to use for receiving.
 *
 * @retval ::kThreadError_None         Successfully transitioned to Receive.
 * @retval ::kThreadError_InvalidState The radio was disabled or transmitting.
 */
ThreadError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    return error;
}

/**
 * Enable/Disable source match for AutoPend.
 *
 * @param[in]  aInstance   The OpenThread instance structure.
 * @param[in]  aEnable     Enable/disable source match for automatical pending.
 */
void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{

}

/**
 * Adding short address to the source match table.
 *
 * @param[in]  aInstance      The OpenThread instance structure.
 * @param[in]  aShortAddress  The short address to be added.
 *
 * @retval ::kThreadError_None     Successfully added short address to the source match table.
 * @retval ::kThreadError_NoBufs   No available entry in the source match table.
 */
ThreadError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    return kThreadError_None;
}

/**
 * Adding extended address to the source match table.
 *
 * @param[in]  aInstance    The OpenThread instance structure.
 * @param[in]  aExtAddress  The extended address to be added.
 *
 * @retval ::kThreadError_None     Successfully added extended address to the source match table.
 * @retval ::kThreadError_NoBufs   No available entry in the source match table.
 */
ThreadError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    return kThreadError_None;
}

/**
 * Removing short address to the source match table.
 *
 * @param[in]  aInstance      The OpenThread instance structure.
 * @param[in]  aShortAddress  The short address to be removed.
 *
 * @retval ::kThreadError_None        Successfully removed short address from the source match table.
 * @retval ::kThreadError_NoAddress   The short address is not in source match table.
 */
ThreadError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    return kThreadError_None;
}

/**
 * Removing extended address to the source match table of the radio.
 *
 * @param[in]  aInstance    The OpenThread instance structure.
 * @param[in]  aExtAddress  The extended address to be removed.
 *
 * @retval ::kThreadError_None        Successfully removed the extended address from the source match table.
 * @retval ::kThreadError_NoAddress   The extended address is not in source match table.
 */
ThreadError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    return kThreadError_None;
}

/**
 * Removing all the short addresses from the source match table.
 *
 * @param[in]  aInstance   The OpenThread instance structure.
 *
 */
void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{

}

/**
 * Removing all the extended addresses from the source match table.
 *
 * @param[in]  aInstance   The OpenThread instance structure.
 *
 */
void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{

}

/**
 * The radio transitions from Transmit to Receive.
 * This method returns a pointer to the transmit buffer.
 *
 * The caller forms the IEEE 802.15.4 frame in this buffer then calls otPlatRadioTransmit() to request transmission.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @returns A pointer to the transmit buffer.
 *
 */
RadioPacket *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    (void) aInstance;
    return &sTransmitFrame;
}

/**
 * This method begins the transmit sequence on the radio.
 *
 * The caller must form the IEEE 802.15.4 frame in the buffer provided by otPlatRadioGetTransmitBuffer() before
 * requesting transmission.  The channel and transmit power are also included in the RadioPacket structure.
 *
 * The transmit sequence consists of:
 * 1. Transitioning the radio to Transmit from Receive.
 * 2. Transmits the psdu on the given channel and at the given transmit power.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 * @param[in] aPacket    A pointer to the packet that will be transmitted.
 *
 * @retval ::kThreadError_None         Successfully transitioned to Transmit.
 * @retval ::kThreadError_InvalidState The radio was not in the Receive state.
 */
ThreadError otPlatRadioTransmit(otInstance *aInstance, RadioPacket *aPacket)
{
    return kThreadError_None;
}

/**
 * Get the most recent RSSI measurement.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @returns The RSSI in dBm when it is valid.  127 when RSSI is invalid.
 */
int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    (void) aInstance;
    return 0;
}

/**
 * Get the radio capabilities.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @returns The radio capability bit vector. The stack enables or disables some functions based on this value.
 */
otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    (void) aInstance;
    return kRadioCapsNone;
}

/**
 * Get the status of promiscuous mode.
 *
 * @param[in] aInstance  The OpenThread instance structure.
 *
 * @retval true   Promiscuous mode is enabled.
 * @retval false  Promiscuous mode is disabled.
 */
bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void) aInstance;

    return false;
}

/**
 * Enable or disable promiscuous mode.
 *
 * @param[in]  aInstance The OpenThread instance structure.
 * @param[in]  aEnable   A value to enable or disable promiscuous mode.
 */
void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void) aInstance;

    otLogInfoPlat("PromiscuousMode=%d", aEnable ? 1 : 0);

}

/**
 * This method begins the energy scan sequence on the radio.
 *
 * @param[in] aInstance      The OpenThread instance structure.
 * @param[in] aScanChannel   The channel to perform the energy scan on.
 * @param[in] aScanDuration  The duration, in milliseconds, for the channel to be scanned.
 *
 * @retval ::kThreadError_None            Successfully started scanning the channel.
 * @retval ::kThreadError_NotImplemented  The radio doesn't support energy scanning.
 */
ThreadError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    (void) aInstance;
    (void) aScanChannel;
    (void) aScanDuration;
    return kThreadError_NotImplemented;
}

static void readFrame(void)
{
    uint8_t length;
    uint8_t crcCorr;
    int i;

    VerifyOrExit(sState == kStateReceive || sState == kStateTransmit, ;);

    // read length
    exit: return;
}

void kw41zRadioProcess(otInstance *aInstance)
{
    readFrame();

    if ((sState == kStateReceive && sReceiveFrame.mLength > 0) || (sState == kStateTransmit && sReceiveFrame.mLength > IEEE802154_ACK_LENGTH))
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
        else if (sReceiveFrame.mLength == IEEE802154_ACK_LENGTH && (sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_ACK && (sReceiveFrame.mPsdu[IEEE802154_DSN_OFFSET] == sTransmitFrame.mPsdu[IEEE802154_DSN_OFFSET]))
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
                otPlatRadioTransmitDone(aInstance, &sTransmitFrame, (sReceiveFrame.mPsdu[0] & IEEE802154_FRAME_PENDING) != 0, sTransmitError);
            }
        }
    }

    sReceiveFrame.mLength = 0;
}

