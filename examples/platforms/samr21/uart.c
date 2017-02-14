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

#define EDBG_COM_TX GPIOGROUP(GPIO_PORTA, 4)
#define EDBG_COM_RX GPIOGROUP(GPIO_PORTA, 5)

enum
{
    kPlatformClock = 48000000,
    kBaudRate = 115200,
    kReceiveBufferSize = 128,
};

extern void SERCOM0_Handler(void);

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

    /* calculate baudrate */
    uint32_t baud = (( (kPlatformClock * 10) / kBaudRate) / 16);

    SercomUsart *uart = (SercomUsart *)SERCOM0;

    /* enable sync and async clocks */
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM0_CORE);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* configure pins */
    setPinFunc(EDBG_COM_TX, FUNCD);
    setPinFunc(EDBG_COM_RX, FUNCD);

    /* reset the UART device */
    uart->CTRLA.reg = SERCOM_USART_CTRLA_SWRST;
    while (uart->SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_SWRST) { }

    /* set asynchronous mode w/o parity, LSB first, TX and RX pad as specified
     * by the board in the periph_conf.h, x16 sampling and use internal clock */
    uart->CTRLA.reg = (SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_SAMPR(0x1)
            | SERCOM_USART_CTRLA_TXPO(0)
            | SERCOM_USART_CTRLA_RXPO(1)
            | SERCOM_USART_CTRLA_MODE_USART_INT_CLK);

    /* set baudrate */
    uart->BAUD.FRAC.FP = (baud % 10);
    uart->BAUD.FRAC.BAUD = (baud / 10);

    /* disable all interrupts Except RX complete */
    uart->INTENCLR.reg = 0xff;
    uart->INTENSET.bit.RXC = 1;

    /* enable receiver and transmitter, use 1 stop bit */
    uart->CTRLB.reg = (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);
    while (uart->SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB)
    {
    }

    /* finally, enable the device */
    uart->CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;

    NVIC_EnableIRQ(SERCOM0_IRQn);

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

    SercomUsart *uart = (SercomUsart *)SERCOM0;

    for (; sTransmitLength > 0; sTransmitLength--)
    {
        while (!(uart->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE)) {}

        uart->DATA.reg = *sTransmitBuffer++;
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

void SERCOM0_Handler(void)
{
    uint32_t intf;
    uint8_t byte;

    SercomUsart *uart = (SercomUsart *)SERCOM0;

    intf = uart->INTFLAG.reg;
    uart->INTFLAG.reg = intf;

    while ( (uart->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) )
    {
        byte = (unsigned char)uart->DATA.reg;

        // We can only write if incrementing mTail doesn't equal mHead
        if (sReceive.mHead != (sReceive.mTail + 1) % kReceiveBufferSize)
        {
            sReceive.mBuffer[sReceive.mTail] = byte;
            sReceive.mTail = (sReceive.mTail + 1) % kReceiveBufferSize;
        }
    }
}
