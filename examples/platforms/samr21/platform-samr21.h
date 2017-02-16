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
 *   This file includes the platform-specific initializers.
 *
 */

#ifndef PLATFORM_SAMR21_H_
#define PLATFORM_SAMR21_H_

#include <stdint.h>
#include <openthread-types.h>

#ifndef __ATSAMR21G18A__
#define __ATSAMR21G18A__
#endif

#include <samr21.h>

// Global OpenThread instance structure
extern otInstance *sInstance;

/**
 * This function initializes the alarm service used by OpenThread.
 *
 */
void samr21AlarmInit(void);

/**
 * This function performs alarm driver processing.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
void samr21AlarmProcess(otInstance *aInstance);

/**
 * This function initializes the radio service used by OpenThread.
 *
 */
void samr21RadioInit(void);

/**
 * This function performs radio driver processing.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
void samr21RadioProcess(otInstance *aInstance);

/**
 * This function initializes the random number service used by OpenThread.
 *
 */
void randomInit(void);

/**
 * This function performs UART driver processing.
 *
 */
void samr21UartProcess(void);

#define GPIOPIN(n)              (((n)&0x1Fu) << 0)
#define GPIOPORT(n)             (((n) >> 5) & 0x7u)
#define GPIOPORTPIN(n)          ((n) & 0xFFu)
#define GPIOFUNC(n)             ((n) >> 8)
#define GPIOGROUP(port, pin)    ((((port)&0x7u) << 5) + ((pin)&0x1Fu))
#define GPIOPINFUNCTIONOFF      0xffffffff

enum FUNCS {FUNCA = 0, FUNCB, FUNCC, FUNCD, FUNCE, FUNCF, FUNCG, FUNCH};

enum gpio_port { GPIO_PORTA, GPIO_PORTB, GPIO_PORTC };

void setPinFunc(uint32_t portpin, enum FUNCS func);
int8_t findExternalInterruptMap(uint32_t portpin);

#define GPIO_DIR_IN             (0 << 0)
#define GPIO_DIR_OUT            (1 << 0)
#define GPIO_DIR_MASK           1

#define GPIO_INT                (1 << 1)
#define GPIO_INT_ACTIVE_LOW     (0 << 2)
#define GPIO_INT_ACTIVE_HIGH    (1 << 2)
#define GPIO_INT_CLOCK_SYNC     (1 << 3)
#define GPIO_INT_DEBOUNCE       (1 << 4)
#define GPIO_INT_LEVEL          (0 << 5)
#define GPIO_INT_EDGE           (1 << 5)
#define GPIO_INT_DOUBLE_EDGE    (1 << 6)

#define GPIO_POL_POS            7
#define GPIO_POL_NORMAL         (0 << GPIO_POL_POS)
#define GPIO_POL_INV            (1 << GPIO_POL_POS)
#define GPIO_POL_MASK           (1 << GPIO_POL_POS)

#define GPIO_PUD_POS            8
#define GPIO_PUD_NORMAL         (0 << GPIO_PUD_POS)
#define GPIO_PUD_PULL_UP        (1 << GPIO_PUD_POS)
#define GPIO_PUD_PULL_DOWN      (2 << GPIO_PUD_POS)
#define GPIO_PUD_MASK           (3 << GPIO_PUD_POS)

#define GPIO_PIN_ENABLE         (1 << 10)
#define GPIO_PIN_DISABLE        (1 << 11)

#define SPI_MODE_CPOL           0x1
#define SPI_MODE_CPHA           0x2
#define SPI_MODE_LOOP           0x4
#define SPI_MODE_MASK           0x7
#define SPI_MODE(_in_)          ((_in_) & SPI_MODE_MASK)
#define SPI_TRANSFER_MSB        (0 << 3)
#define SPI_TRANSFER_LSB        (1 << 3)

#endif  // PLATFORM_SAMR21_H_
