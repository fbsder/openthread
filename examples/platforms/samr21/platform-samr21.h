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

#include <samr21.h>

#define IRQ_EIC         4   /**<  4 SAMR21G18A External Interrupt Controller (EIC) */
#define IRQ_DMAC        6   /**<  6 SAMR21G18A Direct Memory Access Controller (DMAC) */
#define IRQ_EVSYS       8   /**<  8 SAMR21G18A Event System Interface (EVSYS) */
#define IRQ_SERCOM0     9   /**<  9 SAMR21G18A Serial Communication Interface 0 (SERCOM0) */
#define IRQ_SERCOM1     10  /**< 10 SAMR21G18A Serial Communication Interface 1 (SERCOM1) */
#define IRQ_SERCOM2     11  /**< 11 SAMR21G18A Serial Communication Interface 2 (SERCOM2) */
#define IRQ_SERCOM3     12  /**< 12 SAMR21G18A Serial Communication Interface 3 (SERCOM3) */
#define IRQ_SERCOM4     13  /**< 13 SAMR21G18A Serial Communication Interface 4 (SERCOM4) */
#define IRQ_SERCOM5     14  /**< 14 SAMR21G18A Serial Communication Interface 5 (SERCOM5) */

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
void samr21RandomInit(void);

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

void set_pin_func(uint32_t portpin, enum FUNCS func);

#endif  // PLATFORM_SAMR21_H_
