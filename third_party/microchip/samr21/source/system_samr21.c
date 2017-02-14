/**
 * \file
 *
 * \brief Low-level initialization functions called upon chip startup.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include "samr21.h"

/**
 * Initial system clock frequency. The System RC Oscillator (RCSYS) provides
 *  the source for the main clock at chip startup.
 */
#define __SOC_CLOCK_USE_PLL
#define __SYSTEM_CLOCK    (48000000)

uint32_t SystemCoreClock = __SYSTEM_CLOCK;/*!< System Clock Frequency (Core Clock)*/

#define CLOCK_PLL_MUL       (47U)               /* must be >= 31 & <= 95 */
#define CLOCK_PLL_DIV       (1U)                /* adjust to your needs */

/**
 * Initialize the system
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
void SystemInit(void)
{
#if __SYSTEM_CLOCK > 24000000
    /*
     * adjust NVM wait states, see table 42.30 (p. 1070) in the datasheet
     */
    /* PM->APBAMASK.reg |= PM_AHBMASK_NVMCTRL; */
    NVMCTRL->CTRLB.bit.RWS = 1;
    /* PM->APBAMASK.reg &= ~PM_AHBMASK_NVMCTRL; */
#endif
    /* Disable watchdog timer, not used by system */
    WDT->CTRL.bit.ENABLE = 0;

    SystemCoreClockUpdate();

    return;
}

/**
 * Update SystemCoreClock variable
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void)
{
    /* Setup the SoC clocks according to section 28.12 in datasheet.
     *
     * Assumption:
     * SLCK = 32.768kHz
     */
    /* enable clocks for the power, sysctrl and gclk modules */
    PM->APBAMASK.reg = (PM_APBAMASK_PM | PM_APBAMASK_SYSCTRL | PM_APBAMASK_GCLK);

    /* configure internal 8MHz oscillator to run without prescaler */
    SYSCTRL->OSC8M.bit.PRESC = 0;
    SYSCTRL->OSC8M.bit.ONDEMAND = 1;
    SYSCTRL->OSC8M.bit.RUNSTDBY = 0;
    SYSCTRL->OSC8M.bit.ENABLE = 1;
    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC8MRDY)) {}

#ifdef __SOC_CLOCK_USE_PLL
    /* reset the GCLK module so it is in a known state */
    GCLK->CTRL.reg = GCLK_CTRL_SWRST;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* setup generic clock 1 to feed DPLL with 1MHz */
    GCLK->GENDIV.reg = (GCLK_GENDIV_DIV(8)              |
                        GCLK_GENDIV_ID(1));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN             |
                         GCLK_GENCTRL_SRC_OSC8M         |
                         GCLK_GENCTRL_ID(1));
    GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_GEN(1)            |
                         GCLK_CLKCTRL_ID(1)             |
                         GCLK_CLKCTRL_CLKEN);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* enable PLL */
    SYSCTRL->DPLLRATIO.reg = (SYSCTRL_DPLLRATIO_LDR(CLOCK_PLL_MUL));
    SYSCTRL->DPLLCTRLB.reg = (SYSCTRL_DPLLCTRLB_REFCLK_GCLK);
    SYSCTRL->DPLLCTRLA.reg = (SYSCTRL_DPLLCTRLA_ENABLE);
    while(! (SYSCTRL->DPLLSTATUS.reg & (SYSCTRL_DPLLSTATUS_CLKRDY | SYSCTRL_DPLLSTATUS_LOCK) ) ) {}

    /* select the PLL as source for clock generator 0 (CPU core clock) */
    GCLK->GENDIV.reg =  (GCLK_GENDIV_DIV(CLOCK_PLL_DIV) |
                        GCLK_GENDIV_ID(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN             |
                         GCLK_GENCTRL_SRC_FDPLL         |
                         GCLK_GENCTRL_ID(0));
#else /* do not use PLL, use internal 8MHz oscillator directly */
    GCLK->GENDIV.reg =  (GCLK_GENDIV_DIV(CLOCK_DIV)     |
                        GCLK_GENDIV_ID(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN             |
                         GCLK_GENCTRL_SRC_OSC8M         |
                         GCLK_GENCTRL_ID(0));
#endif

    /* make sure we synchronize clock generator 0 before we go on */
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) {}

    /* Setup Clock generator 2 with divider 1 (32.768kHz) */
    GCLK->GENDIV.reg  = (GCLK_GENDIV_ID(2)              |
                         GCLK_GENDIV_DIV(0));
    GCLK->GENCTRL.reg = (GCLK_GENCTRL_ID(2)             |
                         GCLK_GENCTRL_GENEN             |
                      GCLK_GENCTRL_RUNSTDBY             |
                 GCLK_GENCTRL_SRC_OSCULP32K);

    while (GCLK->STATUS.bit.SYNCBUSY) {}

    /* redirect all peripherals to a disabled clock generator (7) by default */
    for (int i = 0x3; i <= 0x22; i++)
    {
        GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_ID(i) | GCLK_CLKCTRL_GEN_GCLK7 );
        while (GCLK->STATUS.bit.SYNCBUSY) {}
    }

    return;
}
