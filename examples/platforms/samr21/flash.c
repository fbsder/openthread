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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <openthread-config.h>
#include <openthread/platform/alarm.h>
#include <utils/flash.h>

#include <common/code_utils.hpp>
#include "platform-samr21.h"

extern uint32_t __flash_data_start;
extern uint32_t __flash_data_end;

#define FLASH_START_ADDR    ((uint32_t)&__flash_data_start)
#define FLASH_END_ADDR      ((uint32_t)&__flash_data_end)
#define NVM_MEMORY        	((volatile uint16_t *)FLASH_ADDR)
#define NVM_ERRORS_MASK   	(NVMCTRL_STATUS_PROGE   | \
                           	   NVMCTRL_STATUS_LOCKE | \
							   NVMCTRL_STATUS_NVME)

static uint16_t page_size;
static uint16_t number_of_pages;
static bool manual_page_write = false;

static inline uint32_t mapAddress(uint32_t aAddress)
{
    return aAddress + FLASH_START_ADDR;
}

/**
 * Perform any initialization for flash driver.
 *
 * @retval ::kThreadError_None    Initialize flash driver success.
 * @retval ::kThreadError_Failed  Initialize flash driver fail.
 */
ThreadError utilsFlashInit(void)
{
    VerifyOrExit((FLASH_START_ADDR % NVMCTRL_PAGE_SIZE) == 0, ;);
    VerifyOrExit((FLASH_END_ADDR % NVMCTRL_PAGE_SIZE) == 0, ;);

    uint8_t wait_state;
    Nvmctrl * const nvm_module = NVMCTRL;

    PM->APBBMASK.reg |= PM_APBBMASK_NVMCTRL;

    /* Clear error flags */
    nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

    /* Check if the module is busy */
    if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    {
        return kThreadError_Failed;
    }

    wait_state = NVMCTRL->CTRLB.bit.RWS;

    nvm_module->CTRLB.reg = NVMCTRL_CTRLB_SLEEPPRM(NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val) | ((manual_page_write & 0x01) << NVMCTRL_CTRLB_MANW_Pos) | NVMCTRL_CTRLB_RWS(wait_state) | ((false & 0x01) << NVMCTRL_CTRLB_CACHEDIS_Pos) | NVMCTRL_CTRLB_READMODE(NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY_Val);

    page_size = (8 << nvm_module->PARAM.bit.PSZ);
    number_of_pages = nvm_module->PARAM.bit.NVMP;

    /* If the security bit is set, the auxiliary space cannot be written */
    if (nvm_module->STATUS.reg & NVMCTRL_STATUS_SB)
    {
        return kThreadError_Failed;
    }

    return kThreadError_None;

exit:
    return kThreadError_Failed;
}

/**
 * Get the size of flash that can be read/write by the caller.
 * The usable flash size is always the multiple of flash page size.
 *
 * @returns The size of the flash.
 */
uint32_t utilsFlashGetSize(void)
{
    return FLASH_END_ADDR - FLASH_START_ADDR;
}

/**
 * Erase one flash page that include the input address.
 * This is a non-blocking function. It can work with utilsFlashStatusWait to check when erase is done.
 *
 * The flash address starts from 0, and this function maps the input address to the physical address of flash for erasing.
 * 0 is always mapped to the beginning of one flash page.
 * The input address should never be mapped to the firmware space or any other protected flash space.
 *
 * @param[in]  aAddress  The start address of the flash to erase.
 *
 * @retval kThreadError_None           Erase flash operation is started.
 * @retval kThreadError_Failed         Erase flash operation is not started.
 * @retval kThreadError_InvalidArgs    aAddress is out of range of flash or not aligend.
 */
ThreadError utilsFlashErasePage(uint32_t aAddress)
{
    int32_t status;
    uint32_t address;
    uint32_t flash_offset;
    Nvmctrl * const nvm_module = NVMCTRL;
    ThreadError error = kThreadError_None;

    flash_offset = (aAddress) & ~(NVMCTRL_ROW_PAGES * page_size - 1);
    address = FLASH_START_ADDR + (aAddress - flash_offset);

    /* Check if the row address is valid */
    if (address > ((uint32_t) page_size * number_of_pages))
    {
        return kThreadError_InvalidArgs;
    }

    /* Check if the address to erase is not aligned to the start of a row */
    if (address & ((page_size * NVMCTRL_ROW_PAGES) - 1))
    {
        return kThreadError_Failed;
    }

    /* Check if the module is busy */
    if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    {
        return kThreadError_Failed;
    }

    /* Clear error flags */
    nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

    /* Set address and command */
    nvm_module->ADDR.reg = (uintptr_t) & NVM_MEMORY[address / 4];
    nvm_module->CTRLA.reg = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY;

    return kThreadError_None;
}

/**
  * Check whether flash is ready or busy.
  *
  * @param[in]  aTimeout  The interval in milliseconds waiting for the flash operation to be done and become ready again.
  *                       zero indicates that it is a polling function, and returns current status of flash immediately.
  *                       non-zero indicates that it is blocking there until the operation is done and become ready, or timeout expires.
  *
  * @retval kThreadError_None           Flash is ready for any operation.
  * @retval kThreadError_Busy           Flash is busy.
  */
ThreadError utilsFlashStatusWait(uint32_t aTimeout)
{
    ThreadError error = kThreadError_None;
    bool busy = true;
    uint32_t start = otPlatAlarmGetNow();
    Nvmctrl * const nvm_module = NVMCTRL;

    while (busy && ((otPlatAlarmGetNow() - start) < aTimeout))
    {
        busy = !(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY);
    }

    VerifyOrExit(!busy, error = kThreadError_Busy);

exit:
    return error;
}

/**
 * Write flash. The write operation only clears bits, but never set bits.
 *
 * The flash address starts from 0, and this function maps the input address to the physical address of flash for writing.
 * 0 is always mapped to the beginning of one flash page.
 * The input address should never be mapped to the firmware space or any other protected flash space.
 *
 * @param[in]  aAddress  The start address of the flash to write.
 * @param[in]  aData     The pointer of the data to write.
 * @param[in]  aSize     The size of the data to write.
 *
 * @returns The actual size of octets write to flash.
 *          It is expected the same as aSize, and may be less than aSize.
 *          0 indicates that something wrong happens when writing.
 */
uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t nvm_address;
    uint32_t ctrlb_bak;

    /* Get a pointer to the module hardware instance */
    Nvmctrl * const nvm_module = NVMCTRL;

    if (aAddress > ((uint32_t) page_size * number_of_pages))
    {
        return 0;
    }

    /* Check if the write address not aligned to the start of a page */
    if (aAddress & (page_size - 1))
    {
        return 0;
    }

    /* Check if the write length is longer than an NVM page */
    if (aSize > page_size)
    {
        return 0;
    }

    if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    {
        return 0;
    }

    /* Erase the page buffer before buffering new data */
    nvm_module->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY;

    while (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    {
    }

    /* Clear error flags */
    nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

    nvm_address = aAddress / 2;

    /* NVM _must_ be accessed as a series of 16-bit words, perform manual copy
     * to ensure alignment */
    for (uint16_t i = 0; i < aSize; i += 2)
    {
        uint16_t data;

        /* Copy first byte of the 16-bit chunk to the temporary buffer */
        data = aData[i];

        /* If we are not at the end of a write request with an odd byte count,
         * store the next byte of data as well */
        if (i < (aSize - 1))
        {
            data |= (aData[i + 1] << 8);
        }

        /* Store next 16-bit chunk to the NVM memory space */
        NVM_MEMORY[nvm_address++] = data;
    }

    /* If automatic page write mode is enable, then perform a manual NVM
     * write when the length of data to be programmed is less than page size
     */
    if ((manual_page_write == false) && (aSize < NVMCTRL_PAGE_SIZE))
    {
        /* Turn off cache before issuing flash commands */
        ctrlb_bak = nvm_module->CTRLB.reg;
        nvm_module->CTRLB.reg = ctrlb_bak | NVMCTRL_CTRLB_CACHEDIS;

        /* Clear error flags */
        nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

        if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
        {
            /* Restore the setting */
            nvm_module->CTRLB.reg = ctrlb_bak;
            return 0;
        }

        /* Set address and command */
        nvm_module->ADDR.reg = (uintptr_t) & NVM_MEMORY[aAddress / 4];
        nvm_module->CTRLA.reg = NVMCTRL_CTRLA_CMD_WP | NVMCTRL_CTRLA_CMDEX_KEY;

        while (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
        {
        }

        /* Restore the setting */
        nvm_module->CTRLB.reg = ctrlb_bak;
    }

    return aSize;
}

/**
 * Read flash.
 *
 * The flash address starts from 0, and this function maps the input address to the physical address of flash for reading.
 * 0 is always mapped to the beginning of one flash page.
 * The input address should never be mapped to the firmware space or any other protected flash space.
 *
 * @param[in]   aAddress  The start address of the flash to read.
 * @param[Out]  aData     The pointer of buffer for reading.
 * @param[in]   aSize     The size of the data to read.
 *
 * @returns The actual size of octets read to buffer.
 *          It is expected the same as aSize, and may be less than aSize.
 *          0 indicates that something wrong happens when reading.
 */
uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t result = 0;
    VerifyOrExit(aData, ;);
    VerifyOrExit(aSize < utilsFlashGetSize(), ;);

    memcpy(aData, (uint8_t *) mapAddress(aAddress), aSize);

    result = aSize;

exit:
    return result;
}
