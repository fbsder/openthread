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
#include <unistd.h>
#include <fcntl.h>

#include <openthread-config.h>
#include <platform/alarm.h>
#include <utils/flash.h>

#include <common/code_utils.hpp>
#include "platform-samr21.h"

extern uint32_t __flash_data_start;
extern uint32_t __flash_data_end;

#define FLASH_START_ADDR     ((uint32_t)&__flash_data_start)
#define FLASH_END_ADDR       ((uint32_t)&__flash_data_end)
#define NVM_MEMORY        ((volatile uint16_t *)FLASH_ADDR)
#define NVM_ERRORS_MASK   (NVMCTRL_STATUS_PROGE | \
                           NVMCTRL_STATUS_LOCKE | \
                           NVMCTRL_STATUS_NVME)

static uint8_t page_buffer[NVMCTRL_PAGE_SIZE];
static uint16_t page_size;
static uint16_t number_of_pages;
static bool manual_page_write = false;

static inline uint32_t mapAddress(uint32_t aAddress)
{
    return aAddress + FLASH_START_ADDR;
}

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

    nvm_module->CTRLB.reg = NVMCTRL_CTRLB_SLEEPPRM(NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val) | ((false & 0x01) << NVMCTRL_CTRLB_MANW_Pos) | NVMCTRL_CTRLB_RWS(wait_state) | ((false & 0x01) << NVMCTRL_CTRLB_CACHEDIS_Pos) | NVMCTRL_CTRLB_READMODE(NVMCTRL_CTRLB_READMODE_NO_MISS_PENALTY_Val);

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

uint32_t utilsFlashGetSize(void)
{
    return FLASH_END_ADDR - FLASH_START_ADDR;
}

ThreadError utilsFlashErasePage(uint32_t aAddress)
{
    int32_t status;
    uint32_t address;
    uint32_t flash_offset;
    Nvmctrl * const nvm_module = NVMCTRL;
    ThreadError error = kThreadError_None;

    flash_offset = (aAddress) & ~(NVMCTRL_ROW_PAGES * page_size - 1);
    address = FLASH_START_ADDR + aAddress - flash_offset;

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

    if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    {
        return kThreadError_Failed;
    }

    /* There existed error in NVM erase operation */
    if ((nvm_module->STATUS.reg & NVM_ERRORS_MASK) != 0)
    {
        return kThreadError_Failed;
    }

    return kThreadError_None;
}

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

uint32_t utilsFlashWrite(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    int32_t status;
    uint32_t busy = 1;
    uint32_t size = 0;
    uint32_t nvm_address;
    uint32_t ctrlb_bak;

    /* Get a pointer to the module hardware instance */
    Nvmctrl * const nvm_module = NVMCTRL;

    if (aAddress > ((uint32_t) page_size * number_of_pages))
    {
        return kThreadError_InvalidArgs;
    }

    /* Check if the write address not aligned to the start of a page */
    if (aAddress & (page_size - 1))
    {
        return kThreadError_InvalidArgs;
    }

    /* Check if the write length is longer than an NVM page */
    if (aSize > page_size)
    {
        return kThreadError_InvalidArgs;
    }

    if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    {
        return kThreadError_Failed;
    }

    /* Erase the page buffer before buffering new data */
    nvm_module->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY;

    if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
    {
        return kThreadError_Failed;
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
        data = page_buffer[i];

        /* If we are not at the end of a write request with an odd byte count,
         * store the next byte of data as well */
        if (i < (aSize - 1))
        {
            data |= (page_buffer[i + 1] << 8);
        }

        /* Store next 16-bit chunk to the NVM memory space */
        NVM_MEMORY[nvm_address++] = data;
    }

    /* If automatic page write mode is enable, then perform a manual NVM
     * write when the length of data to be programmed is less than page size
     */
    if ((manual_page_write == false) && (aSize < NVMCTRL_PAGE_SIZE))
    {
        /* Check that the address given is valid  */
        if (aAddress > ((uint32_t) page_size * number_of_pages) && !(aAddress >= NVMCTRL_AUX0_ADDRESS && aAddress <= NVMCTRL_AUX1_ADDRESS))
        {
            return kThreadError_InvalidArgs;
        }

        /* Turn off cache before issuing flash commands */
        ctrlb_bak = nvm_module->CTRLB.reg;
        nvm_module->CTRLB.reg = ctrlb_bak | NVMCTRL_CTRLB_CACHEDIS;

        /* Clear error flags */
        nvm_module->STATUS.reg = NVMCTRL_STATUS_MASK;

        if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
        {
            /* Restore the setting */
            nvm_module->CTRLB.reg = ctrlb_bak;
            return kThreadError_Failed;
        }

        switch (NVMCTRL_CTRLA_CMD_WP)
        {

        /* Commands requiring address (protected) */
        case NVMCTRL_CTRLA_CMD_EAR:
        case NVMCTRL_CTRLA_CMD_WAP:

            /* Auxiliary space cannot be accessed if the security bit is set */
            if (nvm_module->STATUS.reg & NVMCTRL_STATUS_SB)
            {
                /* Restore the setting */
                nvm_module->CTRLB.reg = ctrlb_bak;
                return kThreadError_Failed;
            }

            /* Set address, command will be issued elsewhere */
            nvm_module->ADDR.reg = (uintptr_t) & NVM_MEMORY[aAddress / 4];
            break;

            /* Commands requiring address (unprotected) */
        case NVMCTRL_CTRLA_CMD_ER:
        case NVMCTRL_CTRLA_CMD_WP:
        case NVMCTRL_CTRLA_CMD_LR:
        case NVMCTRL_CTRLA_CMD_UR:
            /* Set address, command will be issued elsewhere */
            nvm_module->ADDR.reg = (uintptr_t) & NVM_MEMORY[aAddress / 4];
            break;

            /* Commands not requiring address */
        case NVMCTRL_CTRLA_CMD_PBC:
        case NVMCTRL_CTRLA_CMD_SSB:
        case NVMCTRL_CTRLA_CMD_SPRM:
        case NVMCTRL_CTRLA_CMD_CPRM:
            break;

        default:
            /* Restore the setting */
            nvm_module->CTRLB.reg = ctrlb_bak;
            return kThreadError_InvalidArgs;
        }

        /* Set command */
        nvm_module->CTRLA.reg = NVMCTRL_CTRLA_CMD_WP | NVMCTRL_CTRLA_CMDEX_KEY;

        if (!(nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY))
        {
        }

        /* Restore the setting */
        nvm_module->CTRLB.reg = ctrlb_bak;
    }

    return aSize;
}

uint32_t utilsFlashRead(uint32_t aAddress, uint8_t *aData, uint32_t aSize)
{
    uint32_t result = 0;
    uint8_t *from = (uint8_t *) aAddress;
    VerifyOrExit(aData, ;);
    VerifyOrExit(aAddress < utilsFlashGetSize(), ;);

    //memcpy(aData, (uint8_t *) mapAddress(aAddress), aSize);
    for (int i = 0; i < aSize; i++)
    {
        aData[i] = from[i];
    }
    result = aSize;

exit:
    return result;
}
