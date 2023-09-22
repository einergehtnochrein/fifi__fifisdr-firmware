/* $Id::                                                               $
 *
 * Copyright (c) 2011, NXP Semiconductors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 * Neither the name of the author nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 *  \brief IOCON driver implementation.
 *
 *  This file contains the driver code for the IOCON block.
 *
 *  \author NXP Semiconductors
 */

/** \addtogroup IOCON
 *  @{
 */

#include "lpc17xx_iocon.h"



LPCLIB_Result __IOCON_configError;


/** Configure a pin.
 */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
void IOCON_configurePin (IOCON_PinName pin, IOCON_PinConfig config)
{
    uint32_t offset;
    uint32_t index;
    uint32_t temp;

    if ((pin >> 28) != (config >> 28)) {    /* Type encoded into the upper 4 bits */
        __IOCON_configError = LPCLIB_ILLEGAL_PARAMETER;
        return;
    }

    index = ((int)pin & 0xFFFF) / 16;
    offset = 2 * ((int)pin % 16);

    temp = (((uint32_t)config >> 0) & 3) << offset;
    LPC_PINCON->PINSEL[index]     = (LPC_PINCON->PINSEL[index]     & ~(3u << offset)) | temp;
    temp = (((uint32_t)config >> 3) & 3) << offset;
    LPC_PINCON->PINMODE[index]    = (LPC_PINCON->PINMODE[index]    & ~(3u << offset)) | temp;
    temp = (((uint32_t)config >> 10) & 1) << (offset / 2);
    LPC_PINCON->PINMODE_OD[index/2] = (LPC_PINCON->PINMODE_OD[index/2] & ~(1u << offset)) | temp;
}
#endif

/** @} */

