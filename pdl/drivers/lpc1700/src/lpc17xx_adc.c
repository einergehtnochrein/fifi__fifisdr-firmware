/* Copyright (c) 2011-2013, NXP Semiconductors
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

#include "lpc17xx_libconfig.h"

#include <string.h>

#include "lpclib_types.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_adc.h"


/** Field definition for hardware register ADxCR. */
LPCLIB_DefineRegBit(ADC_CR_SEL,                 0,  8);
LPCLIB_DefineRegBit(ADC_CR_CLKDIV,              8,  8);
LPCLIB_DefineRegBit(ADC_CR_BURST,               16, 1);
LPCLIB_DefineRegBit(ADC_CR_PDN,                 21, 1);
LPCLIB_DefineRegBit(ADC_CR_START,               24, 3);
LPCLIB_DefineRegBit(ADC_CR_EDGE,                27, 1);

/** Field definition for hardware register ADxGDR. */
LPCLIB_DefineRegBit(ADC_GDR_RESULT,             4,  12);
LPCLIB_DefineRegBit(ADC_GDR_CHN,                24, 3);
LPCLIB_DefineRegBit(ADC_GDR_OVERRUN,            30, 1);
LPCLIB_DefineRegBit(ADC_GDR_DONE,               31, 1);

/** Field definition for hardware register ADxINTEN. */
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN0,         0,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN1,         1,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN2,         2,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN3,         3,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN4,         4,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN5,         5,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN6,         6,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADINTEN7,         7,  1);
LPCLIB_DefineRegBit(ADC_INTEN_ADGINTEN,         8,  1);

/** Field definition for hardware register ADxDRy. */
LPCLIB_DefineRegBit(ADC_DR_RESULT,              4,  12);
LPCLIB_DefineRegBit(ADC_DR_OVERRUN,             30, 1);
LPCLIB_DefineRegBit(ADC_DR_DONE,                31, 1);

/** Field definition for hardware register ADxSTAT. */
LPCLIB_DefineRegBit(ADC_STAT_DONE0,             0,  1);
LPCLIB_DefineRegBit(ADC_STAT_DONE1,             1,  1);
LPCLIB_DefineRegBit(ADC_STAT_DONE2,             2,  1);
LPCLIB_DefineRegBit(ADC_STAT_DONE3,             3,  1);
LPCLIB_DefineRegBit(ADC_STAT_DONE4,             4,  1);
LPCLIB_DefineRegBit(ADC_STAT_DONE5,             5,  1);
LPCLIB_DefineRegBit(ADC_STAT_DONE6,             6,  1);
LPCLIB_DefineRegBit(ADC_STAT_DONE7,             7,  1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN0,          8,  1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN1,          9,  1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN2,          10, 1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN3,          11, 1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN4,          12, 1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN5,          13, 1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN6,          14, 1);
LPCLIB_DefineRegBit(ADC_STAT_OVERRUN7,          15, 1);
LPCLIB_DefineRegBit(ADC_STAT_ADINT,             16, 1);

/** Field definition for hardware register ADTRIM. */
LPCLIB_DefineRegBit(ADC_TRIM_ADCOFFS,           4,  4);
LPCLIB_DefineRegBit(ADC_TRIM_TRIM,              8,  4);



/** Local context ADC. */
static struct ADC_Context {
    ADC_Name block;                         /**< Identifier */
    ADC_StartMode startMode;                /**< Trigger source */
    LPCLIB_Callback callback;               /**< Event handler */
    uint8_t clockPercentage;                /**< ADC clock in percent of maximum */
} adcContext;



/** Set the ADC clock frequency.
 *
 *  This is not the sample rate, but the clock for the ADC machine, and is usually set to
 *  maximum. Applications may choose a lower frequency if they have high-impedance sources.
 *  This is because a lower clock frequency produces a longer sampling time.
 *
 *  \param[in] handle Device handle
 *  \param[in] clockPercentage Clock frequency in percent of maximum clock frequency.
 */
static void ADC_setClock (ADC_Handle handle, unsigned int clockPercentage)
{
    int divider_minus1;

    if (clockPercentage > 100) {
        clockPercentage = 100;
    }
    handle->clockPercentage = clockPercentage;

    /* (Integer) divider ratio */
    divider_minus1 = (CLKPWR_getBusClock(CLKPWR_CLOCK_ADC) - 1) / (clockPercentage * 130000ul);

    /* Set the maximum possible value (13 MHz) for the ADC clock. */
    LPC_ADC->CR = (LPC_ADC->CR & ~ADC_CR_CLKDIV_Msk) | (divider_minus1 << ADC_CR_CLKDIV_Pos);
}


/* Open access to the ADC. */
LPCLIB_Result ADC_open (ADC_Name block, ADC_Handle *pHandle)
{
    memset(&adcContext, 0, sizeof(adcContext));         /* Prepare ADC context */
    *pHandle = &adcContext;

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_ADC);

    (*pHandle)->block = block;

    /* Set the maximum possible value (100%) for the ADC clock. */
    ADC_setClock(*pHandle, 100);

    return LPCLIB_SUCCESS;
}



/* Close access to the ADC. */
LPCLIB_Result ADC_close (ADC_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_ADC);

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Configure the ADC block. */
LPCLIB_Result ADC_ioctl (ADC_Handle handle, const ADC_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    while (pConfig->opcode != ADC_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case ADC_OPCODE_SET_STARTMODE:
            handle->startMode = pConfig->startMode;
            LPC_ADC->CR = (LPC_ADC->CR & ~ADC_CR_START_Msk) | handle->startMode | ADC_CR_PDN_Msk;
            break;

        case ADC_OPCODE_SET_CHANNELMODE:
            if (pConfig->channel.active) {
                LPC_ADC->CR |= (1u << pConfig->channel.index);
                if (pConfig->channel.sendEvent) {
                    LPC_ADC->INTEN |= (1u << pConfig->channel.index);
                }
            }
            else {
                LPC_ADC->INTEN &= ~(1u << pConfig->channel.index);
                LPC_ADC->CR &= ~(1u << pConfig->channel.index);
            }
            break;

        case ADC_OPCODE_SET_CALLBACK:
            if (pConfig->callback.pOldCallback) {           /* Return current callback if requested */
                *(pConfig->callback.pOldCallback) = handle->callback;
            }
            handle->callback = pConfig->callback.callback;
            break;

        case ADC_OPCODE_SET_CLOCK:
            ADC_setClock(handle, pConfig->clockPercentage);
            break;

        default:
            return LPCLIB_ILLEGAL_PARAMETER;
        }

        ++pConfig;
    }

    return LPCLIB_SUCCESS;
}




void ADC_IRQHandler (void)
{
    LPCLIB_Event event;
    int i;
    uint32_t dr;

    dr = LPC_ADC->GDR;

    for (i = 0; i < 8; i++) {
        dr = LPC_ADC->DR[i];
        if (dr & (uint32_t)ADC_DR_DONE_Msk) {
            if (adcContext.callback) {
                event.id = LPCLIB_EVENTID_ADC;                  /* Prepare event */
                event.block = 0;
                event.channel = i;
                event.parameter = (void *)((dr & ADC_DR_RESULT_Msk) >> ADC_DR_RESULT_Pos);

                adcContext.callback(event);
            }
        }
    }

    if ((LPC_ADC->CR & ADC_CR_START_Msk) == ADC_START_NOW) {
        /* After a single conversion, reset the trigger, and power down the ADC */
        LPC_ADC->CR &= ~(ADC_CR_START_Msk | ADC_CR_PDN_Msk);
    }
}

