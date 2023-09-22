/* Copyright (c) 2012-2013, NXP Semiconductors
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
#include "lpc17xx_dma.h"
#include "lpc17xx_dac.h"


/** Field definition for hardware register DACCR. */
LPCLIB_DefineRegBit(DAC_CR_VALUE,               6,  10);
LPCLIB_DefineRegBit(DAC_CR_BIAS,                16, 1);

/** Field definition for hardware register DACCTRL. */
LPCLIB_DefineRegBit(DAC_CTRL_INT_DMA_REQ,       0,  1);
LPCLIB_DefineRegBit(DAC_CTRL_DBLBUF_ENA,        1,  1);
LPCLIB_DefineRegBit(DAC_CTRL_CNT_ENA,           2,  1);
LPCLIB_DefineRegBit(DAC_CTRL_DMA_ENA,           3,  1);

/** Field definition for hardware register DACCNTVAL. */
LPCLIB_DefineRegBit(DAC_CNTVAL_VALUE,           0,  16);



/** Local DAC context. */
static struct DAC_Context {
    uint32_t sampleRateHz;                  /**< Sample rate (timer mode, or 0 if manual) */
    LPCLIB_Callback callback;               /**< Event handler */
    uint32_t speed;                         /**< 400 kHz or 1 MHz */
#if LPCLIB_DMA
    DMA_ChannelHandle dmaChannel;
    DMA_Job *pDmaJob;
#endif
} dacContext;



/** Set the DAC sample rate.
 *
 *  \param[in] handle Device handle
 *  \param[in] sampleRateHz Desired sample rate
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Sample rate out of range
 */
static LPCLIB_Result _DAC_setSampleRate (DAC_Handle handle, uint32_t sampleRateHz)
{
    int divider;
    LPCLIB_Result result = LPCLIB_SUCCESS;


    if (sampleRateHz == 0) {
        LPC_DAC->CTRL = 0;
        handle->sampleRateHz = 0;
    }
    else {
        handle->sampleRateHz = sampleRateHz;

        /* Find nearest divider value */
        divider =
            (2 * CLKPWR_getBusClock(CLKPWR_CLOCK_DAC) + handle->sampleRateHz)
          / (2 * handle->sampleRateHz);
        if (divider < 1) {
            divider = 1;
        }
        if (divider > 0xFFFF) {
            divider = 0xFFFF;
        }

        LPC_DAC->CNTVAL = divider - 1;
    }

    return result;
}


/* Open access to the DAC. */
LPCLIB_Result DAC_open (DAC_Handle *pHandle)
{
    *pHandle = &dacContext;

    /* Low speed */
    dacContext.speed = (1 << DAC_CR_BIAS_Pos);

    return LPCLIB_SUCCESS;
}



/* Close access to the DAC. */
LPCLIB_Result DAC_close (DAC_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Configure the DAC block. */
LPCLIB_Result DAC_ioctl (DAC_Handle handle, const DAC_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    while (pConfig->opcode != DAC_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case DAC_OPCODE_SET_CALLBACK:
            if (pConfig->callback.pOldCallback) {       /* Return current callback if requested */
                *(pConfig->callback.pOldCallback) = handle->callback;
            }
            handle->callback = pConfig->callback.callback;
            break;

        case DAC_OPCODE_SET_SAMPLERATE:
            result = _DAC_setSampleRate(handle, pConfig->sampleRateHz);
            break;

        default:
            return LPCLIB_ILLEGAL_PARAMETER;
        }

        ++pConfig;
    }

    return result;
}



/* Manually update the DAC output. */
LPCLIB_Result DAC_write (DAC_Handle handle, uint32_t value)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (value > 1023) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

#if LPCLIB_DMA
    /* Halt DMA if still active */
    DMA_cancelJob(handle->dmaChannel);
#endif

    /* Stop timer, and disable double buffering. Just leave DAC enabled. */
    LPC_DAC->CTRL = DAC_CTRL_DMA_ENA_Msk;

    /* Write to output. */
    LPC_DAC->CR = value | handle->speed;

    return LPCLIB_SUCCESS;
}



#if LPCLIB_DMA
/** Callback for DMA events. */
static void _DAC_dmaCallback (LPCLIB_Event event)
{
    const DAC_JobPhase *pPhase = event.parameter;
    LPCLIB_Event dacEvent = {
        .id = LPCLIB_EVENTID_DAC,
        .parameter = pPhase->magic, };

    if (pPhase) {
        if (pPhase->callback) {
            switch (event.opcode) {
            case DMA_EVENT_PHASE_COMPLETE:
                dacEvent.opcode = DAC_EVENT_PHASE_COMPLETE;
                pPhase->callback(dacEvent);             /* Inform about end of phase */
                break;

            case DMA_EVENT_STOP:
                dacEvent.opcode = DAC_EVENT_JOB_COMPLETE;
                pPhase->callback(dacEvent);             /* Inform about end of job */
                DMA_freePhaseMemory(dacContext.dmaChannel, dacContext.pDmaJob);   //TODO: Avoid direct reference to globals
                break;
            }
        }
    }
}
#endif



#if LPCLIB_DMA
/* Submit a new DAC job to the driver. */
LPCLIB_Result DAC_submitJob (DAC_Handle handle, const DAC_Job *pJob, DMA_ChannelHandle dma)
{
    int numPhases;
    const DAC_JobPhase *pPhase;
    DMA_Job *pDmaJob;
    DMA_JobPhase *pDmaPhase;
    int i;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* A DMA handle is mandatory */
    if (dma == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Determine number of phases. */
    numPhases = 0;
    pPhase = pJob->firstPhase;
    while (pPhase) {
        ++numPhases;
        pPhase = pPhase->next;
        if (pPhase == pJob->firstPhase) {
            break;                                      /* Detect loop */
        }
    }

    /* Ask DMA driver for memory */
    pDmaJob = DMA_allocPhaseMemory(dma, numPhases);
    if (pDmaJob == NULL) {
        return LPCLIB_OUT_OF_MEMORY;
    }

    handle->dmaChannel = dma;
    handle->pDmaJob = pDmaJob;

    /* Translate DAC phases to DMA phases */
    pPhase = pJob->firstPhase;
    pDmaPhase = pDmaJob->firstPhase;
    for (i = 0; i < numPhases; i++) {
        /* Prepare DMA job */
        pDmaJob->source = DMA_PERIPHERAL_MEMORY;
        pDmaJob->destination = DMA_PERIPHERAL_DAC;
        pDmaJob->flowControl = DMA_FLOW_MEM2PER_DMA;

        /* NOTE: Writing a 16-bit half-word to DACR will clear bit 16 (BIAS), because the
         *       two 16-bit half-words cannot be written independently.
         */
        DMA_makePhase(
            &pDmaPhase[i],
            (uint32_t)pPhase->data,
            (uint32_t)&LPC_DAC->CR,
            pPhase,
            DMA_makePhaseParameter(
                pPhase->length,     /* Number of samples */
                DMA_BURSTSIZE_1,    /* Source burst size */
                DMA_BURSTSIZE_1,    /* Destination burst size */
                DMA_WIDTH_32,       /* Source width */
                DMA_WIDTH_32,       /* Destination width */
                ENABLE,             /* Source increment */
                DISABLE,            /* Destination increment */
                pPhase->callback ? ENABLE : DISABLE
                )
            );

            LPC_DAC->CTRL |= 0
                    | DAC_CTRL_DBLBUF_ENA_Msk
                    | DAC_CTRL_CNT_ENA_Msk
                    | DAC_CTRL_DMA_ENA_Msk;

        if (i == numPhases - 1) {
            if (pPhase->next == NULL) {
                pDmaPhase[i].next = NULL;
            }
            else {
                pDmaPhase[i].next = pDmaJob->firstPhase;
            }
        }
        else {
            pDmaPhase[i].next = &pDmaPhase[i + 1];
        }

        pPhase = pPhase->next;
    }

    pDmaJob->channel = dma;
    pDmaJob->callback = _DAC_dmaCallback;
    DMA_submitJob(dma, pDmaJob);

    return LPCLIB_SUCCESS;
}
#endif


