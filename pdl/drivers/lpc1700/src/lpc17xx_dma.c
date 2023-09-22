/* Copyright (c) 2011-2012, NXP Semiconductors
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

#if LPCLIB_DMA

/** \file
 *  \brief DMA driver implementation.
 *
 *  This file contains the driver code for the GPDMA peripheral.
 *
 *  \author NXP Semiconductors
 */


/** \addtogroup DMA
 *  @{
 */

#include <stdint.h>
#include <string.h>

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_dma.h"


/** @defgroup DMA_Public_Functions
 *  @{
 */


LPCLIB_DefineRegBit(DMA_CONFIG_E,               0,  1);
LPCLIB_DefineRegBit(DMA_CONFIG_M,               1,  1);

LPCLIB_DefineRegBit(DMACH_CONFIG_E,             0,  1);
LPCLIB_DefineRegBit(DMACH_CONFIG_SRCPERIPHERAL, 1,  5);
LPCLIB_DefineRegBit(DMACH_CONFIG_DESTPERIPHERAL,6,  5);
LPCLIB_DefineRegBit(DMACH_CONFIG_TRANSFERTYPE,  11, 3);
LPCLIB_DefineRegBit(DMACH_CONFIG_IE,            14, 1);
LPCLIB_DefineRegBit(DMACH_CONFIG_ITC,           15, 1);
LPCLIB_DefineRegBit(DMACH_CONFIG_L,             16, 1);
LPCLIB_DefineRegBit(DMACH_CONFIG_A,             17, 1);
LPCLIB_DefineRegBit(DMACH_CONFIG_H,             18, 1);


#define DMA_NUM_CHANNELS                    (8)         /**< Number of channels in DMA block */


/** Local context of DMA block. */
static struct DMA_Context {
    DMA_Name block;                         /**< Block identifier */
    LPCLIB_Switch inUse;                    /**< Set if interface open */
    LPCLIB_Callback systemCallback;         /**< Callback handler (e.g. for memory allocation) */
    osMutexId accessMutex;
} dmaContext;


/** Local context of DMA channel. */
static struct DMA_ChannelContext {
    uint8_t number;                         /**< Hardware channel number */
    LPCLIB_Switch inUse;                    /** Set if this channel is in use. */
    DMA_Handle parent;
    const DMA_Job *pJob;
    uint32_t bytesRemaining;                /**< Used for M2M transfers */
} dmaChannelContext[DMA_NUM_CHANNELS];


osMutexDef(dmaMutex);


/** @} */


/* Open access to a DMA block. */
LPCLIB_Result DMA_open (DMA_Name dmaNum, DMA_Handle *pHandle)
{
    (void) dmaNum;                                      /* There's only one DMA */
    LPC_GPDMA_TypeDef *dma = LPC_GPDMA;


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_deassertPeripheralReset(CLKPWR_RESET_GPDMA); /* Clear reset line */
#endif
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_GPDMA);       /* Enable DMAx peripheral clock */

    if (!dmaContext.inUse) {                            /* Do nothing if already enabled */
        dmaContext.accessMutex = osMutexCreate(osMutex(dmaMutex));
        dmaContext.block = dmaNum;
        dmaContext.inUse = LPCLIB_YES;
        *pHandle = &dmaContext;                         /* Return block handle to caller */
        dma->Config = DMA_CONFIG_E_Msk;                 /* Enable DMA block. Default configuration
                                                         * 'little endian' for the AHB master ports.
                                                         */

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;
}



/* Close a DMA block. */
LPCLIB_Result DMA_close (DMA_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    //TODO Check if channels are still busy, and return BUSY in that case

    LPC_GPDMA->Config = 0;                              /* Disable the DMA block. */
    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_GPDMA);      /* Disable DMAx peripheral clock */

//    LPCLIB_OSALMutexDelete((*pHandle)->accessMutex);

    (*pHandle)->inUse = LPCLIB_NO;
    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Set options of the DMA block. */
void DMA_ioctl (DMA_Handle handle, const DMA_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    switch (pConfig->opcode) {
    case DMA_OPCODE_SET_SYSTEM_CALLBACK:
        handle->systemCallback = pConfig->callback;
        break;
    }
}



/* Acquire access to a DMA channel. */
LPCLIB_Result DMA_acquireChannel (DMA_Handle handle, DMA_ChannelHandle *pChannelHandle)
{
    int n;
    LPCLIB_Result result = LPCLIB_BUSY;


    /* Sanity check */
    if (pChannelHandle == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* We need a valid DMA block handle */
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Nothing to do if we already have a valid handle */
    if (*pChannelHandle != LPCLIB_INVALID_HANDLE) {
        return LPCLIB_SUCCESS;
    }

    /* Get exclusive access, and search for a free channel. */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        for (n = 0; n < DMA_NUM_CHANNELS; n++) {            /* Probe all channels */
            if (dmaChannelContext[n].inUse == LPCLIB_NO) {
                result = LPCLIB_SUCCESS;                    /* Got one! */
                dmaChannelContext[n].number = n;
                dmaChannelContext[n].parent = handle;
                dmaChannelContext[n].pJob = NULL;
                dmaChannelContext[n].bytesRemaining = 0;
                dmaChannelContext[n].inUse = LPCLIB_YES;    /* Mark as used */
                *pChannelHandle = &dmaChannelContext[n];    /* Return this handle */
                break;                                      /* The quest ends here! */
            }
        }

        osMutexRelease(handle->accessMutex);
    }

    return result;
}



/* Release access to a DMA channel. */
LPCLIB_Result DMA_releaseChannel (DMA_ChannelHandle *pChannelHandle)
{
    if (*pChannelHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Make sure all activity has stopped. */
    (*pChannelHandle)->inUse = LPCLIB_NO;
    (*pChannelHandle)->pJob = NULL;
    DMA_cancelJob(*pChannelHandle);

    /* Return ownership of handle (atomic write) */
    *pChannelHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Memory-to-memory copy. */
LPCLIB_Result DMA_memcpy (DMA_MemcpyContext *pContext,
                          void *pDest,
                          void *pSource,
                          int size)
{
    uint32_t thisLength;

    if (pContext == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (pContext->job.channel == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (size == 0) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    pContext->job.source = DMA_PERIPHERAL_MEMORY;
    pContext->job.destination = DMA_PERIPHERAL_MEMORY;
    pContext->job.flowControl = DMA_FLOW_MEM2MEM_DMA;

    /* If we do not start on a 16-byte boundary, transfer the initial 1...15 bytes byte-wise. */
    //TODO: Mainly important for large SDRAM transfers. Check if it's src or dest that needs to be aligned!
    if (((uint32_t)pSource % 16) == 0) {
        thisLength = size;
        if (thisLength > 4092 * 4) {
            thisLength = 4092 * 4;
        }

        pContext->job.firstPhase = DMA_makePhase(
                &pContext->phase,
                (uint32_t)pSource,
                (uint32_t)pDest,
                NULL,
                DMA_makePhaseParameter(
                        thisLength / 4,
                        DMA_BURSTSIZE_4,
                        DMA_BURSTSIZE_4,
                        DMA_WIDTH_32,
                        DMA_WIDTH_32,
                        ENABLE,
                        ENABLE,
                        ENABLE));
    }
    else {
        /* Initial byte-wise transfer */
        thisLength = (uint32_t)pSource % 16;

        pContext->job.firstPhase = DMA_makePhase(
                &pContext->phase,
                (uint32_t)pSource,
                (uint32_t)pDest,
                NULL,
                DMA_makePhaseParameter(
                        thisLength,
                        DMA_BURSTSIZE_1,
                        DMA_BURSTSIZE_1,
                        DMA_WIDTH_8,
                        DMA_WIDTH_8,
                        ENABLE,
                        ENABLE,
                        ENABLE));
    }

    pContext->job.channel->bytesRemaining = size - thisLength;

//TODO allow synchronous operation!!

    return DMA_submitJob(pContext->job.channel, &pContext->job);
}



/* Ask for memory allocation for DMA phases. */
DMA_Job * DMA_allocPhaseMemory (DMA_ChannelHandle channel, int numPhases)
{
    LPCLIB_Event event;
    uint32_t size;
    void *memory;
    int sizeJob;


    if (channel == LPCLIB_INVALID_HANDLE) {
        return NULL;
    }

    /* Is there a system handler? */
    if (!channel->parent->systemCallback) {
        return NULL;
    }

    /* Ask for memory.
     * Make sure to request enough memory such that each component (1x job, Nx phase)
     * can be properly aligned!
     */
    sizeJob = 4 * ((sizeof(DMA_Job) + 3) / 4);
    size = sizeJob + sizeof(DMA_JobPhase[numPhases]);
    event.id = LPCLIB_EVENTID_DMA;
    event.opcode = DMA_EVENT_MEMALLOC;
    memory = (void *)size;
    event.parameter = &memory;
    channel->parent->systemCallback(event);

    if (memory) {
        ((DMA_Job *)memory)->firstPhase = (DMA_JobPhase *)((uint32_t)memory + sizeJob);
    }

    return memory;
}



/* Return memory previously allocated by \ref DMA_allocPhaseMemory. */
LPCLIB_Result DMA_freePhaseMemory (DMA_ChannelHandle channel, DMA_Job *pJob)
{
    LPCLIB_Event event;

    if (channel == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Is there a system handler? */
    if (!channel->parent->systemCallback) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (pJob == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    event.id = LPCLIB_EVENTID_DMA;
    event.opcode = DMA_EVENT_MEMFREE;
    event.parameter = pJob;
    channel->parent->systemCallback(event);

    return LPCLIB_SUCCESS;
}



/* Submit a job to the DMA driver. */
LPCLIB_Result DMA_submitJob (DMA_ChannelHandle handle, const DMA_Job *pJob)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* NOTE:
     * We assume that for any given channel there is only one calling task.
     * Then we do not have to use OS tools to guarantee exclusive access,
     * because hardware allows channels to be controlled independently
     * from each other.
     * Exclusivity is only needed when a channel handle shall be acquired.
     */

    /* Return if selected channel is busy */
    if (LPC_GPDMA->EnbldChns & (1u << handle->number)) {
        return LPCLIB_BUSY;
    }

    handle->pJob = pJob;                                /* Remember job for later callback access */

    /* Adjust the DMA request line multiplexer */
    if (pJob->source & (1 << 15)) {
        LPCLIB_BITBAND(&LPC_SC->DMAREQSEL, (pJob->source >> 8) & 0x0F)
                = (pJob->source >> 14) & 1;
    }
    if (pJob->destination & (1 << 15)) {
        LPCLIB_BITBAND(&LPC_SC->DMAREQSEL, (pJob->destination >> 8) & 0x0F)
                = (pJob->destination >> 14) & 1;
    }

    /* Load job's first phase */
    LPC_GPDMACH[handle->number].CSrcAddr = (uint32_t)pJob->firstPhase->pSource;
    LPC_GPDMACH[handle->number].CDestAddr = (uint32_t)pJob->firstPhase->pDest;
    LPC_GPDMACH[handle->number].CLLI = (uint32_t)pJob->firstPhase->next;
    LPC_GPDMACH[handle->number].CControl = pJob->firstPhase->parameter;
    LPC_GPDMACH[handle->number].CConfig = 0
                  | ((pJob->source & 0x1F) << DMACH_CONFIG_SRCPERIPHERAL_Pos)
                  | ((pJob->destination & 0x1F) << DMACH_CONFIG_DESTPERIPHERAL_Pos)
                  | ((pJob->flowControl << DMACH_CONFIG_TRANSFERTYPE_Pos) & DMACH_CONFIG_TRANSFERTYPE_Msk)
                  | DMACH_CONFIG_IE_Msk                 /* Allow error interrupts */
                  | DMACH_CONFIG_ITC_Msk                /* Allow terminal count interrupts */
                  | DMACH_CONFIG_E_Msk;                 /* Enable */

    return LPCLIB_SUCCESS;
}


/* Prematurely terminate an active job. */
LPCLIB_Result DMA_cancelJob (DMA_ChannelHandle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* TODO: Alternative to channel killing... */
    LPC_GPDMA->Channels[handle->number].CConfig = 0;

    return LPCLIB_SUCCESS;
}



/** IRQ handler for GPDMA block.
 */
void DMA_IRQHandler (void)
{
    LPC_GPDMA_TypeDef * const dma = LPC_GPDMA;
    int channel;
    const void *param;
    DMA_JobPhase *thisLli;
    LPCLIB_Event event;
    DMA_ChannelHandle handle;

    /* Determine the interrupting channel.
     * We do this by looking for a set bit in the interrupt request status register,
     * starting from highest channel. (Channel 0 has highest priority, but this matters
     * only for the transfers, not for checking interrupts).
     * When more than one channel is requesting an interrupt at a time, there will
     * be another (tail-chained) interrupt right after leaving this one.
     */
    channel = 31 - __CLZ(dma->IntStat);
    if (channel >= 0) {                                 /* Should always be true... */
        handle = &dmaChannelContext[channel];

        /* Preset all known fields of possible events */
        event.id = LPCLIB_EVENTID_DMA;
        event.channel = channel;

        if (dma->IntTCStat & (1u << channel)) {         /* A terminal count interrupt? */
            dma->IntTCClear = (1u << channel);

            /* Report completion of this phase. If the whole job is done now,
             * send the STOP message instead.
             * Note: If the current interrupt is from the second to last phase, and the
             *       last phase is very short and already over now, there will be no
             *       separate reporting of the end of phase, but rather a STOP signal only.
             *       It is therefore not useful to enable end-of-phase interrupt in such
             *       a situation.
             */
            if (handle->pJob) {                         //TODO is it valid to check for job==0?
                /* Check if M2M transfer needs continuation */
                if (handle->bytesRemaining >= 4092 * 4) {
                    LPC_GPDMACH[channel].CControl = DMA_makePhaseParameter(
                        4092,
                        DMA_BURSTSIZE_4,
                        DMA_BURSTSIZE_4,
                        DMA_WIDTH_32,
                        DMA_WIDTH_32,
                        ENABLE,
                        ENABLE,
                        ENABLE);
                    LPC_GPDMACH[handle->number].CConfig |= DMACH_CONFIG_E_Msk;
                    handle->bytesRemaining -= 4092 * 4;
                }
                else if (handle->bytesRemaining != 0) {
                    LPC_GPDMACH[channel].CControl = DMA_makePhaseParameter(
                        handle->bytesRemaining,
                        DMA_BURSTSIZE_1,
                        DMA_BURSTSIZE_1,
                        DMA_WIDTH_8,
                        DMA_WIDTH_8,
                        ENABLE,
                        ENABLE,
                        ENABLE);
                    LPC_GPDMACH[handle->number].CConfig |= DMACH_CONFIG_E_Msk;
                    handle->bytesRemaining = 0;
                }
                else {
                    if (handle->pJob->callback) {
                        if (dma->EnbldChns & (1u << channel)) {     /* Job still running? */
                            event.opcode = DMA_EVENT_PHASE_COMPLETE;
                        }
                        else {                                      /* Job done */
                            event.opcode = DMA_EVENT_STOP;
                        }

                        thisLli = (DMA_JobPhase *)dma->Channels[channel].CLLI;
                        if (thisLli) {
                            param = thisLli->callbackParameter;
                        }
                        else {
                            param = NULL;
                        }
                        event.parameter = (void *)param;

                        handle->pJob->callback(event);
                    }
                }
            }
        }

        if (dma->IntErrStat & (1u << channel)) {        /* An error interrupt? */
            /* The error condition has automatically disabled the channel.
             * All that's left to do is inform the calling task.
             */
            dma->IntErrClr = (1u << channel);           /* Clear this request */
            if (handle->pJob) {                         //TODO
                if (handle->pJob->callback) {
                    event.opcode = DMA_EVENT_ERROR;
                    handle->pJob->callback(event);
                }
                handle->pJob = NULL;                    /* Job done */
            }
        }
    }
}


/** @} DMA */

#endif  /* #if LPCLIB_DMA */
