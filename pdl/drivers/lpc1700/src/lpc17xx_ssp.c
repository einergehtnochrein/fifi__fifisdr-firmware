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

#if LPCLIB_SSP

/** \file
 *  \brief SSP driver implementation.
 *
 *  This file contains the driver code for the SSP peripheral.
 *
 *  \author NXP Semiconductors
 */


/** \addtogroup SSP
 *  @{
 */

#include <string.h>

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_ssp.h"


/** @defgroup SSP_Public_Functions
 *  @{
 */


LPCLIB_DefineRegBit(SSP_CR0_DSS,                0,  4);
LPCLIB_DefineRegBit(SSP_CR0_FRF,                4,  2);
LPCLIB_DefineRegBit(SSP_CR0_CPOL,               6,  1);
LPCLIB_DefineRegBit(SSP_CR0_CPHA,               7,  1);
LPCLIB_DefineRegBit(SSP_CR0_SCR,                8,  8);

LPCLIB_DefineRegBit(SSP_CR1_LBM,                0,  1);
LPCLIB_DefineRegBit(SSP_CR1_SSE,                1,  1);
LPCLIB_DefineRegBit(SSP_CR1_MS,                 2,  1);
LPCLIB_DefineRegBit(SSP_CR1_SOD,                3,  1);

LPCLIB_DefineRegBit(SSP_SR_TFE,                 0,  1);
LPCLIB_DefineRegBit(SSP_SR_TNF,                 1,  1);
LPCLIB_DefineRegBit(SSP_SR_RNE,                 2,  1);
LPCLIB_DefineRegBit(SSP_SR_RFF,                 3,  1);
LPCLIB_DefineRegBit(SSP_SR_BSY,                 4,  1);

LPCLIB_DefineRegBit(SSP_IMSC_RORIM,             0,  1);
LPCLIB_DefineRegBit(SSP_IMSC_RTIM,              1,  1);
LPCLIB_DefineRegBit(SSP_IMSC_RXIM,              2,  1);
LPCLIB_DefineRegBit(SSP_IMSC_TXIM,              3,  1);

LPCLIB_DefineRegBit(SSP_MIS_RORMIS,             0,  1);
LPCLIB_DefineRegBit(SSP_MIS_RTMIS,              1,  1);
LPCLIB_DefineRegBit(SSP_MIS_RXMIS,              2,  1);
LPCLIB_DefineRegBit(SSP_MIS_TXMIS,              3,  1);

LPCLIB_DefineRegBit(SSP_ICR_RRORIC,             0,  1);
LPCLIB_DefineRegBit(SSP_ICR_RTIC,               1,  1);

//TODO make sure we have enough (and not more) mutexes/syncs!
osMutexDef(sspAccessMutexDef0);
osMutexDef(sspAccessMutexDef1);
osSemaphoreDef(sspSyncSemaDef0);
osSemaphoreDef(sspSyncSemaDef1);


static osMutexDef_t * const sspMutexes[SSP_NUM_BUSSES] = {
    osMutex(sspAccessMutexDef0), osMutex(sspAccessMutexDef1),
    };
static osSemaphoreDef_t * const sspSemas[SSP_NUM_BUSSES] = {
    osSemaphore(sspSyncSemaDef0), osSemaphore(sspSyncSemaDef1),
    };
static LPC_SSP_TypeDef * const sspPtr[SSP_NUM_BUSSES] = {LPC_SSP0, LPC_SSP1,};
static const CLKPWR_Clockswitch sspClockswitch[SSP_NUM_BUSSES] = {CLKPWR_CLOCKSWITCH_SSP0, CLKPWR_CLOCKSWITCH_SSP1,};
static const CLKPWR_Clock sspClock[SSP_NUM_BUSSES] = {CLKPWR_CLOCK_SSP0, CLKPWR_CLOCK_SSP1,};


static struct SSP_Context {
    SSP_Name bus;                           /**< Bus identifier */
    LPCLIB_Switch inUse;                    /**< Set if interface open */
    SSP_Mode mode;                          /**< Master/slave */
    uint32_t bitrate;                       /**< Bit clock frequency (Hz) */

    LPCLIB_Result errorStatus;              /**< Error status returned with end of transaction */
    uint16_t nRxDummies;
    SSP_Job *job;
    SSP_JobPhase *txPhase;
    SSP_JobPhase *rxPhase;
    osMutexId accessMutex;
    osSemaphoreId syncSema;

} sspContext[SSP_NUM_BUSSES];



/** @} */



/* Open an SSP bus. */
LPCLIB_Result SSP_open (SSP_Name bus, SSP_Handle *pHandle)
{
    LPC_SSP_TypeDef *ssp = sspPtr[bus];
    SSP_Handle handle = &sspContext[bus];

    CLKPWR_enableClock(sspClockswitch[bus]);            /* Enable SSPx peripheral clock */

    if (!handle->inUse) {
        handle->accessMutex = osMutexCreate(sspMutexes[bus]);
        handle->syncSema = osSemaphoreCreate(sspSemas[bus], 1);
        handle->bus = bus;
        handle->inUse = LPCLIB_YES;
        *pHandle = handle;                              /* Return bus handle to caller */
        ssp->CR1 |= SSP_CR1_SSE_Msk;                    /* Enable SSP engine */

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;
}



/* Close an SSP bus. */
void SSP_close (SSP_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    CLKPWR_disableClock(sspClockswitch[(*pHandle)->bus]);   /* Disable I2C peripheral clock */

//    LPCLIB_OSALMutexDelete(sspContext[(*pHandle)->bus].accessMutex);
//    LPCLIB_OSALSemaphoreDelete(sspContext[(*pHandle)->bus].syncSema);

    (*pHandle)->inUse = LPCLIB_NO;
    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Set options of the SSP block. */
void SSP_ioctl (SSP_Handle handle, const SSP_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    uint32_t temp;
    LPC_SSP_TypeDef * const ssp = sspPtr[handle->bus];

    while (pConfig->opcode != SSP_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case SSP_OPCODE_SET_FORMAT:                     /* Set phase/polarity */
            ssp->CR0 = (ssp->CR0 & ~0xFF) |
                        (uint32_t)pConfig->format.bits |
                        (uint32_t)pConfig->format.frameFormat |
                        (uint32_t)pConfig->format.clockFormat;
            break;

        case SSP_OPCODE_SET_MODE:                       /* Select master/slave mode */
            ssp->CR1 = (ssp->CR1 & ~SSP_CR1_MS_Msk) | pConfig->mode;
            break;

        case SSP_OPCODE_SET_BITRATE:                    /* Set clock speed */
            temp = CLKPWR_getBusClock(sspClock[handle->bus]);
            temp /= pConfig->bitrate;

            if (handle->mode == SSP_MODE_SLAVE) {       /* Slave mode */
                temp = 0;                               /* Set all dividers to zero. */
            }
            else {
                //TODO Optimize divider distribution. Range check!
                temp = (2 << 16) | ((temp / 2) - 1);
            }

            ssp->CPSR = temp >> 16;
            ssp->CR0 = (ssp->CR0 & ~SSP_CR0_SCR_Msk) | ((temp << SSP_CR0_SCR_Pos) & SSP_CR0_SCR_Msk);
            break;

        default:
            break;
        }

        ++pConfig;
    }
}




/* Submit a job to the SSP driver. */
LPCLIB_Result SSP_submitJob (SSP_Handle handle, SSP_Job *pJob)
{
    LPCLIB_Event event;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        handle->job = pJob;
        SSP_ioctl(handle, pJob->pConfig);               /* Configure the device */

        /* Make sure to reset the sync flag */
        osSemaphoreWait(handle->syncSema, 0);

    //TODO Check master/slave
    //Here: master
        /* Assert chip select before sending data */
        /* Prepare event */
        if (pJob->pDeviceSelect) {
            event.id = LPCLIB_EVENTID_SSP;
            event.block = handle->bus;
            event.opcode = SSP_EVENT_ASSERT_CHIPSELECT;
            event.channel = pJob->pDeviceSelect->channel;
            event.parameter = pJob->extraParameter;
            pJob->pDeviceSelect->callback(event);
        }

        pJob->nsent = 0;
        pJob->nreceived = 0;
        handle->txPhase = pJob->firstPhase;
        handle->rxPhase = pJob->firstPhase;

        /* Enable TX interrupt to start transmission (RX enabled as well) */
        sspPtr[handle->bus]->IMSC |= SSP_IMSC_TXIM_Msk | SSP_IMSC_RXIM_Msk | SSP_IMSC_RTIM_Msk;

    #if LPCLIB_SSP_ASYNC
        /* In async mode we can leave immediately */
        return LPCLIB_PENDING;
    #else
        /* In sync mode we wait for the end of transaction. */
        if (osSemaphoreWait(handle->syncSema, 1000 /*TODO*/) > 0) {
            osMutexRelease(handle->accessMutex);

            return handle->errorStatus;
        }

        osMutexRelease(handle->accessMutex);

        //TODO: serious error! Restart SSP bus, re-init semaphores
        //TODO: when will semaacquire bring us here??
        return LPCLIB_ERROR;
    #endif
    }

    return LPCLIB_BUSY;
}



/* Fill data into an SSP_JobPhase descriptor. */
SSP_JobPhase *SSP_makePhase (SSP_JobPhase *pPhase, const void *pTx, uint8_t *pRx, uint16_t length)
{
    memset(pPhase, 0, sizeof(SSP_JobPhase));
    pPhase->txstart8 = (uint8_t *)pTx;
    pPhase->rxstart8 = pRx;
    pPhase->length = length;
    pPhase->idlePattern = 0;

    return pPhase;
}



/** Common IRQ handler for SSP blocks.
 *
 *  \param[in] sspNum Indicator for SSP block.
 */
static void SSP_commonIRQHandler (SSP_Name bus)
{
    LPC_SSP_TypeDef * const ssp = sspPtr[bus];
    struct SSP_Context *pCtx = &sspContext[bus];
    uint32_t status;
    uint32_t rxFrame;
    int nFrames;
    LPCLIB_Event event;
    int i;
    _Bool stopTx;
    _Bool done = false;


    status = ssp->MIS;

    /* Prepare event */
    event.id = LPCLIB_EVENTID_SSP;
    event.block = bus;

    /* Determine number of received frames */
    if (status & SSP_MIS_RXMIS_Msk) {                   /* FIFO interrupt (at least 4 frames) */
        nFrames = 4;
    }
    else if (status & SSP_MIS_RTMIS_Msk) {              /* Timeout interrupt (at least 1 frame) */
        nFrames = 1;
        ssp->ICR = status & SSP_ICR_RTIC_Msk;           /* Clear possible timeout interrupt */
    }
    else {
        nFrames = 0;
    }

    if (nFrames > 0) {
        while (nFrames) {
            rxFrame = ssp->DR;                          /* Get RX frame */

            if (pCtx->nRxDummies) {                     /* Ignore dummy frames? */
                --pCtx->nRxDummies;
            }
            else {
                /* Store the frame? */
                if (pCtx->job->nreceived < pCtx->rxPhase->length) {
                    if (pCtx->rxPhase->rxstart8) {
                        pCtx->rxPhase->rxstart8[pCtx->job->nreceived] = rxFrame;
                    }
                    ++(pCtx->job->nreceived);
                }
            }

            /* End of transfer? */
            if (pCtx->job->nreceived >= pCtx->rxPhase->length) {
                pCtx->job->nreceived = 0;

                if (pCtx->rxPhase->interrupt) {         /* Send "interrupt" event to application? */
//                    event.opcode = SSP_EVENT_INTERRUPT;
//                    event.parameter = pCtx->rxPhase;
//                    if (pCtx->job->device) {
//                        pCtx->job->device->callbackFromISR(event);
//                    }

                    ssp->IMSC |= SSP_IMSC_TXIM_Msk | SSP_IMSC_RTIM_Msk; /* TX and timeout interrupts allowed */
                }

                pCtx->rxPhase = pCtx->rxPhase->next;    /* Next RX phase */

                if (!pCtx->rxPhase) {                   /* Will there be another phase? */
                    ssp->IMSC &=                        /* No. Disable further interrupts */
                        ~(SSP_IMSC_RORIM_Msk | SSP_IMSC_RTIM_Msk | SSP_IMSC_RXIM_Msk | SSP_IMSC_TXIM_Msk);

                    if (pCtx->job->pDeviceSelect) {
                        event.opcode = SSP_EVENT_DEASSERT_CHIPSELECT;
                        event.channel = pCtx->job->pDeviceSelect->channel;
                        pCtx->job->pDeviceSelect->callback(event);
                    }

                    done = true;
                    break;
                }
            }

            --nFrames;
        }
    }

    /* Handle TX interrupts */
    if (status & SSP_MIS_TXMIS_Msk) {
        nFrames = pCtx->txPhase->length                 /* How much is left to be sent? */
                  - pCtx->job->nsent;
        if (nFrames > 4) {
            nFrames = 4;                                /* Limit to half the FIFO size */
        }

        for (i = 0; i < nFrames; i++) {                 /* Send that block */
            if (pCtx->txPhase->txstart8) {
                ssp->DR = pCtx->txPhase->txstart8[pCtx->job->nsent];
            }
            else {
                ssp->DR = pCtx->txPhase->idlePattern;
            }
            ++(pCtx->job->nsent);
        }

        if (nFrames <= 0) {                             /* Check for end of transmission (in this phase) */
            pCtx->job->nsent = 0;                       /* Reset counter for next phase */
            stopTx =                                    /* Shall we stop further TX activity for now? */
                (pCtx->txPhase->txBarrier != DISABLE)   /*   Yes if current phase has TX barrier */
             || (pCtx->txPhase->next == NULL);          /*   Yes if we are in the last phase already */
            pCtx->txPhase = pCtx->txPhase->next;        /* Next TX phase */
            if (stopTx) {
                ssp->IMSC &= ~SSP_IMSC_TXIM_Msk;        /* Disable further TX interrupts */
            }
        }
    }

    /* Let the caller know when the transaction is over. */
    if (done) {
#if LPCLIB_SSP_ASYNC
        event.opcode = SSP_EVENT_SUCCESS;
        pCtx->job->device->callback(event);
#else
        osSemaphoreRelease(pCtx->syncSema);
#endif
    }
}


/** Hardware entry for SSP0 interrupt. */
void SSP0_IRQHandler (void)
{
    SSP_commonIRQHandler(SSP0);
}


/** Hardware entry for SSP1 interrupt. */
void SSP1_IRQHandler (void)
{
    SSP_commonIRQHandler(SSP1);
}


/** @} SSP */

#endif  /* #if LPCLIB_SSP */

