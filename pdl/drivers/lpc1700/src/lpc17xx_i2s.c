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

/** \file
 *  \brief I2S driver implementation.
 *
 *  This file contains the driver code for the I2S peripheral.
 *
 *  \author NXP Semiconductors
 */


#include "lpc17xx_i2s.h"
#include "lpc17xx_dma.h"
#include "lpc17xx_clkpwr.h"

LPCLIB_DefineRegBit(I2S_DAO_WORDWIDTH,          0,  2);
LPCLIB_DefineRegBit(I2S_DAO_MONO,               2,  1);
LPCLIB_DefineRegBit(I2S_DAO_STOP,               3,  1);
LPCLIB_DefineRegBit(I2S_DAO_RESET,              4,  1);
LPCLIB_DefineRegBit(I2S_DAO_WS_SEL,             5,  1);
LPCLIB_DefineRegBit(I2S_DAO_WS_HALFPERIOD,      6,  9);
LPCLIB_DefineRegBit(I2S_DAO_MUTE,               15, 1);

LPCLIB_DefineRegBit(I2S_DAI_WORDWIDTH,          0,  2);
LPCLIB_DefineRegBit(I2S_DAI_MONO,               2,  1);
LPCLIB_DefineRegBit(I2S_DAI_STOP,               3,  1);
LPCLIB_DefineRegBit(I2S_DAI_RESET,              4,  1);
LPCLIB_DefineRegBit(I2S_DAI_WS_SEL,             5,  1);
LPCLIB_DefineRegBit(I2S_DAI_WS_HALFPERIOD,      6,  9);

LPCLIB_DefineRegBit(I2S_DAx_WORDWIDTH,          0,  2);
LPCLIB_DefineRegBit(I2S_DAx_MONO,               2,  1);
LPCLIB_DefineRegBit(I2S_DAx_STOP,               3,  1);
LPCLIB_DefineRegBit(I2S_DAx_RESET,              4,  1);
LPCLIB_DefineRegBit(I2S_DAx_WS_SEL,             5,  1);
LPCLIB_DefineRegBit(I2S_DAx_WS_HALFPERIOD,      6,  9);

LPCLIB_DefineRegBit(I2S_STATE_IRQ,              0,  1);
LPCLIB_DefineRegBit(I2S_STATE_DMAREQ1,          1,  1);
LPCLIB_DefineRegBit(I2S_STATE_DMAREQ2,          2,  1);
LPCLIB_DefineRegBit(I2S_STATE_RX_LEVEL,         8,  4);
LPCLIB_DefineRegBit(I2S_STATE_TX_LEVEL,         16, 4);

LPCLIB_DefineRegBit(I2S_DMA1_RX_DMA1_ENABLE,    0,  1);
LPCLIB_DefineRegBit(I2S_DMA1_TX_DMA1_ENABLE,    1,  1);
LPCLIB_DefineRegBit(I2S_DMA1_RX_DEPTH_DMA1,     8,  4);
LPCLIB_DefineRegBit(I2S_DMA1_TX_DEPTH_DMA1,     16, 4);

LPCLIB_DefineRegBit(I2S_DMA2_RX_DMA2_ENABLE,    0,  1);
LPCLIB_DefineRegBit(I2S_DMA2_TX_DMA2_ENABLE,    1,  1);
LPCLIB_DefineRegBit(I2S_DMA2_RX_DEPTH_DMA2,     8,  4);
LPCLIB_DefineRegBit(I2S_DMA2_TX_DEPTH_DMA2,     16, 4);

LPCLIB_DefineRegBit(I2S_IRQ_RX_IRQ_ENABLE,      0,  1);
LPCLIB_DefineRegBit(I2S_IRQ_TX_IRQ_ENABLE,      1,  1);
LPCLIB_DefineRegBit(I2S_IRQ_RX_DEPTH_IRQ,       8,  4);
LPCLIB_DefineRegBit(I2S_IRQ_TX_DEPTH_IRQ,       16, 4);

LPCLIB_DefineRegBit(I2S_TXRATE_Y_DIVIDER,       0,  8);
LPCLIB_DefineRegBit(I2S_TXRATE_X_DIVIDER,       8,  8);

LPCLIB_DefineRegBit(I2S_RXRATE_Y_DIVIDER,       0,  8);
LPCLIB_DefineRegBit(I2S_RXRATE_X_DIVIDER,       8,  8);

LPCLIB_DefineRegBit(I2S_TXBITRATE_TX_BITRATE,   0,  6);

LPCLIB_DefineRegBit(I2S_RXBITRATE_RX_BITRATE,   0,  6);

LPCLIB_DefineRegBit(I2S_TXMODE_TXCLKSEL,        0,  2);
LPCLIB_DefineRegBit(I2S_TXMODE_TX4PIN,          2,  1);
LPCLIB_DefineRegBit(I2S_TXMODE_TXMCENA,         3,  1);

LPCLIB_DefineRegBit(I2S_RXMODE_RXCLKSEL,        0,  2);
LPCLIB_DefineRegBit(I2S_RXMODE_RX4PIN,          2,  1);
LPCLIB_DefineRegBit(I2S_RXMODE_RXMCENA,         3,  1);


/** Fixed threshold for FIFO operation in IRQ and DMA mode. */
#define I2S_THRESHOLD_IRQ                       4
#define I2S_THRESHOLD_DMA                       4


/** Local context of I2S busses. */
static struct I2S_Context {
    I2S_Name bus;                           /**< Bus identifier */
    LPCLIB_Result errorStatus;              /**< Error status returned with end of transaction */
    LPCLIB_Switch txMclkOutput;
    LPCLIB_Switch rxMclkOutput;
    uint32_t txSamplerate;
    uint32_t rxSamplerate;
    int16_t txMclkMultiple;
    int16_t rxMclkMultiple;
    const I2S_Job *pxJob[2];
    const I2S_JobPhase *txPhase;
    const I2S_JobPhase *rxPhase;
    uint32_t txCount;
    uint32_t rxCount;
#if LPCLIB_I2S_DMA
    DMA_ChannelHandle xDmaChannel[2];
    DMA_Job *pxDmaJob[2];
#endif
} i2sContext[__NUM_I2S__];


/** Peripheral bus address of I2S block(s). */
static LPC_I2S_TypeDef * const i2sPtr[__NUM_I2S__] = {LPC_I2S,};

#if LPCLIB_I2S_DMA
/** DMA request lines for I2S block(s). */
static const DMA_Peripheral i2sDmaRequestLines[2] = {DMA_PERIPHERAL_I2S0CH0, DMA_PERIPHERAL_I2S0CH1};
#endif

static const CLKPWR_Clockswitch i2sClockSwitch[__NUM_I2S__] = {CLKPWR_CLOCKSWITCH_I2S,};
//static const CLKPWR_Clock i2sClock[__NUM_I2S__] = {CLKPWR_CLOCK_I2S,};
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
static const CLKPWR_Reset i2sReset[__NUM_I2S__] = {CLKPWR_RESET_I2S,};
#endif

static volatile uint32_t * const i2sDaoPtr[__NUM_I2S__] =
    {&LPCLIB_BITBAND(&LPC_I2S->DAO, 0),};


static void I2S_dmaCallbackI2S0Tx (LPCLIB_Event event);
static void I2S_dmaCallbackI2S0Rx (LPCLIB_Event event);

static const LPCLIB_Callback i2sDmaCallbackPtr[__NUM_I2S__][2] = {
    {I2S_dmaCallbackI2S0Tx, I2S_dmaCallbackI2S0Rx},
};



/* Open an I2S bus. */
LPCLIB_Result I2S_open (I2S_Name bus, I2S_Handle *pHandle)
{
//    LPC_I2S_TypeDef *i2s = i2sPtr[bus];

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_deassertPeripheralReset(i2sReset[bus]);      /* Release I2S reset */
#endif
    CLKPWR_enableClock(i2sClockSwitch[bus]);            /* Enable I2S peripheral clock */

    /* Cannot open an I2S twice */
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        i2sContext[bus].bus = bus;
        i2sContext[bus].rxSamplerate = 0;
        i2sContext[bus].txSamplerate = 0;
        i2sContext[bus].rxMclkMultiple = 0;
        i2sContext[bus].txMclkMultiple = 0;
        i2sContext[bus].rxMclkOutput = DISABLE;
        i2sContext[bus].txMclkOutput = DISABLE;

        *pHandle = &i2sContext[bus];                    /* Return handle */

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;

}



/* Close an I2S bus. */
void I2S_close (I2S_Handle *pHandle)
{
    DMA_Job *pJob;

    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

#if LPCLIB_I2S_DMA
    /* Stop DMA activity, and return memory */
    pJob = (*pHandle)->pxDmaJob[I2S_CHANNEL_TX];
    if ((*pHandle)->xDmaChannel[I2S_CHANNEL_TX] != LPCLIB_INVALID_HANDLE) {
        DMA_cancelJob((*pHandle)->xDmaChannel[I2S_CHANNEL_TX]);
        if (pJob) {
            DMA_freePhaseMemory((*pHandle)->xDmaChannel[I2S_CHANNEL_TX], pJob);
            (*pHandle)->pxDmaJob[I2S_CHANNEL_TX] = NULL;
        }
        (*pHandle)->xDmaChannel[I2S_CHANNEL_TX] = LPCLIB_INVALID_HANDLE;
    }
    pJob = (*pHandle)->pxDmaJob[I2S_CHANNEL_RX];
    if ((*pHandle)->xDmaChannel[I2S_CHANNEL_RX] != LPCLIB_INVALID_HANDLE) {
        DMA_cancelJob((*pHandle)->xDmaChannel[I2S_CHANNEL_RX]);
        if (pJob) {
            DMA_freePhaseMemory((*pHandle)->xDmaChannel[I2S_CHANNEL_RX], pJob);
            (*pHandle)->pxDmaJob[I2S_CHANNEL_RX] = NULL;
        }
        (*pHandle)->xDmaChannel[I2S_CHANNEL_RX] = LPCLIB_INVALID_HANDLE;
    }
#else
    (void) pJob;
#endif

    CLKPWR_disableClock(i2sClockSwitch[(*pHandle)->bus]);   /* Disable I2S peripheral clock */

    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Configure the I2S block. */
void I2S_ioctl (I2S_Handle handle, const I2S_Config *pConfig)
{
    int temp;
    LPC_I2S_TypeDef * const i2s = i2sPtr[handle->bus];


    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    while (pConfig->opcode != I2S_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case I2S_OPCODE_SET_MODE:           /* Configure operating mode and data format. */
            /* Stop and reset the channel while reconfiguring */
            i2s->DAx[pConfig->channel] =                /* Reset + Stop */
                    I2S_DAx_RESET_Msk
                | I2S_DAx_STOP_Msk
                | (31u << I2S_DAx_WS_HALFPERIOD_Pos);
            i2s->DAx[pConfig->channel] =                /* Release Reset */
                    I2S_DAx_STOP_Msk
                | (31u << I2S_DAx_WS_HALFPERIOD_Pos);

            /* Determine word select half period. Must be at least as long as the selected word width */
            temp = pConfig->modeAndSize.bitsPerChannel;
            if (temp < (pConfig->modeAndSize.size >> 8)) {
                temp = (pConfig->modeAndSize.size >> 8);
            }

            i2s->xMODE[pConfig->channel] = pConfig->modeAndSize.mode & 0x0F;
            i2s->DAx[pConfig->channel] =
                    (pConfig->modeAndSize.mode & I2S_DAx_WS_SEL_Msk)
                | (pConfig->modeAndSize.size & (I2S_DAx_WORDWIDTH_Msk | I2S_DAx_MONO_Msk))
                | ((temp - 1) << I2S_DAx_WS_HALFPERIOD_Pos)
                | I2S_DAx_STOP_Msk;                     /* Keep in stop mode */
            break;

        case I2S_OPCODE_SET_SAMPLERATE:
            //TODO
            break;

        case I2S_OPCODE_SET_DIVIDERS:       /* Change sample rate by specifying the dividers */
            if (pConfig->channel == I2S_CHANNEL_RX) {
                i2s->RXBITRATE = pConfig->dividers.bitClockDivider - 1;
                i2s->RXRATE = (pConfig->dividers.fracNominator << 8)
                            | (pConfig->dividers.fracDenominator << 0);
            }
            if (pConfig->channel == I2S_CHANNEL_TX) {
                i2s->TXBITRATE = pConfig->dividers.bitClockDivider - 1;
                i2s->TXRATE = (pConfig->dividers.fracNominator << 8)
                            | (pConfig->dividers.fracDenominator << 0);
            }
            break;

        case I2S_OPCODE_INVALID:
            /* ignore */
            break;
        }

        ++pConfig;
    }
}



#if LPCLIB_I2S_DMA
/** Callback for DMA events. */
static void I2S_dmaCallbackCommon (LPCLIB_Event event, const I2S_Job *pJob)
{
    const I2S_JobPhase *pPhase = event.parameter;
    LPCLIB_Event i2sEvent = {
        .id = LPCLIB_EVENTID_I2S,
        .parameter = pPhase->magic, };

    if (pPhase) {
        if (pJob->callback) {
            switch (event.opcode) {
            case DMA_EVENT_PHASE_COMPLETE:
                i2sEvent.opcode = I2S_EVENT_PHASE_COMPLETE;
                pJob->callback(i2sEvent);               /* Inform about end of phase */
                break;

            case DMA_EVENT_STOP:
                i2sEvent.opcode = I2S_EVENT_JOB_COMPLETE;
                pJob->callback(i2sEvent);               /* Inform about end of job */
                break;
            }
        }
    }
}

static void I2S_dmaCallbackI2S0Tx (LPCLIB_Event event)
{
    I2S_dmaCallbackCommon(event, i2sContext[I2S0].pxJob[I2S_CHANNEL_TX]);
}

static void I2S_dmaCallbackI2S0Rx (LPCLIB_Event event)
{
    I2S_dmaCallbackCommon(event, i2sContext[I2S0].pxJob[I2S_CHANNEL_RX]);
}
#endif



/* Submit a new I2S job to the driver. */
LPCLIB_Result I2S_submitJob (I2S_Handle handle, const I2S_Job *pJob, DMA_ChannelHandle dma)
{
    LPC_I2S_TypeDef *i2s;
    int numPhases;
    const I2S_JobPhase *pI2sPhase;
    DMA_Job *pDmaJob;
    DMA_JobPhase *pDmaPhase;
    int i;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    i2s = i2sPtr[handle->bus];

    handle->pxJob[pJob->channel] = pJob;

#if LPCLIB_I2S_DMA
    if (dma != LPCLIB_INVALID_HANDLE) {                 /* DMA transfer */
        /* Determine number of phases. */
        numPhases = 0;
        pI2sPhase = pJob->firstPhase;
        while (pI2sPhase) {
            ++numPhases;
            pI2sPhase = pI2sPhase->next;
            if (pI2sPhase == pJob->firstPhase) {
                break;                                  /* Detect loop */
            }
        }

        /* Ask DMA driver for memory */
        pDmaJob = DMA_allocPhaseMemory(dma, numPhases);
        if (pDmaJob == NULL) {
            /* Fall back to interrupt mode */
            dma = LPCLIB_INVALID_HANDLE;
        }
        else {
            handle->xDmaChannel[pJob->channel] = dma;
            handle->pxDmaJob[pJob->channel] = pDmaJob;

            /* Translate I2S phases to DMA phases */
            pI2sPhase = pJob->firstPhase;
            pDmaPhase = pDmaJob->firstPhase;
            for (i = 0; i < numPhases; i++) {
                if (pJob->channel == I2S_CHANNEL_RX) {
                    /* Prepare DMA job */
                    pDmaJob->source = i2sDmaRequestLines[pJob->channel];
                    pDmaJob->destination = DMA_PERIPHERAL_MEMORY;
                    pDmaJob->flowControl = DMA_FLOW_PER2MEM_DMA;
                    pDmaJob->callback = i2sDmaCallbackPtr[handle->bus][I2S_CHANNEL_RX];

                    DMA_makePhase(
                        &pDmaPhase[i],
                        (uint32_t)&LPC_I2S->RXFIFO,
                        (uint32_t)pI2sPhase->data,
                        pI2sPhase,
                        DMA_makePhaseParameter(
                            pI2sPhase->length,  /* Number of samples */ //TODO mono/stereo ?
                            DMA_BURSTSIZE_4,    /* Source burst size */
                            DMA_BURSTSIZE_4,    /* Destination burst size */
                            DMA_WIDTH_32,       /* Source width */
                            DMA_WIDTH_32,       /* Destination width */
                            DISABLE,            /* Source increment */
                            ENABLE,             /* Destination increment */
                            pI2sPhase->triggerCallback ? ENABLE : DISABLE
                            )
                        );

                        i2s->DMA2 =                                 /* Enable RX DMA, triggered by FIFO/2 */
                            (i2s->DMA2 & ~(I2S_DMA2_RX_DEPTH_DMA2_Msk))
                          | I2S_DMA2_RX_DMA2_ENABLE_Msk
                          | (I2S_THRESHOLD_DMA << I2S_DMA2_RX_DEPTH_DMA2_Pos);
                }
                else {
                    /* Prepare DMA job */
                    pDmaJob->source = DMA_PERIPHERAL_MEMORY;
                    pDmaJob->destination = i2sDmaRequestLines[pJob->channel];
                    pDmaJob->flowControl = DMA_FLOW_MEM2PER_DMA;
                    pDmaJob->callback = i2sDmaCallbackPtr[handle->bus][I2S_CHANNEL_TX];

                    DMA_makePhase(
                        &pDmaPhase[i],
                        (uint32_t)pI2sPhase->data,
                        (uint32_t)&LPC_I2S->TXFIFO,
                        pI2sPhase,
                        DMA_makePhaseParameter(
                            pI2sPhase->length,  /* Number of samples */ //TODO mono/stereo ?
                            DMA_BURSTSIZE_4,    /* Source burst size */
                            DMA_BURSTSIZE_4,    /* Destination burst size */
                            DMA_WIDTH_32,       /* Source width */
                            DMA_WIDTH_32,       /* Destination width */
                            ENABLE,             /* Source increment */
                            DISABLE,            /* Destination increment */
                            pI2sPhase->triggerCallback ? ENABLE : DISABLE
                            )
                        );

                        i2s->DMA1 =                                 /* Enable TX DMA, triggered by FIFO/2 */
                            (i2s->DMA1 & ~(I2S_DMA1_TX_DEPTH_DMA1_Msk))
                          | I2S_DMA1_TX_DMA1_ENABLE_Msk
                          | (I2S_THRESHOLD_DMA << I2S_DMA1_TX_DEPTH_DMA1_Pos);
                }
                if (i == numPhases - 1) {
                    if (pI2sPhase->next == NULL) {
                        pDmaPhase[i].next = NULL;
                    }
                    else {
                        pDmaPhase[i].next = pDmaJob->firstPhase;
                    }
                }
                else {
                    pDmaPhase[i].next = &pDmaPhase[i + 1];
                }

                pI2sPhase = pI2sPhase->next;
            }

            pDmaJob->channel = dma;
            DMA_submitJob(dma, pDmaJob);
        }
    }
#else
    (void) numPhases;
    (void) pI2sPhase;
    (void) pDmaJob;
    (void) pDmaPhase;
    (void) i;

    dma = LPCLIB_INVALID_HANDLE;                        /* No DMA support */
#endif

    if (dma == LPCLIB_INVALID_HANDLE) {                 /* Interrupt-driven transfer */
        if (pJob->channel == I2S_CHANNEL_RX) {
            if (handle->rxPhase) {
                return LPCLIB_BUSY;
            }
            handle->rxPhase = pJob->firstPhase;
            handle->rxCount = 0;

            i2s->DMA1 = 0;                              /* Disable DMA request */
            i2s->IRQ =                                  /* Enable RX IRQ, triggered by FIFO/2 */
                (i2s->IRQ & ~(I2S_IRQ_RX_DEPTH_IRQ_Msk))
              | I2S_IRQ_RX_IRQ_ENABLE_Msk
              | (I2S_THRESHOLD_IRQ << I2S_IRQ_RX_DEPTH_IRQ_Pos);
        }
        else {
            if (handle->txPhase) {
                return LPCLIB_BUSY;
            }
            handle->txPhase = pJob->firstPhase;
            handle->txCount = 0;

            i2s->DMA2 = 0;                              /* Disable DMA request */
            i2s->IRQ =                                  /* Enable TX IRQ, triggered by FIFO/2 */
                (i2s->IRQ & ~(I2S_IRQ_TX_DEPTH_IRQ_Msk))
              | I2S_IRQ_TX_IRQ_ENABLE_Msk
              | (I2S_THRESHOLD_IRQ << I2S_IRQ_TX_DEPTH_IRQ_Pos);
        }
    }

    return LPCLIB_SUCCESS;
}



/* Starts or stops an I2S channel (clears/sets the STOP condition). */
void I2S_run (I2S_Handle handle, I2S_Channel channel, LPCLIB_Switch enable)
{
    LPC_I2S_TypeDef *i2s;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    i2s = i2sPtr[handle->bus];
    LPCLIB_BITBAND(&i2s->DAx[channel], I2S_DAx_STOP_Pos) = (enable) ? 0 : 1;
}



/* Mute or unmute the TX channel. */
void I2S_muteTx (I2S_Handle handle, I2S_MuteSwitch mute)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    i2sDaoPtr[handle->bus][I2S_DAO_MUTE_Pos] = mute;
}



/** Common (for all I2S blocks) interrupt handler.
 *
 *  \param[in] bus I2S block number
 *  \param pTaskSwitch
 */
static void I2S_commonIRQHandler (I2S_Name bus)
{
    LPC_I2S_TypeDef * const i2s = i2sPtr[bus];
    I2S_Handle handle = &i2sContext[bus];
    int n;
    LPCLIB_Event event = {
        .id = LPCLIB_EVENTID_I2S,
        .block = bus,
        .channel = 0,
        .opcode = I2S_EVENT_PHASE_COMPLETE, };


    /* RX interrupt? */
    if (((i2s->STATE & I2S_STATE_RX_LEVEL_Msk) >> I2S_STATE_RX_LEVEL_Pos) >= I2S_THRESHOLD_IRQ) {
        /* Find next storage location in RX phase. */
        while (handle->rxPhase) {
            if (handle->rxCount < handle->rxPhase->length) {
                break;
            }

            /* Phase ends. Callback? */
            if (handle->pxJob[I2S_CHANNEL_RX] && handle->rxPhase->triggerCallback) {
                event.parameter = handle->rxPhase->magic;
                handle->pxJob[I2S_CHANNEL_RX]->callback(event);
            }

            handle->rxPhase = handle->rxPhase->next;
            handle->rxCount = 0;
        }

        if (handle->rxPhase) {
            for (n = 0; n < I2S_THRESHOLD_IRQ; n++) {
                if (handle->rxCount < handle->rxPhase->length) {
                    handle->rxPhase->data[handle->rxCount] = i2s->RXFIFO;
                    ++handle->rxCount;
                }
            }
        }
    }

    /* TX interrupt? */
    if (((i2s->STATE & I2S_STATE_TX_LEVEL_Msk) >> I2S_STATE_TX_LEVEL_Pos) >= I2S_THRESHOLD_IRQ) {
        /* Find next data in TX phase. */
        while (handle->txPhase) {
            if (handle->txCount < handle->txPhase->length) {
                break;
            }

            /* Phase ends. Callback? */
            if (handle->pxJob[I2S_CHANNEL_TX] && handle->txPhase->triggerCallback) {
                event.parameter = handle->txPhase->magic;
                handle->pxJob[I2S_CHANNEL_TX]->callback(event);
            }

            handle->txPhase = handle->txPhase->next;
            handle->txCount = 0;
        }

        if (handle->txPhase) {
            for (n = 0; n < I2S_THRESHOLD_IRQ; n++) {
                if (handle->txCount < handle->txPhase->length) {
                    i2s->TXFIFO = handle->txPhase->data[handle->txCount];
                    ++handle->txCount;
                }
            }
        }
    }
}


/** I2S block interrupt entry (interrupt number \ref I2S_IRQn).
 */
void I2S_IRQHandler (void)
{
    I2S_commonIRQHandler(I2S0);
}


/** @} */

/** @} addtogroup I2S */

