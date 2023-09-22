/* Copyright (c) 2010-2013, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fifidsp.h"
#include "ssb2200.h"
#include "ssb2700.h"
#include "ssb3300.h"


#define SSB_DECIM_RATE                      4
#define SSBAGC_FILTER_LENGTH                128

struct {
    int32_t filter[SSBAGC_FILTER_LENGTH];
    int index;
    uint32_t currentPeak;
    uint32_t lastPeak;
    uint32_t lastGain131;
} ssbagc __SECTION(".bss3");



/* Initialize the SSB demodulator */
void FIFIDSP_initSsb (FIFIDSP_AlgoContext *pContext)
{
    pContext->pCoeff = ssbfilter2200;
    if (pContext->bandwidth >= 2500) {
        pContext->pCoeff = ssbfilter2700;
    }
    if (pContext->bandwidth >= 3000) {
        pContext->pCoeff = ssbfilter3300;
    }
}


/* Automatic Gain Control */
static int32_t FIFIDSP_runSsbAgc (FIFIDSP_AlgoContext *pAlgo, int32_t sxin)
{
    int32_t sx;
    uint32_t sxinabs;
    int64_t targetLevel = 0x80000000;
    uint32_t maxGain;
    uint32_t delayBlocks;
    uint32_t delay;

    /* Store new sample in buffer. Keep peak value. */
    sx = ssbagc.filter[ssbagc.index];
    ssbagc.filter[ssbagc.index] = sxin;
    sxinabs = (sxin < 0) ? -sxin : sxin;
    if (sxinabs > ssbagc.currentPeak) {
        ssbagc.currentPeak = sxinabs;
    }

    /* Apply gain */
    sx = (((int64_t)sx * targetLevel) * ssbagc.lastGain131) / 0x100000000;

    /* Buffer filled? */
    ++ssbagc.index;
    if (ssbagc.index >= SSBAGC_FILTER_LENGTH) {
        ssbagc.index = 0;
        ssbagc.lastPeak = ssbagc.currentPeak;
        ssbagc.currentPeak = 1;

        /* Calculate gain for the block */
        if (ssbagc.lastGain131 < 1) {
            ssbagc.lastGain131 = 1;
        }
        maxGain = (uint32_t)targetLevel / ssbagc.lastPeak;
        delay = (maxGain > ssbagc.lastGain131) ? pAlgo->pAgcAC->attackTime_ms88 : pAlgo->pAgcAC->releaseTime_ms88;
        delayBlocks = (10 * delay * 3) / (64 * SSBAGC_FILTER_LENGTH);
        if (delayBlocks < 1) {
            delayBlocks = 1;
        }
        ssbagc.lastGain131 += (maxGain - ssbagc.lastGain131) / delayBlocks;
        if (ssbagc.lastGain131 > maxGain) {
            ssbagc.lastGain131 = maxGain;
        }

        pAlgo->rssi = (ssbagc.lastPeak >> 16) * (ssbagc.lastPeak >> 16);
    }

    return sx;
}



/* Process one complex input sample of the SSB demodulator */
int FIFIDSP_demodSsb (FIFIDSP_AlgoContext *pContext, int32_t sx, int32_t sqraw, int16_t *pOut)
{
    (void) sqraw;
    int polyPhase;
    int32_t temp;


    polyPhase = FIFIDSP_runChannelFilter(sx, pContext->pCoeff);

    /* Check if there's a new sample (last polyphase branch) */
    if (polyPhase == SSB_DECIM_RATE - 1) {
        /* Select sideband.
         * NOTE: The filter coefficients have been chosen such that channelFilterSum[n]
         *       has format S.30, and both filter branches can be added/subtract to form
         *       the LSB/USB signal in format S.31 without the risk of clipping.
         */
        if (pContext->mode == FIFIDSP_MODE_LSB) {
            sx = channelFilterSum[0] + channelFilterSum[1];
        }
        else {
            sx = channelFilterSum[0] - channelFilterSum[1];
        }
        channelFilterSum[0] = 0;
        channelFilterSum[1] = 0;

        sx = FIFIDSP_runSsbAgc(pContext, sx);           /* AGC */

        __asm volatile (
            "smull  %[prodl], %[sx], %[sx], %[x]\n\t"   /* *= gain (0dB = 2^31) */
            : [sx]"+r" (sx), [prodl]"=&r" (temp)
            : [x]"r" (pContext->gain131)
        );

        sx = __SSAT(sx / 65536, 16);                    /* Saturate (16 bits) */
        FIFIDSP_interpolate4(sx, pOut);                 /* Upsample to 48k */

        return 4;
    }

    return 0;
}

