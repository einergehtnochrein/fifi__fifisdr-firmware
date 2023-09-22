/* Copyright (c) 2012, DF9DQ
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
#include "fm10000.h"
#include "fm12500.h"
#include "fm15000.h"
#include "fmnf3500.h"


#if 0
#define DEBUGFMSIZE 200
int32_t debugFm[DEBUGFMSIZE] __SECTION(".bss3");
int debugFmIndex;

static void addDebugFm (int32_t value)
{
    if (debugFmIndex < DEBUGFMSIZE) {
        debugFm[debugFmIndex] = value;
        ++debugFmIndex;
    }
}
#endif


static int32_t getSignalPower (void)
{
    int32_t sx;
    int32_t x;
    int32_t temp;

    __asm volatile (
        /* i^2 + q^2. Remove filter coefficient scaling. */
        "smull  %[temp], %[x], %[ai], %[ai]     \n\t"   /* i^2 */
        "smlal  %[temp], %[x], %[aq], %[aq]     \n\t"   /* i^2 + q^2, format 0.60 */
        "lsl    %[x], %[x], #4                  \n\t"
        "lsr    %[temp], %[temp], #28           \n\t"
        "bfi    %[x], %[temp], #0, #4           \n\t"   /* Format 0.32 */

        /* First guess for sx=sqrt(x) */
        "clz    %[temp], %[x]                   \n\t"
        "rsb    %[temp], %[temp], #32           \n\t"
        "lsr    %[temp], #1                     \n\t"
        "lsr    %[sx], %[x], %[temp]            \n\t"

        /* Three iterations of Newton algorithm */
        "cbz    %[sx], _demodFm1_noSqrt         \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
        "cbz    %[sx], _demodFm1_noSqrt         \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
        "cbz    %[sx], _demodFm1_noSqrt         \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"   /* Format 0.16 */
    "_demodFm1_noSqrt:                          \n\t"

        : [x]"=r" (x), [temp]"=r" (temp),
          [sx]"=l" (sx),    /* Operand of 'cbz' must be a low register */
          [ai]"+r" (channelFilterSumFM[0]), [aq]"+r" (channelFilterSumFM[1])
    );

    return sx;
}



/* Initialize the FM demodulator */
void FIFIDSP_initFm (FIFIDSP_AlgoContext *pContext)
{
    /* NOTE: Hamlib has hardcoded filter bandwidth values for 'narrow', 'normal', 'wide'.
     *       The values are 6000, 9000, 12500 Hz.
     *       We check against these values to determine which filter to use, irrespective of the
     *       real bandwidth of each implemented filter!
     */

    pContext->pCoeff = fmfilter12500;
    if (pContext->bandwidth <= 7000) {
        pContext->pCoeff = fmfilter10000;
    }
    if (pContext->bandwidth >= 10000) {
        pContext->pCoeff = fmfilter15000;
    }

    pContext->centerOffset = 0;
}



/* Process one complex input sample of the FM demodulator */
int FIFIDSP_demodFm (FIFIDSP_AlgoContext *pContext, int32_t sx, int32_t sqraw, int16_t *pOut)
{
    (void) sqraw;
    int polyPhase;
    int32_t si, sq;
    static int32_t last_si, last_sq;
    int32_t dc;

    /* Channel filter */
    polyPhase = FIFIDSP_runChannelFilterFM(sx, pContext->pCoeff);

    /* Check if there's a new sample (last polyphase branch) */
    if (polyPhase == 0) {
        /* Remember filter output (will be modified by getSignalPower()!) */
        si = channelFilterSumFM[0];
        sq = channelFilterSumFM[1];
//addDebugFm(si);
//addDebugFm(sq);

        /* Determine signal power for normalization and RSSI. */
        sx = getSignalPower();                          /* sx = sqrt(i^2 + q^2), format 0.15 */
        if (sx == 0) {
            sx = 1;                                     /* Avoid division by zero */
        }
        pContext->rssi = sx;

        /* Apply normalization (= limiter) */
        si = (2 * si) / sx;                             /* si/sq have format S0.30/0.15 = S0.15 */
        sq = (2 * sq) / sx;

        channelFilterSumFM[0] = 0;
        channelFilterSumFM[1] = 0;

        sx = last_si * sq - last_sq * si;               /* Format S0.30 */
        last_si = si;
        last_sq = sq;

        /* Extremely simplified approximation to asin:
         * asin(x) =
         */
sx = sx / 32768;    // linear...  Format S0.15
sx /= 2;

        /* Extract the DC component to determine center frequency offset */
        dc = FIFIDSP_runDcFilter(sx);
        pContext->centerOffset = (dc * 5) / 7;          /* Convert to Hz */
        sx -= dc;                                       /* Make signal DC free */

        /* NF filter */
        polyPhase = FIFIDSP_runNFFilterFM(sx, fmnffilter3500);

        /* Check if there's a new sample (last polyphase branch) */
        if (polyPhase == 0) {
            /* Return result */
            sx = nfFilterSum[0];
            nfFilterSum[0] = 0;
            sx = __SSAT(sx / 32768, 16);                /* Saturate (16 bits) */
            FIFIDSP_interpolate4(sx, pOut);             /* Upsample to 48k */

            return 4;
        }
    }

    return 0;
}
