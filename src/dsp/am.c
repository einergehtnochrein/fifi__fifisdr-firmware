/* Copyright (c) 2011-2013, DF9DQ
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
#include "am6200.h"
#include "am8000.h"
#include "am10000.h"


#define AM_DECIM_RATE   4


static int32_t getSignalPower (void)
{
    int32_t sx;
    int32_t x;
    int32_t temp;

    __asm volatile (
#if 0
        /* i^2 + q^2. Remove filter coefficient scaling first. */
        "asr    %[ai], %[ai], #15               \n\t"
        "asr    %[aq], %[aq], #15               \n\t"

        "mul    %[x], %[ai], %[ai]              \n\t"   /* i^2 */
        "mla    %[x], %[aq], %[aq], %[x]        \n\t"   /* + q^2 */
#else
        "smull  %[temp], %[x], %[ai], %[ai]     \n\t"
        "smlal  %[temp], %[x], %[aq], %[aq]     \n\t"
#endif

        /* First guess for sx=sqrt(x) */
        "clz    %[temp], %[x]                   \n\t"
        "rsb    %[temp], %[temp], #32           \n\t"
        "lsr    %[temp], #1                     \n\t"
        "lsr    %[sx], %[x], %[temp]            \n\t"

        /* Three iterations of Newton algorithm */
        "cbz    %[sx], _demodAm48_noSqrt        \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
        "cbz    %[sx], _demodAm48_noSqrt        \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
        "cbz    %[sx], _demodAm48_noSqrt        \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
    "_demodAm48_noSqrt:                         \n\t"

        : [x]"=r" (x), [temp]"=r" (temp),
          [sx]"=l" (sx),    /* Operand of 'cbz' must be a low register */
          [ai]"+r" (channelFilterSum[0]), [aq]"+r" (channelFilterSum[1])
    );

    return sx;
}


/* Initialize the AM demodulator */
void FIFIDSP_initAm (FIFIDSP_AlgoContext *pContext)
{
    pContext->pCoeff = amfilter6200;
    if (pContext->bandwidth > 7000) {
        pContext->pCoeff = amfilter8000;
    }
    if (pContext->bandwidth > 8500) {
        pContext->pCoeff = amfilter10000;
    }
}


/* Process one complex input sample of the AM demodulator */
int FIFIDSP_demodAm (FIFIDSP_AlgoContext *pContext, int32_t sx, int32_t sqraw, int16_t *pOut)
{
    (void) sqraw;
    int polyPhase;
    int32_t dc;


    /* Channel filter */
    polyPhase = FIFIDSP_runChannelFilter(sx, pContext->pCoeff);

    /* Check if there's a new sample (last polyphase branch) */
    if (polyPhase == 0) {
        sx = getSignalPower();                          /* sx = sqrt(i^2 + q^2) */

        channelFilterSum[0] = 0;
        channelFilterSum[1] = 0;

        dc = FIFIDSP_runDcFilter(sx);

        pContext->rssi = dc;                            /* RSSI(AM) = signal power (sqrt(i^2 + q^2)) */

        sx -= dc;                                       /* - DC offset */
        if (dc < 20) {                                  /* Minimum carrier amplitude avoids */
            dc = 20;                                    /* amplification of noise floor */
        }
        sx *= 32000 / dc;                               /* NF amplitude independent of carrier */
        __asm volatile (
            "lsl    %[sx], %[sx], #1\n\t"
            "smull  %[temp], %[sx], %[sx], %[x]\n\t"    /* *= gain (0dB: gain = 2^31) */
            : [sx]"+r" (sx)
            : [x]"r" (pContext->gain131),
              [temp]"r" (0)
        );
        sx = __SSAT(sx, 16);                            /* Saturate (16 bits) */

        FIFIDSP_interpolate4(sx, pOut);                 /* Upsample to 48k */

        return 4;
    }

    return 0;
}
