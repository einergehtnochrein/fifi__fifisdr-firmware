/* Copyright (c) 2014, DF9DQ
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
//#include "wbfm.h"


static int32_t getSignalPower (int32_t si, int32_t sq)
{
    int32_t sx;
    int32_t x;
    int32_t temp;

    __asm volatile (
        "mul    %[temp], %[ai], %[ai]           \n\t"
        "mla    %[x], %[aq], %[aq], %[temp]     \n\t"

        /* First guess for sx=sqrt(x) */
        "clz    %[temp], %[x]                   \n\t"
        "rsb    %[temp], %[temp], #32           \n\t"
        "lsr    %[temp], #1                     \n\t"
        "lsr    %[sx], %[x], %[temp]            \n\t"

        /* Three iterations of Newton algorithm */
        "cbz    %[sx], _wbfm_sigpwr             \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
        "cbz    %[sx], _wbfm_sigpwr             \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
        "cbz    %[sx], _wbfm_sigpwr             \n\t"
        "udiv   %[temp], %[x], %[sx]            \n\t"
        "add    %[sx], %[temp]                  \n\t"
        "lsr    %[sx], #1                       \n\t"
    "_wbfm_sigpwr:                              \n\t"

        : [x]"=r" (x), [temp]"=r" (temp),
          [sx]"=l" (sx),    /* Operand of 'cbz' must be a low register */
          [ai]"+r" (si), [aq]"+r" (sq)
    );

    return sx;
}



/* Initialize the wideband FM demodulator */
void FIFIDSP_initWbFm (FIFIDSP_AlgoContext *pContext)
{
    pContext->centerOffset = 0;
}



/* Process one complex input sample of the wideband FM demodulator */
int FIFIDSP_demodWbFm (FIFIDSP_AlgoContext *pContext, int32_t si, int32_t sq, int16_t *pOut)
{
    (void) pContext;
    static int polyPhase;
    int32_t sx;
    static int32_t last_si, last_sq;
    int result = 0;


    /* Limit I and Q channels */
    sx = getSignalPower(si, sq);                        /* sx = sqrt(i^2 + q^2), format 0.15 */
    if (sx != 0) {
        si = (32768 * si) / sx;
        sq = (32768 * sq) / sx;
    }

    /* Delay demodulator */
    sx = last_si * sq - last_sq * si;
    last_si = si;
    last_sq = sq;

    /* Compensate for limited bandwidth (192 kHz is less than broadcast FM channel) */
    //TODO can be ~0.6
    sx = sx / 2;

    //TODO: Correction: 1 - sx^2
    //TODO: ASIN(sx / corr)
sx /= 65536;

    /* Decimation filter 192 kHz --> 48 kHz
     * Realized as two halfband filters with decimation ratio 2 each.
     * Halfband filters are realized as polyphase IIR filters (two all-pass branches)
     */
//TODO: optimize
    {
        static int32_t iir1[3];
        static int32_t iir2[3];
        static int32_t iir3[3];
        static int32_t iir4[3];
        int32_t temp;
        const int32_t K01 = (int32_t)(0.0798664464 * (1ll << 15));
        const int32_t K02 = (int32_t)(0.5453236405 * (1ll << 15));
        const int32_t K11 = (int32_t)(0.2838293419 * (1ll << 15));
        const int32_t K12 = (int32_t)(0.8344118932 * (1ll << 15));

        switch (polyPhase) {
        case 0:
            /* First decimation stage, branch A0 */
            temp = iir1[0];
            iir1[0] = sx;
            temp = (K01 * (sx - iir1[1])) / 65536 + temp;
            sx = (K02 * (temp - iir1[2])) / 65536 + iir1[1];
            iir1[1] = temp;
            iir1[2] = sx;
            sx = iir1[2] / 2 + iir2[2] / 2;

            /* Second decimation stage, branch A0 */
            temp = iir3[0];
            iir3[0] = sx;
            temp = (K01 * (sx - iir3[1])) / 65536 + temp;
            sx = (K02 * (temp - iir3[2])) / 65536 + iir3[1];
            iir3[1] = temp;
            iir3[2] = sx;
            *pOut = iir3[2] / 2 + iir4[2] / 2;
            result = 1;

            ++polyPhase;
            break;
        case 1:
            /* First decimation stage, branch A1 */
            temp = iir2[0];
            iir2[0] = sx;
            temp = (K11 * (sx - iir2[1])) / 65536 + temp;
            sx = (K12 * (temp - iir2[2])) / 65536 + iir2[1];
            iir2[1] = temp;
            iir2[2] = sx;

            ++polyPhase;
            break;
        case 2:
            /* First decimation stage, branch A0 */
            temp = iir1[0];
            iir1[0] = sx;
            temp = (K01 * (sx - iir1[1])) / 65536 + temp;
            sx = (K02 * (temp - iir1[2])) / 65536 + iir1[1];
            iir1[1] = temp;
            iir1[2] = sx;
            sx = iir1[2] / 2 + iir2[2] / 2;

            /* Second decimation stage, branch A1 */
            temp = iir4[0];
            iir4[0] = sx;
            temp = (K11 * (sx - iir4[1])) / 65536 + temp;
            sx = (K12 * (temp - iir4[2])) / 65536 + iir4[1];
            iir4[1] = temp;
            iir4[2] = sx;

            ++polyPhase;
            break;
        case 3:
            /* First decimation stage, branch A1 */
            temp = iir2[0];
            iir2[0] = sx;
            temp = (K11 * (sx - iir2[1])) / 65536 + temp;
            sx = (K12 * (temp - iir2[2])) / 65536 + iir2[1];
            iir2[1] = temp;
            iir2[2] = sx;

            polyPhase = 0;
            break;
        }
    }

    return result;
}
