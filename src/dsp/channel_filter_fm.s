/* Copyright (c) 2011, DF9DQ
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

        .syntax unified

/** This function takes as input a real signal centered at 12 kHz, with a sample rate fs=48 kHz.
 *  Multiplication with a complex oscillator (fo=12 kHz) shifts the wanted signal down to f=0,
 *  The signal is then filtered with a complex polyphase filter with decimation rate R=2.
 *  The output is a complex sample after every second call of this function.
 *
 *  A 12-kHz oscillator at fs=48 kHz can be represented by the simple series [0, 1, 0, -1].
 *  For a complex oscillator, the 90° phase shifted signal is also very simple: [1, 0, -1, 0].
 *  This oscillator leads to every other sample in each filter branch being zero. Together
 *  with the selection of R=2, only one filter branch (I or Q) needs to be calculated for
 *  each poly phase, leading to a drastically reduced computation effort.
 *
 *  Two further implementation details speed up the filter calculations:
 *  1. All samples are stored twice: First in the normal tap array (nTaps long), then again in
 *     a copy of the taps right after the first tap array. Since calculation of the filter can
 *     start at any tap, this procedure allows to read all taps linearly without having to check
 *     pointer wrap around at the array bounds.
 *  2. Instead of a software loop, code for filter calculation is copied as often as needed.
 *     That requires all filters to have the same number of taps, and so is not flexible.
 *     However, it is the fastest way to do it, and for FM, channel filters
 *     can reasonably be constructed to be all of the same length.
 */



        .set        N_TAPS, 160
        .set        DECIM_RATE, 2

        .set        N, (N_TAPS + 4)

        .section    ".bss3"
channelTapsFM:      .space (2 * N * 4), 0          /* Real 12 kHz samples (double storage) */

        .bss

channelTapIndexFM:  .long 0

        .global channelFilterSumFM
channelFilterSumFM: .space 2*4

        .text


        .thumb_func
        .global FIFIDSP_runChannelFilterFM


FIFIDSP_runChannelFilterFM:   /* (int32_t sx, int16_t *pCoeff) */
        push    {r4-r9}

        /********** Save sample in taps **********/

        /* Next index */
        ldr     r12, =channelTapIndexFM
        ldr     r3, [r12]
        subs    r3, #1                      /* index-- (we store in descending order) */
        it      mi
        movsmi  r3, #(N - 1)
        str     r3, [r12]

        /* Which phase? */
        ands    r9, r3, #3
        rsb     r9, #3                      /* Count _backwards_ from 0...3! */

        /* Push sample into filter taps (create copy) */
        ldr     r8, =channelTapsFM
        str     r0, [r8, r3, lsl #2]
        add     r2, r3, #N
        str     r0, [r8, r2, lsl #2]

        /********** Run the filter **********/

        /* R0: Pointer into filter taps */
        /* R1: Pointer to coefficients */
        /* R2: Working registers for coefficients */
        /* R3: Working registers for taps */
        /* R4, R6: sums */
        /* R8: Tap base address */
        /* R9: oscillator phase */

        /* Prepare */
        adds    r0, r8, r3, lsl #2          /* Point to current tap */
        movs    r12, #2 * (N_TAPS / DECIM_RATE)
        ands    r4, r9, #1                  /* Get polyphase from oscillator phase */
        mla     r1, r12, r4, r1             /* Go to start of current phase in coefficient table */
        ldr     r12, =channelFilterSumFM
        ldrd    r4, r6, [r12]               /* Current sums */

        /* Execute */
        tbh     [pc, r9, lsl #1]
_OscJumpFM:
        .hword  (_Osc0FM - _OscJumpFM) / 2
        .hword  (_Osc1FM - _OscJumpFM) / 2
        .hword  (_Osc2FM - _OscJumpFM) / 2
        .hword  (_Osc3FM - _OscJumpFM) / 2

        .align
_Osc0FM:
        .rept  (N_TAPS / 4)
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mla     r6, r2, r3, r6
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mls     r6, r2, r3, r6
        .endr
        b       _OscEndFilterFM

_Osc1FM:
        adds    r0, #2*4                        /* Point to previous sample */
        .rept  (N_TAPS / 4)
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mls     r4, r2, r3, r4
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mla     r4, r2, r3, r4
        .endr
        b       _OscEndFilterFM

_Osc2FM:
        .rept  (N_TAPS / 4)
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mls     r6, r2, r3, r6
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mla     r6, r2, r3, r6
        .endr
        b       _OscEndFilterFM

_Osc3FM:
        adds    r0, #2*4                        /* Point to previous sample */
        .rept  (N_TAPS / 4)
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mla     r4, r2, r3, r4
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mls     r4, r2, r3, r4
        .endr

_OscEndFilterFM:

        /* Save results */
        ldr     r12, =channelFilterSumFM
        strd    r4, r6, [r12]                   /* Save filter outputs */

        ands    r0, r9, #1                      /* Return polyPhase */
        rsb     r0, #1

        pop     {r4-r9}
        bx      lr


        .end
