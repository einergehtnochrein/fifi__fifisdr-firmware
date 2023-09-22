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
 *  The signal is then filtered with a complex polyphase filter with decimation rate R=4.
 *  The output is a complex sample after every fourth call of this function.
 *
 *  A 12-kHz oscillator at fs=48 kHz can be represented by the simple series [0, 1, 0, -1].
 *  For a complex oscillator, the 90° phase shifted signal is also very simple: [1, 0, -1, 0].
 *  This oscillator leads to every other sample in each filter branch being zero. Together
 *  with the selection of R=4, only one filter branch (I or Q) needs to be calculated for
 *  each poly phase, leading to a drastically reduced computation effort.
 *
 *  Two further implementation details speed up the filter calculations:
 *  1. All samples are stored twice: First in the normal tap array (nTaps long), then again in
 *     a copy of the taps right after the first tap array. Since calculation of the filter can
 *     start at any tap, this procedure allows to read all taps linearly without having to check
 *     pointer wrap around at the array bounds.
 *  2. Instead of a software loop, code for filter calculation is copied as often as needed.
 *     That requires all filters to have the same number of taps, and so is not flexible.
 *     However, it is the fastest way to do it, and at least for SSB and AM, channel filters
 *     can reasonably be constructed to be all of the same length.
 */



        .set        N_TAPS, 120
        .set        DECIM_RATE, 2

        .section    ".bss3"
nfTaps:             .space (2 * N_TAPS * 4), 0

        .bss

nfTapIndex:         .long 0

        .global nfFilterSum
nfFilterSum:        .space 4

        .text


        .thumb_func
        .global FIFIDSP_runNFFilterFM


FIFIDSP_runNFFilterFM:          /* (int32_t sx, int16_t *pCoeff) */
        push    {r4-r9}

        /********** Save sample in taps **********/

        /* Next index */
        ldr     r12, =nfTapIndex
        ldr     r3, [r12]
        subs    r3, #1                      /* index-- (we store in descending order) */
        it      mi
        movsmi  r3, #(N_TAPS - 1)
        str     r3, [r12]

        /* Which phase? */
        ands    r9, r3, #1
        rsb     r9, #1                      /* Count _backwards_ from 0...1! */

        /* Push sample into filter taps (create copy) */
        ldr     r8, =nfTaps
        str     r0, [r8, r3, lsl #2]
        add     r2, r3, #N_TAPS
        str     r0, [r8, r2, lsl #2]

        /********** Run the filter **********/

        /* R0: Pointer into filter taps */
        /* R1: Pointer to coefficients */
        /* R2: Working registers for coefficients */
        /* R3: Working registers for taps */
        /* R4: sum */
        /* R8: Tap base address */
        /* R9: oscillator phase */

        /* Prepare */
        adds    r0, r8, r3, lsl #2          /* Point to current tap */
        movs    r12, #2 * (N_TAPS / DECIM_RATE)
        mla     r1, r12, r9, r1             /* Go to start of current phase in coefficient table */
        ldr     r12, =nfFilterSum
        ldr     r4, [r12]                   /* Current sum */

        /* Execute */
        tbh     [pc, r9, lsl #1]
_PhaseJumpNF:
        .hword  (_NF0 - _PhaseJumpNF) / 2
        .hword  (_NF1 - _PhaseJumpNF) / 2

        .align
_NF0:
        adds    r0, #4                          /* Point to oldest sample */
        .rept  N_TAPS / DECIM_RATE
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mla     r4, r2, r3, r4
        .endr
        b       _EndFilterNF

_NF1:
        adds    r0, #3*4                        /* Point to oldest sample */
        .rept  N_TAPS / DECIM_RATE
        ldrsh   r2, [r1], #2                    /* Load coefficient */
        ldr     r3, [r0], #4 * DECIM_RATE       /* Load tap */
        mla     r4, r2, r3, r4
        .endr

_EndFilterNF:

        /* Save results */
        ldr     r12, =nfFilterSum
        str     r4, [r12]                       /* Save filter output */

        mov     r0, r9                          /* Return polyPhase */

        pop     {r4-r9}
        bx      lr


        .end
