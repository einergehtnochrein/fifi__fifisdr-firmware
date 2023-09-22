/* Copyright (c) 2011-2014, DF9DQ
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
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY

 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

        .syntax unified


        .text

        /* Filter coefficients */
        .include    "hfa.inc"

        .bss

        /* Align data section to 4 bytes (2^2) */
        .align      2

        /* Anti-aliasing filter. Required size:
         * (Number of taps = N_HFA) * (2 channels I/Q) * (sizeof(int32) = 4).
         * There's a copy of this ring buffer, so we need twice the size:
         * N_HFA * 2 * 4 * 2 bytes.
         * Having a copy of the ring buffer allows to always read the complete buffer linearly!
         */
bpfilter32:         .space (N_HFA * 2 * 4 * 2), 0
        /* Write index into the filter. This is where the next sample goes. */
bpidx:              .word 0


        .text

        .thumb_func
        .global FIFIDSP_bandpass12k_48
FIFIDSP_bandpass12k_48:
        push    {r4-r8,lr}

        /* Get filter write pointer */
        ldr     r12, =bpidx
        ldr     r2, [r12]
        ldr     r3, =bpfilter32
        add     r3, r3, r2, lsl #3

        add     r2, #1                      /* bpidx++ */
        cmp     r2, #N_HFA
        it      ge
        movsge  r2, #0
        str     r2, [r12]

        /* Input samples are 32-bit samples. */
        strd    r0, r1, [r3]                /* Store in ring buffer (filter tap) */
        strd    r0, r1, [r3, #2*4*N_HFA]    /* Store a copy */

        /* Calculate filter */
        add     r3, #(2 * 4)                /* Point to next (=oldest!) sample */
        ldr     r12, =hfa
        mov     r0, #0                      /* sumhi = 0 */
        mov     r1, #0                      /* sumlo = 0 */

        .rept N_HFA/2
        ldmia   r3!, {r2, r4, r5, r6}       /* Fetch i/q of two samples */
        ldmia   r12!, {r7, r8}              /* Coefficients */
        smlal   r1, r0, r5, r7              /* sum += afilter[i].i * consti[m] */
        smlal   r1, r0, r4, r8              /* sumq += afilter[i].q * constq[m] */
        .endr

        /* Scale to 32 bits */
        lsl     r0, r0, #16
        add     r0, r0, r1, lsr #16

        pop     {r4-r8,lr}

        bx      lr

        .end

