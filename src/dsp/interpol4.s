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

        .syntax unified


        .text

        /* Filter coefficients */
        .include    "interpol4.inc"

        .bss

        /* Interpolation. Required size:
         * (Number of taps = N_INTERPOL4) * (sizeof(int32) = 4).
         * There's a copy of this ring buffer, so we need twice the size:
         * N_INTERPOL4 * 4 * 2 bytes.
         * Having a copy of the ring buffer allows to always read the complete buffer linearly!
         */
ip4filter32:    .space (N_INTERPOL4 * 4 * 2), 0
        /* Write index into the filter. This is where the next sample goes. */
ip4idx:         .word 0


        .text

        .thumb_func
        .global FIFIDSP_interpolate4
FIFIDSP_interpolate4:
        push    {r4-r11,lr}

        /* Get filter write pointer */
        ldr     r12, =ip4idx
        ldr     r2, [r12]
        ldr     r3, =ip4filter32
        add     r3, r3, r2, lsl #2

        add     r2, #1                      /* ip4idx++ */
        cmp     r2, #N_INTERPOL4
        it      ge
        movsge  r2, #0
        str     r2, [r12]

        /* Input samples stored as 32-bit samples. */
        str     r0, [r3]                    /* Store in ring buffer (filter tap) */
        str     r0, [r3, #4*N_INTERPOL4]    /* Store a copy */

        /* Prepare */
        add     r3, #4                      /* Point to next (=oldest!) sample */
        mov     r11, r3
        ldr     r12, =interpol4             /* Coefficients */

        /* Calculate filter and upsample by 4 */

        mov     r0, #0                      /* sum = 0 */
        .rept (N_INTERPOL4/R_INTERPOL4)/4
        ldmia   r3!, {r2, r4, r5, r6}       /* Fetch four samples */
        ldmia   r12!, {r7, r8, r9, r10}     /* Coefficients */
        mla     r0, r2, r7, r0              /* sum += filter[i] * const[m] */
        mla     r0, r4, r8, r0              /* sum += filter[i] * const[m] */
        mla     r0, r5, r9, r0              /* sum += filter[i] * const[m] */
        mla     r0, r6, r10, r0             /* sum += filter[i] * const[m] */
        .endr
        asr     r0, r0, #16                 /* Scale to 16 bits */
        strh    r0, [r1],#2                 /* Store upscaled 16-bit sample */

        mov     r0, #0                      /* sum = 0 */
        mov     r3, r11
        .rept (N_INTERPOL4/R_INTERPOL4)/4
        ldmia   r3!, {r2, r4, r5, r6}       /* Fetch four samples */
        ldmia   r12!, {r7, r8, r9, r10}     /* Coefficients */
        mla     r0, r2, r7, r0              /* sum += filter[i] * const[m] */
        mla     r0, r4, r8, r0              /* sum += filter[i] * const[m] */
        mla     r0, r5, r9, r0              /* sum += filter[i] * const[m] */
        mla     r0, r6, r10, r0             /* sum += filter[i] * const[m] */
        .endr
        asr     r0, r0, #16                 /* Scale to 16 bits */
        strh    r0, [r1],#2                 /* Store upscaled 16-bit sample */

        mov     r0, #0                      /* sum = 0 */
        mov     r3, r11
        .rept (N_INTERPOL4/R_INTERPOL4)/4
        ldmia   r3!, {r2, r4, r5, r6}       /* Fetch four samples */
        ldmia   r12!, {r7, r8, r9, r10}     /* Coefficients */
        mla     r0, r2, r7, r0              /* sum += filter[i] * const[m] */
        mla     r0, r4, r8, r0              /* sum += filter[i] * const[m] */
        mla     r0, r5, r9, r0              /* sum += filter[i] * const[m] */
        mla     r0, r6, r10, r0             /* sum += filter[i] * const[m] */
        .endr
        asr     r0, r0, #16                 /* Scale to 16 bits */
        strh    r0, [r1],#2                 /* Store upscaled 16-bit sample */

        mov     r0, #0                      /* sum = 0 */
        mov     r3, r11
        .rept (N_INTERPOL4/R_INTERPOL4)/4
        ldmia   r3!, {r2, r4, r5, r6}       /* Fetch four samples */
        ldmia   r12!, {r7, r8, r9, r10}     /* Coefficients */
        mla     r0, r2, r7, r0              /* sum += filter[i] * const[m] */
        mla     r0, r4, r8, r0              /* sum += filter[i] * const[m] */
        mla     r0, r5, r9, r0              /* sum += filter[i] * const[m] */
        mla     r0, r6, r10, r0             /* sum += filter[i] * const[m] */
        .endr
        asr     r0, r0, #16                 /* Scale to 16 bits */
        strh    r0, [r1],#2                 /* Store upscaled 16-bit sample */

        /* Scale to 16 bits */

        pop     {r4-r11,lr}

        bx      lr

        .end

