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

#include "fifidsp.h"

/** Filter to extract the DC component of its input signal */
#define DCFILTER_LENGTH (128)
struct {
    int32_t filter[DCFILTER_LENGTH];
    int index;
    int32_t previous;
} dc __SECTION(".bss3");



/* Take a new sample, and update the moving average DC extraction filter. */
int32_t FIFIDSP_runDcFilter (int32_t sx)
{
    int32_t temp;


    temp = dc.filter[dc.index];                         /* Put sample into moving average filter */
    dc.filter[dc.index] = sx;
    ++dc.index;
    if (dc.index >= DCFILTER_LENGTH) {
        dc.index = 0;
    }

    temp = sx - temp;                                   /* Calculate output (dc) of filter */
    temp += dc.previous;
    dc.previous = temp;

    return temp / DCFILTER_LENGTH;
}
