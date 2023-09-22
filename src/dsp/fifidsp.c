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
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "fifidsp.h"

extern int32_t FIFIDSP_bandpass12k_48 (int16_t i, int16_t q);


/** Local context. */
typedef struct FIFIDSP_Context {
    uint32_t sampleRateIQ;                  /**< Sample rate of input signal */
    uint32_t sampleRateAudio;               /**< Sample rate of output signal */
    _Bool mute;                             /**< Mute audio output */
    _Bool sampleSize32;                     /** Input sample size is 32 bit */
    int16_t *sampleBuffer;
    int numSamples;
    FIFIDSP_AgcAudioClass agcAC;

    FIFIDSP_AlgoContext algorithm;          /**< A demodulator */
} FIFIDSP_Context;

FIFIDSP_Context fifidsp;

//TODO Buffer size bigger than needed to support debugging
#define DSP_MAX_DEMOD_SAMPLES               50      /* Size of output buffer */
int16_t fifidspDemodSamples[DSP_MAX_DEMOD_SAMPLES] __SECTION(".bss3");



/* Open the DSP. */
LPCLIB_Result FIFIDSP_open (FIFIDSP_Handle *pHandle)
{
    fifidsp.sampleSize32 = false;
    fifidsp.algorithm.demodBuffer = fifidspDemodSamples;
    fifidsp.algorithm.demodBufferSize = DSP_MAX_DEMOD_SAMPLES;

    *pHandle = &fifidsp;

    return LPCLIB_SUCCESS;
}


/* Close the DSP. */
LPCLIB_Result FIFIDSP_close (FIFIDSP_Handle *pHandle)
{
    if ((*pHandle) == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}


/* Configure the DSP. */
LPCLIB_Result FIFIDSP_ioctl (FIFIDSP_Handle handle, const FIFIDSP_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    switch (pConfig->opcode) {
    case FIFIDSP_OPCODE_SET_MODE:
        handle->algorithm.mode = pConfig->mode;
        switch (pConfig->mode) {
        case FIFIDSP_MODE_LSB:
        case FIFIDSP_MODE_USB:
            handle->algorithm.process = FIFIDSP_demodSsb;
            FIFIDSP_initSsb(&handle->algorithm);
            break;
        case FIFIDSP_MODE_AM:
            handle->algorithm.process = FIFIDSP_demodAm;
            FIFIDSP_initAm(&handle->algorithm);
            break;
        case FIFIDSP_MODE_FM:
            handle->algorithm.process = FIFIDSP_demodFm;
            FIFIDSP_initFm(&handle->algorithm);
            break;
        case FIFIDSP_MODE_WBFM:
            handle->algorithm.process = FIFIDSP_demodWbFm;
            FIFIDSP_initWbFm(&handle->algorithm);
            break;
        default:
            handle->algorithm.process = NULL;
            break;
        }
        break;

    case FIFIDSP_OPCODE_GET_MODE:
        *(pConfig->pMode) = handle->algorithm.mode;
        break;

    case FIFIDSP_OPCODE_SET_AGC:
        handle->algorithm.agc = pConfig->agc;
        break;

    case FIFIDSP_OPCODE_GET_AGC:
        *(pConfig->pAgc) = handle->algorithm.agc;
        break;

    case FIFIDSP_OPCODE_SET_AGCAC:
        handle->agcAC = pConfig->agcAC;
        handle->algorithm.pAgcAC = &handle->agcAC;
        break;

    case FIFIDSP_OPCODE_GET_AGCAC:
        *(pConfig->pAgcAC) = handle->agcAC;
        break;

    case FIFIDSP_OPCODE_SET_VOLUME:
        handle->algorithm.gain131 = pConfig->gain131;
        break;

    case FIFIDSP_OPCODE_SET_INPUT_RATE:
        handle->sampleRateIQ = pConfig->sampleRate;
        break;

    case FIFIDSP_OPCODE_SET_OUTPUT_RATE:
        handle->sampleRateAudio = pConfig->sampleRate;
        break;

    case FIFIDSP_OPCODE_SET_BANDWIDTH:
        handle->algorithm.bandwidth = pConfig->bandwidth;
        switch (handle->algorithm.mode) {
        case FIFIDSP_MODE_LSB:
        case FIFIDSP_MODE_USB:
            FIFIDSP_initSsb(&handle->algorithm);
            break;
        case FIFIDSP_MODE_AM:
            FIFIDSP_initAm(&handle->algorithm);
            break;
        case FIFIDSP_MODE_FM:
            FIFIDSP_initFm(&handle->algorithm);
            break;
        case FIFIDSP_MODE_WBFM:
            FIFIDSP_initWbFm(&handle->algorithm);
            break;
        }
        break;

    case FIFIDSP_OPCODE_GET_BANDWIDTH:
        *(pConfig->pBandwidth) = handle->algorithm.bandwidth;
        break;

    case FIFIDSP_OPCODE_SET_SAMPLESIZE:
        handle->sampleSize32 = (pConfig->sampleSize == 32);
        break;
    }

    return LPCLIB_SUCCESS;
}



/* Process samples. */
LPCLIB_Result FIFIDSP_write (FIFIDSP_Handle handle, const FIFIDSP_InputSample *data, int numSamples)
{
    int32_t si, sq;                         /* Complex samples */
    int32_t sx;                             /* Real sample */
    int i;
    int nDemodSamples;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* No support for 96k sample rate. */
    if (handle->sampleRateIQ == 96000) {
        for (i = 0; i < (int)handle->sampleRateAudio / 1000; i++) {
            handle->algorithm.demodBuffer[i] = 0;
        }
        handle->algorithm.numDemodSamples = handle->sampleRateAudio / 1000;

        return LPCLIB_SUCCESS;
    }

    while (numSamples) {
        --numSamples;

        /* Get next pair of input samples */
        if (handle->sampleSize32) {
            si = ((int32_t *)data)[0] / 65536;
            sq = ((int32_t *)data)[1] / 65536;
            data = (void *)((uint32_t)data + 8);
        }
        else {
            si = ((int16_t *)data)[0];
            sq = ((int16_t *)data)[1];
            data = (void *)((uint32_t)data + 4);
        }

        if (handle->algorithm.mode == FIFIDSP_MODE_WBFM) {
            if (handle->algorithm.process) {
                nDemodSamples = handle->algorithm.process(&handle->algorithm, si, sq, &handle->algorithm.demodBuffer[handle->algorithm.numDemodSamples]);
                if (nDemodSamples > 0) {
                    handle->algorithm.numDemodSamples += nDemodSamples;
                    if (handle->algorithm.numDemodSamples >= DSP_MAX_DEMOD_SAMPLES) {
                        /* Avoid buffer overflow. (We should never come here... */
                        break;
                    }
                }
            }
        }
        else {
            /* Bandpass filter to select only positive frequencies around 12 kHz */
            sx = FIFIDSP_bandpass12k_48(si, sq);

            if (handle->algorithm.process) {
                nDemodSamples = handle->algorithm.process(&handle->algorithm, sx, 0, &handle->algorithm.demodBuffer[handle->algorithm.numDemodSamples]);
                if (nDemodSamples > 0) {
                    handle->algorithm.numDemodSamples += nDemodSamples;
                    if (handle->algorithm.numDemodSamples >= DSP_MAX_DEMOD_SAMPLES) {
                        /* Avoid buffer overflow. (We should never come here... */
                        break;
                    }
                }
            }
        }
    }

    return LPCLIB_SUCCESS;
}



/* Read output samples. */
LPCLIB_Result FIFIDSP_read (FIFIDSP_Handle handle, FIFIDSP_OutputSample *data, int *pNumSamples)
{
    int i;
    int n;      /* Number of samples we are allowed to copy (size of target buffer) */


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Determine maximum number of sample we can copy */
    n = handle->algorithm.numDemodSamples;
    if (*pNumSamples < n) {
        n = *pNumSamples;
    }

    for (i = 0; i < n; i++) {
        data[i] = handle->algorithm.demodBuffer[i];
    }

    /* Tell caller how much data we have copied */
    *pNumSamples = n;

    handle->algorithm.numDemodSamples -= n;

    return LPCLIB_SUCCESS;
}



/* Interpolate existing output samples for use with DAC. */
LPCLIB_Result FIFIDSP_interpolateToDac (FIFIDSP_Handle handle, int interpolFactor, DAC_Sample *data, int *pNumSamples)
{
#define N_CIC 4                             /* Number of cascaded CIC stages */
    int i, j, k;
    int n;                                  /* Number of samples we are allowed to copy (size of target buffer) */
    static int32_t cic[2 * N_CIC];          /* CIC taps (comb and integrator sections) */
    int32_t sx, sy;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

//TODO DAC = 48k. Use 48k samples directly.
return LPCLIB_SUCCESS;

    /* Determine maximum number of samples we can copy */
    n = handle->algorithm.numDemodSamples * interpolFactor;
    if (*pNumSamples < n) {
        n = *pNumSamples;
    }

    /* Reject unsupported interpolation factors */
    if (interpolFactor != 4) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    for (i = 0; i < n / interpolFactor; i++) {
        sx = handle->algorithm.demodBuffer[i];  /* Eingang mit 12 kHz */

        /* CIC comb */
        for (j = 0; j < N_CIC; j++) {
            sy = sx;
            sx -= cic[j];
            cic[j] = sy;
        }

        /* CIC integrator */
        for (k = 0; k < interpolFactor; k++) {
            sy = sx;
            for (j = 0; j < N_CIC; j++) {
                cic[N_CIC + j] = sy + cic[N_CIC + j];
                sy = cic[N_CIC + j];
            }
            sy = __SSAT(sy/64, 16);  // sy scaling ok?
            data[interpolFactor * i + k] = DAC_convertFromINT16(sy);
        }
    }

    /* Tell caller how much data we have copied */
    *pNumSamples = n;

    return LPCLIB_SUCCESS;
}


/* Table for lin->log conversion */
static const int lin2log[8] = {
    0, 1, 2, 3, 3, 4, 5, 6,
};


/* Calculate the power of a signal in dBFS (6 dB/Bit, 1 dB resolution, negative!) */
static int32_t FIFIDSP_power2dB (uint32_t signal)
{
    int zeros = __CLZ(signal);
    int power;

    /* Coarse result from number of leading zeros */
    power = -6 * (1 + zeros);

    /* Fine result from the first three significant digits */
    power += lin2log[(signal >> (29 - zeros)) & 7];

    return power;
}



/* Return current RSSI. */
int FIFIDSP_getRssi (FIFIDSP_Handle handle)
{
    int rssi = -200;

    switch (handle->algorithm.mode) {
    case FIFIDSP_MODE_LSB:
    case FIFIDSP_MODE_USB:
        rssi = FIFIDSP_power2dB(handle->algorithm.rssi);
        break;
    case FIFIDSP_MODE_AM:
        rssi = FIFIDSP_power2dB(handle->algorithm.rssi);
        break;
    case FIFIDSP_MODE_FM:
        rssi = FIFIDSP_power2dB(handle->algorithm.rssi);
        break;
    case FIFIDSP_MODE_WBFM:
        rssi = 0;
        break;
    }

    return rssi;
}

