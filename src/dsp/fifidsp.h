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


#ifndef __FIFIDSP_H__
#define __FIFIDSP_H__

#include "lpclib.h"
#include "usbuser.h"

/** \defgroup FIFISDR_DSP_Public_Types Types, enums, macros
 *  @{
 */


/** Handle for the DSP. */
typedef struct FIFIDSP_Context *FIFIDSP_Handle;

/** Format of input (I/Q) samples */
typedef void FIFIDSP_InputSample;                       /* Interpreted by FIFIDSP_write() */
typedef int16_t FIFIDSP_OutputSample;


/** AGC mode. */
typedef enum FIFIDSP_AgcMode {
    FIFIDSP_AGC_OFF = 0,
    FIFIDSP_AGC_SUPERFAST = 1,
    FIFIDSP_AGC_FAST = 2,
    FIFIDSP_AGC_SLOW = 3,
    FIFIDSP_AGC_USER = 4,
    FIFIDSP_AGC_MEDIUM = 5,
    FIFIDSP_AGC_AUTO = 6,
} FIFIDSP_AgcMode;


/** Detailed AGC parameters.
 *  These are the parameters that can be adjusted through USB audio class
 *  (Processing Unit: Dynamic Range Compressor)
 */
typedef struct FIFIDSP_AgcAudioClass {
    bool enable;
    int16_t compressionRatio_dB88;
    int16_t maxAmplitude_dB88;
    int16_t threshold_dB88;
    uint16_t attackTime_ms88;
    uint16_t releaseTime_ms88;
} FIFIDSP_AgcAudioClass;


/** Demodulation mode. */
typedef enum FIFIDSP_Modulation {
    FIFIDSP_MODE_LSB,
    FIFIDSP_MODE_USB,
    FIFIDSP_MODE_AM,
    FIFIDSP_MODE_FM,
    FIFIDSP_MODE_WBFM,
} FIFIDSP_Modulation;


/** Opcodes to specify the configuration command in a call to \ref FIFIDSP_ioctl. */
typedef enum FIFIDSP_Opcode {
    FIFIDSP_OPCODE_SET_MODE,                /**< Select modulation */
    FIFIDSP_OPCODE_GET_MODE,                /**< Get current modulation */
    FIFIDSP_OPCODE_SET_AGC,                 /**< Select AGC mode */
    FIFIDSP_OPCODE_GET_AGC,                 /**< Get current AGC mode */
    FIFIDSP_OPCODE_SET_AGCAC,               /**< Set AGC audio class parameters */
    FIFIDSP_OPCODE_GET_AGCAC,               /**< Get current AGC audio class settings */
    FIFIDSP_OPCODE_SET_VOLUME,              /**< Set audio volume */
    FIFIDSP_OPCODE_SET_INPUT_RATE,          /**< Input signal sample rate */
    FIFIDSP_OPCODE_SET_OUTPUT_RATE,         /**< Output signal sample rate */
    FIFIDSP_OPCODE_SET_BANDWIDTH,           /**< Filter bandwidth */
    FIFIDSP_OPCODE_GET_BANDWIDTH,           /**< Get current filter bandwidth */
    FIFIDSP_OPCODE_SET_SAMPLESIZE,          /**< Set input sample size (16 or 32) */
} FIFIDSP_Opcode;


/** Descriptor to specify the configuration in a call to \ref FIFIDSP_ioctl. */
typedef struct FIFIDSP_Config {
    FIFIDSP_Opcode opcode;                  /**< Config action opcode */

    union {
        FIFIDSP_Modulation mode;            /**< Modulation (LSB, USB, AM, ...) */
        FIFIDSP_Modulation *pMode;
        FIFIDSP_AgcMode agc;                /**< AGC mode (OFF, Fast, Slow, ...) */
        FIFIDSP_AgcMode *pAgc;
        FIFIDSP_AgcAudioClass agcAC;        /**< AGC parameters from USB audio class */
        FIFIDSP_AgcAudioClass *pAgcAC;
        int32_t gain131;                    /**< Audio volume. format 1.31 */
        uint32_t sampleRate;                /**< Input/output sample rate */
        uint32_t bandwidth;                 /**< Filter bandwidth */
        uint32_t *pBandwidth;
        int sampleSize;                     /**< Size of input samples (16 or 32) */
    };
} FIFIDSP_Config;


/** Demodulator that takes one complex baseband as input. */
struct FIFIDSP_AlgoContext;
typedef int (* FIFIDSP_RunAlgorithm)(struct FIFIDSP_AlgoContext *pContext, int32_t sx, int32_t sqraw, int16_t *pOut);

/** Context for a demodulator algorithm. */
typedef struct FIFIDSP_AlgoContext {
    int16_t *demodBuffer;                   /**< Output buffer */
    int demodBufferSize;                    /**< Max number of samples in output buffer */
    int rssi;                               /**< RSSI value. Unit?? TODO */
    int centerOffset;                       /**< Measured center frequency offset. Unit?? TODO */
    int numDemodSamples;                    /**< Number of produced output samples */
    FIFIDSP_RunAlgorithm process;           /**< Handler */
    int32_t gain131;                        /**< Gain factor, format signed 1.31 */
    uint32_t bandwidth;                     /**< Filter bandwidth */
    FIFIDSP_Modulation mode;                /**< Selected modulation */
    const void *pCoeff;                     /**< Pointer to coefficient table */
    FIFIDSP_AgcMode agc;                    /**< Selected AGC mode */
    FIFIDSP_AgcAudioClass *pAgcAC;          /**< Pointer to AGC parameters from USB audio class */
} FIFIDSP_AlgoContext;


/** @} Types, enums, macros */


/** \defgroup FIFISDR_DSP_Public_Functions FiFi-SDR DSP API Functions
 *  @{
 */


/** Open the DSP.
 *
 *  \param[out] pHandle DSP handle
 *  \retval LPCLIB_SUCCESS Success. \ref pHandle contains a valid handle.
 */
LPCLIB_Result FIFIDSP_open (FIFIDSP_Handle *pHandle);


/** Close the DSP.
 *
 *  \param[inout] pHandle DSP handle
 *  \retval LPCLIB_SUCCESS Success.
 */
LPCLIB_Result FIFIDSP_close (FIFIDSP_Handle *pHandle);


/** Configure the DSP.
 *
 *  \param[in] handle DSP handle
 *  \param[in] pConfig Pointer to a configuration descriptor
 */
LPCLIB_Result FIFIDSP_ioctl (FIFIDSP_Handle handle, const FIFIDSP_Config *pConfig);


/** Process samples.
 *
 *  \param[in] handle DSP handle
 *  \param[in] ...
 *  \retval LPCLIB_SUCCESS ok
 */
LPCLIB_Result FIFIDSP_write (FIFIDSP_Handle handle, const FIFIDSP_InputSample *data, int numSamples);


/** Read output samples.
 *
 */
LPCLIB_Result FIFIDSP_read (FIFIDSP_Handle handle, FIFIDSP_OutputSample *data, int *pNumSamples);


/** Interpolate existing output samples for use with DAC.
 *
 *  \param interpolFactor Factor by which to interpolate (currently supported: 4)
 */
LPCLIB_Result FIFIDSP_interpolateToDac (FIFIDSP_Handle handle, int interpolFactor, DAC_Sample *data, int *pNumSamples);


/** Interpolate one sample by factor 4.
 */
void FIFIDSP_interpolate4 (int16_t sx, int16_t *pOut);


/** Return current RSSI
 */
int FIFIDSP_getRssi (FIFIDSP_Handle handle);


/** @} FiFi-SDR DSP API Functions */


/** \defgroup FIFISDR_DSP_Algorithms FiFi-SDR DSP Algorithms
 *  @{
 */

void FIFIDSP_initAm (FIFIDSP_AlgoContext *pContext);
int FIFIDSP_demodAm (FIFIDSP_AlgoContext *pContext, int32_t sx, int32_t sqraw, int16_t *pOut);
void FIFIDSP_initFm (FIFIDSP_AlgoContext *pContext);
int FIFIDSP_demodFm (FIFIDSP_AlgoContext *pContext, int32_t sx, int32_t sqraw, int16_t *pOut);
void FIFIDSP_initSsb (FIFIDSP_AlgoContext *pContext);
int FIFIDSP_demodSsb (FIFIDSP_AlgoContext *pContext, int32_t sx, int32_t sqraw, int16_t *pOut);
void FIFIDSP_initWbFm (FIFIDSP_AlgoContext *pContext);
int FIFIDSP_demodWbFm (FIFIDSP_AlgoContext *pContext, int32_t si, int32_t sq, int16_t *pOut);

extern int32_t channelFilterSum[2];
int FIFIDSP_runChannelFilter (int32_t sx, const int32_t *pCoeff);
extern int32_t channelFilterSumFM[2];
int FIFIDSP_runChannelFilterFM (int32_t sx, const int16_t *pCoeff);
extern int32_t nfFilterSum[1];
int FIFIDSP_runNFFilterFM (int32_t sx, const int16_t *pCoeff);

/** Take a new sample, and update the moving average DC extraction filter.
 *
 *  \param[in sx New sampleRate
 *  \return Output of DC filter
 */
int32_t FIFIDSP_runDcFilter (int32_t sx);

int32_t FIFIDSP_audioCompressor(FIFIDSP_AlgoContext *pContext, int32_t sx);

/** @} FiFi-SDR DSP Algorithms */

#endif


