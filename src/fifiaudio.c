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


#include <string.h>                         /* for memcpy() */

#include "lpclib.h"
#include "bsp-fifisdr.h"

#include "task-sys.h"
#include "usbuser.h"
#include "params.h"

#include "fifiaudio.h"
#include "fifidsp.h"


#define FIFIAUDIO_QUEUE_LENGTH                  10

#define FIFIAUDIO_MAX_DEMOD_SAMPLES             60



typedef struct {
    uint8_t opcode;

    union {
        LPCLIB_Switch enable;
        int32_t numSamples;
        uint32_t sampleRate;
    };
} FIFIAUDIO_Message;


/** Message opcodes for FIFIAUDIO task. */
enum {
    FIFIAUDIO_OPCODE_IQ_ENABLE_16,
    FIFIAUDIO_OPCODE_IQ_ENABLE_32,
    FIFIAUDIO_OPCODE_SET_IQ_SPEED,
    FIFIAUDIO_OPCODE_DEMOD_ENABLE,
    FIFIAUDIO_OPCODE_SET_DEMOD_SPEED,
    FIFIAUDIO_OPCODE_DMA_END,
    FIFIAUDIO_OPCODE_RUN_DSP,
    FIFIAUDIO_OPCODE_COPY_BUFFERSTART,
    FIFIAUDIO_OPCODE_POWERMODE_CHANGE,      /**< System power mode has changed (suspend/resume) */
};


/** State of a streaming interface. */
typedef enum FIFIAUDIO_StreamState {
    FIFIAUDIO_STREAMSTATE_OFF,              /**< Inactive (interface alternate setting = 0) */
    FIFIAUDIO_STREAMSTATE_ARMED,            /**< Armed by interface alternate setting -> 1 */
    FIFIAUDIO_STREAMSTATE_ACTIVE,           /**< Active (after setting endpoint speed) */
} FIFIAUDIO_StreamState;


/** Task context */
static struct {
    osMailQId queue;                    /**< Task message queue */

    I2S_Handle codec;                   /**< I2S interface to codec */
    TIMER_Handle adapter;               /**< Timer for adapting to USB rate */
    uint32_t adapterModulus;            /**< Modulus of high-resolution adapter timer */
    uint32_t lastDmaTimestamp;          /**< Time stamp of last DMA terminal count interrupt */
    _Bool sleeping;                     /**< If set, system is powered down */

    _Bool isActiveCodec;                /**< Codec: active */

    /* Soundcard for I/Q samples */
    int16_t volumeIQ;                   /**< I/Q soundcard: Current volume */
    uint32_t sampleRateIQ;              /**< I/Q soundcard: Current sample rate */
    FIFIAUDIO_StreamState stateIQ;      /**< I/Q soundcard: stream state */
    USB_Buffer buffer;                  /**< USB data buffer */
    _Bool sampleSize32;                 /**< Set if using 32 bit samples */
    _Bool sampleSize32Request;          /**< Use 32 bit samples when enabling the interface */

    /* Soundcard for demodulator */
    int16_t volumeDemod;                /**< Demodulator: current volume */
    _Bool agcDemod;                     /**< Demodulator: AGC on/off */
    uint32_t sampleRateDemod;           /**< Demodulator: current sample rate */
    FIFIAUDIO_StreamState stateDemod;   /**< Demodulator: stream state */
    USB_Buffer demodBuffer;             /**< Demodulator: USB data buffer */
    FIFIDSP_OutputSample demodSamples[FIFIAUDIO_MAX_DEMOD_SAMPLES];

    DMA_ChannelHandle i2sCodecDma;      /**< DMA channel handle for transfer codec->memory */
    DMA_ChannelHandle i2sHelperDma;     /**< DMA channel handle for helper transfer memory->memory */
    DMA_MemcpyContext i2sMemcpy;        /**< Context for using the DMA_memcpy function */
    uint32_t readIndex;

    FIFIDSP_Handle dsp;                 /**< Demodulator function */
    uint32_t dacWriteIndex;             /**< Write index to DAC direct audio buffer */
    uint32_t dacWriteIndexCaptured;     /**< Write index to DAC buffer at time of last DAC DMA event */
} fifiaudio;


/** Magic event numbers used in I2S callback handler. */
enum {
    I2S_MAGIC_BUFFER_COMPLETE,          /**< Wrap-around of buffer DMA */
    I2S_MAGIC_TRIGGER_MEMCPY,           /**< First I2S_NUM_EXTRA_SAMPLES arrived in ring buffer.
                                         *   Triggers memcpy() of these samples.
                                         */
};


/** Volume value that indicates a request for power down */
#define VOLUME_POWERDOWN                    (-9999)


#define I2S_NUM_SAMPLES                     1024
#define I2S_NUM_EXTRA_SAMPLES               193         /* Large enough for 192 kHz sample rate */


/** Ring buffer for audio frames. Large enough for 32-bit samples. */
static uint32_t audioBuffer[2 * (I2S_NUM_SAMPLES + I2S_NUM_EXTRA_SAMPLES)] __SECTION(".bss2");


#define DAC_NUM_SAMPLES                     512
#define DAC_NUM_EXTRA_SAMPLES               64

/** Ring buffer for DAC direct audio output. */
static DAC_Sample dacBuffer[DAC_NUM_SAMPLES + DAC_NUM_EXTRA_SAMPLES] __SECTION(".bss2");


/* Dummy callback handler.
 * Used to make the DMA mempcy an asynchronous function.
 */
static void FIFIAUDIO_dummyMemcpyCallback (LPCLIB_Event event)
{
    (void) event;
    /* Nothing to do */
}



/* USB audio class frame handler. */
void FIFIAUDIO_frameHandler (USB_Handle usbhw, const struct USB_Class *pClass, uint16_t frameNumber)
{
    (void) pClass;
    (void) frameNumber;
    int32_t temp;
    uint32_t expectedReadPos;
    int32_t delta;
    int numSamples;
    FIFIAUDIO_Message *pMessage;


    /* Do nothing if channel isn't active */
    if (!fifiaudio.isActiveCodec) {
        return;
    }

    if (!fifiaudio.queue) {
        return;
    }

    /* Make an estimate of the current write pointer position.
     * The last marker was taken when the write pointer passed the buffer start.
     *
     *                  current_time - marker_time
     * write_position = -------------------------- * buffer_size
     *                           modulus
     *
     * The expected read position should be half a buffer length behind.
     */

    /* Estimated position of write pointer. (Unit: timer ticks) */
    temp = (TIMER_read(fifiaudio.adapter) + fifiaudio.adapterModulus) - fifiaudio.lastDmaTimestamp;
    if ((uint32_t)temp >= fifiaudio.adapterModulus) {
        temp -= fifiaudio.adapterModulus;
    }

    /* Scale to samples (0...I2S_NUM_SAMPLES-1) */
    temp = (temp * I2S_NUM_SAMPLES) / fifiaudio.adapterModulus;

    /* Deviation of read position from expected read position.
     * Range: +/- I2S_NUM_SAMPLES/2, where negative values indicate that read pointer lags behind.
     */
    expectedReadPos = (temp + I2S_NUM_SAMPLES / 2) % I2S_NUM_SAMPLES;

    temp = ((fifiaudio.readIndex + (3 * I2S_NUM_SAMPLES) / 2) - expectedReadPos) % I2S_NUM_SAMPLES
         - I2S_NUM_SAMPLES / 2;

    /* Calculate necessary adjustment. */
    delta = 0;
    if ((temp < -50) || (temp > +50)) {
        /* Too far off. Force sync, and accept losing samples. */
        fifiaudio.readIndex = expectedReadPos;
    }
    else if (temp < -5) {
        /* Internal clock too fast. Long packet (+1 sample) */
        delta = +1;
    }
    else if (temp > +5) {
        /* Internal clock too slow. Short packet (-1 sample) */
        delta = -1;
    }

    if (fifiaudio.stateIQ == FIFIAUDIO_STREAMSTATE_ACTIVE) {
        if (fifiaudio.sampleSize32) {
            fifiaudio.buffer.maxSize = (fifiaudio.sampleRateIQ / 1000ul + delta) * 2 * 2 * 2;
            fifiaudio.buffer.data = &audioBuffer[2 * fifiaudio.readIndex];

            /* Do not try sending ISO packets larger than the endpoint's buffer */
            if (fifiaudio.buffer.maxSize > USBCONFIG_ISOC_SIZE1_32) {
                fifiaudio.buffer.maxSize = USBCONFIG_ISOC_SIZE1_32;
            }
        }
        else {
            fifiaudio.buffer.maxSize = (fifiaudio.sampleRateIQ / 1000ul + delta) * 2 * 2;
            fifiaudio.buffer.data = &audioBuffer[fifiaudio.readIndex];

            /* Do not try sending ISO packets larger than the endpoint's buffer */
            if (fifiaudio.buffer.maxSize > USBCONFIG_ISOC_SIZE1_16) {
                fifiaudio.buffer.maxSize = USBCONFIG_ISOC_SIZE1_16;
            }
        }

        USB_write(usbhw, USBCONFIG_STREAMING_EP1, &fifiaudio.buffer);
    }

    /* Feed demodulator if it is active */
    if (fifiaudio.stateDemod == FIFIAUDIO_STREAMSTATE_ACTIVE) {
        /* Read previous output of demodulator */
        numSamples = FIFIAUDIO_MAX_DEMOD_SAMPLES;       /* Max. #samples allowed */
        FIFIDSP_read(fifiaudio.dsp,
                    fifiaudio.demodBuffer.data,
                    &numSamples);

        /* Send via USB */
        fifiaudio.demodBuffer.maxSize = numSamples * sizeof(FIFIDSP_OutputSample);
        /* Do not try sending ISO packets larger than the endpoint's buffer */
        if (fifiaudio.demodBuffer.maxSize > USBCONFIG_ISOC_SIZE2) {
            fifiaudio.demodBuffer.maxSize = USBCONFIG_ISOC_SIZE2;
        }
        USB_write(usbhw, USBCONFIG_STREAMING_EP2, &fifiaudio.demodBuffer);

        pMessage = osMailAlloc(fifiaudio.queue, 0);
        if (pMessage) {
            pMessage->opcode = FIFIAUDIO_OPCODE_RUN_DSP;
            pMessage->numSamples = fifiaudio.sampleRateIQ / 1000ul + delta;
            osMailPut(fifiaudio.queue, pMessage);
        }
    }

    /* Update read pointer */
    fifiaudio.readIndex += fifiaudio.sampleRateIQ / 1000ul + delta;
    fifiaudio.readIndex %= I2S_NUM_SAMPLES;
}



/** Set DSP gain. */
static void FIFIAUDIO_setDemodVolume (int16_t volume)
{
    int32_t temp;
    int32_t volume131;
    FIFIDSP_Config config;

    /* Get the gain factor (format signed 1.31) from the volume in dB. */
    temp = volume / 256;                        /* Ignore dB fractions */
    if (temp > 0) {                             /* Max volume = 0 dB */
        temp = 0;
    }
    volume131 = 0x7FFFFFFF;                     /* (= 0 dB) */
    while (temp <= -6) {
        volume131 >>= 1;                        /* -6 dB */
        temp += 6;
    }
    while (temp < 0) {
        volume131 = (volume131 / 9) * 8;        /* (close to) 1 dB */
        temp += 1;
    }
    config.opcode = FIFIDSP_OPCODE_SET_VOLUME;
    config.gain131 = volume131;
    FIFIDSP_ioctl(fifiaudio.dsp, &config);
}



/* Handle events from generic audio class. */
void FIFIAUDIO_handleEventIQ (LPCLIB_Event event)
{
    FIFIAUDIO_Message *pMessage;
    FIFIDSP_Config dspConfig;


    if (!fifiaudio.queue) {
        return;
    }

    switch (event.opcode) {
    case USBAUDIO_EVENT_INTERFACE_CHANGE:
        if (event.block == USBCONFIG_INTERFACE_AUDIO_STREAMING_1) {
            pMessage = osMailAlloc(fifiaudio.queue, 0);
            if (pMessage) {
                pMessage->opcode = FIFIAUDIO_OPCODE_IQ_ENABLE_16;
                if (event.channel == 1) {
                    pMessage->opcode = FIFIAUDIO_OPCODE_IQ_ENABLE_32;
                }
                pMessage->enable = (event.channel != 0);      /* Non-zero alternate settings enable codec */
                osMailPut(fifiaudio.queue, pMessage);
            }
        }
        break;

    case USBAUDIO_EVENT_SET_CONTROL:
        if ((event.block == USBCONFIG_UNIT_FEATURE_IQ) &&
            (event.channel == USBAC_CS_FU_VOLUME_CONTROL)) {
            fifiaudio.volumeIQ = (int32_t)event.parameter;  /* Extract volume (16 bit) from event */
            BSP_setCodecVolume(fifiaudio.volumeIQ, fifiaudio.sampleRateIQ, fifiaudio.sampleSize32);
        }
        break;

    case USBAUDIO_EVENT_ENDPOINT:
        if ((event.block == USBCONFIG_STREAMING_EP1) &&
            (event.channel == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
            pMessage = osMailAlloc(fifiaudio.queue, 0);
            if (pMessage) {
                pMessage->opcode = FIFIAUDIO_OPCODE_SET_IQ_SPEED;
                pMessage->sampleRate = (uint32_t)event.parameter;
                osMailPut(fifiaudio.queue, pMessage);
            }

            dspConfig.opcode = FIFIDSP_OPCODE_SET_INPUT_RATE;
            dspConfig.sampleRate = (uint32_t)event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
        }
        break;
    }
}


/* Handle events from generic audio class. */
void FIFIAUDIO_handleEventDemod (LPCLIB_Event event)
{
    FIFIAUDIO_Message *pMessage;
    FIFIDSP_Config dspConfig;
    FIFIDSP_Modulation mode;
    FIFIDSP_AgcAudioClass agcAC;
    int temp;
    int rssi;


    if (!fifiaudio.queue) {
        return;
    }

    if (event.id == LPCLIB_EVENTID_USBAUDIO) {
        switch (event.opcode) {
        case USBAUDIO_EVENT_INTERFACE_CHANGE:
            if (event.block == USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING) {
                pMessage = osMailAlloc(fifiaudio.queue, 0);
                if (pMessage) {
                    pMessage->opcode = FIFIAUDIO_OPCODE_DEMOD_ENABLE;
                    pMessage->enable = (event.channel == 1);  /* Alternate setting = 1 enables codec */
                    osMailPut(fifiaudio.queue, pMessage);
                }
            }
            break;

        case USBAUDIO_EVENT_SET_CONTROL:
            switch (event.block) {
            case USBCONFIG_UNIT_FEATURE_DSP_AM:
            case USBCONFIG_UNIT_FEATURE_DSP_FM:
            case USBCONFIG_UNIT_FEATURE_DSP_LSB:
            case USBCONFIG_UNIT_FEATURE_DSP_USB:
                /* NOTE: All volumes are the same! They exist as separate units because of
                 *       limitations in the way Linux and Windows handle USB audio function topology.
                 * TODO: Send notification to host. Otherwise the volumes get out of sync!
                 */
                if (event.channel == USBAC_CS_FU_VOLUME_CONTROL) {
                    fifiaudio.volumeDemod = (int32_t)event.parameter;   /* Extract volume (16 bit) from event */
                    FIFIAUDIO_setDemodVolume(fifiaudio.volumeDemod);
                }
                break;
            }
            if ((event.block == USBCONFIG_UNIT_SELECTOR_DSP) && (event.channel == 0)) {
                dspConfig.opcode = FIFIDSP_OPCODE_SET_MODE;
                switch ((uint32_t)event.parameter) {
                case 2:
                    dspConfig.mode = FIFIDSP_MODE_FM;
                    break;
                case 3:
                    dspConfig.mode = FIFIDSP_MODE_LSB;
                    break;
                case 4:
                    dspConfig.mode = FIFIDSP_MODE_USB;
                    break;
                default:
                    dspConfig.mode = FIFIDSP_MODE_AM;
                    break;
                }
                FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            }
            if (event.block == USBCONFIG_UNIT_PROCESSING_DSP) {
                /* Get complete set of audio class AGC settings. */
                dspConfig.opcode = FIFIDSP_OPCODE_GET_AGCAC;
                dspConfig.pAgcAC = &agcAC;
                FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);

                switch (event.channel) {
                case USBAC_CS_PU_DR_ENABLE_CONTROL:
                    agcAC.enable = (uint32_t)event.parameter ? 1 : 0;
                    break;
                case USBAC_CS_PU_COMPRESSION_RATIO_CONTROL:
                    agcAC.compressionRatio_dB88 = (int32_t)event.parameter;
                    break;
                case USBAC_CS_PU_MAXAMPL_CONTROL:
                    agcAC.maxAmplitude_dB88 = (int32_t)event.parameter;
                    break;
                case USBAC_CS_PU_THRESHOLD_CONTROL:
                    agcAC.threshold_dB88 = (int32_t)event.parameter;
                    break;
                case USBAC_CS_PU_ATTACK_TIME:
                    agcAC.attackTime_ms88 = (uint32_t)event.parameter;
                    break;
                case USBAC_CS_PU_RELEASE_TIME:
                    agcAC.releaseTime_ms88 = (uint32_t)event.parameter;
                    break;
                }

                dspConfig.opcode = FIFIDSP_OPCODE_SET_AGCAC;
                dspConfig.agcAC = agcAC;
                FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            }
            break;

        case USBAUDIO_EVENT_ENDPOINT:
            if ((event.block == USBCONFIG_STREAMING_EP2) &&
                (event.channel == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
                pMessage = osMailAlloc(fifiaudio.queue, 0);
                if (pMessage) {
                    pMessage->opcode = FIFIAUDIO_OPCODE_SET_DEMOD_SPEED;
                    pMessage->sampleRate = (uint32_t)event.parameter;
                    osMailPut(fifiaudio.queue, pMessage);
                }

                dspConfig.opcode = FIFIDSP_OPCODE_SET_OUTPUT_RATE;
                dspConfig.sampleRate = (uint32_t)event.parameter;
                FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            }
            break;
        }
    }

    if (event.id == LPCLIB_EVENTID_APPLICATION) {
        switch (event.opcode) {
        case FIFISDR_EVENT_DEMOD_SET_MODE:
            dspConfig.opcode = FIFIDSP_OPCODE_SET_MODE;
            dspConfig.mode = (uint32_t)event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            break;

        case FIFISDR_EVENT_DEMOD_GET_MODE:
            dspConfig.opcode = FIFIDSP_OPCODE_GET_MODE;
            dspConfig.pMode = event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            break;

        case FIFISDR_EVENT_DEMOD_SET_BANDWIDTH:
            dspConfig.opcode = FIFIDSP_OPCODE_SET_BANDWIDTH;
            dspConfig.bandwidth = (uint32_t)event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            break;

        case FIFISDR_EVENT_DEMOD_GET_BANDWIDTH:
            dspConfig.opcode = FIFIDSP_OPCODE_GET_BANDWIDTH;
            dspConfig.pBandwidth = event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            break;

        case FIFISDR_EVENT_DEMOD_SET_VOLUME:
            /* Value has range of 0...100 (%). Check limits */
            temp = (int32_t)event.parameter;
            if (temp < 0) {
                temp = 0;
            }
            if (temp > 100) {
                temp = 100;
            }
            /* Transform into dB domain (TODO 0...-60Db is hardcoded) */
            fifiaudio.volumeDemod = 256 * (((temp - 100) * 60) / 100);
            FIFIAUDIO_setDemodVolume(fifiaudio.volumeDemod);
            break;

        case FIFISDR_EVENT_DEMOD_GET_VOLUME:
            temp = (((fifiaudio.volumeDemod / 256) + 60) * 100) / 60;
            if (temp < 0) {
                temp = 0;
            }
            if (temp > 100) {
                temp = 100;
            }
            *((int32_t *)event.parameter) = temp;
            break;

        case FIFISDR_EVENT_DEMOD_SET_PREAMP:
            //TODO
            break;

        case FIFISDR_EVENT_DEMOD_GET_PREAMP:
            *((int32_t *)event.parameter) = fifiaudio.volumeIQ;
            break;

        case FIFISDR_EVENT_DEMOD_SET_AGCTEMPLATE:
            dspConfig.opcode = FIFIDSP_OPCODE_SET_AGC;
            dspConfig.agc = (uint32_t)event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            break;

        case FIFISDR_EVENT_DEMOD_GET_AGCTEMPLATE:
            dspConfig.opcode = FIFIDSP_OPCODE_GET_AGC;
            dspConfig.pAgc = event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
            break;

        case FIFISDR_EVENT_DEMOD_GET_RSSI:
            dspConfig.opcode = FIFIDSP_OPCODE_GET_MODE;
            dspConfig.pMode = &mode;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);

            rssi = -9*6;    /* (0 = S9) --> -54 = S0 */
            switch (mode) {
            case FIFIDSP_MODE_LSB:
            case FIFIDSP_MODE_USB:
                temp = FIFIDSP_getRssi(fifiaudio.dsp);
                rssi = 132 + temp;
                break;
            case FIFIDSP_MODE_AM:
            case FIFIDSP_MODE_FM:
                temp = FIFIDSP_getRssi(fifiaudio.dsp);
                rssi = 132 + temp;
                break;
            case FIFIDSP_MODE_WBFM:
                rssi = 0;
                break;
            }
            *((int32_t *)event.parameter) = rssi;
            break;

        case FIFISDR_EVENT_DEMOD_GET_FMCENTER:
            *((int32_t *)event.parameter) = 0;
            //TODO
            break;
        }
    }
}



LPCLIB_Result usbaudio_handleEndpointData (
                    USBDEV_Handle handle,
                    const struct USB_Class *pClass,
                    uint8_t logicalEndpoint)
{
    (void) handle;
    (void) pClass;
    (void) logicalEndpoint;

    /* Check for returned buffers for iso endpoints */
    ;
    ;
    ;

    /* Check for sync endpoints */
    ;
    ;
    ;

    return LPCLIB_ILLEGAL_PARAMETER;
}



/** Callback for I2S phase. */
static void FIFIAUDIO_i2sCallback (LPCLIB_Event event)
{
    FIFIAUDIO_Message *pMessage;


    if (!fifiaudio.queue) {
        return;
    }

    /* Take a time stamp every time a buffer wrap-around occurs. */
    if ((uint32_t)event.parameter == I2S_MAGIC_BUFFER_COMPLETE) {
        /* Take a time stamp */
        fifiaudio.lastDmaTimestamp = TIMER_read(fifiaudio.adapter);

        /* Inform task about end of DMA transfer */
        if (event.opcode == I2S_EVENT_JOB_COMPLETE) {
            pMessage = osMailAlloc(fifiaudio.queue, 0);
            if (pMessage) {
                pMessage->opcode = FIFIAUDIO_OPCODE_DMA_END;
                osMailPut(fifiaudio.queue, pMessage);
            }
        }
    }

    /* After the first I2S_NUM_EXTRA_SAMPLES have arrived, make a copy of these samples
     * beyond the end of the ring buffer.
     */
    if ((uint32_t)event.parameter == I2S_MAGIC_TRIGGER_MEMCPY) {
        pMessage = osMailAlloc(fifiaudio.queue, 0);
        if (pMessage) {
            pMessage->opcode = FIFIAUDIO_OPCODE_COPY_BUFFERSTART;
            osMailPut(fifiaudio.queue, pMessage);
        }
    }
}



static void FIFIAUDIO_dacCallback (LPCLIB_Event event)
{
    (void) event;

    /* Remember state of write pointer when DMA begin at buffer start. */
    fifiaudio.dacWriteIndexCaptured = fifiaudio.dacWriteIndex;
}



static const TIMER_Config adapterConfig[] = {
    {.opcode = TIMER_OPCODE_MODE,
     {.mode = {.mode = TIMER_MODE_TIMER, }}},

    {.opcode = TIMER_OPCODE_CONFIG_MATCH,
     {.match = {
         .channel = TIMER_MATCH0,
         .intOnMatch = DISABLE,
         .resetOnMatch = ENABLE,
         .stopOnMatch = DISABLE,
         .function = TIMER_MATCH_OUTPUT_NONE,
         .pwm = DISABLE, }}},

    TIMER_CONFIG_END
};


static const I2S_JobPhase codecJobPhases16[2] = {
    {.next = &codecJobPhases16[1],
     .data = &audioBuffer[0],
     .length = I2S_NUM_EXTRA_SAMPLES,
     .triggerCallback = LPCLIB_YES,
     .magic = (void *)I2S_MAGIC_TRIGGER_MEMCPY, },
    {.next = &codecJobPhases16[0],
     .data = &audioBuffer[I2S_NUM_EXTRA_SAMPLES],
     .length = I2S_NUM_SAMPLES - I2S_NUM_EXTRA_SAMPLES,
     .triggerCallback = LPCLIB_YES,
     .magic = (void *)I2S_MAGIC_BUFFER_COMPLETE, },
};

static const I2S_JobPhase codecJobPhases32[2] = {
    {.next = &codecJobPhases32[1],
     .data = &audioBuffer[0],
     .length = 2 * I2S_NUM_EXTRA_SAMPLES,
     .triggerCallback = LPCLIB_YES,
     .magic = (void *)I2S_MAGIC_TRIGGER_MEMCPY, },
    {.next = &codecJobPhases32[0],
     .data = &audioBuffer[2 * I2S_NUM_EXTRA_SAMPLES],
     .length = 2 * (I2S_NUM_SAMPLES - I2S_NUM_EXTRA_SAMPLES),
     .triggerCallback = LPCLIB_YES,
     .magic = (void *)I2S_MAGIC_BUFFER_COMPLETE, },
};

static const I2S_Job i2sJob16 = {
    .channel = I2S_CHANNEL_RX,
    .callback = FIFIAUDIO_i2sCallback,
    .firstPhase = &codecJobPhases16[0],
};

static const I2S_Job i2sJob32 = {
    .channel = I2S_CHANNEL_RX,
    .callback = FIFIAUDIO_i2sCallback,
    .firstPhase = &codecJobPhases32[0],
};

static const DAC_JobPhase dacJobPhase[] = {
    {.next = &dacJobPhase[0],
     .data = dacBuffer,
     .length = DAC_NUM_SAMPLES,
     .callback = FIFIAUDIO_dacCallback,
     .magic = (void *)0,
    },
};

static const DAC_Job dacJob = {
    .firstPhase = &dacJobPhase[0],
};



/** Start or stop the codec.
 */
static void FIFIAUDIO_runCodec (LPCLIB_Switch enable)
{
    /* Stop the codec? */
    if (!enable && fifiaudio.isActiveCodec) {
        BSP_closeDac();                                 /* Stop local audio output */

        I2S_close(&fifiaudio.codec);                    /* Close the I2S channel */

        BSP_setCodecVolume(BSP_VOLUME_POWERDOWN, fifiaudio.sampleRateIQ, fifiaudio.sampleSize32);

        DMA_releaseChannel(&fifiaudio.i2sHelperDma);
        DMA_releaseChannel(&fifiaudio.i2sCodecDma);
        TIMER_close(&fifiaudio.adapter);

        fifiaudio.isActiveCodec = false;                /* Mark as inactive */
    }

    /* Start the codec? */
    if (enable && !fifiaudio.isActiveCodec) {
        if (fifiaudio.sampleRateIQ == 0) {
            fifiaudio.sampleRateIQ = 48000ul;
        }

        DMA_acquireChannel(gpdma, &fifiaudio.i2sCodecDma);
        DMA_acquireChannel(gpdma, &fifiaudio.i2sHelperDma);

        BSP_setCodecVolume(fifiaudio.volumeIQ, fifiaudio.sampleRateIQ, fifiaudio.sampleSize32);

        I2S_open(I2S0, &fifiaudio.codec);           /* Open the I2S channel */

        TIMER_open(TIMER3, &fifiaudio.adapter);
        TIMER_ioctl(fifiaudio.adapter, &adapterConfig[0]);
        TIMER_stop(fifiaudio.adapter);
        TIMER_write(fifiaudio.adapter, 0);
        fifiaudio.adapterModulus =
            (1024u * I2S_NUM_SAMPLES * 69u * 2u) / (4u * 68u * (fifiaudio.sampleRateIQ / 48000u));
        TIMER_writeMatch(fifiaudio.adapter, TIMER_MATCH0, fifiaudio.adapterModulus - 1);
        TIMER_run(fifiaudio.adapter);

        BSP_setupI2S(fifiaudio.codec, fifiaudio.sampleRateIQ, fifiaudio.sampleSize32);

        I2S_submitJob(fifiaudio.codec,
                      fifiaudio.sampleSize32 ? &i2sJob32 : &i2sJob16,
                      fifiaudio.i2sCodecDma);
        I2S_run(fifiaudio.codec, I2S_CHANNEL_RX, ENABLE);

        BSP_openDac(&dacJob);

        fifiaudio.isActiveCodec = true;                 /* Mark as active */
    }
}



/** Start or stop the DSP.
 */
static void FIFIAUDIO_runDemod (LPCLIB_Switch enable)
{
    /* Stop the DSP? */
    if (!enable) {
    }

    /* Start the DSP? */
    if (enable) {
        if (fifiaudio.sampleRateDemod == 0) {
            fifiaudio.sampleRateDemod = 12000ul;
        }

        FIFIAUDIO_setDemodVolume(fifiaudio.volumeDemod);
    }
}



static void FIFIAUDIO_setDspDefaults (FIFIDSP_Handle handle)
{
    FIFIDSP_Config config;

    config.opcode = FIFIDSP_OPCODE_SET_MODE;
    config.mode = FIFIDSP_MODE_AM;  //TODO
    FIFIDSP_ioctl(handle, &config);

    config.opcode = FIFIDSP_OPCODE_SET_BANDWIDTH;
    config.bandwidth = 8000;  //TODO
    FIFIDSP_ioctl(handle, &config);

    config.opcode = FIFIDSP_OPCODE_SET_VOLUME;
    config.gain131 = 0x7FFFFFFF;  //TODO
    FIFIDSP_ioctl(handle, &config);
}



/* Inform FIFIAUDIO about change in system power state. */
void FIFIAUDIO_setPowerState (LPCLIB_Switch sleeping)
{
    fifiaudio.sleeping = sleeping;

    if (fifiaudio.sleeping) {
        /* Stop codec activity, power down the I2S interface */
        FIFIAUDIO_runCodec(DISABLE);
    }
}



osMailQDef(audioQueue, FIFIAUDIO_QUEUE_LENGTH, FIFIAUDIO_Message);


void FIFIAUDIO_task (const void *pArgs)
{
    (void) pArgs;
    FIFIAUDIO_Message *pMessage;
    FIFIDSP_Config dspConfig;
    FIFIDSP_Modulation mode;
    osEvent event;
    LPCLIB_Event sysEvent;
    int numSamples;


    fifiaudio.queue = osMailCreate(osMailQ(audioQueue), NULL);

    USB_initBuffer(&fifiaudio.buffer);
    USB_initBuffer(&fifiaudio.demodBuffer);
    fifiaudio.demodBuffer.data = fifiaudio.demodSamples;

    FIFIDSP_open(&fifiaudio.dsp);
    FIFIAUDIO_setDspDefaults(fifiaudio.dsp);

    while (1) {
        /* Is there a new message? */
        event = osMailGet(fifiaudio.queue, osWaitForever);
        if (event.status == osEventMail) {
            pMessage = (FIFIAUDIO_Message *)event.value.p;

//TODO: Ignore selected commands if in sleep mode
            switch (pMessage->opcode) {
            case FIFIAUDIO_OPCODE_IQ_ENABLE_16:
            case FIFIAUDIO_OPCODE_IQ_ENABLE_32:
                fifiaudio.stateIQ = pMessage->enable ? FIFIAUDIO_STREAMSTATE_ARMED : FIFIAUDIO_STREAMSTATE_OFF;
                fifiaudio.sampleSize32Request = (pMessage->opcode == FIFIAUDIO_OPCODE_IQ_ENABLE_32);
                if (!pMessage->enable) {
                    if (fifiaudio.stateDemod != FIFIAUDIO_STREAMSTATE_ACTIVE) {
                        FIFIAUDIO_runCodec(DISABLE);
                    }
                    else {
                    }
                }
                break;

            case FIFIAUDIO_OPCODE_SET_IQ_SPEED:
                if (fifiaudio.stateIQ == FIFIAUDIO_STREAMSTATE_ARMED) {
                    /* If the sample size changes, we must force a reinitialization of the codec. */
                    if ((fifiaudio.sampleSize32 != fifiaudio.sampleSize32Request) ||
                        (pMessage->sampleRate != fifiaudio.sampleRateIQ)) {
                        FIFIAUDIO_runCodec(DISABLE);
                    }
                    fifiaudio.sampleSize32 = fifiaudio.sampleSize32Request;
                    fifiaudio.sampleRateIQ = pMessage->sampleRate;

                    FIFIAUDIO_runCodec(ENABLE);
                    fifiaudio.stateIQ = FIFIAUDIO_STREAMSTATE_ACTIVE;

                    /* Inform DSP about sample format (16 or 32 bits) */
                    dspConfig.opcode = FIFIDSP_OPCODE_SET_SAMPLESIZE;
                    dspConfig.sampleSize = fifiaudio.sampleSize32 ? 32 : 16;
                    FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);

                    /* Inform system task about sample format (16 or 32 bits). Needed for I/Q swap in mixer! */
                    sysEvent.id = LPCLIB_EVENTID_APPLICATION;
                    sysEvent.opcode = FIFISDR_EVENT_SET_IQ_SWAP;
                    sysEvent.parameter = (void *)(fifiaudio.sampleSize32 ? 32 : 16);
                    SYS_submitJob(sysEvent);
                }
                break;

            case FIFIAUDIO_OPCODE_DEMOD_ENABLE:
                fifiaudio.stateDemod = pMessage->enable ? FIFIAUDIO_STREAMSTATE_ARMED : FIFIAUDIO_STREAMSTATE_OFF;
                if (fifiaudio.stateDemod == FIFIAUDIO_STREAMSTATE_OFF) {
                    FIFIAUDIO_runDemod(DISABLE);
                    if (fifiaudio.stateIQ == FIFIAUDIO_STREAMSTATE_OFF) {
                        FIFIAUDIO_runCodec(DISABLE);
                    }
                }
                break;

            case FIFIAUDIO_OPCODE_SET_DEMOD_SPEED:
                fifiaudio.sampleRateDemod = pMessage->sampleRate;
                if (fifiaudio.stateDemod == FIFIAUDIO_STREAMSTATE_ARMED) {
                    FIFIAUDIO_runCodec(ENABLE);
                    FIFIAUDIO_runDemod(ENABLE);
                    fifiaudio.stateDemod = FIFIAUDIO_STREAMSTATE_ACTIVE;
                }
                break;

            case FIFIAUDIO_OPCODE_DMA_END:
                FIFIAUDIO_runCodec(DISABLE);
                break;

            case FIFIAUDIO_OPCODE_COPY_BUFFERSTART:     /* Make a copy of the sample buffer start */
#if 1
if(fifiaudio.isActiveCodec) {
                DMA_prepareMemcpyContext(&fifiaudio.i2sMemcpy,
                                        fifiaudio.i2sHelperDma,
                                        FIFIAUDIO_dummyMemcpyCallback);
                if (fifiaudio.sampleSize32) {
                    DMA_memcpy(&fifiaudio.i2sMemcpy,
                            &audioBuffer[2 * I2S_NUM_SAMPLES],
                            &audioBuffer[0],
                            2 * I2S_NUM_EXTRA_SAMPLES * sizeof(audioBuffer[0]));
                }
                else {
                    DMA_memcpy(&fifiaudio.i2sMemcpy,
                            &audioBuffer[I2S_NUM_SAMPLES],
                            &audioBuffer[0],
                            I2S_NUM_EXTRA_SAMPLES * sizeof(audioBuffer[0]));
                }
}
#endif
                break;

            case FIFIAUDIO_OPCODE_RUN_DSP:              /* Run DSP for a new chunk of samples */
                /* Determine demodulator mode */
                dspConfig.opcode = FIFIDSP_OPCODE_GET_MODE;
                dspConfig.pMode = &mode;
                FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);

                /* Set the soundcard sample rate depending on DSP mode (only if not in use by PC) */
                if (fifiaudio.stateIQ == FIFIAUDIO_STREAMSTATE_OFF) {
                    if ((mode == FIFIDSP_MODE_WBFM) && (fifiaudio.sampleRateIQ != 192000)) {
                        FIFIAUDIO_runCodec(DISABLE);
                        fifiaudio.sampleRateIQ = 192000;
                        FIFIAUDIO_runCodec(ENABLE);

                        dspConfig.opcode = FIFIDSP_OPCODE_SET_INPUT_RATE;
                        dspConfig.sampleRate = fifiaudio.sampleRateIQ;
                        FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
                    }
                    if ((mode != FIFIDSP_MODE_WBFM) && (fifiaudio.sampleRateIQ != 48000)) {
                        FIFIAUDIO_runCodec(DISABLE);
                        fifiaudio.sampleRateIQ = 48000;
                        FIFIAUDIO_runCodec(ENABLE);

                        dspConfig.opcode = FIFIDSP_OPCODE_SET_INPUT_RATE;
                        dspConfig.sampleRate = fifiaudio.sampleRateIQ;
                        FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
                    }
                }

                /* Feed demodulator */
                FIFIDSP_write(fifiaudio.dsp,
                              fifiaudio.sampleSize32 ?
                                (const FIFIDSP_InputSample *)&audioBuffer[2 * fifiaudio.readIndex]
                              : (const FIFIDSP_InputSample *)&audioBuffer[fifiaudio.readIndex],
                              pMessage->numSamples);

                /* Take the generated output samples, and interpolate for direct DAC output. */
                numSamples = DAC_NUM_EXTRA_SAMPLES;     /* Maximum #samples we can accept */
                if (FIFIDSP_interpolateToDac(fifiaudio.dsp,
                                             4,
                                             &dacBuffer[fifiaudio.dacWriteIndex],
                                             &numSamples) == LPCLIB_SUCCESS) {
                    fifiaudio.dacWriteIndex += numSamples;
                    if (fifiaudio.dacWriteIndex > DAC_NUM_SAMPLES) {
                        memcpy(&dacBuffer[0],
                               &dacBuffer[DAC_NUM_SAMPLES],
                               (fifiaudio.dacWriteIndex - DAC_NUM_SAMPLES) * sizeof(dacBuffer[0]));
                    }
                    fifiaudio.dacWriteIndex %= DAC_NUM_SAMPLES;
                }
                break;
            }

            osMailFree(fifiaudio.queue, event.value.p);
       }
    }
}

