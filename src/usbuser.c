/* Copyright (c) 2010-2014, DF9DQ
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


#include "lpclib.h"
#include "bsp-fifisdr.h"

#include "task-sys.h"
#include "usbuser.h"
#include "softrock.h"



const uint8_t audioEndpointList1[] = {USBCONFIG_STREAMING_EP1};
const uint8_t audioEndpointList2[] = {USBCONFIG_STREAMING_EP2};



/*************** Audio Class **********************/

/** Describe the range for all audio controls. */

static USBAUDIO_ControlRange audioControls1[] = {
    { .unit_id          = USBCONFIG_UNIT_FEATURE_IQ,
      .control_id       = USBAC_CS_FU_VOLUME_CONTROL,
      .min              = -6 * 0x100,
      .max              = 0,
      .current          = 0,
      .resolution       = 6 * 0x100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventIQ,
    },
};

static USBAUDIO_ControlRange audioControls2[] = {
    { .unit_id          = USBCONFIG_UNIT_FEATURE_DSP_AM,
      .control_id       = USBAC_CS_FU_VOLUME_CONTROL,
      .min              = -40 * 0x100,
      .max              = 0,
      .current          = 0,
      .resolution       = 1 * 0x100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_FEATURE_DSP_FM,
      .control_id       = USBAC_CS_FU_VOLUME_CONTROL,
      .min              = -40 * 0x100,
      .max              = 0,
      .current          = 0,
      .resolution       = 1 * 0x100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_FEATURE_DSP_LSB,
      .control_id       = USBAC_CS_FU_VOLUME_CONTROL,
      .min              = -40 * 0x100,
      .max              = 0,
      .current          = 0,
      .resolution       = 1 * 0x100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_FEATURE_DSP_USB,
      .control_id       = USBAC_CS_FU_VOLUME_CONTROL,
      .min              = -40 * 0x100,
      .max              = 0,
      .current          = 0,
      .resolution       = 1 * 0x100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },

    { .unit_id          = USBCONFIG_UNIT_SELECTOR_DSP,
      .control_id       = 0,
      .min              = 1,
      .max              = 4,
      .current          = 1,
      .resolution       = 1,
      .length           = 1,
      .callback         = FIFIAUDIO_handleEventDemod,
    },

    { .unit_id          = USBCONFIG_UNIT_PROCESSING_DSP,
      .control_id       = USBAC_CS_PU_DR_ENABLE_CONTROL,    /* NOTE: currently not used */
      .min              = 0,
      .max              = 1,
      .current          = 0,
      .resolution       = 1,
      .length           = 1,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_PROCESSING_DSP,
      .control_id       = USBAC_CS_PU_COMPRESSION_RATIO_CONTROL,    /* NOTE: currently not used */
      .min              = 1 * 0x0100,       /* 1 dB/dB */
      .max              = 30 * 0x0100,      /* 30 dB/dB */
      .current          = 1 * 0x0100,
      .resolution       = 0x0100,           /* 1 dB/dB */
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_PROCESSING_DSP,
      .control_id       = USBAC_CS_PU_MAXAMPL_CONTROL,      /* NOTE: currently not used */
      .min              = -40 * 0x0100,
      .max              = -6 * 0x0100,
      .current          = -20 * 0x0100,
      .resolution       = 1 * 0x0100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_PROCESSING_DSP,
      .control_id       = USBAC_CS_PU_THRESHOLD_CONTROL,    /* NOTE: currently not used */
      .min              = -100 * 0x0100,
      .max              = -10 * 0x0100,
      .current          = -60 * 0x0100,
      .resolution       = 1 * 0x0100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_PROCESSING_DSP,
      .control_id       = USBAC_CS_PU_ATTACK_TIME,
      .min              = 20 * 0x0100,
      .max              = 255 * 0x0100,
      .current          = 150 * 0x0100,
      .resolution       = 1 * 0x0100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
    { .unit_id          = USBCONFIG_UNIT_PROCESSING_DSP,
      .control_id       = USBAC_CS_PU_RELEASE_TIME,
      .min              = 20 * 0x0100,
      .max              = 255 * 0x0100,
      .current          = 255 * 0x0100,
      .resolution       = 1 * 0x0100,
      .length           = 2,
      .callback         = FIFIAUDIO_handleEventDemod,
    },
};


const USBAUDIO_ControlInterface fifisdr_audioControlInterface1 = {
    .interfaceNumber            = USBCONFIG_INTERFACE_AUDIO_CONTROL_1,
    .usesInterrupt              = false,
    .endpointNumberInterrupt    = 0,
    .numControls                = sizeof(audioControls1) / sizeof(audioControls1[0]),
    .controls                   = audioControls1,
};


const USBAUDIO_ControlInterface fifisdr_audioControlInterface2 = {
    .interfaceNumber            = USBCONFIG_INTERFACE_AUDIO_DSP_CONTROL,
    .usesInterrupt              = false,
    .endpointNumberInterrupt    = 0,
    .numControls                = sizeof(audioControls2) / sizeof(audioControls2[0]),
    .controls                   = audioControls2,
};

const uint32_t fifisdr_samplingFrequencies1_16_96k[] = {48000ul, 96000ul};
const uint32_t fifisdr_samplingFrequencies1_16_192k[] = {48000ul, 96000ul, 192000ul};
const uint32_t fifisdr_samplingFrequencies1_32[] = {48000ul, 96000ul};

const USBAUDIO_StreamingInterfaceParams fifisdr_streamingParams1_96k[] = {
    { .numEndpoints             = 0,
    },
    { .numEndpoints             = 1,
      .linkedTerminal           = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
      .endpointNumber           = USBCONFIG_STREAMING_EP1,
      .packetSize               = USBCONFIG_ISOC_SIZE1_32,
      .supportedControls        = 0x01,     /* Sampling frequency */
      .numSamplingFrequencies   = 2,
      .samplingFrequencies      = fifisdr_samplingFrequencies1_32,
    },
    { .numEndpoints             = 1,
      .linkedTerminal           = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
      .endpointNumber           = USBCONFIG_STREAMING_EP1,
      .packetSize               = USBCONFIG_ISOC_SIZE1_16,
      .supportedControls        = 0x01,     /* Sampling frequency */
      .numSamplingFrequencies   = 2,
      .samplingFrequencies      = fifisdr_samplingFrequencies1_16_96k,
    },
};


const USBAUDIO_StreamingInterfaceParams fifisdr_streamingParams1_192k[] = {
    { .numEndpoints             = 0,
    },
    { .numEndpoints             = 1,
      .linkedTerminal           = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
      .endpointNumber           = USBCONFIG_STREAMING_EP1,
      .packetSize               = USBCONFIG_ISOC_SIZE1_32,
      .supportedControls        = 0x01,     /* Sampling frequency */
      .numSamplingFrequencies   = 2,
      .samplingFrequencies      = fifisdr_samplingFrequencies1_32,
    },
    { .numEndpoints             = 1,
      .linkedTerminal           = USBCONFIG_UNIT_TERMINAL_OUT_IQ,
      .endpointNumber           = USBCONFIG_STREAMING_EP1,
      .packetSize               = USBCONFIG_ISOC_SIZE1_16,
      .supportedControls        = 0x01,     /* Sampling frequency */
      .numSamplingFrequencies   = 3,
      .samplingFrequencies      = fifisdr_samplingFrequencies1_16_192k,
    },
};


const uint32_t fifisdr_samplingFrequencies2[] = {48000ul};

const USBAUDIO_StreamingInterfaceParams fifisdr_streamingParams2[] = {
    { .numEndpoints             = 0,
    },
    { .numEndpoints             = 1,
      .linkedTerminal           = USBCONFIG_UNIT_TERMINAL_OUT_DSP,
      .endpointNumber           = USBCONFIG_STREAMING_EP2,
      .packetSize               = USBCONFIG_ISOC_SIZE2,
      .supportedControls        = 0x00,
      .numSamplingFrequencies   = 1,
      .samplingFrequencies      = fifisdr_samplingFrequencies2,
    },
};


USBAUDIO_StreamingInterface fifisdr_streamingInterfaces1_96k[] = {
    { .interfaceNumber          = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
      .numAltSettings           = 3,
      .activeSetting            = 0,
      .currentSamplerate        = 1,
      .isMuted                  = false,
      .params                   = fifisdr_streamingParams1_96k,
    },
};

USBAUDIO_StreamingInterface fifisdr_streamingInterfaces1_192k[] = {
    { .interfaceNumber          = USBCONFIG_INTERFACE_AUDIO_STREAMING_1,
      .numAltSettings           = 3,
      .activeSetting            = 0,
      .currentSamplerate        = 1,
      .isMuted                  = false,
      .params                   = fifisdr_streamingParams1_192k,
    },
};

USBAUDIO_StreamingInterface fifisdr_streamingInterfaces2[] = {
    { .interfaceNumber          = USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING,
      .numAltSettings           = 2,
      .activeSetting            = 0,
      .currentSamplerate        = 1,
      .isMuted                  = false,
      .params                   = fifisdr_streamingParams2,
    },
};


static const USBAUDIO_FunctionDeclaration fifisdr_audioFunctionIQ_96k = {
    .controlInterface       = &fifisdr_audioControlInterface1,
    .numStreamingInterfaces = 1,
    .streamingInterfaces    = fifisdr_streamingInterfaces1_96k,
    .callback               = FIFIAUDIO_handleEventIQ,
};

static const USBAUDIO_FunctionDeclaration fifisdr_audioFunctionIQ_192k = {
    .controlInterface       = &fifisdr_audioControlInterface1,
    .numStreamingInterfaces = 1,
    .streamingInterfaces    = fifisdr_streamingInterfaces1_192k,
    .callback               = FIFIAUDIO_handleEventIQ,
};

static const USBAUDIO_FunctionDeclaration fifisdr_audioFunctionDemod = {
    .controlInterface       = &fifisdr_audioControlInterface2,
    .numStreamingInterfaces = 1,
    .streamingInterfaces    = fifisdr_streamingInterfaces2,
    .callback               = FIFIAUDIO_handleEventDemod,
};


static USBAUDIO_Handle handleAudioIQ;
#if USBCONFIG_DSP
static USBAUDIO_Handle handleAudioDemod;
#endif


static const USB_Class fifisdr_classes[] = {
    {
      .configuration    = 1,
      .numEndpoints     = 0,
      .pEndpointList    = NULL,
      .init             = NULL,
      .requestHandler   = softrock_handleRequests,
      .frameHandler     = NULL,
      .endpointHandler  = NULL,
      .pInstance        = NULL,
    },
#if USBCONFIG_SOUNDCARD
    {
      .configuration    = 1,
      .numEndpoints     = 1,
      .pEndpointList    = audioEndpointList1,
      .init             = NULL,
      .requestHandler   = USBAUDIO_handleRequests,
      .frameHandler     = FIFIAUDIO_frameHandler,
      .endpointHandler  = usbaudio_handleEndpointData,
      .pInstance        = &handleAudioIQ,
    },
  #if USBCONFIG_DSP
    {
      .configuration    = 1,
      .numEndpoints     = 1,
      .pEndpointList    = audioEndpointList2,
      .init             = NULL,
      .requestHandler   = USBAUDIO_handleRequests,
      .frameHandler     = NULL,
      .endpointHandler  = usbaudio_handleEndpointData,
      .pInstance        = &handleAudioDemod,
    },
  #endif
#endif
};
const uint8_t fifisdr_numClasses = sizeof(fifisdr_classes) / sizeof(fifisdr_classes[0]);



/* Definition of USB task with restricted resource access (MPU) */
//#define USB_TASK_STACKSIZE      (1024)


USBDEV_Handle usbdev;



/** Callback for USB events. */
static void USBUSER_handleEvents (LPCLIB_Event event)
{
    if (event.id == LPCLIB_EVENTID_USBDEV) {
        /* Transform into system events */
        event.id = LPCLIB_EVENTID_APPLICATION;

        switch (event.opcode) {
        case USBDEV_EVENT_SUSPEND:
            /* Hardware holds USB_NEED_CLK set for another 2 ms after suspend condition is detected! */
//            osDelay(5);
osDelay(20);    /* NOTE: RTX CMSIS-RTOS doesn't guarantee a minimum delay! Use 20ms to ensure we wait at least one tick period. */

            event.opcode = FIFISDR_EVENT_USB_SUSPEND;
            SYS_submitJob(event);
            break;
        case USBDEV_EVENT_RESUME:
            event.opcode = FIFISDR_EVENT_USB_RESUME;
            SYS_submitJob(event);
            break;
        }
    }
}



/** USB handler task.
 */
void USBUSER_task (const void *pArgs)
{
    (void) pArgs;

    USBDEV_Config config;
    int have192k;


    /* Check for 192 kHz support */
    switch (BSP_getBoardType()) {
    case BSP_BOARD_KAI3:
    case BSP_BOARD_ROLF2:
    case BSP_BOARD_ROLF3:
        have192k = 1;
        break;

    default:
        have192k = 0;
        break;
    }

    fifisdr_initDescriptors();

    /* Prepare the classes. */
#if USBCONFIG_SOUNDCARD
    if (have192k) {
        USBAUDIO_open(&fifisdr_audioFunctionIQ_192k, &handleAudioIQ);
    }
    else {
        USBAUDIO_open(&fifisdr_audioFunctionIQ_96k, &handleAudioIQ);
    }
  #if USBCONFIG_DSP
    USBAUDIO_open(&fifisdr_audioFunctionDemod, &handleAudioDemod);
  #endif
#endif

    USBDEV_open(usbHardware, &usbdev);

    config.opcode = USBDEV_OPCODE_ATTACH_CLASSES;
    config.count = fifisdr_numClasses;
    config.pClassList = (USB_Class *)fifisdr_classes;
    USBDEV_ioctl(usbdev, &config);                      /* Attach class descriptors */

    config.opcode = USBDEV_OPCODE_ATTACH_DESCRIPTORS;
    if (have192k) {
        config.pDescriptors = (USB_DescriptorList *)&fifisdr_descriptors_192k;
    }
    else {
        config.pDescriptors = (USB_DescriptorList *)&fifisdr_descriptors_96k;
    }
    USBDEV_ioctl(usbdev, &config);

    config.opcode = USBDEV_OPCODE_SET_CALLBACK;
    config.callback.callback = USBUSER_handleEvents;
    config.callback.pOldCallback = NULL;
    USBDEV_ioctl(usbdev, &config);

config.opcode = USBDEV_OPCODE_CONNECT;
USBDEV_ioctl(usbdev, &config);    //TODO

    while (1) {
        USBDEV_worker(usbdev);
    }
}

