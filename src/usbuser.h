/* Copyright (c) 2010-2013, DF9DQ
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

#ifndef __USBUSER_H
#define __USBUSER_H

#include "usbdevice.h"
#include "usbdesc.h"
#include "usbclass_audio.h"
#include "fifiaudio.h"

/* Options */
#define USBCONFIG_SOUNDCARD                     1
#define USBCONFIG_DSP                           1



/** Number of configurations. */
#define USBCONFIG_NUM_CONFIGURATIONS            1

/** Interface numbers for configuration 1. */
#if USBCONFIG_SOUNDCARD
  #if USBCONFIG_DSP
    #define USBCONFIG_NUM_INTERFACES            5
  #else
    #define USBCONFIG_NUM_INTERFACES            3
  #endif
#else
    #define USBCONFIG_NUM_INTERFACES            1
#endif
#define USBCONFIG_INTERFACE_SOFTROCK_1          0
#define USBCONFIG_INTERFACE_AUDIO_CONTROL_1     1
#define USBCONFIG_INTERFACE_AUDIO_STREAMING_1   2
#define USBCONFIG_INTERFACE_AUDIO_DSP_CONTROL   3
#define USBCONFIG_INTERFACE_AUDIO_DSP_STREAMING 4

/** Endpoints for Audio Class streaming. */
#define USBCONFIG_STREAMING_EP1                 0x83
#define USBCONFIG_STREAMING_EP2                 0x86

/** Max packet size for the isochronous endpoint. */
#define USBCONFIG_ISOC_SIZE1_16                 800
#define USBCONFIG_ISOC_SIZE1_32                 800
#define USBCONFIG_ISOC_SIZE2                    100

/** Audio controls. */
#define USBCONFIG_UNIT_TERMINAL_IN_IQ           1
#define USBCONFIG_UNIT_FEATURE_IQ               2
#define USBCONFIG_UNIT_TERMINAL_OUT_IQ          3

#define USBCONFIG_UNIT_TERMINAL_IN_DSP1         4
#define USBCONFIG_UNIT_FEATURE_DSP_AM           5
#define USBCONFIG_UNIT_TERMINAL_IN_DSP2         6
#define USBCONFIG_UNIT_FEATURE_DSP_FM           7
#define USBCONFIG_UNIT_TERMINAL_IN_DSP3         8
#define USBCONFIG_UNIT_FEATURE_DSP_LSB          9
#define USBCONFIG_UNIT_TERMINAL_IN_DSP4         10
#define USBCONFIG_UNIT_FEATURE_DSP_USB          11
#define USBCONFIG_UNIT_SELECTOR_DSP             12
#define USBCONFIG_UNIT_PROCESSING_DSP           13
#define USBCONFIG_UNIT_TERMINAL_OUT_DSP         14

extern USBDEV_Handle usbdev;

extern const USB_DescriptorList fifisdr_descriptors_96k;
extern const USB_DescriptorList fifisdr_descriptors_192k;
void fifisdr_initDescriptors (void);

void USBUSER_task (const void *pArgs);


#endif /* #ifndef __USBUSER_CONFIG_H */

