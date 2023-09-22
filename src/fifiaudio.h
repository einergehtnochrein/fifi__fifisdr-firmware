/* Copyright (c) 2010, DF9DQ
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


#ifndef _FIFIAUDIO_H_
#define _FIFIAUDIO_H_

#include "lpclib.h"
#include "usbclass_audio.h"

void FIFIAUDIO_handleEventIQ (LPCLIB_Event event);
void FIFIAUDIO_handleEventDemod (LPCLIB_Event event);
#if 1
void FIFIAUDIO_streamingInterfaceCallback (uint8_t endpoint,
                                           uint8_t selector,
                                           uint32_t currentValue);
#endif


/** Inform FIFIAUDIO about change in system power state.
 *
 *  \param[in] enable ENABLE if powered down, DISABLE if system up.
 */
void FIFIAUDIO_setPowerState (LPCLIB_Switch enable);


/** USB audio class frame handler. */
void FIFIAUDIO_frameHandler (USB_Handle usbhw, const struct USB_Class *pClass, uint16_t frameNumber);


/** Handle endpoint data transfers. */
LPCLIB_Result usbaudio_handleEndpointData (USBDEV_Handle handle, const struct USB_Class *pClass, uint8_t logicalEndpoint);

void FIFIAUDIO_task (const void *pArgs);

#endif
