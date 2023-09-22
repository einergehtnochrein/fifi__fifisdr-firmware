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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "lpclib.h"

#include "usbclass.h"
#include "usbclass_audio.h"


#define USBAUDIO_NUM_INSTANCES              2   /* TODO */


typedef struct USBAUDIO_Context {
    const USBAUDIO_FunctionDeclaration *pFunction;
} USBAUDIO_Context;


static USBAUDIO_Context usbaudio[USBAUDIO_NUM_INSTANCES];
static uint8_t usbaudioNumInstances;


/** Determine the index of the given control in the list.
 *
 *  Search the list of known controls, and return the index of the given control into that list.
 *
 *  \param [in] unit Index of the unit (feature, mixer, terminal, ...)
 *  \param [in] control Identifies the control (volume, mute, ...)
 *  \param [out] index Index in the control list
 *
 *  \return true=control found; false=control not found
 */
static _Bool USBAUDIO_findControl (USBAUDIO_Handle handle,
                                   uint8_t unit,
                                   uint8_t controlId,
                                   USBAUDIO_ControlRange **control)
{
    uint8_t n;


    /* Search the list */
    for (n = 0; n < handle->pFunction->controlInterface->numControls; n++) {
        if ((handle->pFunction->controlInterface->controls[n].unit_id == unit) &&
            (handle->pFunction->controlInterface->controls[n].control_id == controlId)) {
            /* Found it! */
            *control = &(handle->pFunction->controlInterface->controls[n]);
            return true;
        }
    }

    /* This is an unknown unit/control. */
    return false;
}



/** Handle audio class requests to the control interface.
 *
 *  \param [in] instance The instance of this audio class (starts with 0).
 *  \param [in] setup The setup packet
 *  \param [out] length Length of data to be returned. Maximum possible length is in setup->wLength.
 *  \param [out] data Points to the buffer that can take the data. Buffer must be able to take
 *                    setup->wLength bytes.
 *  \return
 */
static LPCLIB_Result USBAUDIO_handleControlRequests (USBAUDIO_Handle handle,
                                                     USB_SetupPacket *pSetup,
                                                     USB_Buffer *pBuffer)
{
    USBAUDIO_ControlRange *pControl;
    union {
        uint8_t *b;
        uint16_t *w;
        uint32_t *d;
    } p = {.b = pBuffer->data};
    int length = 0;
    LPCLIB_Result result = LPCLIB_SUCCESS;
    LPCLIB_Event event;


    /* Prepare event */
    event.id = LPCLIB_EVENTID_USBAUDIO;

    /* Is there a control like this? */
    if (USBAUDIO_findControl(handle,
                             ((pSetup->wIndex >> 8) & 0xFF),
                             ((pSetup->wValue >> 8) & 0xFF),
                             &pControl)) {
        switch (pSetup->bRequest) {
        case GET_MIN:
            p.w[0] = pControl->min;
            length = pControl->length;
            break;

        case GET_MAX:
            p.w[0] = pControl->max;
            length = pControl->length;
            break;

        case GET_CUR:
            p.w[0] = pControl->current;
            length = pControl->length;
            break;

        case GET_RES:
            p.w[0] = pControl->resolution;
            length = pControl->length;
            break;

        case SET_CUR:
            /* Is there enough data? (Accept longer data fields) */
            if (pSetup->wLength >= pControl->length) {
                if (pControl->length == 1) {
                    pControl->current = p.b[0];
                }
                else {
                    pControl->current = p.w[0];
                }

                /* Check if there is a callback */
                if (pControl->callback) {
                    event.opcode = USBAUDIO_EVENT_SET_CONTROL;
                    event.block = ((pSetup->wIndex >> 8) & 0xFF);
                    event.channel = pControl->control_id;
                    event.parameter = (void *)((int32_t)pControl->current);
                    pControl->callback(event);
                }
            }
            else {
                result = LPCLIB_ILLEGAL_PARAMETER;
            }
            break;

        case SET_RES:
            /* Value ignored */
            break;

        default:
            result = LPCLIB_ILLEGAL_PARAMETER;
            break;
        }
    }

    pBuffer->maxSize = length;

    return result;
}



static const LPCLIB_Event eventSamplerate = {
    .id = LPCLIB_EVENTID_USBAUDIO,
    .opcode = USBAUDIO_EVENT_ENDPOINT,
    .channel = USBAC_CS_EP_SAMPLING_FREQ_CONTROL,
};



/** Handle audio class requests to an endpoint..
 *
 *  \param [in] instance The instance of this audio class (starts with 0).
 *  \param [in] setup The setup packet
 *  \param [out] length Length of data to be returned. Maximum possible length is in setup->wLength.
 *  \param [out] data Points to the buffer that can take the data. Buffer must be able to take
 *                    setup->wLength bytes.
 *  \return
 */
static LPCLIB_Result USBAUDIO_handleEndpointRequests (USBAUDIO_Handle handle,
                                                      USB_SetupPacket *pSetup,
                                                      USB_Buffer *pBuffer)
{
    uint8_t n, m;
    uint32_t temp;
    USBAUDIO_StreamingInterface *streaming;
    bool found;
    int length = 0;
    LPCLIB_Result result = LPCLIB_SUCCESS;
    LPCLIB_Event event;


    /* Determine the interface to which this endpoint belongs. */
    streaming = NULL;
    found = false;
    for (n = 0; n < handle->pFunction->numStreamingInterfaces; n++) {
        streaming = &(handle->pFunction->streamingInterfaces[n]);
        for (m = 0; m < streaming->numAltSettings; m++) {
            if (streaming->params[m].numEndpoints > 0) {
                if (streaming->params[m].endpointNumber == pSetup->endpoint) {
                    found = true;
                    break;
                }
            }
        }

        if (found ) {
            break;
        }

    }

    if (!found) {
        /* Unknown endpoint! */
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    switch (pSetup->bRequest) {
    case SET_CUR:
        temp = ((uint32_t *)pBuffer->data)[0];
        if (pSetup->wLength < 4) {
            temp &= (1u << (8 * pSetup->wLength)) - 1;
        }

        if (handle->pFunction->callback) {
            event = eventSamplerate;
            event.block = pSetup->endpoint;
            event.parameter = (void *)((uint32_t)temp);
//event.channel = pSetup->wValue >> 8;
            handle->pFunction->callback(event);
        }

        if ((pSetup->wLength == 3) && ((pSetup->wValue >> 8) == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
            streaming->currentSamplerate = temp;
            USB_DEBUG(2, printf("  SET_CUR rate=%d\n", (int)streaming->currentSamplerate));
        }
        else {
            result = LPCLIB_ILLEGAL_PARAMETER;          /* unsupported */
        }
        break;

    case GET_CUR:
        if ((pSetup->wLength == 3) && ((pSetup->wValue >> 8) == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
            memcpy(pBuffer->data, &(streaming->currentSamplerate), 3);
            length = 3;
        }
        else {
            result = LPCLIB_ILLEGAL_PARAMETER;          /* unsupported */
        }
        break;
    }

    pBuffer->maxSize = length;

    return result;
}



/** Handle audio class requests.
 *
 *  \param [in] classInstance The instance of this audio class (starts with 0).
 *  \param [in] setup The setup packet
 *  \param [in] buffer Data buffer
 *  \return
 */
LPCLIB_Result USBAUDIO_handleRequests (USBDEV_Handle device,
                                       const USB_Class *pClass,
                                       USB_SetupPacket *pSetup,
                                       USB_Buffer *pBuffer)
{
(void)device;//TODO

    int n;
    LPCLIB_Event event;
    USBAUDIO_Handle handle = LPCLIB_INVALID_HANDLE;
    LPCLIB_Result result = LPCLIB_UNDEFINED;


    if (pClass->pInstance) {
        handle = *((USBAUDIO_Handle *)pClass->pInstance);
    }

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_UNDEFINED;
    }


    /* TODO TODO TODO
     * Workaround for a horrible Linux bug.
     * Linux (since kernel 3.x) tries to access audio controls of the second audio interface through the
     * control interface of the first audio interface.
     * Check the control ID, and change the interface number if it is a DSP interface control.
     */
    if ((pSetup->Type == USB_REQUEST_TYPE_CLASS) && (pSetup->Recipient == USB_REQUEST_RECIPIENT_INTERFACE)) {
        if ((pSetup->interface == 1) && (((pSetup->wIndex >> 8) & 0xFF) >= 4)) {
            pSetup->interface = 3;
        }
    }

    /* Prepare event */
    event.id = LPCLIB_EVENTID_USBAUDIO;
    event.block = pSetup->interface;

    /* Filter standard requests. This gives us a chance to follow changes in alternate settings.
     */
    if ((pSetup->Type == USB_REQUEST_TYPE_STANDARD) && (pSetup->bRequest == USBR_STANDARD_SET_INTERFACE)) {
        for (n = 0; n < handle->pFunction->numStreamingInterfaces; n++) {
            if (handle->pFunction->streamingInterfaces[n].interfaceNumber == pSetup->interface) {
                handle->pFunction->streamingInterfaces[n].activeSetting = pSetup->wValue;   //TODO check if index is valid!

                /* Inform the application */
                if (handle->pFunction->callback) {
                    event.opcode = USBAUDIO_EVENT_INTERFACE_CHANGE;
                    event.channel = pSetup->wValue & 0xFF;
                    handle->pFunction->callback(event);
                }
            }
        }
    }

    /* Filter class requests to the control interface. */
    if ((pSetup->Type == USB_REQUEST_TYPE_CLASS) && (pSetup->Recipient == USB_REQUEST_RECIPIENT_INTERFACE)) {
        if (pSetup->interface == handle->pFunction->controlInterface->interfaceNumber) {
            USB_DEBUG(2, printf(" Audio: control\r\n"));
            result = USBAUDIO_handleControlRequests(handle, pSetup, pBuffer);
        }
    }

    /* Filter class requests to the control (interrupt) or streaming endpoints. */
    if ((pSetup->Type == USB_REQUEST_TYPE_CLASS) && (pSetup->Recipient == USB_REQUEST_RECIPIENT_ENDPOINT)) {
        for (n = 0; n < pClass->numEndpoints; n++) {
            if (pClass->pEndpointList[n] == pSetup->endpoint) {
                USB_DEBUG(2, printf(" Audio: endpoint\r\n"));
                result = USBAUDIO_handleEndpointRequests(handle, pSetup, pBuffer);
            }
        }
    }

    /* Send response now if possible */
    if (result == LPCLIB_BUSY) {
        /* Response will be sent later. This is a success! */
        result = LPCLIB_SUCCESS;
    }
    else if (result != LPCLIB_UNDEFINED) {
        /* Truncate response buffer if too long. */
        if (pBuffer->maxSize > pSetup->wLength) {
            pBuffer->maxSize = pSetup->wLength;
        }
        USBDEV_submitSetupResponse(device,
                                   (result == LPCLIB_SUCCESS) ? pBuffer : NULL);
    }

    return result;
}



/* Open a USB Audio Class instance. */
LPCLIB_Result USBAUDIO_open (const USBAUDIO_FunctionDeclaration *pFunction, USBAUDIO_Handle *pHandle)
{
    /* Room for another instance? */
    if (usbaudioNumInstances >= USBAUDIO_NUM_INSTANCES) {
        *pHandle = LPCLIB_INVALID_HANDLE;
        return LPCLIB_BUSY;
    }

    usbaudio[usbaudioNumInstances].pFunction = pFunction;
    *pHandle = &usbaudio[usbaudioNumInstances];
    ++usbaudioNumInstances;

    return LPCLIB_SUCCESS;
}


/* Configure the USB audio class. */
void USBAUDIO_ioctl (USBAUDIO_Handle handle, const USBAUDIO_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    while (1) {
        switch (pConfig->opcode) {
        case USBAUDIO_OPCODE_xxx:
            break;
        }

        if (!pConfig->continued) {
            break;
        }

        ++pConfig;                                      /* More config's follow */
    }
}


