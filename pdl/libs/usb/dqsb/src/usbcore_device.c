/* Copyright (c) 2010-2012, DF9DQ
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

#include "usb.h"
#include "usbdevice.h"
#include "usbdesc.h"
#include "usbclass.h"



/** USB device status.
 *  (USB spec 2.0, chapter 9.1)
 *
 *  Do not change the sequence of the elements. It must be guaranteed that for instance
 *  ...ADDRESS > ...DEFAULT, etc.
 */
typedef enum {
    USBDEV_STATE_ATTACHED,
    USBDEV_STATE_POWERED,
    USBDEV_STATE_DEFAULT,
    USBDEV_STATE_ADDRESS,
    USBDEV_STATE_CONFIGURED,
} USBDEV_Status;


/** USB device features. */
enum {
    USBDEV_FEATURE_DEVICE_REMOTE_WAKEUP = 1,
    USBDEV_FEATURE_ENDPOINT_HALT = 0,
    USBDEV_FEATURE_TEST_MODE = 2,
};


#define USBDEV_CONTROL_BUFFER_SIZE          128


/** Length of queue for USB device task. */
#define USBDEV_QUEUE_LENGTH                 10
/** Length of queue for USB deferred interrupt handling. */
#define USBDEV_DEFERRED_QUEUE_LENGTH        10


/** USB device queue messages */
enum {
    USBDEV_MESSAGE_DEVICE_STATUS,
    USBDEV_MESSAGE_FRAME_INTERRUPT,
    USBDEV_MESSAGE_SETUP,
    USBDEV_MESSAGE_SETUP_RESPONSE,
    USBDEV_MESSAGE_BUFFER_COMPLETED,
    USBDEV_MESSAGE_BUFFER_REQUEST,
    USBDEV_MESSAGE_DEFERRED,
    USBDEV_MESSAGE_CONNECT,                 /**< Activate SoftConnect */
};

/** Element of USB device queue. */
typedef struct {
    uint8_t opcode;
    uint8_t logicalEndpoint;
    uint8_t reserved;
    union {
        uint32_t    deviceStatus;
        uint32_t    eventMask;
        uint32_t    endpointMask;
        USB_Buffer  *pBuffer;
        uint16_t    frameNumber;
        USB_SetupPacket   setup;
    };
} USBDEV_Message;


/** State of control endpoint */
typedef enum {
    USBDEV_CONTROLSTATE_IDLE,
    USBDEV_CONTROLSTATE_SETUP_OUT,          /**< OUT phase expected/ongoing */
    USBDEV_CONTROLSTATE_SETUP_IN,           /**< IN phase expected/ongoing */
} USBDEV_ControlState;


//TODO Try putting everything into the normal queue, and get rid of the deferred queue.
osMailQId usbdevDeferredQueue;          /**< Queue for deferred handling requests. */



/** State of the USB device core driver. */
static struct USBDEV_Context {
    USB_Handle usbhw;
    USB_SetupPacket setup;
    osMailQId queue;
    USBDEV_Status state;
    bool remote_wakeup_enabled;
    bool self_powered;
    bool isSuspended;                   /**< Currently in suspended state */
    USBDEV_ControlState controlState;   /**< State machine for control endpoint */
    const USB_Class *classes;
    uint8_t numClasses;
    uint8_t activeConfiguration;
    uint8_t alternateSetting[USB_MAX_INTERFACES_PER_CONFIG];
    USB_Buffer controlBuffer;
    const USB_DescriptorList *descriptors;
    uint8_t data[USBDEV_CONTROL_BUFFER_SIZE] __ALIGN(4);
    USBDEV_RequestHandler *endpointHandlerIn[16];
    USBDEV_RequestHandler *endpointHandlerOut[16];
    LPCLIB_Callback callback;
} usbdev;




void USBDEV_submitSetupResponse (USBDEV_Handle handle, USB_Buffer *pBuffer)
{
    (void) handle;
    USBDEV_Message *pMessage;

    pMessage = osMailAlloc(usbdev.queue, 0);
    if (pMessage) {
        pMessage->opcode = USBDEV_MESSAGE_SETUP_RESPONSE;
        pMessage->pBuffer = pBuffer;
        osMailPut(usbdev.queue, pMessage);
    }
}



/** Handle events from USB hardware layer. */
static void USBDEV_handleEvent (LPCLIB_Event event)
{
    USBDEV_Message *pMessage;
    USBDEV_Message *pMessageDeferred;
    USB_CallbackContext *pCbc = (USB_CallbackContext *)event.parameter;


    pMessage = osMailAlloc(usbdev.queue, 0);
    if (pMessage == NULL) {
        return;
    }

    switch (event.opcode) {
    case USB_EVENT_FRAME:
        pMessage->opcode = USBDEV_MESSAGE_FRAME_INTERRUPT;
        break;

    case USB_EVENT_DEVICE_STATUS:
        pMessage->opcode = USBDEV_MESSAGE_DEVICE_STATUS;
        break;

    case USB_EVENT_DEFERRED:
        /* Hardware event goes sequentially into separate queue. */
        pMessageDeferred = osMailAlloc(usbdevDeferredQueue, 0);
        if (pMessageDeferred) {
            pMessageDeferred->opcode = USBDEV_MESSAGE_DEFERRED;
            pMessageDeferred->endpointMask = pCbc->endpointMask;
            osMailPut(usbdevDeferredQueue, pMessageDeferred);
        }

        pMessage->opcode = USBDEV_MESSAGE_DEFERRED;
        pMessage->endpointMask = pCbc->endpointMask;
        break;

    case USB_EVENT_NEED_BUFFER:
        pMessage->opcode = USBDEV_MESSAGE_BUFFER_REQUEST;
        pMessage->logicalEndpoint = event.channel;
        pMessage->eventMask = pCbc->deviceStatus;
        break;

    case USB_EVENT_BUFFER_DONE:
        pMessage->opcode = USBDEV_MESSAGE_BUFFER_COMPLETED;
        pMessage->logicalEndpoint = event.channel;
        break;

    case USB_EVENT_SETUP:
        pMessage->opcode = USBDEV_MESSAGE_SETUP;
        pMessage->setup = *((USB_SetupPacket *)pCbc->pSetup);
        break;

    default:
        osMailFree(usbdev.queue, pMessage);
        return;
    }

    osMailPut(usbdev.queue, pMessage);
}


/** Set device address.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] address Device address
 */
static void USBDEV_setAddress (USBDEV_Handle handle, uint8_t address)
{
    USB_Config config;

    config.opcode = USB_OPCODE_SET_ADDRESS;
    config.address = address;
    USB_ioctl(handle->usbhw, &config);
}


/** Configure/unconfigure the device hardware.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] configure ENABLE/DISABLE flag
 */
static void USBDEV_configureDevice (USBDEV_Handle handle, LPCLIB_Switch configure)
{
    USB_Config config;

    config.opcode = USB_OPCODE_CONFIGURE_DEVICE;
    config.configure = configure;
    USB_ioctl(handle->usbhw, &config);
}


/** Deactivate all non-control endpoints.
 *
 *  \param[in] handle Device stack handle
 */
static void USBDEV_deactivateNonControlEndpoints (USBDEV_Handle handle)
{
    USB_Config config;

    config.opcode = USB_OPCODE_DEACTIVATE_NON_CONTROL;
    USB_ioctl(handle->usbhw, &config);
}



/** Connect/disconnect the device (SoftConnect).
 *
 *  \param[in] handle Device stack handle
 *  \param[in] connect ENABLE/DISABLE SoftConnect
 */
static void USBDEV_connectBus (USBDEV_Handle handle, LPCLIB_Switch connect)
{
    USB_Config config;

    config.opcode = USB_OPCODE_SOFTCONNECT;
    config.connect = connect;
    USB_ioctl(handle->usbhw, &config);
}



/** Select which endpoints shall have DMA support.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] dmaMask Mask for 32 physical endpoints (1 = DMA enabled)
 */
static void USBDEV_selectDmaEndpoints (USBDEV_Handle handle, uint32_t dmaMask)
{
    USB_Config config;

    config.opcode = USB_OPCODE_SELECT_DMA_ENDPOINTS;
    config.dmaEndpointEnable = dmaMask;
    USB_ioctl(handle->usbhw, &config);
}


/** Stall/unstall an endpoint.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] logicalEndpoint Endpoint number
 *  \param[in] stall ENABLE(stall)/DISABLE(unstall)
 */
#if 0
static void USBDEV_stallEndpoint (USBDEV_Handle handle, uint8_t logicalEndpoint, LPCLIB_Switch stall)
{
    USB_Config config;

    config.opcode = USB_OPCODE_ENDPOINT_STALL;
    config.logicalEndpoint = logicalEndpoint;
    config.stall = stall;
    USB_ioctl(handle->usbhw, &config);
}
#endif



/** Activate/deactivate an endpoint.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] logicalEndpoint Endpoint number
 *  \param[in] activate ENABLE/DISABLE endpoint
 *  \param[in] packetSize Max packet size
 */
static void USBDEV_activateEndpoint (USBDEV_Handle handle,
                                     uint8_t logicalEndpoint,
                                     LPCLIB_Switch activate,
                                     uint16_t packetSize)
{
    USB_Config config;

    config.opcode = USB_OPCODE_ENDPOINT_ACTIVATION;
    config.logicalEndpoint = logicalEndpoint;
    config.endpointActivation.activate = activate;
    config.endpointActivation.packetSize = packetSize;
    USB_ioctl(handle->usbhw, &config);
}



/** Select which conditions can trigger NAK interrupts.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] nakOption Conditions
 */
#if 0
static void USBDEV_setNakOption (USBDEV_Handle handle, USB_ConfigNakInterrupt nakOption)
{
    USB_Config config;

    config.opcode = USB_OPCODE_NAK_INTERRUPT;
    config.nak = nakOption;
    USB_ioctl(handle->usbhw, &config);
}
#endif



/** Sets a configuration.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] newConfig Number of new configuration
 *  \retval LPCLIB_SUCCESS New configuration activated.
 *  \retval LPCLIB_ILLEGAL_PARAMETER Unknown configuration
 */
static LPCLIB_Result USBDEV_setConfiguration (USBDEV_Handle handle, uint8_t newConfig)
{
    const void *p;
    uint16_t length;
    int thisLength;
    uint8_t n;
    uint8_t thisAlternateSetting;
    uint8_t interfaceIndex;


//TODO Für alle EPs muß das Halt-Feature gelöscht werden, auch wenn sich die Configuration nicht ändert!


    /* As this function is called only if the configuration is different from the active configuration,
     * we deactivate all non-control endpoints first.
     */
    USBDEV_deactivateNonControlEndpoints(handle);

    /* Search the list of configurations for the desired configuration value. */
    for (n = 0; n < handle->descriptors->numConfigurations; n++) {
        /* Is this the configuration we are looking for? */
        p = handle->descriptors->configurations[n].descriptor;
        if (((USB_ConfigurationDescriptor *)p)->bConfigurationValue == newConfig) {
            /* Parse the descriptor list for endpoint descriptors.
             * Activate all endpoints in interfaces with alternate setting 0.
             */
            length = 0;
            interfaceIndex = 0;
            thisAlternateSetting = 0;
            thisLength = -1;
            while ((length < handle->descriptors->configurations[n].length) && (thisLength != 0)) {
                /* Check descriptor type (always the second element in a descriptor) */
                switch (((uint8_t *)p)[1]) {
                case USBD_INTERFACE:
                    /* Remember the alternate setting of the current interface. */
                    thisAlternateSetting = ((USB_InterfaceDescriptor *)p)->bAlternateSetting;
                    if ((thisAlternateSetting == 0) &&
                        (interfaceIndex < USB_MAX_INTERFACES_PER_CONFIG)) {
                        handle->alternateSetting[interfaceIndex] = 0;
                        interfaceIndex++;
                    }
                break;

                case USBD_ENDPOINT:
                    /* Activate the endpoint if it belongs to an interface with alternate setting zero. */
                    if (thisAlternateSetting == 0) {
                        USBDEV_activateEndpoint(handle,
                                                ((USB_EndpointDescriptor *)p)->bEndpointAddress,
                                                ENABLE,
                                                ((USB_EndpointDescriptor *)p)->wMaxPacketSize);
                    }
                break;
                }

                /* Go to next descriptor (bLength is always the first descriptor element) */
                thisLength = ((uint8_t *)p)[0];
                length += thisLength;
                p = ((uint8_t *)p) + thisLength;

                /* NOTE: If we reach a descriptor of zero length, we stop parsing. */
            }

            return LPCLIB_SUCCESS;
        }
    }

    /* We will end up here if the configuration value is unknown. */
    return LPCLIB_ILLEGAL_PARAMETER;
}



/** Return descriptor */
static LPCLIB_Result USBDEV_getDescriptor (USBDEV_Handle handle,
                                           USB_SetupPacket *pSetup,
                                           USB_Buffer *pBuffer)
{
    uint8_t descriptorIndex;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    switch ((pSetup->wValue >> 8) & 0xFF) {
    case USBD_DEVICE:
        pBuffer->data = (void *)((handle->descriptors)->device);
        pBuffer->maxSize = USBD_SIZE_DEVICE;
        USB_DEBUG(2, printf("  DEV\r\n"));
        result = LPCLIB_SUCCESS;
        break;

    case USBD_CONFIGURATION:
        descriptorIndex = pSetup->wValue & 0xFF;
        pBuffer->data = (void *)handle->descriptors->configurations[descriptorIndex].descriptor;
        pBuffer->maxSize = handle->descriptors->configurations[descriptorIndex].length;
        USB_DEBUG(2, printf("  CONF\r\n"));
        result = LPCLIB_SUCCESS;
        break;

    case USBD_STRING:
        descriptorIndex = pSetup->wValue & 0xFF;
        USB_DEBUG(2, printf("  STR\r\n"));
        result = handle->descriptors->callbackGetString(descriptorIndex, pSetup->wIndex, (const void **)&pBuffer->data);
        if (result == LPCLIB_SUCCESS) {
            /* Length is in first byte of string descriptor */
            pBuffer->maxSize = ((uint8_t *)pBuffer->data)[0];
        }
        break;

    case USBD_DEVICE_QUALIFIER:
        break;
    }

    /* Truncate reply if longer than request buffer. */
    if (pBuffer->maxSize > pSetup->wLength) {
        pBuffer->maxSize = pSetup->wLength;
    }

    return result;
}



/** Get pointer to the class addressed by the endpoint.
 *
 *  \param [in] logicalEndpoint Endpoint number
 *  \return Class pointer; NULL if no class has this endpoint
 */
static const USB_Class* USBDEV_getClassFromEndpoint (USBDEV_Handle handle, uint8_t logicalEndpoint)
{
    const USB_Class *theClass;
    uint8_t n, m;


    for (n = 0; n < handle->numClasses; n++) {
        theClass = &handle->classes[n];

        /* Is the class active in the current configuration? */
        if (theClass->configuration == handle->activeConfiguration) {
            /* Does this class know the endpoint? */
            for (m = 0; m < theClass->numEndpoints; m++) {
                if (theClass->pEndpointList[m] == logicalEndpoint) {
                    return theClass;
                }
            }
        }
    }

    return NULL;
}



/** Tell the USB device core about the class driver configuration. */
static void USBDEV_attachClasses (USBDEV_Handle handle, const USB_Class *classList, uint8_t numClasses)
{
    handle->classes = classList;
    handle->numClasses = numClasses;
}



static const USB_Config usbSetCallback = {
    .opcode = USB_OPCODE_SET_CALLBACK,
    .continued = DISABLE,
    .logicalEndpoint = 0,
    {.callback = USBDEV_handleEvent,}
};


osMailQDef(devQueue, USBDEV_QUEUE_LENGTH, USBDEV_Message);
osMailQDef(devDeferredQueue, USBDEV_DEFERRED_QUEUE_LENGTH, USBDEV_Message);


/* Open a USB device stack. */
LPCLIB_Result USBDEV_open (USB_Handle bus, USBDEV_Handle *pHandle)
{
    if (bus == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    *pHandle = &usbdev;

    /* Task communication */
    usbdev.queue = osMailCreate(osMailQ(devQueue), NULL);
    usbdevDeferredQueue = osMailCreate(osMailQ(devDeferredQueue), NULL);

    usbdev.usbhw = bus;

    usbdev.controlBuffer.maxSize = USBDEV_CONTROL_BUFFER_SIZE;
    usbdev.controlBuffer.callback = NULL;               /* Only API calls from HW to DEV */

    usbdev.state = USBDEV_STATE_POWERED;
    usbdev.controlState = USBDEV_CONTROLSTATE_IDLE;

    /* Configure the hardware */
    USBDEV_selectDmaEndpoints(*pHandle, 0x00000000);   //TODO hardware dependent!!
//    USBDEV_selectDmaEndpoints(*pHandle, 0x00000080);   //TODO

    /* Activate the control endpoints (IN and OUT) */
    USBDEV_activateEndpoint(*pHandle, 0x00, ENABLE, 64);    //TODO
    USBDEV_activateEndpoint(*pHandle, 0x80, ENABLE, 64);    //TODO
//USBDEV_setNakOption(*pHandle, 0x20);//TODO

    USB_ioctl(bus, (USB_Config *)&usbSetCallback);

    return LPCLIB_SUCCESS;
}



/* Configure the USB device stack. */
LPCLIB_Result USBDEV_ioctl (USBDEV_Handle handle, const USBDEV_Config *pConfig)
{
    USBDEV_Message *pMessage;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    while (pConfig->opcode != USBDEV_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case USBDEV_OPCODE_CONNECT:
            pMessage = osMailAlloc(usbdev.queue, 0);
            if (pMessage) {
                pMessage->opcode = USBDEV_MESSAGE_CONNECT;
                osMailPut(usbdev.queue, pMessage);
            }
            break;

        case USBDEV_OPCODE_ATTACH_DESCRIPTORS:
            handle->descriptors = pConfig->pDescriptors;
            break;

        case USBDEV_OPCODE_ATTACH_CLASSES:
            USBDEV_attachClasses(handle, pConfig->pClassList, pConfig->count);
            break;

        case USBDEV_OPCODE_SET_CALLBACK:
            if (pConfig->callback.pOldCallback) {       /* Return current callback if requested */
                *(pConfig->callback.pOldCallback) = handle->callback;
            }
            handle->callback = pConfig->callback.callback;
            break;

        default:
            return LPCLIB_ILLEGAL_PARAMETER;
        }

        ++pConfig;
    }

    return LPCLIB_SUCCESS;
}


/** Activate/deactivate classes based on the current configuration.
 *
 *  \param [in] instance Device instance
 */
static void usbdevice_activateClasses (void)
{
    uint8_t n;
    USB_Class *theClass;


    for (n = 0; n < usbdev.numClasses; n++) {
        theClass = (USB_Class *)&usbdev.classes[n];

        if (theClass->init) {
            if (theClass->configuration == usbdev.activeConfiguration) {
                theClass->init(theClass, ENABLE);
            }
            else {
                theClass->init(theClass, DISABLE);
            }
        }
    }
}



/** Get pointer to interface descriptor.
 *
 *  Search the descriptors for an interface with the specified interface number.
 *  Return a pointer to that interface (first occurrence, alternate setting 0).
 *
 *  \param [in] interfaceNumber Number of the interface
 *  \param [out] currentAlternateSetting Current alternate setting (if interface valid)
 *  \return Pointer to interface; NULL if not found.
 */
static const void * USBDEV_getInterfaceDescriptor (uint8_t interfaceNumber, uint8_t *currentAlternateSetting)
{
    const void *p;
    uint16_t length;
    int thisLength;
    uint8_t n;
    uint8_t interfaceIndex;
    uint8_t thisAlternateSetting;


    /* Search the list of configurations for the active configuration. */
    for (n = 0; n < usbdev.descriptors->numConfigurations; n++) {
        /* Is this the active configuration */
        p = usbdev.descriptors->configurations[n].descriptor;
        if (((USB_ConfigurationDescriptor *)p)->bConfigurationValue == usbdev.activeConfiguration) {
            /* Parse the descriptor list for interface descriptors. */
            length = 0;
            interfaceIndex = 0;
            thisLength = -1;
            while ((length < usbdev.descriptors->configurations[n].length) && (thisLength != 0)) {
                /* Check descriptor type (always the second element in a descriptor) */
                if (((uint8_t *)p)[1] == USBD_INTERFACE) {
                    /* Remember the alternate setting of the current interface. */
                    thisAlternateSetting = ((USB_InterfaceDescriptor *)p)->bAlternateSetting;

                    /* Do nothing if the desired alternate setting is the active one. */
                    if (((USB_InterfaceDescriptor *)p)->bInterfaceNumber == interfaceNumber) {
                        if (currentAlternateSetting) {
                            *currentAlternateSetting = usbdev.alternateSetting[interfaceIndex];
                        }
                        return p;
                    }

                    if ((thisAlternateSetting == 0) &&
                        (interfaceIndex < USB_MAX_INTERFACES_PER_CONFIG)) {
                        interfaceIndex++;
                    }
                }

                /* Go to next descriptor (bLength is always the first descriptor element) */
                thisLength = ((uint8_t *)p)[0];
                length += thisLength;
                p = ((uint8_t *)p) + thisLength;

                /* NOTE: If we reach a descriptor of zero length, we stop parsing. */
            }
        }
    }

    /* We will end up here if the configuration value is unknown. */
    return NULL;
}



/** Sets the alternate setting for an interface.
 *
 *  \param [in] interfaceNumber Number of the interface
 *  \param [in] newAlternateSetting Index of the alternate setting
 */
static LPCLIB_Result USBDEV_setInterface (USBDEV_Handle handle,
                                          uint8_t interfaceNumber,
                                          uint8_t newAlternateSetting)
{
    const uint8_t *p;
    uint16_t length;
    int thisLength;
    uint8_t n;
    uint8_t thisAlternateSetting;
    uint8_t currentAlternateSetting;
    uint8_t thisInterfaceNumber;
    uint8_t phase;
    uint8_t interfaceIndex;
    uint16_t wMaxPacketSize;
    bool found = false;

//TODO Für alle EPs muß das Halt-Feature gelöscht werden, auch wenn sich Interface/AlternateSetting nicht ändert!

    /* Search the list of configurations for the active configuration. */
    for (n = 0; n < handle->descriptors->numConfigurations; n++) {
        /* Run in two phases:
         * Phase 1 deactivates all endpoints of the currently active alternate setting.
         * Phase 2 activates all endpoints of the new alternate setting.
         */
        for (phase = 0; phase < 2; phase++) {
            /* Is this the active configuration */
            p = handle->descriptors->configurations[n].descriptor;
            if (((USB_ConfigurationDescriptor *)p)->bConfigurationValue == handle->activeConfiguration) {
                /* Parse the descriptor list for endpoint descriptors.
                 * Activate all endpoints in interfaces with alternate setting 0.
                 */
                length = 0;
                interfaceIndex = 0;
                thisInterfaceNumber = 0;
                thisAlternateSetting = 0;
                currentAlternateSetting = 0;
                thisLength = -1;
                while ((length < handle->descriptors->configurations[n].length) && (thisLength != 0)) {
                    /* Check descriptor type (always the second element in a descriptor) */
                    switch (((uint8_t *)p)[1]) {
                    case USBD_INTERFACE:
                        /* Remember the alternate setting of the current interface. */
                        thisInterfaceNumber = ((USB_InterfaceDescriptor *)p)->bInterfaceNumber;
                        thisAlternateSetting = ((USB_InterfaceDescriptor *)p)->bAlternateSetting;

                        /* Remember a match of interface and alternate setting. */
                        if ((thisInterfaceNumber == interfaceNumber) &&
                            (thisAlternateSetting == newAlternateSetting)) {
                            found = true;
                        }

                        if (thisAlternateSetting == 0) {
                            currentAlternateSetting = handle->alternateSetting[interfaceIndex];

                            if (thisInterfaceNumber == interfaceNumber) {
                                /* Do nothing if the desired alternate setting is the active one. */
                                if (currentAlternateSetting == newAlternateSetting) {
                                    return LPCLIB_SUCCESS;
                                }

                                if (phase == 1) {
                                    usbdev.alternateSetting[interfaceIndex] = newAlternateSetting;
                                }
                            }

                            if (interfaceIndex < USB_MAX_INTERFACES_PER_CONFIG) {
                                interfaceIndex++;
                            }
                        }
                    break;

                    case USBD_ENDPOINT:
                        /* In first phase deactivate the endpoints of the current alternate setting */
                        if ((phase == 0) &&
                            (thisInterfaceNumber == interfaceNumber) &&
                            (thisAlternateSetting == currentAlternateSetting)) {
                            USBDEV_activateEndpoint(handle,
                                                    ((USB_EndpointDescriptor *)p)->bEndpointAddress,
                                                    DISABLE,
                                                    0);
                        }

                        /* In second phase activate the endpoints of the new alternate setting */
                        if ((phase == 1) &&
                            (thisInterfaceNumber == interfaceNumber) &&
                            (thisAlternateSetting == newAlternateSetting)) {
                            /* Make copy of wMaxPacketSize field to avoid unaligned access isuues */
                            wMaxPacketSize =
                                        ((uint8_t *)(&((USB_EndpointDescriptor *)p)->wMaxPacketSize))[0]
                                + 256 * ((uint8_t *)(&((USB_EndpointDescriptor *)p)->wMaxPacketSize))[1];
                            USBDEV_activateEndpoint(handle,
                                                    ((USB_EndpointDescriptor *)p)->bEndpointAddress,
                                                    ENABLE,
                                                    wMaxPacketSize);
                        }
                    break;
                    }

                    /* Go to next descriptor (bLength is always the first descriptor element) */
                    thisLength = ((uint8_t *)p)[0];
                    length += thisLength;
                    p = ((uint8_t *)p) + thisLength;

                    /* NOTE: If we reach a descriptor of zero length, we stop parsing. */
                }
            }
        }
    }

    /* We will end up here if the configuration value is unknown. */
    return (found ? LPCLIB_SUCCESS : LPCLIB_ILLEGAL_PARAMETER);
}



/** Handle USB device standard requests ("chapter 9").
 */
static LPCLIB_Result USBDEV_handleStandardRequest (USBDEV_Handle handle,
                                                   USB_SetupPacket *pSetup,
                                                   USB_Buffer *pBuffer)
{
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;
    const void *p;
    uint8_t alternateSetting;


    /* In case of a positive reply, default to zero reply length. */
    pBuffer->maxSize = 0;
//    pBuffer->data = handle->data;

    switch (pSetup->Recipient) {
    case USB_REQUEST_RECIPIENT_DEVICE:
        switch (pSetup->bRequest) {
        case USBR_STANDARD_CLEAR_FEATURE:
            /* Only the remote wakeup feature can be cleared */
            if (pSetup->wValue == USBDEV_FEATURE_DEVICE_REMOTE_WAKEUP) {
                handle->remote_wakeup_enabled = false;
                result = LPCLIB_SUCCESS;
            }
            break;

        case USBR_STANDARD_GET_CONFIGURATION:
            if (pSetup->wLength == 1) {
                ((uint8_t *)pBuffer->data)[0] = handle->activeConfiguration;
                pBuffer->maxSize = 1;
                result = LPCLIB_SUCCESS;
            }
            break;

        case USBR_STANDARD_GET_DESCRIPTOR:
            USB_DEBUG(2, printf(" GET_DESCRIPTOR\r\n"));
            result = USBDEV_getDescriptor(handle, pSetup, pBuffer);
            break;

        case USBR_STANDARD_GET_STATUS:
            if (pSetup->wLength == 2) {
                ((uint16_t *)pBuffer->data)[0] =
                    (handle->remote_wakeup_enabled ? 0x0002 : 0x0000)
                  | (handle->self_powered ? 0x0001 : 0x0000);
                pBuffer->maxSize = 2;
                result = LPCLIB_SUCCESS;
            }
            break;

        case USBR_STANDARD_SET_ADDRESS:
            USB_DEBUG(2, printf(" SET_ADDRESS %d\r\n", pSetup->wValue));
            switch (handle->state) {
            case USBDEV_STATE_DEFAULT:
                if (pSetup->wValue != 0) {
                    /* From now on respond to the new address */
                    handle->state = USBDEV_STATE_ADDRESS;
                    USBDEV_setAddress(handle, pSetup->wValue);
                }
                result = LPCLIB_SUCCESS;
                break;

            case USBDEV_STATE_ADDRESS:
                if (pSetup->wValue == 0) {
                    /* Forget our current address */
                    //TODO Tell the hardware to return to address 0
                    handle->state = USBDEV_STATE_DEFAULT;
                }
                else {
                    /* From now on respond to the new address */
                    //TODO Tell hardware layer to change address
                }
                result = LPCLIB_SUCCESS;
                break;

            default:
                /* Standard doesn't specify the behavior in other states. */
                break;
            }
            break;

        case USBR_STANDARD_SET_CONFIGURATION:
            USB_DEBUG(2, printf(" SET_CONFIGURATION\r\n"));
            switch (handle->state) {
            case USBDEV_STATE_ADDRESS:
            case USBDEV_STATE_CONFIGURED:
                /* A configuration value of zero unconfigures the device, and takes it to ADDRESS state */
                if (pSetup->wValue == 0) {
                    USBDEV_configureDevice(handle, DISABLE);
                    handle->state = USBDEV_STATE_ADDRESS;
                    handle->activeConfiguration = 0;
                    result =LPCLIB_SUCCESS;
                }
                /* Check whether the requested configuration is already active. */
                else if (pSetup->wValue == handle->activeConfiguration) {
                    result = LPCLIB_SUCCESS;
                }
                else {
                    /* Select this as the new configuration. */
                    result = USBDEV_setConfiguration(handle, pSetup->wValue);
                    /* Enable the new configuration. */
                    USBDEV_configureDevice(handle, ENABLE);
                    handle->activeConfiguration = pSetup->wValue;
                    handle->state = USBDEV_STATE_CONFIGURED;
                    /* Activate classes */
                    usbdevice_activateClasses();
                }
                break;

            default:
                /* 'Request error' in all other states */
                ;
                break;
            }
            break;

        case USBR_STANDARD_SET_DESCRIPTOR:
            /* We do not support this request */
            break;

        case USBR_STANDARD_SET_FEATURE:
            if (pSetup->wValue == USBDEV_FEATURE_DEVICE_REMOTE_WAKEUP) {
                handle->remote_wakeup_enabled = true;
                result = LPCLIB_SUCCESS;
            }
            else if (pSetup->wValue == USBDEV_FEATURE_TEST_MODE) {
                //TODO support test modes
            }
            break;
        }
        break;

    case USB_REQUEST_RECIPIENT_INTERFACE:
        switch (pSetup->bRequest) {
        case USBR_STANDARD_CLEAR_FEATURE:
            /* Nothing to do for an interface */
            break;

        case USBR_STANDARD_GET_INTERFACE:
            p = USBDEV_getInterfaceDescriptor(pSetup->wIndex, &alternateSetting);
            if (p != NULL) {
                ((uint8_t *)pBuffer->data)[0] = alternateSetting;
                pBuffer->maxSize = 1;
                result = LPCLIB_SUCCESS;
            }
            break;

        case USBR_STANDARD_GET_STATUS:
            /* Currently no status info is defined for an interface, but a response is required! */
           ((uint16_t *)pBuffer->data)[0] = 0;
           pBuffer->maxSize = 2;
           result = LPCLIB_SUCCESS;
           break;

        case USBR_STANDARD_SET_FEATURE:
            /* Nothing to do for an interface */
            break;

        case USBR_STANDARD_SET_INTERFACE:
            USB_DEBUG(2, printf(" SET_INTERFACE\r\n"));
            switch (usbdev.state) {
            case USBDEV_STATE_ADDRESS:
                /* MUST return 'request error' */
                break;

            case USBDEV_STATE_CONFIGURED:
                //TODO Let the class driver know!
                result = USBDEV_setInterface(handle, pSetup->wIndex, pSetup->wValue);
                break;

            default:
                /* Undefined behavior. Return 'request error' */
                break;
            }
            break;
        }
    break;

    case USB_REQUEST_RECIPIENT_ENDPOINT:
        switch (pSetup->bRequest) {
        case USBR_STANDARD_CLEAR_FEATURE:
            if (pSetup->wValue == USBDEV_FEATURE_ENDPOINT_HALT) {
                //TODO
                result = LPCLIB_SUCCESS;
            }
            break;

        case USBR_STANDARD_GET_STATUS:
           ((uint16_t *)pBuffer->data)[0] = 0;  //TODO return endpoint status
           pBuffer->maxSize = 2;
           result = LPCLIB_SUCCESS;
           break;

        case USBR_STANDARD_SET_FEATURE:
            if (pSetup->wValue == USBDEV_FEATURE_ENDPOINT_HALT) {
                //TODO
                result = LPCLIB_SUCCESS;
            }
            break;
        }
        break;
    }

    /* Always send a response */
    USBDEV_submitSetupResponse(handle,
                               (result == LPCLIB_SUCCESS) ? pBuffer : NULL);

    return result;
}



/** Handle a request (standard or class/vendor).
 *
 *  \retval LPCLIB_SUCCESS
 *  \retval LPCLIB_ILLEGAL_PARAMETER
 */
static LPCLIB_Result USBDEV_handleRequest (USBDEV_Handle handle,
                                           USB_SetupPacket *pSetup,
                                           USB_Buffer *pBuffer)
{
    LPCLIB_Result status;
    int n;


    /* Send request to all registered classes first.
     * A class handler must return LPCLIB_UNDEFINED if it hasn't fully handled the request.
     * Any other return value is immediately returned, and no othe classes (or the standard handler)
     * will see this request.
     */
    for (n = 0; n < handle->numClasses; n++) {
        status = handle->classes[n].requestHandler(handle, &handle->classes[n], pSetup, pBuffer);
        if (status != LPCLIB_UNDEFINED) {
            return status;                              /* Class has handled the request */
        }
    }

    /* Anything that's left now (not handled by class/vendor driver) is either a standard
     * request, or something illegal.
     */
    if (pSetup->Type == USB_REQUEST_TYPE_STANDARD) {    /* Filter for standard requests */
        status = USBDEV_handleStandardRequest(handle, pSetup, pBuffer);

        return status;
    }

    return LPCLIB_ILLEGAL_PARAMETER;
}


/* Config actions for stalling both control endpoints */
static const USB_Config stallControlEndpoints[] = {
//TODO: Better use the "conditional stall" in LPC1700 for control EP stall
#if 0
	{.opcode = USB_OPCODE_ENDPOINT_STALL, .continued = ENABLE,
     .logicalEndpoint = 0x00,
     {.stall = ENABLE, }},
#endif
	{.opcode = USB_OPCODE_ENDPOINT_STALL, .continued = DISABLE,
     .logicalEndpoint = 0x80,
     {.stall = ENABLE, }},
};


/** Check for a complete control transaction, and handle it. */
static void USBDEV_handleControlTransaction (USBDEV_Handle handle)
{
    LPCLIB_Result result;


//    handle->controlBuffer.data = usbdev.data;
    handle->controlState = USBDEV_CONTROLSTATE_SETUP_IN;
    result = USBDEV_handleRequest(handle, &handle->setup, &handle->controlBuffer);

    if (result != LPCLIB_SUCCESS) {
        /* Unknown or illegal request. Stall control endpoints. */
        USB_ioctl(handle->usbhw, &stallControlEndpoints[0]);
        handle->controlState = USBDEV_CONTROLSTATE_IDLE;
    }
}



/** Change device state based on device status events. */
static void USBDEV_handleDeviceStatusEvent (USBDEV_Handle handle, USB_DeviceStatus hwStatus)
{
    LPCLIB_Event event;

    if (hwStatus & USBDEVICE_STATUS_BUSRESET_Msk) {
        handle->state = USBDEV_STATE_DEFAULT;
        handle->activeConfiguration = 0;
        USB_DEBUG(2, printf("Reset\r\n"));
    }

    /* Change in suspend status? */
    if (hwStatus & USBDEVICE_STATUS_SUSPENDCHANGE_Msk) {
        /* Prepare event */
        event.id = LPCLIB_EVENTID_USBDEV;

        /* Enter suspend mode? */
        if (hwStatus & USBDEVICE_STATUS_SUSPENDED_Msk) {
            handle->isSuspended = true;
            event.opcode = USBDEV_EVENT_SUSPEND;
            USB_DEBUG(2, printf("Suspend\r\n"));
        }
        else {
            handle->isSuspended = false;
            event.opcode = USBDEV_EVENT_RESUME;
            USB_DEBUG(2, printf("Resume\r\n"));
        }

        /* Send event if someone has registered.
         * Do not report before having reached ADDRESS state.
         */
        if (handle->state >= USBDEV_STATE_ADDRESS) {
            if (handle->callback) {
                handle->callback(event);
            }
        }
    }
}



/* Worker function. */
void USBDEV_worker (USBDEV_Handle handle)
{
    USBDEV_Message *pMessage;
    USBDEV_Message *pMessageDeferred;
    uint32_t i;
    uint16_t frameNumber;
    USB_DeviceStatus status;
    const USB_Class *theClass;
    osEvent queueEvent;
    osEvent deferredQueueEvent;


    /* Wait here until a message wakes us up */
    queueEvent = osMailGet(usbdev.queue, osWaitForever);

    /* Check if there is a message for deferred handling (non-blocking). */
    deferredQueueEvent = osMailGet(usbdevDeferredQueue, 0);
    if (deferredQueueEvent.status == osEventMail) {
        pMessageDeferred = (USBDEV_Message *)deferredQueueEvent.value.p;
        USB_DEBUG(4, printf("(D)"));
        USB_doDeferredHandling(handle->usbhw, pMessageDeferred->endpointMask);
        osMailFree(usbdevDeferredQueue, deferredQueueEvent.value.p);
    }

    if (queueEvent.status == osEventMail) {
        pMessage = (USBDEV_Message *)queueEvent.value.p;

        USB_allowAccess(handle->usbhw, ENABLE);

        switch (pMessage->opcode) {

        case USBDEV_MESSAGE_DEFERRED:
            /* This is a dummy message (wake up only!) */
            break;

        /*******************************************************************/

        case USBDEV_MESSAGE_DEVICE_STATUS:
            USB_getDeviceStatus(handle->usbhw, &status);
            USBDEV_handleDeviceStatusEvent(handle, status);
            break;

        case USBDEV_MESSAGE_FRAME_INTERRUPT:
            /* Read the current frame number */
            USB_getFrameNumber(handle->usbhw, &frameNumber);
            pMessage->frameNumber = frameNumber;

            /* Send to all classes that have a frame handler. */
            for (i = 0; i < handle->numClasses; i++) {
                if (handle->classes[i].frameHandler) {
                    handle->classes[i].frameHandler(
                        handle->usbhw,
                        &handle->classes[i],
                        pMessage->frameNumber);
                }
            }

            /* Re-enable interrupts (if needed) */
            USB_manageFrameInterrupt();
            break;

        /********************************************/

        case USBDEV_MESSAGE_SETUP:
            /* Make a local copy of the setup packet */
            handle->setup = pMessage->setup;
            handle->controlState = USBDEV_CONTROLSTATE_IDLE;
            handle->controlBuffer.maxSize = USBDEV_CONTROL_BUFFER_SIZE;
            handle->controlBuffer.data = usbdev.data;

            /* Is this already complete, or will more data come in the next packet? */
            if ((handle->setup.wLength == 0) ||
                (handle->setup.DataDirection == USB_REQUEST_DIRECTION_DEVICE_TO_HOST)) {
                USBDEV_handleControlTransaction(handle);
            }
            else {
                /* More data expected */
                handle->controlState = USBDEV_CONTROLSTATE_SETUP_OUT;
                USB_read(handle->usbhw, 0x00, &(handle->controlBuffer));
            }
            break;

        case USBDEV_MESSAGE_SETUP_RESPONSE:
            /* Ignore if nobody is waiting for a response... */
            if (handle->controlState == USBDEV_CONTROLSTATE_SETUP_IN) {
                /* A NULL pointer instead of a buffer indicates a reject. */
                if (pMessage->pBuffer == NULL) {
                    /* Unknown or illegal request. Stall control endpoints. */
                    USB_ioctl(handle->usbhw, &stallControlEndpoints[0]);
                }
                else {
                    USB_write(handle->usbhw, 0x80, pMessage->pBuffer);
                }
                USB_DEBUG(4, printf("(SR):"));
            }
            else {
                USB_DEBUG(4, printf("(SRe):"));
            }
            break;

        /********************************************/

        case USBDEV_MESSAGE_BUFFER_COMPLETED:
            if (pMessage->logicalEndpoint == 0x00) {
                /* This should belong to a previously started setup packet. */
                if (handle->controlState == USBDEV_CONTROLSTATE_SETUP_OUT) {
                    USBDEV_handleControlTransaction(handle);
                }
            }
            else if (pMessage->logicalEndpoint == 0x80) {
            }
            else {
                /* Check if there is a class/handler for this endpoint */
                theClass = USBDEV_getClassFromEndpoint(handle, pMessage->logicalEndpoint);
                if (theClass) {
#if 0
                    if (theClass->endpointHandler) {
                        theClass->endpointHandler(handle,
                                                  theClass->classInstance,
                                                  message.logicalEndpoint,
                                                  message.buffer);
                    }
#endif
                }
            }
            break;

#if 1
        case USBDEV_MESSAGE_BUFFER_REQUEST:
            if (pMessage->logicalEndpoint == 0x00) {
                /* TODO: This should not happen! Just do a dummy read to empty the buffer */
//                USB_read(handle, 0x00, usbdev.controlBuffer);
            }
            else {
                /* Check if there is a class/handler for this endpoint */
                theClass = USBDEV_getClassFromEndpoint(handle, pMessage->logicalEndpoint);
                if (theClass) {
                    if (theClass->endpointHandler) {
                        theClass->endpointHandler(handle,
                                                 theClass,
                                                 pMessage->logicalEndpoint);
                    }
                }
            }
            break;
#endif

        case USBDEV_MESSAGE_CONNECT:
            USBDEV_connectBus(handle, ENABLE);
            break;
        }

        osMailFree(usbdev.queue, pMessage);

        /* Clocks may now be disabled */
        USB_allowAccess(handle->usbhw, DISABLE);
    }
}


