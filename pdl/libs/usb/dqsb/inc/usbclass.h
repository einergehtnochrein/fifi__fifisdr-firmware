/* Copyright (c) 2011-2012, DF9DQ
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

#ifndef __USBCLASS_H
#define __USBCLASS_H

#include "usb.h"
#include "usbdevice.h"


/** Interface of a class handler. */
typedef void (*USBCLASS_InitFunc)(const struct USB_Class *pClass, LPCLIB_Switch enable);
typedef void (*USBCLASS_FrameHandler)(USB_Handle usbhw,
                                      const struct USB_Class *pClass,
                                      uint16_t frameNumber);
typedef LPCLIB_Result (*USBCLASS_EndpointHandler) (USBDEV_Handle handle,
                                                   const struct USB_Class *pClass,
                                                   uint8_t logicalEndpoint);



typedef struct USB_Class {
    uint8_t configuration;                      /**< Number of configuration for this class */
    uint8_t numEndpoints;                       /**< Number of endpoints in endpointList */
    const uint8_t *pEndpointList;               /**< Points to array with supported endpoints */

    USBCLASS_InitFunc init;                     /**< Called when the configuration is
                                                 *   activated or deactivated.
                                                 */
    USBDEV_RequestHandler requestHandler;       /**< Handles SETUP requests. */
    USBCLASS_FrameHandler frameHandler;
    USBCLASS_EndpointHandler endpointHandler;

    const void *pInstance;                      /**< Private instance data */
} USB_Class;


/* init             Called when configurations are activated/deactivated.
 *                  Parameter ENABLE if current configuration matches the class configuration.
 *                  Parameter DISABLE if current configuration is not the class configuration.
 *
 */

#endif

