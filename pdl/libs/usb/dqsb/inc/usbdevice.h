/* Copyright (c) 2011-2012, NXP Semiconductors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 * Neither the name of the author nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 *  \brief USB device stack interface.
 *  This file defines all interface objects needed to use the USB device stack.
 *
 *  \author DF9DQ
 */


#ifndef __USBDEV_H
#define __USBDEV_H

/** \defgroup USB
 *  \ingroup API
 *  @{
 */

#include <stdbool.h>

#include "lpclib.h"

#include "usb.h"


/** Maximum number of interfaces in a configuration.
 *  Interfaces with the same bInterfaceNumber but different bAlternateSetting
 *  count as one interface here!
 */
#define USB_MAX_INTERFACES_PER_CONFIG       8



typedef struct USBDEV_Context *USBDEV_Handle;

/** USB request handler. */
typedef LPCLIB_Result (*USBDEV_RequestHandler)(USBDEV_Handle handle,
                                               const struct USB_Class *pClass,
                                               USB_SetupPacket *pSetup,
                                               USB_Buffer *pBuffer);


/** Opcodes to specify the configuration command in a call to \ref USBDEV_ioctl. */
typedef enum USBDEV_Opcode {
    USBDEV_OPCODE_INVALID = 0,              /**< (List terminator) */
    USBDEV_OPCODE_ATTACH_CLASSES,           /**< Attach a list of class handlers */
    USBDEV_OPCODE_ATTACH_DESCRIPTORS,       /**< Attach a list of descriptors */
    USBDEV_OPCODE_CONNECT,                  /**< Connect to bus */
    USBDEV_OPCODE_SET_CALLBACK,             /**< Install callback */
} USBDEV_Opcode;

/** Callback configuration. */
struct USBDEV_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};


/** Descriptor to specify the configuration in a call to \ref USBDEV_ioctl. */
typedef struct USBDEV_Config {
    USBDEV_Opcode opcode;                   /**< Config action opcode */
    uint8_t count;                          /**< Generic count value */

    union {
        const struct USB_Class *pClassList;             /**< List of class descriptors */
        const struct USB_DescriptorList *pDescriptors;  /**< Descriptors */
        struct USBDEV_ConfigCallback callback;          /**< Callback handler */
    };
} USBDEV_Config;

/** Config list terminator. */
#define USBDEV_CONFIG_END \
    {.opcode = USBDEV_OPCODE_INVALID}


typedef enum USBDEV_CallbackEvent {
    USBDEV_EVENT_SUSPEND = 0,               /**< Device is suspended */
    USBDEV_EVENT_RESUME,                    /**< Device has resumed */
} USBDEV_CallbackEvent;


/** \defgroup USBDEV_Public_Functions USBDEV API Functions
 *  @{
 */



/** Open a USB device stack.
 *
 *  \param[in] bus Handle of the hardware bus.
 *  \param[out] pHandle Handle to be used in future API calls to the USBDEV module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 *  \retval LPCLIB_BUSY Failure
 */
LPCLIB_Result USBDEV_open (USB_Handle bus, USBDEV_Handle *pHandle);


/** Configure the USB device stack.
 *
 *  Pass a configuration command to the USB device stack.
 *
 *  \param[in] handle USB device stack handle.
 *  \param[in] pConfig Pointer to a configuration descriptor
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle
 */
LPCLIB_Result USBDEV_ioctl (USBDEV_Handle handle, const USBDEV_Config *pConfig);


/** Worker function.
 *
 *  In an RTOS environment, this function is at the heart of the USB (device) task. The function
 *  blocks (in RTOS calls) if there is nothing to do.
 *  Non-RTOS applications must call this function regularly. In this case the function returns
 *  immediately if there is nothing to do for it.
 *
 *  \param[in] handle Device stack handle
 */
void USBDEV_worker (USBDEV_Handle handle);


/** Send response to SETUP packet (device to host).
 *
 *  This is called by upper layers (classes or application) in order to submit a buffer for
 *  the IN phase of a SETUP transaction.
 *
 *  \param[in] handle Device stack handle
 *  \param[in] pBuffer Data buffer
 */
void USBDEV_submitSetupResponse (USBDEV_Handle handle, USB_Buffer *pBuffer);


/** @} USBDEV API Functions */

/** @} USB */

#endif /* #ifndef __USBDEV_H__ */

