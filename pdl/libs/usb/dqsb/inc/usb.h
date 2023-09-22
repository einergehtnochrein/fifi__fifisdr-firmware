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

#ifndef __USB_H
#define __USB_H

#include "lpclib.h"

#include <stdint.h>


/** USB Standard Requests */
#define USBR_STANDARD_GET_STATUS                0
#define USBR_STANDARD_CLEAR_FEATURE             1
#define USBR_STANDARD_SET_FEATURE               3
#define USBR_STANDARD_SET_ADDRESS               5
#define USBR_STANDARD_GET_DESCRIPTOR            6
#define USBR_STANDARD_SET_DESCRIPTOR            7
#define USBR_STANDARD_GET_CONFIGURATION         8
#define USBR_STANDARD_SET_CONFIGURATION         9
#define USBR_STANDARD_GET_INTERFACE             10
#define USBR_STANDARD_SET_INTERFACE             11
#define USBR_STANDARD_SYNCH_FRAME               12


/** Setup packet definition. */
typedef __PACKED(struct USB_SetupPacket {
    __PACKED(union {
        uint8_t     bmRequestType;      /**< characteristics of the specific request */
        __PACKED(struct {
            uint8_t Recipient:5;
            uint8_t Type:2;
            uint8_t DataDirection:1;
        });
    });
    uint8_t     bRequest;               /**< specific request */
    uint16_t    wValue;
    __PACKED(union {
        uint16_t    wIndex;
        __PACKED(struct {
            uint16_t interface:8;
            uint16_t reserved1:8;
        });
        __PACKED(struct {
            uint16_t endpoint:8;
            uint16_t reserved2:8;
        });
    });
    uint16_t    wLength;                /**< length of data transfered in data phase */
} ) USB_SetupPacket;



enum {
    USB_REQUEST_DIRECTION_HOST_TO_DEVICE = 0,
    USB_REQUEST_DIRECTION_DEVICE_TO_HOST = 1,
};

enum {
    USB_REQUEST_TYPE_STANDARD = 0,
    USB_REQUEST_TYPE_CLASS = 1,
    USB_REQUEST_TYPE_VENDOR = 2,
};

enum {
    USB_REQUEST_RECIPIENT_DEVICE = 0,
    USB_REQUEST_RECIPIENT_INTERFACE = 1,
    USB_REQUEST_RECIPIENT_ENDPOINT = 2,
    USB_REQUEST_RECIPIENT_OTHER = 3,
};


#define USB_DEBUG_LEVEL 0x00
#define USB_DEBUG(level,x)              \
    if(level & USB_DEBUG_LEVEL) {       \
        if(level & 0x80) {              \
            extern void outbyte(int c); \
            outbyte(x);                 \
        }                               \
        else {                          \
            x;                          \
        }                               \
    }

#endif

/* Forward declaration for USB class descriptor */
struct USB_Class;


