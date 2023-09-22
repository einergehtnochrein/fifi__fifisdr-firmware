/* Copyright (c) 2011, NXP Semiconductors
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
 *  \brief USB device stack, descriptors
 *
 *  \author DF9DQ
 */


#ifndef __USBDESC_H
#define __USBDESC_H

#include "lpclib.h"


/* Descriptor Types */
#define USBD_DEVICE                                     (1)
#define USBD_CONFIGURATION                              (2)
#define USBD_STRING                                     (3)
#define USBD_INTERFACE                                  (4)
#define USBD_ENDPOINT                                   (5)
#define USBD_DEVICE_QUALIFIER                           (6)
#define USBD_OTHER_SPEED_CONFIGURATION                  (7)
#define USBD_INTERFACE_POWER                            (8)
#define USBD_OTG                                        (9)
#define USBD_DEBUG                                      (10)
#define USBD_INTERFACE_ASSOCIATION                      (11)
#define USBD_CS_DEVICE                                  (0x21)
#define USBD_CS_CONFIGURATION                           (0x22)
#define USBD_CS_STRING                                  (0x23)
#define USBD_CS_INTERFACE                               (0x24)
#define USBD_CS_ENDPOINT                                (0x25)


/* Length of descriptors (standard) */
#define USBD_SIZE_DEVICE                                18
#define USBD_SIZE_DEVICE_QUALIFIER                      10
#define USBD_SIZE_CONFIGURATION                         9
#define USBD_SIZE_INTERFACE                             9
#define USBD_SIZE_ENDPOINT                              7
#define USBD_SIZE_ENDPOINT_ISO                          9
#define USBD_SIZE_INTERFACE_ASSOCIATION                 8

/* Subtype and length of descriptors (Audio class) */
#define USBD_AUDIO_ST_HEADER                            (1)
#define USBD_SIZE_AUDIO_ST_HEADER(n)                    (8+(n))
#define USBD_AUDIO_ST_INPUT_TERMINAL                    (2)
#define USBD_SIZE_AUDIO_ST_INPUT_TERMINAL               (12)
#define USBD_AUDIO_ST_OUTPUT_TERMINAL                   (3)
#define USBD_SIZE_AUDIO_ST_OUTPUT_TERMINAL              (9)
#define USBD_AUDIO_ST_MIXER_UNIT                        (4)
#define USBD_AUDIO_ST_SELECTOR_UNIT                     (5)
#define USBD_SIZE_AUDIO_ST_SELECTOR_UNIT(n)             (6+(n))
#define USBD_AUDIO_ST_FEATURE_UNIT                      (6)
#define USBD_SIZE_AUDIO_ST_FEATURE_UNIT(ch,n)           (7+((ch)+1)*(n))
#define USBD_AUDIO_ST_PROCESSING_UNIT                   (7)
#define USBD_SIZE_AUDIO_ST_PROCESSING_UNIT(p,n)         (13+(p)+(n))
#define USBD_AUDIO_ST_EXTENSION_UNIT                    (8)


/* USB Class Codes and applicable sub classes and protocols */
#define USBC_BASE                                       (0x00)
#define USBC_AUDIO                                      (0x01)
#define   USBCS_AUDIO_AUDIOCONTROL                        (0x01)
#define   USBCS_AUDIO_AUDIOSTREAMING                      (0x02)
#define   USBCS_AUDIO_MIDISTREAMING                       (0x03)
#define USBC_CDC_CONTROL                                (0x02)
#define USBC_HID                                        (0x03)
#define USBC_PHYSICAL                                   (0x05)
#define USBC_IMAGE                                      (0x06)
#define USBC_PRINTER                                    (0x07)
#define USBC_MASS_STORAGE                               (0x08)
#define   USBCS_MASS_STORAGE_RBC                          (0x01)
#define   USBCS_MASS_STORAGE_MMC2                         (0x02)
#define   USBCS_MASS_STORAGE_QIC157                       (0x03)
#define   USBCS_MASS_STORAGE_UFI                          (0x04)
#define   USBCS_MASS_STORAGE_SFF8070I                     (0x05)
#define   USBCS_MASS_STORAGE_SCSI                         (0x06)
#define     USBCP_MASS_STORAGE_CBI_WITH                     (0x00)
#define     USBCP_MASS_STORAGE_CBI_WITHOUT                  (0x01)
#define     USBCP_MASS_STORAGE_BULK_ONLY                    (0x50)
#define USBC_HUB                                        (0x09)
#define USBC_CDC_DATA                                   (0x0A)
#define USBC_SMART_CARD                                 (0x0B)
#define USBC_CONTENT_SECURITY                           (0x0D)
#define USBC_VIDEO                                      (0x0E)
#define USBC_PERSONAL_HEALTHCARE                        (0x0F)
#define USBC_DIAGNOSTIC_DEVICE                          (0xDC)
#define USBC_WIRELESS_CONTROLLER                        (0xE0)
#define USBC_MISCELLANEOUS                              (0xEF)
#define USBC_APPLICATION_SPECIFIC                       (0xFE)
#define   USBCS_APPLICATION_SPECIFIC_DFU                  (0x01)
#define     USBCP_APPLICATION_SPECIFIC_DFU                  (0x01)
#define   USBCS_APPLICATION_SPECIFIC_IRDA_BRIDGE          (0x02)
#define   USBCS_APPLICATION_SPECIFIC_TAM                  (0x03)
#define     USBCP_APPLICATION_SPECIFIC_TAM                  (0x00)
#define     USBCP_APPLICATION_SPECIFIC_TAM_USB488           (0x01)
#define USBC_VENDOR_SPECIFIC                            (0xFF)


/* For convenience */
#define LE16(w) ((w) % 256UL), ((w) / 256UL)
#define LE24(w) ((w) % 256UL), (((w) % 65536UL) / 256UL), ((w) / 65536UL)

/* If you want the serial number string (device descriptor) to be replaced
 * by the 128-bit device serial number (as ASCII string), set this to the
 * same value as iSerialNumber in the device descriptor. If not, set to 0.
 */
#define USB_SERIAL_STRING                               (3)


/** Table that combines string descriptors for all languages. */
typedef LPCLIB_Result (*USB_CallbackGetStringDescriptor)(uint8_t index,
                                                         uint16_t languageId,
                                                         const void **pString);


/** USB String Descriptor */
#define DECLARE_STRING_DESCRIPTOR(name,theText)     \
    const __PACKED(struct {                         \
        uint8_t bLength;                            \
        uint8_t bDescriptorType;                    \
        wchar_t text[sizeof(theText)/2-1];})        \
        name = {                                    \
            .bLength = sizeof(theText),             \
            .bDescriptorType = USBD_STRING,         \
            .text = theText,                        \
        }


/** String descriptor 0 with list of language ID's. */
#define DECLARE_LANGUAGE_ID_DESCRIPTOR(name,num,...)\
    const __PACKED(struct {                         \
        uint8_t bLength;                            \
        uint8_t bDescriptorType;                    \
        uint16_t langId[num];})                     \
        name = {                                    \
            .bLength = 2 + 2 * num,                 \
            .bDescriptorType = USBD_STRING,         \
            .langId = {__VA_ARGS__},                \
        }


/** USB device descriptor. */
typedef __PACKED(struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint16_t    bcdUSB;
    uint8_t     bDeviceClass;
    uint8_t     bDeviceSubClass;
    uint8_t     bDeviceProtocol;
    uint8_t     bMaxPacketSize0;
    uint16_t    idVendor;
    uint16_t    idProduct;
    uint16_t    bcdDevice;
    uint8_t     iManufacturer;
    uint8_t     iProduct;
    uint8_t     iSerialNumber;
    uint8_t     bNumConfigurations;
}) USB_DeviceDescriptor;

/** USB Device Qualifier Descriptor. */
typedef struct __attribute__((packed)) {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint16_t    bcdUSB;
    uint8_t     bDeviceClass;
    uint8_t     bDeviceSubClass;
    uint8_t     bDeviceProtocol;
    uint8_t     bMaxPacketSize0;
    uint8_t     bNumConfigurations;
    uint8_t     bReserved;
} USB_DeviceQualifierDescriptor;

/** USB configuration descriptor. */
typedef struct __attribute__((packed)) {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint16_t    wTotalLength;
    uint8_t     bNumInterfaces;
    uint8_t     bConfigurationValue;
    uint8_t     iConfiguration;
    uint8_t     bmAttributes;
    uint8_t     bMaxPower;
} USB_ConfigurationDescriptor;

/** USB interface descriptor. */
typedef struct __attribute__((packed)) {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bInterfaceNumber;
    uint8_t     bAlternateSetting;
    uint8_t     bNumEndpoints;
    uint8_t     bInterfaceClass;
    uint8_t     bInterfaceSubClass;
    uint8_t     bInterfaceProtocol;
    uint8_t     iInterface;
} USB_InterfaceDescriptor;

/** USB standard endpoint descriptor. */
typedef  __PACKED(struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bEndpointAddress;
    uint8_t     bmAttributes;
    uint16_t    wMaxPacketSize;
    uint8_t     bInterval;
}) USB_EndpointDescriptor;

/** Isochronous endpoint descriptor. */
typedef __PACKED(struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bEndpointAddress;
    uint8_t     bmAttributes;
    uint16_t    wMaxPacketSize;
    uint8_t     bInterval;
    uint8_t     bRefresh;
    uint8_t     bSyncAddress;
}) USB_EndpointDescriptorIso;

/** USB interface association descriptor. */
typedef __PACKED(struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bFirstInterface;
    uint8_t     bInterfaceCount;
    uint8_t     bFunctionClass;
    uint8_t     bFunctionSubClass;
    uint8_t     bFunctionProtocol;
    uint8_t     iFunction;
}) USB_InterfaceAssociationDescriptor;


/** Element of an array of configuration descriptors. */
typedef struct {
    uint16_t    length;
    const void  *descriptor;
} USB_ConfigurationArrayElement;


typedef struct USB_DescriptorList {
    const USB_DeviceDescriptor *device;
    uint8_t numConfigurations;
    USB_CallbackGetStringDescriptor callbackGetString;
    const USB_ConfigurationArrayElement *configurations;
} USB_DescriptorList;


#endif  /* __USBDESC_H */

