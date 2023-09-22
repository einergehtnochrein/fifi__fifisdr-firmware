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
 *  \brief USB driver interface.
 *  This file defines all interface objects needed to use the USB driver.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17XX_USB_H__
#define __LPC17XX_USB_H__

/** \defgroup USB
 *  \ingroup API
 *  @{
 */

#include "lpc17xx_libconfig.h"

#if LPCLIB_USB

#include "lpclib_types.h"



/** \defgroup USB_Public_Types USB Types, enums, macros
 *  @{
 */


/** Enumerator for the USB block.
 */
typedef enum USB_Name {
    USB0 = 0,           /**< First USB device block */
    __NUM_USB__         /* In order for this element to reflect the number of USB busses,
                         * you mustn't assign an explicit value to any of the other elements
                         * of this enum! (except for the first element which may get assigned to 0).
                         */
} USB_Name;


/** Handle for an open USB block, as obtained by \ref USB_open. */
typedef struct USB_Context *USB_Handle;


/** Opcodes to specify the configuration command in a call to \ref USB_ioctl. */
typedef enum USB_Opcode {
    USB_OPCODE_SELECT_DMA_ENDPOINTS,        /**< Specify endpoints with DMA support */
    USB_OPCODE_SET_ADDRESS,                 /**< Set bus address */
    USB_OPCODE_SOFTCONNECT,                 /**< Connect/disconnect to/from bus */
    USB_OPCODE_ENDPOINT_ACTIVATION,         /**< Activate/deactivate an endpoint */
    USB_OPCODE_DEACTIVATE_NON_CONTROL,      /**< Deactivate all non-control endpoints */
    USB_OPCODE_CONFIGURE_DEVICE,            /**< Configure/unconfigure device */
    USB_OPCODE_ENDPOINT_STALL,              /**< Stall/unstall an endpoint. */
    USB_OPCODE_NAK_INTERRUPT,               /**< Enable/disable NAK interrupt */
    USB_OPCODE_SET_CALLBACK,                /**< Set event handler callback */
    USB_OPCODE_SET_ISOC_ENDPOINTS,          /**< Determine which endpoints are isochronous
                                             *   This is hard-wired on the LPC1700!
                                             */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    USB_OPCODE_SELECT_PORT,                 /**< DSelect physical port (1 or 2) */
#endif
} USB_Opcode;


/** Config data for endpoint activation */
struct USB_ConfigEndpoint {
    LPCLIB_Switch activate;                 /**< ENABLE/DISABLE endpoint */
    uint16_t packetSize;                    /**< max. packet size */
};

/** Physical port selector. */
typedef enum USB_PhysicalPort {
    USB_PHYSICALPORT_1 = 0,                 /**< Port 1 (USB_D-1, USB_D+1) */
    USB_PHYSICALPORT_2 = 3,                 /**< Port 2 (USB_D-2, USB_D+2) */
} USB_PhysicalPort;


/** NAK interrupt configuration */
typedef uint8_t USB_ConfigNakInterrupt;


/** Descriptor to specify the configuration in a call to \ref USB_ioctl. */
typedef struct USB_Config {
    USB_Opcode opcode;                      /**< Config action opcode */
    LPCLIB_Switch continued;                /**< Set if further config struct's follow in an array */
    uint8_t logicalEndpoint;

    union {
        uint32_t dmaEndpointEnable;         /**< Bitmask to enable DMA for physical endpoints */
        uint8_t address;                    /**< Device address */
        LPCLIB_Switch connect;              /**< ENABLE/DISABLE SoftConnect */
        LPCLIB_Switch configure;            /**< ENABLE/DISABLE configuration */
        LPCLIB_Switch stall;                /**< ENABLE/DISABLE endpoint stall */
        USB_ConfigNakInterrupt nak;         /**< Configure NAK interrupts for all endpoint types. */
        struct USB_ConfigEndpoint endpointActivation;   /**< Endpoint activation */
        LPCLIB_Callback callback;           /**< Event handler */
        USB_PhysicalPort port;              /**< Port number */
    };
} USB_Config;


typedef enum USB_CallbackEvent {
    USB_EVENT_FRAME,                        /**< Frame interrupt */
    USB_EVENT_DEVICE_STATUS,                /**< Change in device status */
    USB_EVENT_FAST_ENDPOINT,                /**< Fast endpoint interrupt */
    USB_EVENT_SLOW_ENDPOINT,                /**< Slow endpoint interrupt */
    USB_EVENT_DMA_EOT,                      /**< DMA End Of Transfer */
    USB_EVENT_DMA_NDDR,                     /**< DMA New DD Request */
    USB_EVENT_NEED_BUFFER,                  /**< Request for a new buffer */
    USB_EVENT_BUFFER_DONE,                  /**< Return buffer to the device stack */
    USB_EVENT_DEFERRED,                     /**< Request for deferred interrupt handling */
    USB_EVENT_SETUP,                        /**< SETUP received via control pipe */
} USB_CallbackEvent;


typedef struct USB_CallbackContext {
    union {
        uint32_t endpointMask;              /**< Bit mask for 32 physical endpoints */
        uint32_t deviceStatus;              /**< Device status bits */
        void *pSetup;                       /**< Pointer to setup packet */
    };
} USB_CallbackContext;


enum {
    /* >> NOTE: These definitions must match the bit positions in USBSIE_SELEctEndpoint register! */
    USB_BUFFEREVENT_EP_STALLED = (1u << 1),     /**< Endpoint is stalled. */
    USB_BUFFEREVENT_EP_SETUP = (1u << 2),       /**< Received packet was a SETUP packet. */
    USB_BUFFEREVENT_EP_OVERRUN = (1u << 3),     /**< RX packet overwritten by SETUP packet. */
    USB_BUFFEREVENT_EP_NAK = (1u << 4),         /**< NAK has been sent. */
    /* << */

    /* NOTE: Next definitions must start with bit 8. */
    USB_BUFFEREVENT_EP_RXDATA = (1u << 8),      /**< New RX data (packet not necessarily complete) */
    USB_BUFFEREVENT_EP_COMPLETE = (1u << 9),    /**< RX or TX packet completed */
    USB_BUFFEREVENT_EP_RXOVERFLOW = (1u << 10), /**< RX data didin't fit into the buffer. */
};


typedef struct USB_Buffer {
    struct USB_Buffer *next;                /**< Next buffer in list */
    uint16_t someStatus;
    void *data;                             /**< The data! */
    uint8_t dmaStatus;
    uint8_t dummy;
    uint16_t dmaCount;
    uint32_t dmaIsocCountAddress;

    uint16_t maxSize;                       /**< Maximum number of bytes the buffer can hold (read)
                                             *   Total number of bytes in buffer (write)
                                             */
    int16_t currentSize;                    /**< Current number of bytes in buffer (read)
                                             *   Current number of bytes sent (write)
                                             */
    LPCLIB_Callback callback;               /**< Called when buffer ready/done or in case of an error. */
} USB_Buffer;


/** Bitmask for device status */
typedef uint8_t USB_DeviceStatus;
LPCLIB_DefineRegBit(USBDEVICE_STATUS_CONNECTED,     0,  1);
LPCLIB_DefineRegBit(USBDEVICE_STATUS_CONNECTCHANGE, 1,  1);
LPCLIB_DefineRegBit(USBDEVICE_STATUS_SUSPENDED,     2,  1);
LPCLIB_DefineRegBit(USBDEVICE_STATUS_SUSPENDCHANGE, 3,  1);
LPCLIB_DefineRegBit(USBDEVICE_STATUS_BUSRESET,      4,  1);



/** @} USB Types, enums, macros */



/** \defgroup USB_Public_Functions USB API Functions
 *  @{
 */


/** Open a USB (device) bus.
 *
 *  \param[in] bus Indicator that selects a USB interface block
 *  \param[out] pHandle Handle to be used in future API calls to the USB module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 *  \retval LPCLIB_BUSY Failure (interface already open). \ref pHandle does not
 *  contain a valid handle in this case.
 */
LPCLIB_Result USB_open (USB_Name bus, USB_Handle *pHandle);


/** Close a USB device.
 *
 *  \param[in] pHandle Device handle.
 */
void USB_close (USB_Handle *pHandle);


/** Configure the USB block.
 *
 *  Pass a configuration command to the USB block.
 *
 *  \param[in] handle Device handle.
 *  \param[in] pConfig Pointer to a configuration descriptor
 */
void USB_ioctl (USB_Handle handle, const USB_Config *pConfig);


/** Read from an endpoint.
 *
 *  //TODO
 */
LPCLIB_Result USB_read (USB_Handle handle, uint8_t logicalEndpoint, USB_Buffer *pBuffer);


/** Write to an endpoint.
 *
 *  //TODO
 */
LPCLIB_Result USB_write (USB_Handle handle, uint8_t logicalEndpoint, USB_Buffer *pBuffer);


/** Get frame number.
 *
 *  \param[in] handle Device handle.
 *  \param[out] pFrameNumber Current frame number.
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle
 */
LPCLIB_Result USB_getFrameNumber (USB_Handle handle, uint16_t *FrameNumber);


/** Get current device status.
 *
 *  \param[in] handle Device handle.
 *  \param[out] pDeviceStatus Current device status.
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle
 */
LPCLIB_Result USB_getDeviceStatus (USB_Handle handle, USB_DeviceStatus *pDeviceStatus);


/** Submit a data buffer.
 *
 *  \param[in] handle Device handle.
 *  \param[in] buffer Buffer descriptor
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle
 */
LPCLIB_Result USB_submitBuffer (USB_Handle handle, USB_Buffer *pBuffer);


/** Acquire/release the right to make calls to the device API.
 *
 *  You must call this function with ENABLE before you attempt to call other API
 *  functions that may try to access the hardware. Failure to do so may cause a
 *  hardfault. Calling this function multiple times is allowed.
 *  Call this function with DISABLE to save power when the register access
 *  is not needed.
 *
 *  \param[in] handle Device handle.
 *  \param[in] activate ENABLE/DISABLE access to API.
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle
 */
LPCLIB_Result USB_allowAccess (USB_Handle handle, LPCLIB_Switch activate);


/** Enable/disable frame interrupt as needed.
 *
 *  Ensures that frame interrupts are enable if needed by any isochronous endpoint.
 */
void USB_manageFrameInterrupt (void);


/** Deferred interrupt handling.
 *
 *  Must be called by the task in response to a USB_EVENT_DEFERRED event.
 *
 *  \param [in] endpointMask Taken from the event (....) TODO
 */
void USB_doDeferredHandling (USB_Handle handle, uint32_t endpointMask);


/** Initialize a buffer variable.
 *
 *  \param pBuffer Pointer to buffer variable
 *  \returns Same buffer pointer
 */
USB_Buffer * USB_initBuffer (USB_Buffer *pBuffer);


/** @} USB API Functions */


#endif /* #if LPCLIB_USB */

/** @} USB */

#endif /* #ifndef __LPC17XX_USB_H__ */

