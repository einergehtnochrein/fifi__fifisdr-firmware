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


#include "lpc17xx_libconfig.h"

#if LPCLIB_USB

/** \file
 *  \brief USB driver implementation.
 *
 *  This file contains the driver code for the USB peripheral.
 *
 *  \author NXP Semiconductors
 */


/** \addtogroup USB
 *  @{
 */

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_usb.h"


LPCLIB_DefineRegBit(USB_PORTSEL_PORTSEL,        0,  2);

LPCLIB_DefineRegBit(USB_CLKCTRL_HOST_CLK_EN,    0,  1);
LPCLIB_DefineRegBit(USB_CLKCTRL_DEV_CLK_EN,     1,  1);
LPCLIB_DefineRegBit(USB_CLKCTRL_I2C_CLK_EN,     2,  1);
LPCLIB_DefineRegBit(USB_CLKCTRL_OTG_CLK_EN,     3,  1); /* Name used in OTG context */
LPCLIB_DefineRegBit(USB_CLKCTRL_PORTSEL_CLK_EN, 3,  1); /* Name used in DEVICE context */
LPCLIB_DefineRegBit(USB_CLKCTRL_AHB_CLK_EN,     4,  1);

LPCLIB_DefineRegBit(USB_CLKST_HOST_CLK_ON,      0,  1);
LPCLIB_DefineRegBit(USB_CLKST_DEV_CLK_ON,       1,  1);
LPCLIB_DefineRegBit(USB_CLKST_I2C_CLK_ON,       2,  1);
LPCLIB_DefineRegBit(USB_CLKST_OTG_CLK_ON,       3,  1); /* Name used in OTG context */
LPCLIB_DefineRegBit(USB_CLKST_PORTSEL_CLK_ON,   3,  1); /* Name used in DEVICE context */
LPCLIB_DefineRegBit(USB_CLKST_AHB_CLK_ON,       4,  1);

LPCLIB_DefineRegBit(USB_DEVINTST_FRAME,         0,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_EP_FAST,       1,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_EP_SLOW,       2,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_DEV_STAT,      3,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_CCEMPTY,       4,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_CDFULL,        5,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_RXENDPKT,      6,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_TXENDPKT,      7,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_EP_RLZED,      8,  1);
LPCLIB_DefineRegBit(USB_DEVINTST_ERR_INT,       9,  1);

LPCLIB_DefineRegBit(USB_DEVINTEN_FRAME,         0,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_EP_FAST,       1,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_EP_SLOW,       2,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_DEV_STAT,      3,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_CCEMPTY,       4,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_CDFULL,        5,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_RXENDPKT,      6,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_TXENDPKT,      7,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_EP_RLZED,      8,  1);
LPCLIB_DefineRegBit(USB_DEVINTEN_ERR_INT,       9,  1);

LPCLIB_DefineRegBit(USB_DEVINTCLR_FRAME,        0,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_EP_FAST,      1,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_EP_SLOW,      2,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_DEV_STAT,     3,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_CCEMPTY,      4,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_CDFULL,       5,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_RXENDPKT,     6,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_TXENDPKT,     7,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_EP_RLZED,     8,  1);
LPCLIB_DefineRegBit(USB_DEVINTCLR_ERR_INT,      9,  1);

LPCLIB_DefineRegBit(USB_RXPLEN_PKT_LNGTH,       0,  10);
LPCLIB_DefineRegBit(USB_RXPLEN_DV,              10, 1);
LPCLIB_DefineRegBit(USB_RXPLEN_PKT_RDY,         11, 1);

LPCLIB_DefineRegBit(USB_CTRL_RD_EN,             0,  1);
LPCLIB_DefineRegBit(USB_CTRL_WR_EN,             1,  1);
LPCLIB_DefineRegBit(USB_CTRL_LOG_ENDPOINT,      2,  4);

LPCLIB_DefineRegBit(USB_CMDCODE_CMD_PHASE,      8,  8);
LPCLIB_DefineRegBit(USB_CMDCODE_CMD_CODE,       16, 8);
LPCLIB_DefineRegBit(USB_CMDCODE_CMD_WDATA,      16, 8);

LPCLIB_DefineRegBit(USB_CMDDATA_CMD_RDATA,      0,  8);

LPCLIB_DefineRegBit(USB_DMAINTST_EOT,           0,  1);
LPCLIB_DefineRegBit(USB_DMAINTST_NDDR,          1,  1);
LPCLIB_DefineRegBit(USB_DMAINTST_ERR,           2,  1);

LPCLIB_DefineRegBit(USB_DMAINTEN_EOT,           0,  1);
LPCLIB_DefineRegBit(USB_DMAINTEN_NDDR,          1,  1);
LPCLIB_DefineRegBit(USB_DMAINTEN_ERR,           2,  1);



/** SIE command opcodes */
enum {
    USBSIE_SetAddress                       = 0xD0,
    USBSIE_ConfigureDevice                  = 0xD8,
    USBSIE_SetMode                          = 0xF3,
    USBSIE_ReadCurrentFrameNumber           = 0xF5,
    USBSIE_ReadTestRegister                 = 0xFD,
    USBSIE_SetDeviceStatus                  = 0xFE,
    USBSIE_GetDeviceStatus                  = 0xFE,
    USBSIE_GetErrorCode                     = 0xFF,
    USBSIE_ReadErrorStatus                  = 0xFB,
    USBSIE_SelectEndpoint0                  = 0x00,
    USBSIE_SelectEndpoint0_ClearInterrupt   = 0x40,
    USBSIE_SetEndpointStatus0               = 0x40,
    USBSIE_ClearBuffer                      = 0xF2,
    USBSIE_ValidateBuffer                   = 0xFA,
};


/** SIE register bits. */
LPCLIB_DefineRegBit(USBSIE_SETADDRESS_DEV_ADDR,             0,  7);
LPCLIB_DefineRegBit(USBSIE_SETADDRESS_DEV_EN,               7,  1);
LPCLIB_DefineRegBit(USBSIE_CONFIGUREDEVICE_CONF_DEVICE,     0,  1);
LPCLIB_DefineRegBit(USBSIE_SETMODE_AP_CLK,                  0,  1);
LPCLIB_DefineRegBit(USBSIE_SETMODE_INAK_CI,                 1,  1);
LPCLIB_DefineRegBit(USBSIE_SETMODE_INAK_CO,                 2,  1);
LPCLIB_DefineRegBit(USBSIE_SETMODE_INAK_II,                 3,  1);
LPCLIB_DefineRegBit(USBSIE_SETMODE_INAK_IO,                 4,  1);
LPCLIB_DefineRegBit(USBSIE_SETMODE_INAK_BI,                 5,  1);
LPCLIB_DefineRegBit(USBSIE_SETMODE_INAK_BO,                 6,  1);
LPCLIB_DefineRegBit(USBSIE_SETDEVICESTATUS_CON,             0,  1);
LPCLIB_DefineRegBit(USBSIE_SETDEVICESTATUS_CON_CH,          1,  1);
LPCLIB_DefineRegBit(USBSIE_SETDEVICESTATUS_SUS,             2,  1);
LPCLIB_DefineRegBit(USBSIE_SETDEVICESTATUS_SUS_CH,          3,  1);
LPCLIB_DefineRegBit(USBSIE_SETDEVICESTATUS_RST,             4,  1);
LPCLIB_DefineRegBit(USBSIE_GETDEVICESTATUS_CON,             0,  1);
LPCLIB_DefineRegBit(USBSIE_GETDEVICESTATUS_CON_CH,          1,  1);
LPCLIB_DefineRegBit(USBSIE_GETDEVICESTATUS_SUS,             2,  1);
LPCLIB_DefineRegBit(USBSIE_GETDEVICESTATUS_SUS_CH,          3,  1);
LPCLIB_DefineRegBit(USBSIE_GETDEVICESTATUS_RST,             4,  1);
LPCLIB_DefineRegBit(USBSIE_GETERRORCODE_EC,                 0,  4);
LPCLIB_DefineRegBit(USBSIE_GETERRORCODE_EA,                 4,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_PID_ERR,         0,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_UEPKT,           1,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_DCRC,            2,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_TIMEOUT,         3,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_EOP,             4,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_B_OVRN,          5,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_BTSTF,           6,  1);
LPCLIB_DefineRegBit(USBSIE_READERRORSTATUS_TGL_ERR,         7,  1);
LPCLIB_DefineRegBit(USBSIE_SELECTENDPOINT_FE,               0,  1);
LPCLIB_DefineRegBit(USBSIE_SELECTENDPOINT_ST,               1,  1);
LPCLIB_DefineRegBit(USBSIE_SELECTENDPOINT_STP,              2,  1);
LPCLIB_DefineRegBit(USBSIE_SELECTENDPOINT_PO,               3,  1);
LPCLIB_DefineRegBit(USBSIE_SELECTENDPOINT_EPN,              4,  1);
LPCLIB_DefineRegBit(USBSIE_SELECTENDPOINT_B_1_FULL,         5,  1);
LPCLIB_DefineRegBit(USBSIE_SELECTENDPOINT_B_2_FULL,         6,  1);
LPCLIB_DefineRegBit(USBSIE_SETENDPOINTSTATUS_ST,            0,  1);
LPCLIB_DefineRegBit(USBSIE_SETENDPOINTSTATUS_DA,            5,  1);
LPCLIB_DefineRegBit(USBSIE_SETENDPOINTSTATUS_RF_MO,         6,  1);
LPCLIB_DefineRegBit(USBSIE_SETENDPOINTSTATUS_CND_ST,        7,  1);
LPCLIB_DefineRegBit(USBSIE_CLEARBUFFER_PO,                  0,  1);


/** Endpoint bit mask. Contains a 1 for every isochronous endpoint. */
#define USBHWD_ISO_ENDPOINT_BITMASK     (0x030C30C0)


/** Endpoint context. */
typedef struct USB_EndpointContext {
    USB_Buffer *pBuffer;                /**< R/W data */
    uint16_t packetSize;                /**< Max size of packets */
    LPCLIB_Switch busy;                 /**< Set if buffer is in use */
} USB_EndpointContext;


/** Local context of USB block. */
static struct USB_Context {
    USB_Name bus;
    uint32_t dmaEnabledEndpoint;        /**< Bitmask (32 physical endpoints);
                                         *   DMA enabled for that endpoint */
    uint32_t fastEndpoint;              /**< Bitmask (32 physical endpoints);
                                         *   Fast endpoint (high priority).
                                         */

    USB_EndpointContext endpoints[32];  /**< Endpoint context */
    LPCLIB_Switch needClock;            /**< Clock is required by task level. */
    osMutexId accessMutex;
    LPCLIB_Callback callback;           /**< Callback (device stack) */
    uint32_t errorSpuriousEpInt;        /**< Counts spurious endpoint interrupts.
                                         *   TODO Such interrupts (self-clearing USBEpIntSt bit)
                                         *        have been seen for Control IN endpoint. Maybe
                                         *        caused by a wrong init sequence?
                                         */
    uint32_t setupPacket[2];            /**< SETUP packet (organized as two 32-bit words
                                         *   for convenience)
                                         */
} usbContext;



osMutexDef(usbAccessMutex);


/** USB DMA Descriptor. */
typedef struct usbdevice_DmaDescriptor_tag {
    struct usbdevice_DmaDescriptor_tag *next;
    uint32_t value1;
    uint32_t *dma_buffer_start;     /* Start address of DMA buffer */
    uint32_t value3;
    uint32_t *isoc_packetsize_memory_address;
} usbhwd_DmaDescriptor;

/** USB Device Communication Area. */
static usbhwd_DmaDescriptor *usbhwd_udca[32] __attribute__ ((section(".bss2"),aligned (128)));

#if 0
#define FRAMES_PER_DD (20)
static usbhwd_DmaDescriptor usb_dmaDesc[3] __attribute__ ((section(".bss2")));
static uint32_t usbhwd_isocPackets[2][FRAMES_PER_DD] __attribute__ ((section(".bss2")));
#endif

/** Enable the clock for USB device register access.
 *
 *  Access to any device controller register requires the 'usbclk' to be active. Since usbclk is
 *  deactivated by hardware in case of a bus suspend, there is no guarantee that usbclk is
 *  active when needed. Therefore, usbclk must be forced ON before any register access is
 *  attempted. It should be set to automatic mode afterwards.
 */
static void USB_enableUsbClk (void)
{
    if (usbContext.needClock == DISABLE) {
        usbContext.needClock = ENABLE;
        LPC_USB->USBClkCtrl |= USB_CLKCTRL_DEV_CLK_EN_Msk;
        while ((LPC_USB->USBClkSt & USB_CLKST_DEV_CLK_ON_Msk) == 0)
            ;
    }
}


/** Disable the clock for USB device register access. */
static void USB_disableUsbClk (void)
{
    usbContext.needClock = DISABLE;
    LPC_USB->USBClkCtrl &= ~USB_CLKCTRL_DEV_CLK_EN_Msk;
}


/** Enable the clock for USB device register access (call from ISR). */
static void USB_enableUsbClkFromISR (void)
{
    if ((LPC_USB->USBClkSt & USB_CLKST_DEV_CLK_ON_Msk) == 0) {
        LPC_USB->USBClkCtrl |= USB_CLKCTRL_DEV_CLK_EN_Msk;
        while ((LPC_USB->USBClkSt & USB_CLKST_DEV_CLK_ON_Msk) == 0)
            ;
    }
}


/** Disable the clock for USB device register access (call from ISR). */
static void USB_disableUsbClkFromISR (void)
{
    if (usbContext.needClock == DISABLE) {
        LPC_USB->USBClkCtrl &= ~USB_CLKCTRL_DEV_CLK_EN_Msk;
    }
}



/** Conversion from logical endpoint (0x82/0x02) to physical endpoint (0x05/0x04). */
static uint8_t USB_log2phy (uint8_t logicalEndpoint)
{
    return ((logicalEndpoint & 0x0F) << 1) | ((logicalEndpoint & 0x80) >> 7);
}



/** Conversion from physical endpoint (0x0C/0x13) to logical endpoint (0x06/0x89). */
static uint8_t USB_phy2log (uint8_t physicalEndpoint)
{
    return ((physicalEndpoint & 0x1E) >> 1) | ((physicalEndpoint & 0x01) << 7);
}


/** Wait for (and then clear) a device status bit. */
static void USB_waitStatus (uint32_t bitMask)
{
    while (!(LPC_USB->DevIntSt & bitMask))
        ;
    LPC_USB->DevIntClr = bitMask;
}



/** Send command without data phase to SIE.
 *
 *  \param [in] command SIE command
 */
static void USB_cmdNodata (uint8_t command)
{
    /* Enter command code (command phase) */
    LPC_USB->CmdCode = (5 << USB_CMDCODE_CMD_PHASE_Pos)
                     | (command << USB_CMDCODE_CMD_CODE_Pos);

    /* Wait until it completes */
    USB_waitStatus(USB_DEVINTST_CCEMPTY_Msk);
}



/** Send command and write 1 byte to SIE.
 *
 *  \param [in] command SIE command
 *  \param [in] data data
 */
static void USB_cmdWrite (uint8_t command, uint8_t data)
{
    /* Send command */
    USB_cmdNodata(command);

    /* Append data */
    LPC_USB->CmdCode = (1 << USB_CMDCODE_CMD_PHASE_Pos)
                     | (data << USB_CMDCODE_CMD_WDATA_Pos);

    /* Wait until it completes */
    USB_waitStatus(USB_DEVINTST_CCEMPTY_Msk);
}



/** Send command and read 1 byte from SIE.
 *
 *  \param [in] command SIE command
 *  \return data
 */
static uint8_t USB_cmdRead1 (uint8_t command)
{
    /* Send command */
    USB_cmdNodata(command);

    /* Read data */
    LPC_USB->CmdCode = (2 << USB_CMDCODE_CMD_PHASE_Pos)
                     | (command << USB_CMDCODE_CMD_CODE_Pos);

    /* Wait until it completes */
    USB_waitStatus(USB_DEVINTST_CDFULL_Msk);

    /* Return received data */
    return LPC_USB->CmdData;
}



/** Send command and read 2 bytes from SIE.
 *
 *  \param [in] command SIE command
 *  \return Two bytes as half-word
 */
static uint16_t USB_cmdRead2 (uint8_t command)
{
    uint16_t data;


    /* Send command */
    USB_cmdNodata(command);

    /* Read first byte */
    LPC_USB->CmdCode = (2 << USB_CMDCODE_CMD_PHASE_Pos)
                     | (command << USB_CMDCODE_CMD_CODE_Pos);

    /* Wait until it completes */
    USB_waitStatus(USB_DEVINTST_CDFULL_Msk);

    /* Remember first byte */
    data = LPC_USB->CmdData;

    /* Read second byte */
    LPC_USB->CmdCode = (2 << USB_CMDCODE_CMD_PHASE_Pos)
                     | (command << USB_CMDCODE_CMD_CODE_Pos);

    /* Wait until it completes */
    USB_waitStatus(USB_DEVINTST_CDFULL_Msk);

    /* Return complete halfword */
    data |= (LPC_USB->CmdData << 8);
    return data;
}



/** Enable/disable frame interrupt as needed. */
void USB_manageFrameInterrupt (void)
{
    /* Enable/disable frame interrupt based on whether iso endpoints are enabled,
     * and whether or nor they are served by DMA.
     */
    if ((LPC_USB->ReEp & ~(usbContext.dmaEnabledEndpoint)) & USBHWD_ISO_ENDPOINT_BITMASK) {
        LPC_USB->DevIntEn |= USB_DEVINTEN_FRAME_Msk;
    }
    else {
        LPC_USB->DevIntEn &= ~USB_DEVINTEN_FRAME_Msk;
    }
}



/** Activate/deactivate a single endpoint.
 *
 *  \param [in] endpoint Number of endpoint
 *  \param [in] packet_size Maximum packet size (set to 0 if you deactivate an endpoint)
 *  \param [in] enable boolean value to enable/disable the endpoint.
 */
static void USB_activateEndpoint (uint8_t logicalEndpoint, LPCLIB_Switch activate, uint16_t packetSize)
{
    uint8_t phy_ep;


    /* Calculate the register index */
    phy_ep = USB_log2phy(logicalEndpoint);

    /* Remember endpoint size */
    usbContext.endpoints[phy_ep].packetSize = packetSize;

    if (activate) {
        if ((phy_ep < 2) || !(LPC_USB->ReEp & (1u << phy_ep))) {
            /* Realize the endpoint. */
            LPC_USB->ReEp |= (1u << phy_ep);
            LPC_USB->EpInd = phy_ep;
            LPC_USB->MaxPSize = packetSize;

            /* Wait until it's done */
            USB_waitStatus(USB_DEVINTST_EP_RLZED_Msk);

            /* Enable the endpoint */
            USB_cmdWrite(USBSIE_SetEndpointStatus0 + phy_ep, 0);

            /* Enable endpoint interrupt or DMA mode */
            if (usbContext.dmaEnabledEndpoint & (1u << phy_ep)) {
                LPC_USB->EpIntEn &= ~(1u << phy_ep);
                LPC_USB->EpDMAEn = (1u << phy_ep);
            }
            else {
                LPC_USB->EpDMADis = (1u << phy_ep);
                LPC_USB->EpIntEn |= (1u << phy_ep);
            }
        }
    }
    else {
        LPC_USB->ReEp &= ~(1u << phy_ep);
    }

    USB_manageFrameInterrupt();
}



/** Perform the read from an endpoint.
 *
 *  \param[in] handle Device handle
 *  \param[in] buffer Data buffer
 *  \param[in] pLastPacket
 *  \retval LPC_SUCCESS Data fit into the buffer
 *  \retval LPCLIB_ERROR Data did not fit into the buffer
 */
static LPCLIB_Result USB_performEndpointRead (USB_Handle handle,
                                              uint8_t physicalEndpoint,
                                              LPCLIB_Switch *pLastPacket
                                             )
{
    uint32_t rxData;
    uint16_t packetLength;
    uint16_t n;
    uint8_t *p;
    USB_EndpointContext *pEp = &(handle->endpoints[physicalEndpoint]);
    USB_Buffer *pBuffer = pEp->pBuffer;
    LPCLIB_Result result = LPCLIB_SUCCESS;


    /* Select read mode for the endpoint */
    LPC_USB->Ctrl =
            USB_CTRL_RD_EN_Msk |
            ((USB_phy2log(physicalEndpoint) << USB_CTRL_LOG_ENDPOINT_Pos) & USB_CTRL_LOG_ENDPOINT_Msk);

    /* Wait for the packet length to become available in USBRxPLen. */
    while (!(LPC_USB->RxPLen & USB_RXPLEN_PKT_RDY_Msk))
        ;

    /* Extract the packet length field. */
    packetLength = (LPC_USB->RxPLen & USB_RXPLEN_PKT_LNGTH_Msk) >> USB_RXPLEN_PKT_LNGTH_Pos;
    *pLastPacket = (packetLength < pEp->packetSize) ? ENABLE : DISABLE;

    /* Read data. Make sure we read at least once, even for zero-length packets. */
    rxData = 0;
    n = 0;
    p = NULL;
    if (pBuffer) {
        p = pBuffer->data;
    }

    do {
        /* RX register is word-wide. Read it every fourth byte. */
        if ((n % 4) == 0) {
            rxData = LPC_USB->RxData;
        }

        if (p == NULL) {
            result = LPCLIB_ERROR;
        }
        else if (n < packetLength) {
            /* Write byte-wise. Not too inefficient... */
            if (pBuffer->currentSize < pBuffer->maxSize) {
                p[pBuffer->currentSize] = rxData & 0xFF;
                ++(pBuffer->currentSize);
            }
            else {
                result = LPCLIB_ERROR;
            }
        }
        rxData >>= 8;

        ++n;
    } while (n < packetLength);

    if (*pLastPacket || ((1u << physicalEndpoint) & USBHWD_ISO_ENDPOINT_BITMASK)) {
        pEp->busy = DISABLE;
    }

    /* Buffer has been processed. Clear it! */
    USB_cmdNodata(USBSIE_SelectEndpoint0 + physicalEndpoint);
    USB_cmdNodata(USBSIE_ClearBuffer);

//printf("(R%d=%d)\r\n", physicalEndpoint, packetLength);

    return result;
}



/** Read setup packet from endpoint 0.
 *
 *  \param[in] handle Device handle
 *  \retval LPC_SUCCESS Ok
 *  \retval LPCLIB_ERROR Wrong packet length (!= 8)
 */
static LPCLIB_Result USB_readSetupFromEndpoint0 (USB_Handle handle)
{
    uint16_t packetLength;
    LPCLIB_Result result = LPCLIB_SUCCESS;


    /* No more EP0(RX) interrupts until the stack sends a response */
    LPC_USB->EpIntEn &= ~(1u << 0);

    /* Invalidate control endpoint data buffers */
    handle->endpoints[0].busy = DISABLE;
    handle->endpoints[1].busy = DISABLE;

    /* Select read mode for the endpoint */
    LPC_USB->Ctrl =
            USB_CTRL_RD_EN_Msk |
            ((0x00 << USB_CTRL_LOG_ENDPOINT_Pos) & USB_CTRL_LOG_ENDPOINT_Msk) ;

    /* Wait for the packet length to become available in USBRxPLen. */
    while (!(LPC_USB->RxPLen & USB_RXPLEN_PKT_RDY_Msk))
        ;

    /* Extract the packet length field. */
    packetLength = (LPC_USB->RxPLen & USB_RXPLEN_PKT_LNGTH_Msk) >> USB_RXPLEN_PKT_LNGTH_Pos;
    if (packetLength != 8) {                /* SETUP packets must be 8 bytes long */
        result = LPCLIB_ERROR;
        LPC_USB->Ctrl = 0; //TODO
    }
    else {
        /* Read SETUP packet. */
        handle->setupPacket[0] = LPC_USB->RxData;
        handle->setupPacket[1] = LPC_USB->RxData;
    }

    /* Buffer has been processed. Clear it! */
    USB_cmdNodata(USBSIE_SelectEndpoint0 + 0);
    USB_cmdNodata(USBSIE_ClearBuffer);

    return result;
}



/** Perform the write to an endpoint.
 *
 *  \param[in] handle Device handle
 *  \param[in] physicalEndpoint Number of physical endpoint
 *  \param[out] pLastPacket Returns true if this was the last packet sent
 */
static void USB_performEndpointWrite (USB_Handle handle,
                                      uint8_t physicalEndpoint,
                                      LPCLIB_Switch *pLastPacket
                                     )
{
    union {
        uint32_t txData32;
        uint8_t  txData8[4];
    } txData;
    uint16_t packetLength;
    uint16_t n;
    USB_EndpointContext *pEp = &(handle->endpoints[physicalEndpoint]);
    USB_Buffer *pBuffer = pEp->pBuffer;
    uint8_t *p;


    if (pEp->busy) {
        p = pBuffer->data;

        /* Clear end-of-packet indicator */
        LPC_USB->DevIntClr = USB_DEVINTCLR_TXENDPKT_Msk;

        /* Select write mode for the endpoint */
        LPC_USB->Ctrl =
            USB_CTRL_WR_EN_Msk |
            ((USB_phy2log(physicalEndpoint) << USB_CTRL_LOG_ENDPOINT_Pos) & USB_CTRL_LOG_ENDPOINT_Msk) ;

        /* Calculate packet length */
        *pLastPacket = ENABLE;
        packetLength = pBuffer->maxSize - pBuffer->currentSize;
        if (packetLength > pEp->packetSize) {
            packetLength = pEp->packetSize;
            *pLastPacket = DISABLE;
        }
        LPC_USB->TxPLen = packetLength;

        /* Write data.
        * Note: Even if the packet length is zero, we still have to write once to USBTxData before
        *       the TxENDPKT bit is set the packet is sent!
        *       --> Make sure to not read input data beyond the *real* packet length! (MPU!)
        *       --> Make sure to write at least once! (n >= packetLength)
        */
        txData.txData32 = 0;
        n = 0;
        while (!(LPC_USB->DevIntSt & USB_DEVINTST_TXENDPKT_Msk)) {
            if (n < packetLength) {
                txData.txData8[n % 4] = p[pBuffer->currentSize];
                ++(pBuffer->currentSize);
            }

            /* TX register is word-wide. Write every fourth byte. */
            n++;
            if (((n % 4) == 0) || (n >= packetLength)) {
                LPC_USB->TxData = txData.txData32;
            }
        }

        if (*pLastPacket) {
            pEp->busy = DISABLE;
        }

        /* Validate buffer */
        USB_cmdNodata(USBSIE_SelectEndpoint0 + physicalEndpoint);
        USB_cmdNodata(USBSIE_ValidateBuffer);
    }
}



/** Deferred interrupt handling.
 *
 *  Scan all endpoints for events.
 *
 *  \param [in] endpointMask Bitmask for 32 physical endpoints
 */
void USB_doDeferredHandling (USB_Handle handle, uint32_t endpointMask)
{
    uint32_t i;
    LPCLIB_Event event;
    uint8_t status;
    LPCLIB_Switch shortPacket;
    USB_CallbackContext cbc;



    event.id = LPCLIB_EVENTID_USB;
    event.block = handle->bus;
    event.parameter = &cbc;

    /* Scan the mask for requesting endpoints */
#if LPCLIB_USB_DMA
#else
    for (i = 0; i < 32; i++) {
        if (endpointMask & (1u << i)) {
            /* TODO Workaround for spurious endpoint interrupts. Such interrupts have occurred for
             *      control IN endpoint 0x80. USBEpIntSt.1 was set (that's why we came here!), but
             *      it wasn't set anymore when we arrived here. The code then hung forever in the
             *      loop below waiting for CDFULL.
             */
            if (!(LPC_USB->EpIntSt & (1u << i))) {
                usbContext.errorSpuriousEpInt++;
                continue;
            }

            /* Clear this request. This also returns the EP status. */
            LPC_USB->EpIntClr = (1u << i);
            USB_waitStatus(USB_DEVINTST_CDFULL_Msk);
            status = LPC_USB->CmdData;

            /* Check if we have a buffer for this endpoint (Non-control endpoints only. Control
             * endpoints are assumed to always have a valid buffer).
             * If not, ask for one. If we have a buffer, we can use it for TX/RX.
             */
            if ((i >= 2) && handle->endpoints[i].pBuffer == NULL) {
                /* No buffer. Ask for one. */
                event.opcode = USB_EVENT_NEED_BUFFER;
                event.channel = USB_phy2log(i);
                cbc.deviceStatus = status;
                if (handle->callback) {
                    handle->callback(event);   //TODO what if no callback?
                }
            }
            else {
                shortPacket = DISABLE;

                /* RX or TX? */
                if (i & 1) {
                    /* IN endpoint. Send data from buffer. */
                    USB_performEndpointWrite(handle, i, &shortPacket);
                }
                else {
                    if (status & USBSIE_SELECTENDPOINT_STP_Msk) {
                        USB_readSetupFromEndpoint0(handle);

                        event.opcode = USB_EVENT_SETUP;
                        cbc.pSetup = &handle->setupPacket;
                        if (handle->callback) {
                            handle->callback(event);
                        }
                    }
                    else {
                        /* OUT endpoint. Use the buffer for this RX packet. */
                        if (status & USBSIE_SELECTENDPOINT_FE_Msk) {
                            USB_performEndpointRead(handle, i, &shortPacket);
                        }
                    }
                }

                /* Let the device know if the buffer is complete. */
                if (shortPacket) {
                    event.opcode = USB_EVENT_BUFFER_DONE;
                    event.channel = USB_phy2log(i);
                    if ((i >= 2) && handle->endpoints[i].pBuffer->callback) {
                        handle->endpoints[i].pBuffer->callback(event);
                    }
                    else {
                        if (handle->callback) {
                            handle->callback(event);
                        }
                    }
                    handle->endpoints[i].pBuffer = NULL;
                }
            }
        }
    }
#endif  /* LPCLIB_USB_DMA */

    /* Acknowledge and re-enable the interrupts */
    LPC_USB->DevIntClr = USB_DEVINTCLR_EP_FAST_Msk | USB_DEVINTCLR_EP_SLOW_Msk;
    LPC_USB->DevIntEn |= USB_DEVINTEN_EP_FAST_Msk | USB_DEVINTEN_EP_SLOW_Msk;    //TODO not atomic!!
}



/************************************************************************************/



/* Open the USB (device) block. */
LPCLIB_Result USB_open (USB_Name bus, USB_Handle *pHandle)
{
    (void) bus;

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_deassertPeripheralReset(CLKPWR_RESET_USB);   /* Release USB reset */
#endif
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_USB);         /* Enable USB peripheral clock */

//TODO protect against opening the device twice

    usbContext.needClock = DISABLE;
    USB_enableUsbClk();                                 /* Enable internal clocks for register access. */

    LPC_USB->DevIntEn = 0;                              /* Reset device interrupts */
    LPC_USB->DevIntClr = 0xFFFFFFFF;
    LPC_USB->EpIntEn = 0;                               /* Reset endpoint interrupts */
//    LPC_USB->EpIntClr = 0xFFFFFFFF;
    LPC_USB->DMAIntEn = 0;                              /* Reset DMA interrupts */
    LPC_USB->EpDMADis = 0xFFFFFFFF;
    LPC_USB->DMARClr = 0xFFFFFFFF;
    LPC_USB->EoTIntClr = 0xFFFFFFFF;
    LPC_USB->NDDRIntClr = 0xFFFFFFFF;
    LPC_USB->SysErrIntClr = 0xFFFFFFFF;

    usbContext.bus = bus;
    usbContext.dmaEnabledEndpoint = 0;                  /* Init context */
    usbContext.fastEndpoint = 0;
    usbContext.callback = NULL;
    usbContext.accessMutex = osMutexCreate(osMutex(usbAccessMutex));

    LPC_USB->UDCAH = (uint32_t)&usbhwd_udca;            /* Set link to DMA descriptor table */

    /* Enable interrupts. (Frame interrupts will be enabled if needed by isochronous endpoints.) */
    LPC_USB->DevIntEn =  USB_DEVINTEN_EP_FAST_Msk       /* Fast endpoints */
                      |  USB_DEVINTEN_EP_SLOW_Msk       /* Slow endpoints */
                      |  USB_DEVINTEN_DEV_STAT_Msk;     /* Device status */

    *pHandle = &usbContext;                             /* Return handle */

    return LPCLIB_SUCCESS;
}



/* Close a USB device. */
void USB_close (USB_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_USB);        /* Disable peripheral clock */

//    LPCLIB_OSALMutexDelete(usbContext.accessMutex);

    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Configure the USB block. */
void USB_ioctl (USB_Handle handle, const USB_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    switch (pConfig->opcode) {
    case USB_OPCODE_SELECT_DMA_ENDPOINTS:               /* Select which endpoints use DMA */
//TODO When should we allow this? Reject if endpoint in use/busy?
        LPC_USB->EpDMAEn = pConfig->dmaEndpointEnable;
        LPC_USB->DMAIntEn = USB_DMAINTEN_EOT_Msk        //TODO Disable if mask==0?
                          | USB_DMAINTEN_NDDR_Msk
                          /*| USB_DMAINTEN_ERR_Msk*/;
        break;

    case USB_OPCODE_SET_ADDRESS:                        /* Set the USB bus address. */
        USB_cmdWrite(USBSIE_SetAddress,
                     USBSIE_SETADDRESS_DEV_EN_Msk | pConfig->address);
        break;

    case USB_OPCODE_SOFTCONNECT:
        USB_cmdWrite(USBSIE_SetDeviceStatus, pConfig->connect ? USBSIE_SETDEVICESTATUS_CON_Msk : 0);
        break;

    case USB_OPCODE_ENDPOINT_ACTIVATION:
        USB_activateEndpoint(pConfig->logicalEndpoint,
                             pConfig->endpointActivation.activate,
                             pConfig->endpointActivation.packetSize);
        break;

    case USB_OPCODE_DEACTIVATE_NON_CONTROL:
        LPC_USB->ReEp &= ~0xFFFFFFFC;                   /* Leave only EP0 and EP1 active */
        break;

    case USB_OPCODE_CONFIGURE_DEVICE:
        USB_cmdWrite(USBSIE_ConfigureDevice, pConfig->configure ? USBSIE_CONFIGUREDEVICE_CONF_DEVICE_Msk : 0);
        break;

    case USB_OPCODE_ENDPOINT_STALL:
        /* Any control EP activity re-enables the control OUT EP interrupt. */
        if (USB_log2phy(pConfig->logicalEndpoint) < 2) {
            LPC_USB->EpIntEn |= (1u << 0);
        }

        USB_cmdWrite(USBSIE_SetEndpointStatus0 + USB_log2phy(pConfig->logicalEndpoint),
                    pConfig->stall ? USBSIE_SETENDPOINTSTATUS_ST_Msk : 0);
        break;

    case USB_OPCODE_NAK_INTERRUPT:
        //TODO: If any endpoint is enabled for DMA,
        //      ensure that INAK_BO and INAK_IO are cleared!
        USB_cmdWrite(USBSIE_SetMode, pConfig->nak);
        break;

    case USB_OPCODE_SET_CALLBACK:
        handle->callback = pConfig->callback;
        break;

    case USB_OPCODE_SET_ISOC_ENDPOINTS:
        ;
        ; //TODO
        ;
        break;

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    case USB_OPCODE_SELECT_PORT:
        LPCLIB_BITBAND(&LPC_USB->USBClkCtrl, USB_CLKCTRL_PORTSEL_CLK_EN_Pos) = 1;
        while (LPCLIB_BITBAND(&LPC_USB->USBClkSt, USB_CLKST_PORTSEL_CLK_ON_Pos) == 0)
            ;
        LPC_USB->PortSel =
                (LPC_USB->PortSel & ~USB_PORTSEL_PORTSEL_Msk)
              | (pConfig->port << USB_PORTSEL_PORTSEL_Pos);
        LPCLIB_BITBAND(&LPC_USB->USBClkCtrl, USB_CLKCTRL_PORTSEL_CLK_EN_Pos) = 0;
        break;
#endif

    }
}



/* Read from an endpoint. */
LPCLIB_Result USB_read (USB_Handle handle, uint8_t logicalEndpoint, USB_Buffer *pBuffer)
{
    uint8_t physicalEndpoint = USB_log2phy(logicalEndpoint);
    USB_EndpointContext *pEp = &(handle->endpoints[physicalEndpoint]);
    LPCLIB_Result result = LPCLIB_BUSY;


    /* Any control EP read/write activity re-enables the control OUT EP interrupt. */
    if (physicalEndpoint < 2) {
        LPC_USB->EpIntEn |= (1u << 0);
    }

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        if (!pEp->busy || (1u << physicalEndpoint & USBHWD_ISO_ENDPOINT_BITMASK)) { //TODO iso endpoints
            pBuffer->currentSize = 0;
            pEp->pBuffer = pBuffer;
            pEp->busy = ENABLE;

            LPC_USB->EpIntSet = (1u << physicalEndpoint);

            result = LPCLIB_SUCCESS;
        }

        osMutexRelease(handle->accessMutex);
    }

    return result;
}



/* Write to an endpoint. */
LPCLIB_Result USB_write (USB_Handle handle, uint8_t logicalEndpoint, USB_Buffer *pBuffer)
{
    uint8_t physicalEndpoint = USB_log2phy(logicalEndpoint);
    USB_EndpointContext *pEp = &(handle->endpoints[physicalEndpoint]);
    LPCLIB_Result result = LPCLIB_BUSY;

    /* Any control EP read/write activity re-enables the control OUT EP interrupt. */
    if (physicalEndpoint < 2) {
        LPC_USB->EpIntEn |= (1u << 0);
    }

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        if (!pEp->busy) {
            pBuffer->currentSize = 0;
            pEp->pBuffer = pBuffer;
            pEp->busy = ENABLE;

            LPC_USB->EpIntSet = (1u << physicalEndpoint);

            result = LPCLIB_SUCCESS;
        }

        osMutexRelease(handle->accessMutex);
    }

    return result;
}




/* Get frame number. */
LPCLIB_Result USB_getFrameNumber (USB_Handle handle, uint16_t *pFrameNumber)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    *pFrameNumber = USB_cmdRead2(USBSIE_ReadCurrentFrameNumber);

    return LPCLIB_SUCCESS;
}



/* Get current device status. */
LPCLIB_Result USB_getDeviceStatus (USB_Handle handle, USB_DeviceStatus *pDeviceStatus)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    uint8_t value = USB_cmdRead1(USBSIE_GetDeviceStatus);
    *pDeviceStatus = *((USB_DeviceStatus *)&value);

    return LPCLIB_SUCCESS;
}



/* Acquire/release the right to make calls to the device API. */
LPCLIB_Result USB_allowAccess (USB_Handle handle, LPCLIB_Switch activate)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (activate == DISABLE) {
        USB_disableUsbClk();
    }
    else {
        USB_enableUsbClk();
    }

    return LPCLIB_SUCCESS;
}



/* Initialize a buffer variable. */
USB_Buffer * USB_initBuffer (USB_Buffer *pBuffer)
{
    pBuffer->callback = NULL;
    pBuffer->data = NULL;
    pBuffer->maxSize = 0;
    pBuffer->next = NULL;

    return pBuffer;
}



/** USB interrupt handler. */
static void USB_commonIRQHandler (USB_Name bus)
{
    uint32_t status;
    LPCLIB_Event event;
    USB_CallbackContext cbContext;
    LPCLIB_Result result;


    /* Force clocks ON for register access */
    USB_enableUsbClkFromISR();

    /* Prepare event */
    event.id = LPCLIB_EVENTID_USB;
    event.block = bus;
    event.parameter = &cbContext;

    /* Read relevant device interrupt flags */
    status = (LPC_USB->DevIntSt & LPC_USB->DevIntEn);

    /* Frame interrupt */
    if (status & USB_DEVINTST_FRAME_Msk) {
        /* Acknowledge the interrupt */
        LPC_USB->DevIntClr = USB_DEVINTCLR_FRAME_Msk;

        /* Disable further interrupts, until we have handled this one.
         * Avoids flooding the queue with SOF interrupts in case we have to do more important things.
         */
        LPC_USB->DevIntEn &= ~(USB_DEVINTEN_FRAME_Msk);

        /* Inform the device */
        event.opcode = USB_EVENT_FRAME;
        if (usbContext.callback) {
            usbContext.callback(event);
        }
    }

    /* Change in device status */
    if (status & USB_DEVINTST_DEV_STAT_Msk) {
        /* Acknowledge the interrupt */
        LPC_USB->DevIntClr = USB_DEVINTCLR_DEV_STAT_Msk;

        /* Inform the device */
        event.opcode = USB_EVENT_DEVICE_STATUS;
        if (usbContext.callback) {
            usbContext.callback(event);
        }
    }

    /* It is not recommended to clear the endpoint interrupts here!
     * Reason: Clearing EP interrupts is only possible by issuing a SIE command (writing to
     * USBEpIntClr is a hidden SIE command). The documents do not specify how long it takes
     * to execute a SIE command worst case. Also that time must be multiplied by the number
     * of EP's requesting an interrupt simultaneously. As the task also uses SIE commands,
     * one would also have to disable USB device interrupts while a SIE operation
     * is ongoing on task level.
     *
     * Therefore, all EP_FAST/EP_SLOW interrupts are disabled here, and the info about the requesting
     * endpoints is passed to a deferred handler task. That handler processes all the endpoints
     * without a resource conflict. At the end of the endpoint processing, the task acknowledges
     * the EP_FAST/EP_SLOW requests in the device status register, and re-enables the EP interrupts.
     */

    /* Fast endpoint interrupt? */
    result = LPCLIB_ERROR;
    if (status & USB_DEVINTST_EP_FAST_Msk) {
        /* Tell the handler task about the event */
        event.opcode = USB_EVENT_DEFERRED;
        cbContext.endpointMask = LPC_USB->EpIntSt & usbContext.fastEndpoint;
        if (usbContext.callback) {
            usbContext.callback(event);
if(1){//            if (cbContext.error == LPCLIB_OSAL_ERROR_NONE) {
                /* Disable further EP_FAST and EP_SLOW interrupts.
                 * EP_SLOW interrupts will be handled only if no EP_FAST interrupts are pending.
                 */
                LPC_USB->DevIntEn &= ~(USB_DEVINTEN_EP_FAST_Msk | USB_DEVINTEN_EP_SLOW_Msk);
                status &= ~USB_DEVINTST_EP_SLOW_Msk;
                result = LPCLIB_SUCCESS;
            }
        }

        if (result != LPCLIB_SUCCESS) {
            /* This is a serious error! (either no callback, or callback failed)
             * We can try to recover by acknowledging the EP interrupts, but all data is lost!
             * TODO: Reading status is mandatory!?
             */
            LPC_USB->EpIntClr = cbContext.endpointMask;
        }
    }

    /* Slow endpoint interrupt? */
    if (status & USB_DEVINTST_EP_SLOW_Msk) {
        /* Tell the handler task about the event */
        event.opcode = USB_EVENT_DEFERRED;
        cbContext.endpointMask = LPC_USB->EpIntSt & ~(usbContext.fastEndpoint);
        if (usbContext.callback) {
            usbContext.callback(event);
if(1){//            if (cbContext.error == LPCLIB_OSAL_ERROR_NONE) {
                /* Disable further EP_SLOW interrupts. */
                LPC_USB->DevIntEn &= ~(USB_DEVINTEN_EP_SLOW_Msk);
                result = LPCLIB_SUCCESS;
            }
        }

        if (result != LPCLIB_SUCCESS) {
            /* This is a serious error!
             * We can try to recover by acknowledging the EP interrupts, but all data is lost!
             * TODO: Reading status is mandatory!?
             */
            LPC_USB->EpIntClr = cbContext.endpointMask;
        }
    }

    /* Read DMA interrupt flags */
    status = (LPC_USB->DMAIntSt & LPC_USB->DMAIntEn);

    /* DMA End-of-Transfer interrupt? */
    if (status & USB_DMAINTST_EOT_Msk) {
        event.opcode = USB_EVENT_DMA_EOT;               /* Inform the handler task */
        cbContext.endpointMask = LPC_USB->EoTIntSt;
        if (usbContext.callback) {
            usbContext.callback(event);
        }

        LPC_USB->EoTIntClr = cbContext.endpointMask;    /* Interrupt acknowledge */
#if 0
if(usb_dmaDesc[0].value3 & 1) {
  usb_dmaDesc[0].dma_buffer_start = &baffa[0][0];
  usb_dmaDesc[0].value3 = 0;
  usb_dmaDesc[0].isoc_packetsize_memory_address = &usbhwd_isocPackets[0][0];
  usb_dmaDesc[0].value1 = (FRAMES_PER_DD << 16) | (1 << 4) | (1 << 2);
}
if(usb_dmaDesc[1].value3 & 1) {
  usb_dmaDesc[1].dma_buffer_start = &baffa[0][0];
  usb_dmaDesc[1].value3 = 0;
  usb_dmaDesc[1].isoc_packetsize_memory_address = &usbhwd_isocPackets[0][0];
  usb_dmaDesc[1].value1 = (FRAMES_PER_DD << 16) | (1 << 4) | (1 << 2);
}
#endif
    }

    /* DMA New-DD-Request interrupt? */
    if (status & USB_DMAINTST_NDDR_Msk) {
        event.opcode = USB_EVENT_DMA_NDDR;              /* Inform the handler task */
        cbContext.endpointMask = LPC_USB->NDDRIntSt;
        if (usbContext.callback) {
            usbContext.callback(event);
        }

        LPC_USB->NDDRIntClr = cbContext.endpointMask;   /* Interrupt acknowledge */
    }

    /* Release clocks for register access */
    USB_disableUsbClkFromISR();
}


/** USB block interrupt entry (interrupt number \ref USB_IRQn).
 */
void USB_IRQHandler (void)
{
    USB_commonIRQHandler(USB0);
}



/** USB block activity interrupt entry (interrupt number \ref USBActivity_IRQn).
 */
void USBActivity_IRQHandler (void)
{
    /* There's nothing to do! */
}

/** @} USB */

#endif  /* #if LPCLIB_USB */

