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
 *  \brief MCI driver interface.
 *  This file defines all interface objects needed to use the MCI driver.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17XX_MCI_H__
#define __LPC17XX_MCI_H__

/** \defgroup MCI
 *  \ingroup API
 *  @{
 */


#include "lpc17xx_libconfig.h"

#if LPCLIB_MCI

#include "lpclib_types.h"



/** \defgroup MCI_Public_Types MCI Types, enums, macros
 *  @{
 */


/** Enumerator for the MCI block.
 */
typedef enum MCI_Name {
    MCI0 = 0,           /**< First (and only...) MCI interface block */
} MCI_Name;


/** Handle for an MCI block. */
typedef struct MCI_Context *MCI_Handle;


/** Opcodes to specify the configuration command in a call to \ref MCI_ioctl. */
typedef enum MCI_Opcode {
    MCI_OPCODE_BUS_CONFIG,                  /**< Config action: Bus width and bus maximum clock */
    MCI_OPCODE_CONNECT,                     /**< Config action: SDCARD connect/disconnect */
    MCI_OPCODE_SET_CALLBACK,                /**< Config action: Install event handler */
    MCI_OPCODE_SELECT_CARD,                 /**< Config action: Select a card (standby --> transfer) */
} MCI_Opcode;

/** MCI bus configuration parameter */
struct MCI_ConfigBus {
    uint32_t maxClockHz;                    /**< Maximum bus clock in Hz */
    LPCLIB_Switch allow4Bit;                /**< Allow 4-bit data bus */
    LPCLIB_Switch pwrLowActive;             /**< MCIPWR is active low */
};

/** MCI card connection and write protect */
struct MCI_ConfigConnect {
    LPCLIB_Switch isInserted;               /**< Card is inserted */
    LPCLIB_Switch isWriteProtected;         /**< Card is write protected
                                             *   (only valid if card connected)
                                             */
};

/** Parameter for 'select' action. */
struct MCI_ConfigSelect {
    LPCLIB_Switch action;
};


/** Descriptor to specify the configuration in a call to \ref MCI_ioctl. */
typedef struct MCI_Config {
    MCI_Opcode opcode;                      /**< Config action opcode */

    union {
        struct MCI_ConfigBus bus;           /**< Config bus width and clock speed */
        struct MCI_ConfigConnect connect;   /**< Card connect/disconnect */
        LPCLIB_Callback callback;           /**< Callback handler */
        struct MCI_ConfigSelect select;     /**< Card selection (--> transfer state) */
    };
} MCI_Config;


typedef enum MCI_Event {
    MCI_EVENT_CARD_INVALID = 1,             /**< Card is invalid, and has been deactivated */
    MCI_EVENT_CARD_STANDBY = 2,             /**< Card has reached STANDBY state */
    MCI_EVENT_CARD_SELECTED = 3,            /**< Card has reached TRANSFER state */
    MCI_EVENT_READ_WRITE_COMPLETE = 3,      /**< R/W operation completed */
} MCI_Event;


/** @} MCI Types, enums, macros */



/** \defgroup MCI_Public_Functions MCI API Functions
 *  @{
 */


/** Open the MCI interface for use.
 *
 *  Enable the peripheral clock for the MCI block, then enable
 *  the block. Enable the interrupts.
 *  The block is ready for use after that.
 *
 *  \param[in] mciNum Indicator that selects the MCI interface block. This is always \ref MCI0
 *      since there is only one MCI block.
 *  \param[out] pHandle Handle to be used in future API calls to the MCI module.
 *  \retval LPCLIB_SUCCESS Success. \ref pHandle contains a valid handle.
 */
LPCLIB_Result MCI_open (MCI_Name mciNum, MCI_Handle *pHandle);


/** Close an MCI interface.
 *
 *  Disables interrupts for the MCI block, and cuts the clock to the block.
 *  Make sure to call this only if there is no ongoing transaction, since no
 *  check is made to prevent this.
 *
 *  \param[in] pHandle MCI Pointer to handle as obtained by \ref MCI_open.
 */
void MCI_close (MCI_Handle *pHandle);


/** Configure the MCI block.
 *
 *  Pass a configuration command to the MCI block.
 *
 *  \param[in] handle MCI bus handle as obtained by \ref MCI_open.
 *  \param[in] pConfig Pointer to a configuration descriptor of type
 *             \ref MCI_Config.
 */
void MCI_ioctl (MCI_Handle handle, const MCI_Config *pConfig);


/** Read sector(s) from card.
 *
 *  \param[in] handle MCI bus handle
 *  \param[in] pData Pointer to a data buffer.
 *  \param[in] sectorIndex Index of the first sector to be read
 *  \param[in] numSectors Number of sectors to be read.
 *  \param[in] dma DMA channel (or NULL for interrupt-driven transfer)
 */
struct DMA_ChannelContext;
LPCLIB_Result MCI_read (MCI_Handle handle,
                        void *pData,
                        int sectorIndex,
                        int numSectors,
                        struct DMA_ChannelContext *dma);

/** Write sector(s) to card.
 *
 *  \param[in] handle MCI bus handle
 *  \param[in] pData Pointer to a data buffer.
 *  \param[in] firstSector Index of the first sector to be written
 *  \param[in] numSectors Number of sectors to be written
 *  \param[in] dma DMA channel (or NULL for interrupt-driven transfer)
 */
LPCLIB_Result MCI_write (MCI_Handle handle,
                         const void *pData,
                         int firstSector,
                         int numSectors,
                         struct DMA_ChannelContext *dma);


/** @} MCI API Functions */


#endif /* #if LPCLIB_MCI */

/** @} MCI */

#endif /* #ifndef __LPC17XX_MCI_H__ */

