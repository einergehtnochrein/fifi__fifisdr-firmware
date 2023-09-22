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
 *  \brief I2S driver interface.
 *  This file defines all interface objects needed to use the I2S driver.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17XX_I2S_H__
#define __LPC17XX_I2S_H__

/** \defgroup I2S
 *  \ingroup API
 *  @{
 */

#include "lpc17xx_libconfig.h"

#include "lpclib_types.h"



/** \defgroup I2S_Public_Types I2S Types, enums, macros
 *  @{
 */


/** Enumerator for the I2S block.
 */
typedef enum I2S_Name {
    I2S0 = 0,           /**< First I2S interface block */
    __NUM_I2S__         /* In order for this element to reflect the number of I2S busses,
                         * you mustn't assign an explicit value to any of the other elements
                         * of this enum! (except for the first element which may get assigned to 0).
                         */
} I2S_Name;


/** Handle for an I2S block. */
typedef struct I2S_Context *I2S_Handle;


/** Opcodes to specify the configuration command in a call to \ref I2S_ioctl. */
typedef enum I2S_Opcode {
    I2S_OPCODE_INVALID = 0,                 /**< (List terminator) */
    I2S_OPCODE_SET_SAMPLERATE,              /**< Config action: Set dividers by sample rate (WS frequency) */
    I2S_OPCODE_SET_DIVIDERS,                /**< Config action: Set sample rate by dividers */
    I2S_OPCODE_SET_MODE,                    /**< Config action: Set operating mode */
} I2S_Opcode;


/** I2S transfer channel. */
typedef enum I2S_Channel {
    I2S_CHANNEL_TX = 0,
    I2S_CHANNEL_RX = 1,
} I2S_Channel;


/** Operating modes. */
typedef enum I2S_Mode {
    I2S_MODE_MASTER                     = 0x00,
    I2S_MODE_MASTER_SHARED_REFCLOCK     = 0x02,
    I2S_MODE_MASTER_4WIRE_SHARED_CLOCK  = 0x04,
    I2S_MODE_MASTER_MCLK_OUT            = 0x08,
    I2S_MODE_SLAVE                      = 0x20,
    I2S_MODE_SLAVE_SHARED_REFCLOCK      = 0x22,
    I2S_MODE_SLAVE_4WIRE_SHARED_CLOCK   = 0x24,
} I2S_Mode;


/** Data word format. */
typedef enum I2S_WordSize {
    I2S_WORDSIZE_8BIT_MONO              = ( 8 << 8) | 0x04, /**< 8 bit data */
    I2S_WORDSIZE_16BIT_MONO             = (16 << 8) | 0x05, /**< 16 bit data */
    I2S_WORDSIZE_32BIT_MONO             = (32 << 8) | 0x07, /**< 32 bit data */
    I2S_WORDSIZE_8BIT_STEREO            = ( 8 << 8) | 0x00, /**< 8 bit data, WS=0 -> L, WS=1 -> R */
    I2S_WORDSIZE_16BIT_STEREO           = (16 << 8) | 0x01, /**< 16 bit data, WS=0 -> L, WS=1 -> R */
    I2S_WORDSIZE_32BIT_STEREO           = (32 << 8) | 0x03, /**< 32 bit data, WS=0 -> L, WS=1 -> R */
} I2S_WordSize;


/** Mode and data format descriptor. */
struct I2S_ConfigMode {
    uint16_t bitsPerChannel;                /**< Must be at least the number of bits required by format. */
    I2S_WordSize size;                      /**< Mono/Stereo, bits per channel */
    I2S_Mode mode;
};


/** Divider descriptor. */
struct I2S_ConfigDividers {
    uint8_t fracNominator;                  /**< Fractional divider: nominator */
    uint8_t fracDenominator;                /**< Fractional divider: denominator */
    uint16_t bitClockDivider;               /**< Integer bit clock divider */
};


/** Descriptor to specify the configuration in a call to \ref I2S_ioctl. */
typedef struct I2S_Config {
    I2S_Opcode opcode;                      /**< Config action opcode */
    I2S_Channel channel;                    /**< Selects the transfer direction (TX/RX) */

    union {
        uint32_t sampleRate;                /**< Samplerate (WS frequency) */
        struct I2S_ConfigDividers dividers; /**< Fractional divider, bit clock divider */
        struct I2S_ConfigMode modeAndSize;  /**< Mode and data format */
    };
} I2S_Config;

/** Config list terminator. */
#define I2S_CONFIG_END \
    {.opcode = I2S_OPCODE_INVALID}


typedef enum I2S_CallbackEvent {
    I2S_EVENT_PHASE_COMPLETE = 0,           /**< Phase completed successfully */
    I2S_EVENT_JOB_COMPLETE,                 /**< Job completed successfully */
} I2S_CallbackEvent;


typedef enum I2S_MuteSwitch {
    I2S_MUTE_ON = 1,
    I2S_MUTE_OFF = 0,
} I2S_MuteSwitch;



/** I2S transfer phase descriptor. */
typedef struct I2S_JobPhase {
    const struct I2S_JobPhase *next;        /**< Pointer to next transaction phase (or NULL) */
    uint32_t *data;                         /**< Pointer to source or destination data */
    uint32_t length;                        /**< Transfer length (in FIFO elements) */
    LPCLIB_Switch triggerCallback;          /**< Invoke callback at end of phase */
    void *magic;                            /**< Magic value sent by callback (to identify phase) */
} I2S_JobPhase;


/** Transfer job descriptor for I2S (one direction).
 *  Consists of one or more phases. A job can end after the last phase, or it may continue
 *  to run endlessly if the last phase links back to the first phase.
 */
typedef struct I2S_Job {
    I2S_Channel channel;                    /**< RX or TX */
    LPCLIB_Callback callback;               /**< Callback for terminal count or error event */
    const I2S_JobPhase *firstPhase;         /**< Descriptor of first job phase */
} I2S_Job;



/** @} I2S Types, enums, macros */



/** \defgroup I2S_Public_Functions I2S API Functions
 *  @{
 */


/** Open an I2S bus.
 *
 *  Enable the peripheral clock to the indicated I2S block, then enable
 *  the block.
 *
 *  \param[in] bus Indicator that selects an I2S interface block
 *  \param[out] pHandle Handle to be used in future API calls to the I2S module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 *  \retval LPCLIB_BUSY Failure (interface already open). \ref pHandle does not
 *  contain a valid handle in this case.
 */
LPCLIB_Result I2S_open (I2S_Name bus, I2S_Handle *pHandle);


/** Close an I2S bus.
 *
 *  Disables interrupts for the I2S block, and cuts the clock to the block.
 *
 *  \param[in] pHandle I2S bus handle.
 */
void I2S_close (I2S_Handle *pHandle);


/** Configure the I2S block.
 *
 *  Pass a configuration command to the I2S block.
 *
 *  \param[in] handle I2S bus handle.
 *  \param[in] pConfig Pointer to a configuration descriptor of type
 *             \ref I2S_Config.
 */
void I2S_ioctl (I2S_Handle handle, const I2S_Config *pConfig);


/** Submit a new I2S job to the driver.
 *
 *  \param[in] handle I2S block handle.
 *  \param[in] pJob Pointer to a job descriptor that describes the details
 *      of the transaction.
 *  \param[in] dma Handle of a DMA channel. If this is a valid handle, that DMA channel
 *      will be used instead of interrupt driven transfer.
 *
 *  \retval LPCLIB_SUCCESS Success. Job will be executed, and the callback function
 *      called once the phase/transaction ends.
 *  \retval LPCLIB_BUSY Failure. Job NOT submitted.
 */
struct DMA_ChannelContext;
LPCLIB_Result I2S_submitJob (I2S_Handle handle, const I2S_Job *pJob, struct DMA_ChannelContext *dma);


/** Starts or stops an I2S channel (clears/sets the STOP condition).
 *
 *  \param[in] handle I2S block handle.
 *  \param[in] channel Selects RX or TX channel
 *  \param[in] enable ENABLE=run, DISABLE=stop
 */
void I2S_run (I2S_Handle handle, I2S_Channel channel, LPCLIB_Switch enable);


/** Mute or unmute the TX channel.
 *
 *  \param[in] handle I2S bus handle.
 *  \param[in] mute mute on/off
 */
void I2S_muteTx (I2S_Handle handle, I2S_MuteSwitch mute);


/** @} I2S API Functions */

/** @} I2S */

#endif /* #ifndef __LPC17XX_I2S_H__ */

