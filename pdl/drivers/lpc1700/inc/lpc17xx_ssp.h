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

/** \defgroup SSP
 *  \ingroup API
 *  @{
 */

#ifndef __LPC17XX_SSP_H__
#define __LPC17XX_SSP_H__

#include "lpc17xx_libconfig.h"
#include "lpclib_types.h"



/** \defgroup SSP_Public_Types SSP Types, enums, macros
 *  @{
 */


typedef enum SSP_Name {
    SSP0 = 0,           /**< SSP 0 */
    SSP1,               /**< SSP 1 */
    SSP_NUM_BUSSES      /* In order for this element to reflect the number of SSP busses,
                         * you mustn't assign an explicit value to any of the other elements
                         * of this enum! (except for the first element which may get assigned to 0).
                         */
} SSP_Name;

/** Handle for an open SSP block, as obtained by \ref SSP_open. */
typedef struct SSP_Context *SSP_Handle;

typedef enum SSP_Opcode {
    SSP_OPCODE_INVALID = 0,                 /**< (List terminator) */
    SSP_OPCODE_SET_MODE,                    /**< Config action: Set mode (M/S) */
    SSP_OPCODE_SET_FORMAT,                  /**< Config action: Set format */
    SSP_OPCODE_SET_BITRATE,                 /**< Config action: Set bit rate */
} SSP_Opcode;

typedef enum SSP_Mode {
    SSP_MODE_MASTER = (0  << 2),            /**< Master mode */
    SSP_MODE_SLAVE =  (1u << 2),            /**< Slave mode */
} SSP_Mode;

typedef enum SSP_ClockFormat {
    SSP_CLOCK_CPHA0_CPOL0 = (0  << 6),      /**< CPHA = 0, CPOL = 0 */
    SSP_CLOCK_CPHA0_CPOL1 = (1u << 6),      /**< CPHA = 0, CPOL = 1 */
    SSP_CLOCK_CPHA1_CPOL0 = (2u << 6),      /**< CPHA = 1, CPOL = 0 */
    SSP_CLOCK_CPHA1_CPOL1 = (3u << 6),      /**< CPHA = 1, CPOL = 1 */
    SSP_CLOCK_MODE0 = SSP_CLOCK_CPHA0_CPOL0,    /**< alias */
    SSP_CLOCK_MODE1 = SSP_CLOCK_CPHA1_CPOL0,    /**< alias */
    SSP_CLOCK_MODE2 = SSP_CLOCK_CPHA0_CPOL1,    /**< alias */
    SSP_CLOCK_MODE3 = SSP_CLOCK_CPHA1_CPOL1,    /**< alias */
} SSP_ClockFormat;

typedef enum SSP_FrameFormat {
    SSP_FRAMEFORMAT_SPI = (0 << 4),         /**< Frame format: SPI */
    SSP_FRAMEFORMAT_TI = (1u << 4),         /**< Frame format: TI SSI */
    SSP_FRAMEFORMAT_MICROWIRE = (2u << 4),  /**< Frame format: Microwire */
} SSP_FrameFormat;

typedef enum SSP_Bits {
    SSP_BITS_4  = (3u << 0),                /**< 4 bits/frame */
    SSP_BITS_5  = (4u << 0),                /**< 5 bits/frame */
    SSP_BITS_6  = (5u << 0),                /**< 6 bits/frame */
    SSP_BITS_7  = (6u << 0),                /**< 7 bits/frame */
    SSP_BITS_8  = (7u << 0),                /**< 8 bits/frame */
    SSP_BITS_9  = (8u << 0),                /**< 9 bits/frame */
    SSP_BITS_10 = (9u << 0),                /**< 10 bits/frame */
    SSP_BITS_11 = (10u << 0),               /**< 11 bits/frame */
    SSP_BITS_12 = (11u << 0),               /**< 12 bits/frame */
    SSP_BITS_13 = (12u << 0),               /**< 13 bits/frame */
    SSP_BITS_14 = (13u << 0),               /**< 14 bits/frame */
    SSP_BITS_15 = (14u << 0),               /**< 15 bits/frame */
    SSP_BITS_16 = (15u << 0),               /**< 16 bits/frame */
} SSP_Bits;

struct SSP_ConfigFormat {
    SSP_Bits bits;                          /**< Format config: bits/frame */
    SSP_ClockFormat clockFormat;            /**< Format config: clock phase/polarity */
    SSP_FrameFormat frameFormat;            /**< Format config: SPI/TI/Microwire */
};

typedef struct SSP_Config {
    SSP_Opcode opcode;                      /**< Config action opcode */

    union {
        SSP_Mode mode;                      /**< Config mode */
        struct SSP_ConfigFormat format;     /**< Config format */
        uint32_t bitrate;                   /**< Config clock speed */
    };
} SSP_Config;

/** Config list terminator. */
#define SSP_CONFIG_END \
    {.opcode = SSP_OPCODE_INVALID}


typedef enum SSP_CallbackEvent {
    SSP_EVENT_SUCCESS = 0,                  /**< Transaction completed successfully */
    SSP_EVENT_ASSERT_CHIPSELECT,            /**< Request to assert chip select line */
    SSP_EVENT_DEASSERT_CHIPSELECT,          /**< Request to deassert chip select line */
    SSP_EVENT_INTERRUPT,                    /**< "Interrupt" RX event */
} SSP_CallbackEvent;


/** Method to select a device (SSEL demuxing). */
typedef struct SSP_DeviceSelect {
    LPCLIB_Callback callback;               /**< Callback handler */
    uint8_t channel;                        /**< Channel number (arbitrary ID to identify device) */
} SSP_DeviceSelect;


typedef struct SSP_JobPhase {
    struct SSP_JobPhase *next;              /**< Pointer to following phase (or NULL) */
    union {
        const uint8_t *txstart8;            /**< Pointer to TX block start (8 bits/frame) */
        const uint16_t *txstart16;          /**< Pointer to TX block start (16 bits/frame) */
    };
    union {
        uint8_t *rxstart8;                  /**< Pointer to RX block start (8 bits/frame) */
        uint16_t *rxstart16;                /**< Pointer to RX block start (16 bits/frame) */
    };
    uint16_t length;                        /**< Number of frames in this phase */
    uint16_t idlePattern;                   /**< Idle pattern to be sent if no TX data available */
    LPCLIB_Switch interrupt;                /**< Trigger interrupt (= callback) at end of phase */
    LPCLIB_Switch txBarrier;                /**< Do not preload TX FIFO beyond this point */
} SSP_JobPhase;


typedef struct SSP_Job {
    SSP_JobPhase *firstPhase;               /**< Descriptor of first job phase */
    const SSP_DeviceSelect *pDeviceSelect;  /**< Device callback */
    const SSP_Config *pConfig;              /**< Interface parameters */
    uint16_t nsent;                         /**< Local context: TX frame counter */
    uint16_t nreceived;                     /**< Local context: RX frame counter */
    void *extraParameter;                   /**< Extra parameter to be passed on to device select */
} SSP_Job;


/** @} SSP Types, enums, macros */



/** \defgroup SSP_Public_Functions SSP API Functions
 *  @{
 */

/** Open an SSP bus.
 *
 *  Prepare the selected bus. Obtain a handle that must be used in calls to all other
 *  SSP module functions.
 *
 *  \param[in] bus Indicator that selects a bus.
 *  \param[out] pHandle Receives the handle
 *  \retval LPCLIB_SUCCESS Ok. Valid handle returned.
 *  \retval LPCLIB_BUSY Error. No valid handle returned. (Bus already open)
 */
LPCLIB_Result SSP_open (SSP_Name bus, SSP_Handle *pHandle);


/** Close an SSP bus.
 *
 *  Disable interrupts and clocks (power saving). Make sure there are no more
 *  active transactions.
 *
 *  \param pBus Pointer to handle of the SSP bus to be closed.
 */
void SSP_close (SSP_Handle *pBus);


/** Set options of the SSP block.
 *
 *  \param[in] bus Handle of the SSP bus
 *  \param[in] config Configuration descriptor
 */
void SSP_ioctl (SSP_Handle bus, const SSP_Config *pConfig);


/** Submit a job to the SSP driver.
 *
 *  \param[in] bus Handle for SSP bus
 *  \param[in] pJob Job descriptor
 *  \return ....
 */
LPCLIB_Result SSP_submitJob (SSP_Handle bus, SSP_Job *pJob);


/** Fill data into an SSP_JobPhase descriptor.
 *
 *  \param[in] phase Pointer to existing phase descriptor.
 *  \param[in] pTx Pointer to TX data (or NULL to send idle frames).
 *  \param[in] pRx Pointer to RX buffer (or NULL for no reception).
 *  \param[in] length Number of frames in this phase.
 *  \return Pointer to the phase descriptor (same as \a phase).
 */
SSP_JobPhase *SSP_makePhase (SSP_JobPhase *pPhase, const void *pTx, uint8_t *pRx, uint16_t length);

/** @} SSP API Functions */

#endif /* #ifndef __LPC17XX_SSP_H__ */

/** @} */



