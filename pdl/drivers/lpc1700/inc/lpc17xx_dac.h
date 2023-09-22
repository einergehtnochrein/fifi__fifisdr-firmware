/* Copyright (c) 2012-2013, NXP Semiconductors
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
 *  \brief DAC driver interface.
 *  This file defines all interface objects needed to use the DAC driver.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17XX_DAC_H__
#define __LPC17XX_DAC_H__

/** \defgroup DAC
 *  \ingroup API
 *  @{
 */

#include "lpc17xx_libconfig.h"

#include "lpclib_types.h"


/** \defgroup DAC_Public_Types DAC Types, enums, macros
 *  @{
 */


/** Handle for a DAC block. */
typedef struct DAC_Context *DAC_Handle;

/** Format of a DAC sample */
typedef uint32_t DAC_Sample;

/** Some selected output values. */
enum {
    DAC_VALUE_MINIMUM = 0,                  /** 0 V */
    DAC_VALUE_MAXIMUM = 1023,               /** 3.3 V */
    DAC_VALUE_HALF = (DAC_VALUE_MAXIMUM + 1 - DAC_VALUE_MINIMUM) / 2,
};


/** Opcodes to specify the configuration command in a call to \ref DAC_ioctl. */
typedef enum DAC_Opcode {
    DAC_OPCODE_INVALID = 0,                 /**< (List terminator) */
    DAC_OPCODE_SET_CALLBACK,                /**< Config action: Set callback handler */
    DAC_OPCODE_SET_SAMPLERATE,              /**< Config action: Set DAC sample rate in Hz.
                                             *                  or to select manual mode.
                                             */
} DAC_Opcode;

/** Callback configuration. */
struct DAC_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};


/** Descriptor to specify the configuration in a call to \ref DAC_ioctl. */
typedef struct DAC_Config {
    DAC_Opcode opcode;                      /**< Config action opcode */

    union {
        struct DAC_ConfigCallback callback; /**< Callback function */
        uint32_t sampleRateHz;              /**< DAC sample rate (or manual mode) */
    };
} DAC_Config;

/** Config list terminator. */
#define DAC_CONFIG_END \
    {.opcode = DAC_OPCODE_INVALID}


typedef enum DAC_CallbackEvent {
    DAC_EVENT_PHASE_COMPLETE = 0,           /**< Phase completed successfully */
    DAC_EVENT_JOB_COMPLETE,                 /**< Job completed successfully */
} DAC_CallbackEvent;


/** DAC transfer phase descriptor. */
typedef struct DAC_JobPhase {
    const struct DAC_JobPhase *next;        /**< Pointer to next transaction phase (or NULL) */
    DAC_Sample *data;                       /**< Pointer to data */
    uint32_t length;                        /**< Transfer length (in samples) */
    LPCLIB_Callback callback;               /**< Callback for terminal count */
    void *magic;                            /**< Magic value sent by callback (to identify phase) */
} DAC_JobPhase;


/** Transfer job descriptor for DAC.
 *  Consists of one or more phases. A job can end after the last phase, or it may continue
 *  to run endlessly if the last phase links back to the first phase.
 */
typedef struct DAC_Job {
    const DAC_JobPhase *firstPhase;         /**< Descriptor of first job phase */
} DAC_Job;

/** @} DAC Types, enums, macros */



/** \defgroup DAC_Public_Functions DAC API Functions
 *  @{
 */


/** Open the DAC.
 *
 *  \param[out] pHandle Handle to be used in future API calls to the DAC module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 */
LPCLIB_Result DAC_open (DAC_Handle *pHandle);


/** Close the DAC.
 *
 *  \param[out] pHandle Handle.
 *  \retval LPCLIB_SUCCESS Success.
 *  \retval LPCLIB_ILLEGAL_PARAMETER Handle is invalid.
 */
LPCLIB_Result DAC_close (DAC_Handle *pHandle);


/** Adjust the configuration of the DAC block.
 *
 *  \param[in] handle DAC handle
 *  \param[in] pConfig Describes the configuration change
 */
LPCLIB_Result DAC_ioctl (DAC_Handle handle, const DAC_Config *pConfig);


/** Manually update the DAC output.
 *
 *  \param[in] handle DAC handle
 *  \param[in] value
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Illegal handle, or value too large.
 */
LPCLIB_Result DAC_write (DAC_Handle handle, DAC_Sample value);


/** Submit a new DAC job to the driver.
 *
 *  DMA driven DAC output always uses the fast 1 MHz setting.
 *
 *  \param[in] handle Device handle.
 *  \param[in] pJob Pointer to a job descriptor that describes the details
 *      of the transaction.
 *  \param[in] dma Handle of a DMA channel. DMA transfer is mandatory.
 *
 *  \retval LPCLIB_SUCCESS Success. Job will be executed, and the callback function
 *      called once the phase/transaction ends.
 *  \retval LPCLIB_BUSY Failure. Job NOT submitted.
 */
struct DMA_ChannelContext;
LPCLIB_Result DAC_submitJob (DAC_Handle handle, const DAC_Job *pJob, struct DMA_ChannelContext *dma);


/** Assert the reset signal of the selected peripheral block.
 *
 *  \param[in] peripheral Peripheral block selector
 */
static DAC_Sample DAC_convertFromINT16 (int16_t value);



__FORCEINLINE(DAC_Sample DAC_convertFromINT16 (int16_t value))
{
    return (value ^ 0x8000) & 0x0000FFFF;
}


/** @} DAC API Functions */

/** @} DAC */

#endif /* #ifndef __LPC17XX_DAC_H__ */

