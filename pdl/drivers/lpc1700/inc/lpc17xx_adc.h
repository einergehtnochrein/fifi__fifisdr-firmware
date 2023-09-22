/* Copyright (c) 2011-2013, NXP Semiconductors
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
 *  \brief ADC driver interface.
 *  This file defines all interface objects needed to use the ADC driver.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17XX_ADC_H__
#define __LPC17XX_ADC_H__

/** \defgroup ADC
 *  \ingroup API
 *  @{
 */

#include "lpc17xx_libconfig.h"

#include "lpclib_types.h"


/** \defgroup ADC_Public_Types ADC Types, enums, macros
 *  @{
 */


/** Enumerator for the ADC block.
 */
typedef enum ADC_Name {
    ADC0 = 0,
    ADC_NUM_BLOCKS,     /* In order for this element to reflect the number of ADC blocks,
                         * you mustn't assign an explicit value to any of the other elements
                         * of this enum! (except for the first element which may get assigned to 0).
                         */
} ADC_Name;


typedef enum ADC_Channel {
    ADC_CH0 = 0,                            /**< ADC channel 0 */
    ADC_CH1 = 1,                            /**< ADC channel 1 */
    ADC_CH2 = 2,                            /**< ADC channel 2 */
    ADC_CH3 = 3,                            /**< ADC channel 3 */
    ADC_CH4 = 4,                            /**< ADC channel 4 */
    ADC_CH5 = 5,                            /**< ADC channel 5 */
    ADC_CH6 = 6,                            /**< ADC channel 6 */
    ADC_CH7 = 7,                            /**< ADC channel 7 */
} ADC_Channel;


/** Trigger mode for a channel. */
typedef enum ADC_TriggerMode {
    ADC_TRIGGERMODE_OFF,                    /**< No trigger function */
    ADC_TRIGGERMODE_RISING,                 /**< Trigger event: rising edge */
    ADC_TRIGGERMODE_FALLING,                /**< Trigger event: falling edge */
} ADC_TriggerMode;



typedef enum ADC_StartMode {
    ADC_START_OFF                   = 0,
    ADC_START_BURST                 = (0  << 27) | (0  << 24) | (1u << 16),
    ADC_START_NOW                   = (0  << 27) | (1u << 24) | (0  << 16),
    ADC_START_CT16B0_CAP0_RISING    = (0  << 27) | (2u << 24) | (0  << 16),
    ADC_START_CT16B0_CAP0_FALLING   = (1u << 27) | (2u << 24) | (0  << 16),
    ADC_START_CT32B0_CAP0_RISING    = (0  << 27) | (3u << 24) | (0  << 16),
    ADC_START_CT32B0_CAP0_FALLING   = (1u << 27) | (3u << 24) | (0  << 16),
    ADC_START_CT32B0_MAT0_RISING    = (0  << 27) | (4u << 24) | (0  << 16),
    ADC_START_CT32B0_MAT0_FALLING   = (1u << 27) | (4u << 24) | (0  << 16),
    ADC_START_CT32B0_MAT1_RISING    = (0  << 27) | (5u << 24) | (0  << 16),
    ADC_START_CT32B0_MAT1_FALLING   = (1u << 27) | (5u << 24) | (0  << 16),
    ADC_START_CT16B0_MAT0_RISING    = (0  << 27) | (6u << 24) | (0  << 16),
    ADC_START_CT16B0_MAT0_FALLING   = (1u << 27) | (6u << 24) | (0  << 16),
    ADC_START_CT16B0_MAT1_RISING    = (0  << 27) | (7u << 24) | (0  << 16),
    ADC_START_CT16B0_MAT1_FALLING   = (1u << 27) | (7u << 24) | (0  << 16),
} ADC_StartMode;


/** Handle for an open ADC block, as obtained by \ref ADC_open. */
typedef struct ADC_Context *ADC_Handle;


/** Opcodes to specify the configuration command in a call to \ref ADC_ioctl. */
typedef enum ADC_Opcode {
    ADC_OPCODE_INVALID = 0,                 /**< (List terminator) */
    ADC_OPCODE_SET_CALLBACK,                /**< Config action: Set callback handler */
    ADC_OPCODE_SET_STARTMODE,               /**< Config action: Set start mode (match/capture/etc) */
    ADC_OPCODE_SET_CHANNELMODE,             /**< Config action: enable/disable a channel */
    ADC_OPCODE_SET_CLOCK,                   /**< Config action: Set ADC clock (in % of maximum) */
} ADC_Opcode;


/** Config options for a channel. */
struct ADC_ConfigChannel {
    ADC_Channel index;                      /**< The channel number */
    LPCLIB_Switch active;                   /**< Channel activation */
    LPCLIB_Switch sendEvent;                /**< Send an event after conversion */
};


/** Callback configuration. */
struct ADC_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};

/** Descriptor to specify the configuration in a call to \ref ADC_ioctl. */
typedef struct ADC_Config {
    ADC_Opcode opcode;                      /**< Config action opcode */

    union {
        ADC_StartMode startMode;            /**< Start mode (trigger) */
        struct ADC_ConfigCallback callback; /**< Callback handler */
        struct ADC_ConfigChannel channel;   /**< Channel configuration */
        uint8_t clockPercentage;            /**< ADC clock (in percent of maximum) */
    };
} ADC_Config;

/** Config list terminator. */
#define ADC_CONFIG_END \
    {.opcode = ADC_OPCODE_INVALID}


/** @} ADC Types, enums, macros */



/** \defgroup ADC_Public_Functions ADC API Functions
 *  @{
 */


/** Open an ADC for use.
 *
 *  \param[in] block Indicator that selects an ADC block
 *  \param[out] pHandle Handle to be used in future API calls to the ADC module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 */
LPCLIB_Result ADC_open (ADC_Name block, ADC_Handle *pHandle);


/** Close an ADC.
 *
 *  \param[out] pHandle Handle.
 *  \retval LPCLIB_SUCCESS Success.
 *  \retval LPCLIB_ILLEGAL_PARAMETER Handle is invalid.
 */
LPCLIB_Result ADC_close (ADC_Handle *pHandle);



static uint32_t ADC_read (ADC_Channel channel);
LPCLIB_Result ADC_ioctl (ADC_Handle handle, const ADC_Config *pConfig);


__FORCEINLINE(uint32_t ADC_read (ADC_Channel channel))
{
    return ((uint32_t)LPC_ADC->DR[channel] >> 6);
}


/** @} ADC API Functions */

/** @} ADC */

#endif /* #ifndef __LPC17XX_ADC_H__ */

