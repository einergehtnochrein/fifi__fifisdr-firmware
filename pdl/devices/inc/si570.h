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


#ifndef _SI570_H_
#define _SI570_H_

#include "lpclib.h"


/** \defgroup SI570_Public_Types SI570 Types, enums, macros
 *  @{
 */

/** Convert frequency from "double" to one of the fractional integer formats. */
#define  _8_24(double_freq) (uint32_t)(double_freq*(1<<24))
#define _11_21(double_freq) (uint32_t)(double_freq*(1<<21))
#define _43_21(double_freq) (uint64_t)(double_freq*(1<<21))


/** Maximum frequency (depends on speed grade and output option). */
#define SI570_MAX_FREQUENCY     _11_21(160.0)



/** Handle for a SI570 device. */
typedef struct SI570_Context *SI570_Handle;


/** Opcodes to specify the configuration command in a call to \ref SI570_ioctl. */
typedef enum SI570_Opcode {
    SI570_OPCODE_SET_FREQUENCY,             /**< Set a new output frequency */
    SI570_OPCODE_GET_REGISTERS,             /**< Get register content */
    SI570_OPCODE_GET_FACTORY_STARTUP_REGISTERS, /**< Get registers after startup */
    SI570_OPCODE_SET_XTAL_FREQUENCY,        /**< Set XTAL frequency (8.24 format) */
    SI570_OPCODE_SET_SMOOTHTUNE,            /**< Set range for smooth tuning (ppm, default: 3500) */
    SI570_OPCODE_FLUSH_SMOOTHTUNE,          /**< Force full register write on next frequency setting */
} SI570_Opcode;


/** Descriptor to specify the configuration in a call to \ref SI570_ioctl. */
typedef struct SI570_Config {
    SI570_Opcode opcode;                    /**< Config action opcode */

    union {
        uint8_t (*pRegs)[6];                /**< Register content */
        uint32_t frequency1121;             /**< Frequency in 11.21 format */
        uint32_t frequency824;              /**< Frequency in 8.24 format */
        uint32_t smoothtune;                /**< Smooth tuning range (ppm) */
    };
} SI570_Config;



/** @} SI570 Types, enums, macros */


/** \defgroup SI570_Public_Functions SI570 API Functions
 *  @{
 */


/** Open the device.
 *
 *  Enable access to a SI570 oscillator connected to an already open I2C bus.
 *
 *  \param[in] bus Handle of I2C bus to which the device is connected.
 *  \param[in] address I2C device address
 *  \param[out] pHandle Device handle
 *  \retval LPCLIB_SUCCESS Success. \ref pHandle contains a valid handle.
 */
LPCLIB_Result SI570_open (I2C_Handle bus, const uint8_t address, SI570_Handle *pHandle);


/** Close the device.
 *
 *  Disable access to a SI570 oscillator.
 *
 *  \param[out] pHandle Device handle
 *  \retval LPCLIB_SUCCESS Success..
 */
LPCLIB_Result SI570_close (SI570_Handle *pHandle);


/** Configure the device.
 *
 *  Send request to SI570.
 *
 *  \param[in] handle Device handle
 *  \param[in] pConfig Pointer to a configuration descriptor
 */
LPCLIB_Result SI570_ioctl (SI570_Handle handle, const SI570_Config *pConfig);


/** Set a new frequency.
 *
 *  \param[in] handle Device handle
 *  \param[in] frequency1121 New frequency (format 11.21)
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Frequency out of range
 */
LPCLIB_Result SI570_write (SI570_Handle handle, uint32_t frequency1121);


/** Read current frequency.
 *
 *  \param[in] handle Device handle
 *  \param[in] pFrequency1121 Takes current frequency (format 11.21)
 *  \retval LPCLIB_SUCCESS ok
 */
LPCLIB_Result SI570_read (SI570_Handle handle, uint32_t *pFrequency1121);


/** Return register set for a given frequency (debug function).
 *
 *  This transformation calculates the register settings, even if the Si570 cannot
 *  be used for that frequency. The only limit is the range of parameters in the register
 *  fields. Caclculations use the real XTAL frequency.
 *
 *  \param[in] handle Device handle
 *  \param[in] frequency1121 Frequency value
 *  \param[out] pRegs Register settings for the six relevant registers.
 */
void SI570_calcVirtualRegisters (SI570_Handle handle,
                                 uint32_t frequency1121,
                                 uint8_t (*pRegs)[6]);

/** @} SI570 API Functions */

#endif
