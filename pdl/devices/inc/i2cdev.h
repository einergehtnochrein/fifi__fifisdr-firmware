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
 *  \brief Utility functions for I2C devices.
 *
 *  \author NXP Semiconductors
 */


#ifndef __I2CDEV_H
#define __I2CDEV_H

#include <stdint.h>

#include "lpclib.h"

#if LPCLIB_I2C

/** \defgroup I2CDEV_Public_Types I2CDEV Types, enums, macros
 *  @{
 */


/** @} I2CDEV Types, enums, macros */



/** \defgroup I2CDEV_Public_Functions I2C device utility API functions
 *  @{
 */


/** Single buffer write operation to a device.
 *
 *  \param[in] bus I2C bus handle.
 *  \param[in] address Device slave address
 *  \param[in] numBytes Number of bytes to be written.
 *  \param[in] txBuffer Pointer to TX data
 *
 *  \retval LPCLIB_SUCCESS Success.
 */
LPCLIB_Result I2CDEV_write (I2C_Handle bus, uint8_t address, int numBytes, const void *txBuffer);


/** Single buffer read operation from a device.
 *
 *  \param[in] bus I2C bus handle.
 *  \param[in] address Device slave address
 *  \param[in] numBytes Number of bytes to be received.
 *  \param[in] rxBuffer Pointer to RX data
 *
 *  \retval LPCLIB_SUCCESS Success.
 */
LPCLIB_Result I2CDEV_read (I2C_Handle bus, uint8_t address, int numBytes, void *rxBuffer);


/** Single buffer write+read operation.
 *
 *  \param[in] bus I2C bus handle.
 *  \param[in] address Device slave address
 *  \param[in] numTxBytes Number of bytes to be sent.
 *  \param[in] txBuffer Pointer to TX data
 *  \param[in] numRxBytes Number of bytes to be received.
 *  \param[in] rxBuffer Pointer to RX data
 *
 *  \retval LPCLIB_SUCCESS Success.
 */
LPCLIB_Result I2CDEV_writeAndRead (I2C_Handle bus,
                                   uint8_t address,
                                   int numTxBytes,
                                   const void *txBuffer,
                                   int numRxBytes,
                                   void *rxBuffer);


/** @} I2CDEV API Functions */

#endif

#endif /* #ifndef __I2CDEV_H */
