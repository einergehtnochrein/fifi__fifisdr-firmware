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
 *  \brief GPIO driver interface.
 *  This file defines all interface objects needed to use the GPIO driver.
 *
 *  \author NXP Semiconductors
 */

#ifndef __LPC17XX_GPIO_H__
#define __LPC17XX_GPIO_H__

/** \defgroup GPIO
 *  \ingroup API
 *  @{
 */


#include "lpc17xx_libconfig.h"


#include "lpclib_types.h"


/** \defgroup GPIO_Public_Types GPIO Types, enums, macros
 *  @{
 */

typedef enum GPIO_Port32 {
    GPIO_PORT0 = 0,                     /**< GPIO port 0, 32-bit access */
    GPIO_PORT1,                         /**< GPIO port 1, 32-bit access */
    GPIO_PORT2,                         /**< GPIO port 2, 32-bit access */
    GPIO_PORT3,                         /**< GPIO port 3, 32-bit access */
    GPIO_PORT4,                         /**< GPIO port 4, 32-bit access */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    GPIO_PORT5,                         /**< GPIO port 5, 32-bit access */
#endif
    __NUM_GPIO__,
} GPIO_Port32;


typedef enum GPIO_Port16 {
    GPIO_PORT0_L = (0 << 4) | (0 << 0),  /**< GPIO port 0, 16-bit access, lower half-word */
    GPIO_PORT0_H = (1 << 4) | (0 << 0),  /**< GPIO port 0, 16-bit access, higher half-word */
    GPIO_PORT1_L = (0 << 4) | (1 << 0),  /**< GPIO port 1, 16-bit access, lower half-word */
    GPIO_PORT1_H = (1 << 4) | (1 << 0),  /**< GPIO port 1, 16-bit access, higher half-word */
    GPIO_PORT2_L = (0 << 4) | (2 << 0),  /**< GPIO port 2, 16-bit access, lower half-word */
    GPIO_PORT2_H = (1 << 4) | (2 << 0),  /**< GPIO port 2, 16-bit access, higher half-word */
    GPIO_PORT3_L = (0 << 4) | (3 << 0),  /**< GPIO port 3, 16-bit access, lower half-word */
    GPIO_PORT3_H = (1 << 4) | (3 << 0),  /**< GPIO port 3, 16-bit access, higher half-word */
    GPIO_PORT4_L = (0 << 4) | (4 << 0),  /**< GPIO port 4, 16-bit access, lower half-word */
    GPIO_PORT4_H = (1 << 4) | (4 << 0),  /**< GPIO port 4, 16-bit access, higher half-word */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    GPIO_PORT5_L = (0 << 4) | (5 << 0),  /**< GPIO port 5, 16-bit access, lower half-word */
#endif
} GPIO_Port16;


typedef enum GPIO_Port8 {
    GPIO_PORT0_0 = (0 << 4) | (0 << 0),  /**< GPIO port 0, 8-bit access, byte 0 */
    GPIO_PORT0_1 = (1 << 4) | (0 << 0),  /**< GPIO port 0, 8-bit access, byte 1 */
    GPIO_PORT0_2 = (2 << 4) | (0 << 0),  /**< GPIO port 0, 8-bit access, byte 2 */
    GPIO_PORT0_3 = (3 << 4) | (0 << 0),  /**< GPIO port 0, 8-bit access, byte 3 */
    GPIO_PORT1_0 = (0 << 4) | (1 << 0),  /**< GPIO port 1, 8-bit access, byte 0 */
    GPIO_PORT1_1 = (1 << 4) | (1 << 0),  /**< GPIO port 1, 8-bit access, byte 1 */
    GPIO_PORT1_2 = (2 << 4) | (1 << 0),  /**< GPIO port 1, 8-bit access, byte 2 */
    GPIO_PORT1_3 = (3 << 4) | (1 << 0),  /**< GPIO port 1, 8-bit access, byte 3 */
    GPIO_PORT2_0 = (0 << 4) | (2 << 0),  /**< GPIO port 2, 8-bit access, byte 0 */
    GPIO_PORT2_1 = (1 << 4) | (2 << 0),  /**< GPIO port 2, 8-bit access, byte 1 */
    GPIO_PORT2_2 = (2 << 4) | (2 << 0),  /**< GPIO port 2, 8-bit access, byte 2 */
    GPIO_PORT2_3 = (3 << 4) | (2 << 0),  /**< GPIO port 2, 8-bit access, byte 3 */
    GPIO_PORT3_0 = (0 << 4) | (3 << 0),  /**< GPIO port 3, 8-bit access, byte 0 */
    GPIO_PORT3_1 = (1 << 4) | (3 << 0),  /**< GPIO port 3, 8-bit access, byte 1 */
    GPIO_PORT3_2 = (2 << 4) | (3 << 0),  /**< GPIO port 3, 8-bit access, byte 2 */
    GPIO_PORT3_3 = (3 << 4) | (3 << 0),  /**< GPIO port 3, 8-bit access, byte 3 */
    GPIO_PORT4_0 = (0 << 4) | (4 << 0),  /**< GPIO port 4, 8-bit access, byte 0 */
    GPIO_PORT4_1 = (1 << 4) | (4 << 0),  /**< GPIO port 4, 8-bit access, byte 1 */
    GPIO_PORT4_2 = (2 << 4) | (4 << 0),  /**< GPIO port 4, 8-bit access, byte 2 */
    GPIO_PORT4_3 = (3 << 4) | (4 << 0),  /**< GPIO port 4, 8-bit access, byte 3 */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    GPIO_PORT5_0 = (0 << 4) | (5 << 0),  /**< GPIO port 5, 8-bit access, byte 0 */
#endif
} GPIO_Port8;


typedef enum GPIO_Pin {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    GPIO_0_0  = ( 0u << 3) | (0 << 0),      /**< GPIO 0.0 */
    GPIO_0_1  = ( 1u << 3) | (0 << 0),      /**< GPIO 0.1 */
    GPIO_0_2  = ( 2u << 3) | (0 << 0),      /**< GPIO 0.2 */
    GPIO_0_3  = ( 3u << 3) | (0 << 0),      /**< GPIO 0.3 */
    GPIO_0_4  = ( 4u << 3) | (0 << 0),      /**< GPIO 0.4 */
    GPIO_0_5  = ( 5u << 3) | (0 << 0),      /**< GPIO 0.5 */
    GPIO_0_6  = ( 6u << 3) | (0 << 0),      /**< GPIO 0.6 */
    GPIO_0_7  = ( 7u << 3) | (0 << 0),      /**< GPIO 0.7 */
    GPIO_0_8  = ( 8u << 3) | (0 << 0),      /**< GPIO 0.8 */
    GPIO_0_9  = ( 9u << 3) | (0 << 0),      /**< GPIO 0.9 */
    GPIO_0_10 = (10u << 3) | (0 << 0),      /**< GPIO 0.10 */
    GPIO_0_11 = (11u << 3) | (0 << 0),      /**< GPIO 0.11 */
    GPIO_0_15 = (15u << 3) | (0 << 0),      /**< GPIO 0.15 */
    GPIO_0_16 = (16u << 3) | (0 << 0),      /**< GPIO 0.16 */
    GPIO_0_17 = (17u << 3) | (0 << 0),      /**< GPIO 0.17 */
    GPIO_0_18 = (18u << 3) | (0 << 0),      /**< GPIO 0.18 */
    GPIO_0_19 = (19u << 3) | (0 << 0),      /**< GPIO 0.19 */
    GPIO_0_20 = (20u << 3) | (0 << 0),      /**< GPIO 0.20 */
    GPIO_0_21 = (21u << 3) | (0 << 0),      /**< GPIO 0.21 */
    GPIO_0_22 = (22u << 3) | (0 << 0),      /**< GPIO 0.22 */
    GPIO_0_23 = (23u << 3) | (0 << 0),      /**< GPIO 0.23 */
    GPIO_0_24 = (24u << 3) | (0 << 0),      /**< GPIO 0.24 */
    GPIO_0_25 = (25u << 3) | (0 << 0),      /**< GPIO 0.25 */
    GPIO_0_26 = (26u << 3) | (0 << 0),      /**< GPIO 0.26 */
    GPIO_0_27 = (27u << 3) | (0 << 0),      /**< GPIO 0.27 */
    GPIO_0_28 = (28u << 3) | (0 << 0),      /**< GPIO 0.28 */
    GPIO_0_29 = (29u << 3) | (0 << 0),      /**< GPIO 0.29 */
    GPIO_0_30 = (30u << 3) | (0 << 0),      /**< GPIO 0.30 */
    GPIO_1_0  = ( 0u << 3) | (1 << 0),      /**< GPIO 1.0 */
    GPIO_1_1  = ( 1u << 3) | (1 << 0),      /**< GPIO 1.1 */
    GPIO_1_4  = ( 4u << 3) | (1 << 0),      /**< GPIO 1.4 */
    GPIO_1_8  = ( 8u << 3) | (1 << 0),      /**< GPIO 1.8 */
    GPIO_1_9  = ( 9u << 3) | (1 << 0),      /**< GPIO 1.9 */
    GPIO_1_10 = (10u << 3) | (1 << 0),      /**< GPIO 1.10 */
    GPIO_1_14 = (14u << 3) | (1 << 0),      /**< GPIO 1.14 */
    GPIO_1_15 = (15u << 3) | (1 << 0),      /**< GPIO 1.15 */
    GPIO_1_16 = (16u << 3) | (1 << 0),      /**< GPIO 1.16 */
    GPIO_1_17 = (17u << 3) | (1 << 0),      /**< GPIO 1.17 */
    GPIO_1_18 = (18u << 3) | (1 << 0),      /**< GPIO 1.18 */
    GPIO_1_19 = (19u << 3) | (1 << 0),      /**< GPIO 1.19 */
    GPIO_1_20 = (20u << 3) | (1 << 0),      /**< GPIO 1.20 */
    GPIO_1_21 = (21u << 3) | (1 << 0),      /**< GPIO 1.21 */
    GPIO_1_22 = (22u << 3) | (1 << 0),      /**< GPIO 1.22 */
    GPIO_1_23 = (23u << 3) | (1 << 0),      /**< GPIO 1.23 */
    GPIO_1_24 = (24u << 3) | (1 << 0),      /**< GPIO 1.24 */
    GPIO_1_25 = (25u << 3) | (1 << 0),      /**< GPIO 1.25 */
    GPIO_1_26 = (26u << 3) | (1 << 0),      /**< GPIO 1.26 */
    GPIO_1_27 = (27u << 3) | (1 << 0),      /**< GPIO 1.27 */
    GPIO_1_28 = (28u << 3) | (1 << 0),      /**< GPIO 1.28 */
    GPIO_1_29 = (29u << 3) | (1 << 0),      /**< GPIO 1.29 */
    GPIO_1_30 = (30u << 3) | (1 << 0),      /**< GPIO 1.30 */
    GPIO_1_31 = (31u << 3) | (1 << 0),      /**< GPIO 1.31 */
    GPIO_2_0  = ( 0u << 3) | (2 << 0),      /**< GPIO 2.0 */
    GPIO_2_1  = ( 1u << 3) | (2 << 0),      /**< GPIO 2.1 */
    GPIO_2_2  = ( 2u << 3) | (2 << 0),      /**< GPIO 2.2 */
    GPIO_2_3  = ( 3u << 3) | (2 << 0),      /**< GPIO 2.3 */
    GPIO_2_4  = ( 4u << 3) | (2 << 0),      /**< GPIO 2.4 */
    GPIO_2_5  = ( 5u << 3) | (2 << 0),      /**< GPIO 2.5 */
    GPIO_2_6  = ( 6u << 3) | (2 << 0),      /**< GPIO 2.6 */
    GPIO_2_7  = ( 7u << 3) | (2 << 0),      /**< GPIO 2.7 */
    GPIO_2_8  = ( 8u << 3) | (2 << 0),      /**< GPIO 2.8 */
    GPIO_2_9  = ( 9u << 3) | (2 << 0),      /**< GPIO 2.9 */
    GPIO_2_10 = (10u << 3) | (2 << 0),      /**< GPIO 2.10 */
    GPIO_2_11 = (11u << 3) | (2 << 0),      /**< GPIO 2.11 */
    GPIO_2_12 = (12u << 3) | (2 << 0),      /**< GPIO 2.12 */
    GPIO_2_13 = (13u << 3) | (2 << 0),      /**< GPIO 2.13 */
    GPIO_3_25 = (25u << 3) | (3 << 0),      /**< GPIO 3.25 */
    GPIO_3_26 = (26u << 3) | (3 << 0),      /**< GPIO 3.26 */
    GPIO_4_28 = (28u << 3) | (4 << 0),      /**< GPIO 4.28 */
    GPIO_4_29 = (29u << 3) | (4 << 0),      /**< GPIO 4.29 */
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    GPIO_0_0  = ( 0u << 3) | (0 << 0),      /**< GPIO 0.0 */
    GPIO_0_1  = ( 1u << 3) | (0 << 0),      /**< GPIO 0.1 */
    GPIO_0_2  = ( 2u << 3) | (0 << 0),      /**< GPIO 0.2 */
    GPIO_0_3  = ( 3u << 3) | (0 << 0),      /**< GPIO 0.3 */
    GPIO_0_4  = ( 4u << 3) | (0 << 0),      /**< GPIO 0.4 */
    GPIO_0_5  = ( 5u << 3) | (0 << 0),      /**< GPIO 0.5 */
    GPIO_0_6  = ( 6u << 3) | (0 << 0),      /**< GPIO 0.6 */
    GPIO_0_7  = ( 7u << 3) | (0 << 0),      /**< GPIO 0.7 */
    GPIO_0_8  = ( 8u << 3) | (0 << 0),      /**< GPIO 0.8 */
    GPIO_0_9  = ( 9u << 3) | (0 << 0),      /**< GPIO 0.9 */
    GPIO_0_10 = (10u << 3) | (0 << 0),      /**< GPIO 0.10 */
    GPIO_0_11 = (11u << 3) | (0 << 0),      /**< GPIO 0.11 */
    GPIO_0_12 = (12u << 3) | (0 << 0),      /**< GPIO 0.12 */
    GPIO_0_13 = (13u << 3) | (0 << 0),      /**< GPIO 0.13 */
    GPIO_0_14 = (14u << 3) | (0 << 0),      /**< GPIO 0.14 */
    GPIO_0_15 = (15u << 3) | (0 << 0),      /**< GPIO 0.15 */
    GPIO_0_16 = (16u << 3) | (0 << 0),      /**< GPIO 0.16 */
    GPIO_0_17 = (17u << 3) | (0 << 0),      /**< GPIO 0.17 */
    GPIO_0_18 = (18u << 3) | (0 << 0),      /**< GPIO 0.18 */
    GPIO_0_19 = (19u << 3) | (0 << 0),      /**< GPIO 0.19 */
    GPIO_0_20 = (20u << 3) | (0 << 0),      /**< GPIO 0.20 */
    GPIO_0_21 = (21u << 3) | (0 << 0),      /**< GPIO 0.21 */
    GPIO_0_22 = (22u << 3) | (0 << 0),      /**< GPIO 0.22 */
    GPIO_0_23 = (23u << 3) | (0 << 0),      /**< GPIO 0.23 */
    GPIO_0_24 = (24u << 3) | (0 << 0),      /**< GPIO 0.24 */
    GPIO_0_25 = (25u << 3) | (0 << 0),      /**< GPIO 0.25 */
    GPIO_0_26 = (26u << 3) | (0 << 0),      /**< GPIO 0.26 */
    GPIO_0_27 = (27u << 3) | (0 << 0),      /**< GPIO 0.27 */
    GPIO_0_28 = (28u << 3) | (0 << 0),      /**< GPIO 0.28 */
    GPIO_0_29 = (29u << 3) | (0 << 0),      /**< GPIO 0.29 */
    GPIO_0_30 = (30u << 3) | (0 << 0),      /**< GPIO 0.30 */
    GPIO_0_31 = (31u << 3) | (0 << 0),      /**< GPIO 0.31 */
    GPIO_1_0  = ( 0u << 3) | (1 << 0),      /**< GPIO 1.0 */
    GPIO_1_1  = ( 1u << 3) | (1 << 0),      /**< GPIO 1.1 */
    GPIO_1_2  = ( 2u << 3) | (1 << 0),      /**< GPIO 1.2 */
    GPIO_1_3  = ( 3u << 3) | (1 << 0),      /**< GPIO 1.3 */
    GPIO_1_4  = ( 4u << 3) | (1 << 0),      /**< GPIO 1.4 */
    GPIO_1_5  = ( 5u << 3) | (1 << 0),      /**< GPIO 1.5 */
    GPIO_1_6  = ( 6u << 3) | (1 << 0),      /**< GPIO 1.6 */
    GPIO_1_7  = ( 7u << 3) | (1 << 0),      /**< GPIO 1.7 */
    GPIO_1_8  = ( 8u << 3) | (1 << 0),      /**< GPIO 1.8 */
    GPIO_1_9  = ( 9u << 3) | (1 << 0),      /**< GPIO 1.9 */
    GPIO_1_10 = (10u << 3) | (1 << 0),      /**< GPIO 1.10 */
    GPIO_1_11 = (11u << 3) | (1 << 0),      /**< GPIO 1.11 */
    GPIO_1_12 = (12u << 3) | (1 << 0),      /**< GPIO 1.12 */
    GPIO_1_13 = (13u << 3) | (1 << 0),      /**< GPIO 1.13 */
    GPIO_1_14 = (14u << 3) | (1 << 0),      /**< GPIO 1.14 */
    GPIO_1_15 = (15u << 3) | (1 << 0),      /**< GPIO 1.15 */
    GPIO_1_16 = (16u << 3) | (1 << 0),      /**< GPIO 1.16 */
    GPIO_1_17 = (17u << 3) | (1 << 0),      /**< GPIO 1.17 */
    GPIO_1_18 = (18u << 3) | (1 << 0),      /**< GPIO 1.18 */
    GPIO_1_19 = (19u << 3) | (1 << 0),      /**< GPIO 1.19 */
    GPIO_1_20 = (20u << 3) | (1 << 0),      /**< GPIO 1.20 */
    GPIO_1_21 = (21u << 3) | (1 << 0),      /**< GPIO 1.21 */
    GPIO_1_22 = (22u << 3) | (1 << 0),      /**< GPIO 1.22 */
    GPIO_1_23 = (23u << 3) | (1 << 0),      /**< GPIO 1.23 */
    GPIO_1_24 = (24u << 3) | (1 << 0),      /**< GPIO 1.24 */
    GPIO_1_25 = (25u << 3) | (1 << 0),      /**< GPIO 1.25 */
    GPIO_1_26 = (26u << 3) | (1 << 0),      /**< GPIO 1.26 */
    GPIO_1_27 = (27u << 3) | (1 << 0),      /**< GPIO 1.27 */
    GPIO_1_28 = (28u << 3) | (1 << 0),      /**< GPIO 1.28 */
    GPIO_1_29 = (29u << 3) | (1 << 0),      /**< GPIO 1.29 */
    GPIO_1_30 = (30u << 3) | (1 << 0),      /**< GPIO 1.30 */
    GPIO_1_31 = (31u << 3) | (1 << 0),      /**< GPIO 1.31 */
    GPIO_2_0  = ( 0u << 3) | (2 << 0),      /**< GPIO 2.0 */
    GPIO_2_1  = ( 1u << 3) | (2 << 0),      /**< GPIO 2.1 */
    GPIO_2_2  = ( 2u << 3) | (2 << 0),      /**< GPIO 2.2 */
    GPIO_2_3  = ( 3u << 3) | (2 << 0),      /**< GPIO 2.3 */
    GPIO_2_4  = ( 4u << 3) | (2 << 0),      /**< GPIO 2.4 */
    GPIO_2_5  = ( 5u << 3) | (2 << 0),      /**< GPIO 2.5 */
    GPIO_2_6  = ( 6u << 3) | (2 << 0),      /**< GPIO 2.6 */
    GPIO_2_7  = ( 7u << 3) | (2 << 0),      /**< GPIO 2.7 */
    GPIO_2_8  = ( 8u << 3) | (2 << 0),      /**< GPIO 2.8 */
    GPIO_2_9  = ( 9u << 3) | (2 << 0),      /**< GPIO 2.9 */
    GPIO_2_10 = (10u << 3) | (2 << 0),      /**< GPIO 2.10 */
    GPIO_2_11 = (11u << 3) | (2 << 0),      /**< GPIO 2.11 */
    GPIO_2_12 = (12u << 3) | (2 << 0),      /**< GPIO 2.12 */
    GPIO_2_13 = (13u << 3) | (2 << 0),      /**< GPIO 2.13 */
    GPIO_2_14 = (14u << 3) | (2 << 0),      /**< GPIO 2.14 */
    GPIO_2_15 = (15u << 3) | (2 << 0),      /**< GPIO 2.15 */
    GPIO_2_16 = (16u << 3) | (2 << 0),      /**< GPIO 2.16 */
    GPIO_2_17 = (17u << 3) | (2 << 0),      /**< GPIO 2.17 */
    GPIO_2_18 = (18u << 3) | (2 << 0),      /**< GPIO 2.18 */
    GPIO_2_19 = (19u << 3) | (2 << 0),      /**< GPIO 2.19 */
    GPIO_2_20 = (20u << 3) | (2 << 0),      /**< GPIO 2.20 */
    GPIO_2_21 = (21u << 3) | (2 << 0),      /**< GPIO 2.21 */
    GPIO_2_22 = (22u << 3) | (2 << 0),      /**< GPIO 2.22 */
    GPIO_2_23 = (23u << 3) | (2 << 0),      /**< GPIO 2.23 */
    GPIO_2_24 = (24u << 3) | (2 << 0),      /**< GPIO 2.24 */
    GPIO_2_25 = (25u << 3) | (2 << 0),      /**< GPIO 2.25 */
    GPIO_2_26 = (26u << 3) | (2 << 0),      /**< GPIO 2.26 */
    GPIO_2_27 = (27u << 3) | (2 << 0),      /**< GPIO 2.27 */
    GPIO_2_28 = (28u << 3) | (2 << 0),      /**< GPIO 2.28 */
    GPIO_2_29 = (29u << 3) | (2 << 0),      /**< GPIO 2.29 */
    GPIO_2_30 = (30u << 3) | (2 << 0),      /**< GPIO 2.30 */
    GPIO_2_31 = (31u << 3) | (2 << 0),      /**< GPIO 2.31 */
    GPIO_3_0  = ( 0u << 3) | (3 << 0),      /**< GPIO 3.0 */
    GPIO_3_1  = ( 1u << 3) | (3 << 0),      /**< GPIO 3.1 */
    GPIO_3_2  = ( 2u << 3) | (3 << 0),      /**< GPIO 3.2 */
    GPIO_3_3  = ( 3u << 3) | (3 << 0),      /**< GPIO 3.3 */
    GPIO_3_4  = ( 4u << 3) | (3 << 0),      /**< GPIO 3.4 */
    GPIO_3_5  = ( 5u << 3) | (3 << 0),      /**< GPIO 3.5 */
    GPIO_3_6  = ( 6u << 3) | (3 << 0),      /**< GPIO 3.6 */
    GPIO_3_7  = ( 7u << 3) | (3 << 0),      /**< GPIO 3.7 */
    GPIO_3_8  = ( 8u << 3) | (3 << 0),      /**< GPIO 3.8 */
    GPIO_3_9  = ( 9u << 3) | (3 << 0),      /**< GPIO 3.9 */
    GPIO_3_10 = (10u << 3) | (3 << 0),      /**< GPIO 3.10 */
    GPIO_3_11 = (11u << 3) | (3 << 0),      /**< GPIO 3.11 */
    GPIO_3_12 = (12u << 3) | (3 << 0),      /**< GPIO 3.12 */
    GPIO_3_13 = (13u << 3) | (3 << 0),      /**< GPIO 3.13 */
    GPIO_3_14 = (14u << 3) | (3 << 0),      /**< GPIO 3.14 */
    GPIO_3_15 = (15u << 3) | (3 << 0),      /**< GPIO 3.15 */
    GPIO_3_16 = (16u << 3) | (3 << 0),      /**< GPIO 3.16 */
    GPIO_3_17 = (17u << 3) | (3 << 0),      /**< GPIO 3.17 */
    GPIO_3_18 = (18u << 3) | (3 << 0),      /**< GPIO 3.18 */
    GPIO_3_19 = (19u << 3) | (3 << 0),      /**< GPIO 3.19 */
    GPIO_3_20 = (20u << 3) | (3 << 0),      /**< GPIO 3.20 */
    GPIO_3_21 = (21u << 3) | (3 << 0),      /**< GPIO 3.21 */
    GPIO_3_22 = (22u << 3) | (3 << 0),      /**< GPIO 3.22 */
    GPIO_3_23 = (23u << 3) | (3 << 0),      /**< GPIO 3.23 */
    GPIO_3_24 = (24u << 3) | (3 << 0),      /**< GPIO 3.24 */
    GPIO_3_25 = (25u << 3) | (3 << 0),      /**< GPIO 3.25 */
    GPIO_3_26 = (26u << 3) | (3 << 0),      /**< GPIO 3.26 */
    GPIO_3_27 = (27u << 3) | (3 << 0),      /**< GPIO 3.27 */
    GPIO_3_28 = (28u << 3) | (3 << 0),      /**< GPIO 3.28 */
    GPIO_3_29 = (29u << 3) | (3 << 0),      /**< GPIO 3.29 */
    GPIO_3_30 = (30u << 3) | (3 << 0),      /**< GPIO 3.30 */
    GPIO_3_31 = (31u << 3) | (3 << 0),      /**< GPIO 3.31 */
    GPIO_4_0  = ( 0u << 3) | (4 << 0),      /**< GPIO 4.0 */
    GPIO_4_1  = ( 1u << 3) | (4 << 0),      /**< GPIO 4.1 */
    GPIO_4_2  = ( 2u << 3) | (4 << 0),      /**< GPIO 4.2 */
    GPIO_4_3  = ( 3u << 3) | (4 << 0),      /**< GPIO 4.3 */
    GPIO_4_4  = ( 4u << 3) | (4 << 0),      /**< GPIO 4.4 */
    GPIO_4_5  = ( 5u << 3) | (4 << 0),      /**< GPIO 4.5 */
    GPIO_4_6  = ( 6u << 3) | (4 << 0),      /**< GPIO 4.6 */
    GPIO_4_7  = ( 7u << 3) | (4 << 0),      /**< GPIO 4.7 */
    GPIO_4_8  = ( 8u << 3) | (4 << 0),      /**< GPIO 4.8 */
    GPIO_4_9  = ( 9u << 3) | (4 << 0),      /**< GPIO 4.9 */
    GPIO_4_10 = (10u << 3) | (4 << 0),      /**< GPIO 4.10 */
    GPIO_4_11 = (11u << 3) | (4 << 0),      /**< GPIO 4.11 */
    GPIO_4_12 = (12u << 3) | (4 << 0),      /**< GPIO 4.12 */
    GPIO_4_13 = (13u << 3) | (4 << 0),      /**< GPIO 4.13 */
    GPIO_4_14 = (14u << 3) | (4 << 0),      /**< GPIO 4.14 */
    GPIO_4_15 = (15u << 3) | (4 << 0),      /**< GPIO 4.15 */
    GPIO_4_16 = (16u << 3) | (4 << 0),      /**< GPIO 4.16 */
    GPIO_4_17 = (17u << 3) | (4 << 0),      /**< GPIO 4.17 */
    GPIO_4_18 = (18u << 3) | (4 << 0),      /**< GPIO 4.18 */
    GPIO_4_19 = (19u << 3) | (4 << 0),      /**< GPIO 4.19 */
    GPIO_4_20 = (20u << 3) | (4 << 0),      /**< GPIO 4.20 */
    GPIO_4_21 = (21u << 3) | (4 << 0),      /**< GPIO 4.21 */
    GPIO_4_22 = (22u << 3) | (4 << 0),      /**< GPIO 4.22 */
    GPIO_4_23 = (23u << 3) | (4 << 0),      /**< GPIO 4.23 */
    GPIO_4_24 = (24u << 3) | (4 << 0),      /**< GPIO 4.24 */
    GPIO_4_25 = (25u << 3) | (4 << 0),      /**< GPIO 4.25 */
    GPIO_4_26 = (26u << 3) | (4 << 0),      /**< GPIO 4.26 */
    GPIO_4_27 = (27u << 3) | (4 << 0),      /**< GPIO 4.27 */
    GPIO_4_28 = (28u << 3) | (4 << 0),      /**< GPIO 4.28 */
    GPIO_4_29 = (29u << 3) | (4 << 0),      /**< GPIO 4.29 */
    GPIO_4_30 = (30u << 3) | (4 << 0),      /**< GPIO 4.30 */
    GPIO_4_31 = (31u << 3) | (4 << 0),      /**< GPIO 4.31 */
    GPIO_5_0  = ( 0u << 3) | (5 << 0),      /**< GPIO 5.0 */
    GPIO_5_1  = ( 1u << 3) | (5 << 0),      /**< GPIO 5.1 */
    GPIO_5_2  = ( 2u << 3) | (5 << 0),      /**< GPIO 5.2 */
    GPIO_5_3  = ( 3u << 3) | (5 << 0),      /**< GPIO 5.3 */
    GPIO_5_4  = ( 4u << 3) | (5 << 0),      /**< GPIO 5.4 */
#endif
} GPIO_Pin;


/** Opcodes to specify the configuration command in a call to \ref GPIO_ioctl. */
typedef enum GPIO_Opcode {
    GPIO_OPCODE_INVALID = 0,                /**< (List terminator) */
#if LPCLIB_GPIO_INTERRUPTS
    GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,    /**< Config action: Interrupt configuration */
#endif
} GPIO_Opcode;


#if LPCLIB_GPIO_INTERRUPTS
/** GPIO interrupt mode (edge vs. level) */
typedef enum GPIO_InterruptMode {
    GPIO_INT_FALLING_EDGE,                  /**< GPIO interrupt on falling edge */
    GPIO_INT_RISING_EDGE,                   /**< GPIO interrupt on rising edge */
    GPIO_INT_BOTH_EDGES,                    /**< GPIO interrupt on both edges */
} GPIO_InterruptMode;


/** Configuration parameters for GPIO interrupt. */
struct GPIO_ConfigInterrupt {
    GPIO_Pin pin;                           /**< The pin to be configured */
    LPCLIB_Switch enable;                   /**< Flag: Enable interrupt */
    GPIO_InterruptMode mode;                /**< Level (vs edge) sensitive */
    LPCLIB_Callback callback;               /**< Callback handler (or NULL) */
};
#endif


/** Descriptor to specify the configuration in a call to \ref I2C_Ioctl. */
typedef struct GPIO_Config {
    GPIO_Opcode opcode;                     /**< Config action opcode */

    union {
    #if LPCLIB_GPIO_INTERRUPTS
        struct GPIO_ConfigInterrupt pinInterrupt;   /**< Interrupts */
    #endif
        uint8_t __dummy__;
    };
} GPIO_Config;


/** Config list terminator. */
#define GPIO_CONFIG_END \
    {.opcode = GPIO_OPCODE_INVALID}


/** @} GPIO Types, enums, macros */

/** \defgroup GPIO_Public_Functions GPIO API Functions
 *  @{
 */


/** Prepare the use of the GPIO block.
 */
void GPIO_open (void);


/** Configure the GPIO block.
 *
 *  Pass a configuration command to the GPIO block.
 *
 *  \param[in] pConfig Pointer to a configuration descriptor
 */
void GPIO_ioctl (const GPIO_Config *pConfig);


/** Set GPIO pin direction for whole 32-bit port.
 *
 *  \param[in] port Port number
 *  \param[in] value 32-bit value where each bit corresponds to the pin in the
 *                   same bit position of the port. (0=input, 1=output).
 *  \param[in] mask 32-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_setDir32 (GPIO_Port32 port, uint32_t value, uint32_t mask);

/** Set GPIO pin direction for whole 32-bit port.
 *
 *  \param[in] port Pin selector
 *  \param[in] outputEnable Boolean value to enable the output.
 */
void GPIO_setDirBit (GPIO_Pin pin, LPCLIB_Switch outputEnable);

/** Write value to a 32-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value to be written to port pins.
 */
static void GPIO_write32 (GPIO_Port32 port, uint32_t value);

/** Write value to a 16-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value to be written to port pins.
 */
static void GPIO_write16 (GPIO_Port16 port, uint16_t value);

/** Write value to an 8-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value to be written to port pins.
 */
static void GPIO_write8  (GPIO_Port8  port, uint8_t  value);

/** Write value (0/1) to a GPIO port pin.
 *
 *  \param[in] port Pin selector

 *  \param[in] value Binary value to be written to port pin.
 */
static void GPIO_writeBit(GPIO_Pin pin, uint8_t value);

/** Write value to a 32-bit GPIO port through a mask.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value to be written to port pins.
 *  \param[in] mask 32-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_write32WithMask (GPIO_Port32 port, uint32_t value, uint32_t mask);

/** Write value to a 16-bit GPIO port through a mask.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value to be written to port pins.
 *  \param[in] mask 16-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_write16WithMask (GPIO_Port16 port, uint16_t value, uint16_t mask);

/** Write value to an 8-bit GPIO port through a mask.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value to be written to port pins.
 *  \param[in] mask 8-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_write8WithMask (GPIO_Port8 port, uint8_t value, uint8_t mask);

/** Return current state of the pins of a 32-bit GPIO port.
 *
 *  \param port Port selector
 *  \return 32-bit value with state of all port pins.
 */
static uint32_t GPIO_read32 (GPIO_Port32 port);

/** Return current state of the pins of a 16-bit GPIO port.
 *
 *  \param port Port selector
 *  \return 16-bit value with state of all port pins.
 */
static uint16_t GPIO_read16 (GPIO_Port16 port);

/** Return current state of the pins of an 8-bit GPIO port.
 *
 *  \param port Port selector
 *  \return 8-bit value with state of all port pins.
 */
static uint8_t GPIO_read8 (GPIO_Port8 port);

/** Return current state of a single GPIO port pin.
 *
 *  \param port Pin selector
 *  \return (32-bit) value with state (0/1) of the port pin.
 */
static uint32_t GPIO_readBit (GPIO_Pin pin);

/** Set pins of a 32-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value where a 1 in bit position n sets pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_set32 (GPIO_Port32 port, uint32_t value);

/** Set pins of a 16-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value where a 1 in bit position n sets pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_set16 (GPIO_Port16 port, uint16_t value);

/** Set pins of an 8-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value where a 1 in bit position n sets pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_set8 (GPIO_Port8 port, uint8_t value);

/** Clear pins of a 32-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value where a 1 in bit position n clears pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_clr32 (GPIO_Port32 port, uint32_t value);

/** Clear pins of a 16-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value where a 1 in bit position n clears pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_clr16 (GPIO_Port16 port, uint16_t value);

/** Clear pins of an 8-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value where a 1 in bit position n clears pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_clr8 (GPIO_Port8 port, uint8_t value);


static LPC_GPIO_TypeDef * const __gpio[__NUM_GPIO__] = {
    LPC_GPIO0,
    LPC_GPIO1,
    LPC_GPIO2,
    LPC_GPIO3,
    LPC_GPIO4,
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    LPC_GPIO5,
#endif
};


static volatile uint32_t * const __bbgpio[__NUM_GPIO__] = {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    (volatile uint32_t *)(0x23380280),
    (volatile uint32_t *)(0x23380680),
    (volatile uint32_t *)(0x23380A80),
    (volatile uint32_t *)(0x23380E80),
    (volatile uint32_t *)(0x23381280),
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    (volatile uint32_t *)(0x23300280),
    (volatile uint32_t *)(0x23300680),
    (volatile uint32_t *)(0x23300A80),
    (volatile uint32_t *)(0x23300E80),
    (volatile uint32_t *)(0x23301280),
    (volatile uint32_t *)(0x23301680),
#endif
};



__FORCEINLINE(void GPIO_write32(GPIO_Port32 port, uint32_t value))
{
    __gpio[port]->FIOPIN32 = value;
}

__FORCEINLINE(void GPIO_write16(GPIO_Port16 port, uint16_t value))
{
    __gpio[port & 0x0F]->FIOPIN16[(port >> 4) & 0x01] = value;
}

__FORCEINLINE(void GPIO_write8(GPIO_Port8 port, uint8_t value))
{
    __gpio[port & 0x0F]->FIOPIN8[(port >> 4) & 0x03] = value;
}

__FORCEINLINE(void GPIO_writeBit(GPIO_Pin pin, uint8_t value))
{
    __bbgpio[(pin >> 0) & 7][(pin >> 3) & 0x1F] = value;
    __DSB();    /* NOTE: This data barrier is required to ensure correct operation in case
                 *       the compiler places two STR instructions to the PIN register
                 *       via bit-banding directly after each other.
                 */
}



__FORCEINLINE(uint32_t GPIO_read32(GPIO_Port32 port))
{
    return __gpio[port]->FIOPIN32;
}

__FORCEINLINE(uint16_t GPIO_read16(GPIO_Port16 port))
{
    return __gpio[port & 0x0F]->FIOPIN16[(port >> 4) & 0x01];
}

__FORCEINLINE(uint8_t GPIO_read8(GPIO_Port8 port))
{
    return __gpio[port & 0x0F]->FIOPIN8[(port >> 4) & 0x03];
}

__FORCEINLINE(uint32_t GPIO_readBit(GPIO_Pin pin))
{
    return __bbgpio[(pin >> 0) & 3][(pin >> 3) & 0x1F];
}



__FORCEINLINE(void GPIO_set32 (GPIO_Port32 port, uint32_t value))
{
    __gpio[port & 0x0F]->FIOSET32 = value;
}

__FORCEINLINE(void GPIO_set16 (GPIO_Port16 port, uint16_t value))
{
    __gpio[port & 0x0F]->FIOSET16[(port >> 4) & 0x01] = value;
}

__FORCEINLINE(void GPIO_set8 (GPIO_Port8 port, uint8_t value))
{
    __gpio[port & 0x0F]->FIOSET8[(port >> 4) & 0x03] = value;
}

__FORCEINLINE(void GPIO_clr32 (GPIO_Port32 port, uint32_t value))
{
    __gpio[port & 0x0F]->FIOCLR32 = value;
}

__FORCEINLINE(void GPIO_clr16 (GPIO_Port16 port, uint16_t value))
{
    __gpio[port & 0x0F]->FIOCLR16[(port >> 4) & 0x01] = value;
}

__FORCEINLINE(void GPIO_clr8 (GPIO_Port8 port, uint8_t value))
{
    __gpio[port & 0x0F]->FIOCLR8[(port >> 4) & 0x03] = value;
}


/** @} GPIO API Functions */

/** @} GPIO */

#endif /* #ifndef __LPC17XX_GPIO_H__ */

