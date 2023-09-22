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
 *  \brief EMC driver interface.
 *  This file defines all interface objects needed to use the EMC driver.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17XX_EMC_H__
#define __LPC17XX_EMC_H__

/** \defgroup EMC
 *  \ingroup API
 *  @{
 */

#include "lpc17xx_libconfig.h"

#if LPCLIB_EMC

#include "lpclib_types.h"



/** \defgroup EMC_Public_Types EMC Types, enums, macros
 *  @{
 */


/** Handle for an open EMC block, as obtained by \ref EMC_open. */
typedef struct EMC_Context *EMC_Handle;


/** Opcodes to specify the configuration command in a call to \ref EMC_ioctl. */
typedef enum EMC_Opcode {
    EMC_OPCODE_INVALID = 0,                 /**< (List terminator) */
    EMC_OPCODE_DYNAMIC_GLOBALS,             /**< Config action: Set global timings for dynamic memory */
    EMC_OPCODE_DYNAMIC_DEVICE,              /**< Config action: Configure an SDRAM */
    EMC_OPCODE_STATIC_DEVICE,               /**< Config action: Configure a static memory */
} EMC_Opcode;


typedef struct EMC_ConfigDynamicGlobals {
    uint16_t refreshCycles;                 /**< Number of refresh cycles (in CCLKs) */
    uint8_t trp;                            /**< Precharge Command Period (n + 1) */
    uint8_t tras;                           /**< Active to Precharge Command Period (n + 1) */
    uint8_t tsrex;                          /**< Self-refresh exit time (n + 1) */
    uint8_t tapr;                           /**< Last-data-out to active command time (n + 1) */
    uint8_t tdal;                           /**< Data-in to active command time (n) */
    uint8_t twr;                            /**< Write recovery time (n + 1) */
    uint8_t trc;                            /**< Active to active command period (n + 1) */
    uint8_t trfc;                           /**< Auto-refresh period (n + 1) */
    uint8_t txsr;                           /**< Exit self-refresh to active command time (n + 1) */
    uint8_t trrd;                           /**< Active bank A to active bank B latency (n + 1) */
    uint8_t tmrd;                           /**< Load mode register to active command time (n + 1) */
} EMC_ConfigDynamicGlobals;


typedef enum EMC_DynamicDevice {
    EMC_DYCS0 = 0,                          /**< Dynamic chip select 0 (DYCS0) */
    EMC_DYCS1 = 1,                          /**< Dynamic chip select 1 (DYCS1) */
    EMC_DYCS2 = 2,                          /**< Dynamic chip select 2 (DYCS2) */
    EMC_DYCS3 = 3,                          /**< Dynamic chip select 3 (DYCS3) */
} EMC_DynamicDevice;

typedef enum EMC_RasDelay {
    EMC_RAS_DELAY_1 = 1,                    /**< RAS delay = 1 clock cycle */
    EMC_RAS_DELAY_2 = 2,                    /**< RAS delay = 2 clock cycles */
    EMC_RAS_DELAY_3 = 3,                    /**< RAS delay = 3 clock cycles */
} EMC_RasDelay;

typedef enum EMC_CasDelay {
    EMC_CAS_DELAY_1 = 1,                    /**< CAS delay = 1 clock cycle */
    EMC_CAS_DELAY_2 = 2,                    /**< CAS delay = 2 clock cycles */
    EMC_CAS_DELAY_3 = 3,                    /**< CAS delay = 3 clock cycles */
} EMC_CasDelay;

typedef enum EMC_DynamicBusWidth {
    EMC_DYNAMIC_BUS_16 = 0,                 /**< 16 data bits */
    EMC_DYNAMIC_BUS_32 = 1,                 /**< 32 data bits */
} EMC_DynamicBusWidth;

typedef enum EMC_DynamicOrganization {
    EMC_DYNAMIC_ORG_1Mx8BITx2_11R_9C    = ( 9 << 8) | 0,  /**< 16 MBit (2Mx8), B=2, R=11, C=9 */
    EMC_DYNAMIC_ORG_512Kx16BITx2_11R_8C = ( 8 << 8) | 1,  /**< 16 MBit (1Mx16), B=2, R=11, C=8 */
    EMC_DYNAMIC_ORG_2Mx8BITx4_12R_9C    = ( 9 << 8) | 4,  /**< 64 MBit (8Mx8), B=4, R=12, C=9 */
    EMC_DYNAMIC_ORG_1Mx16BITx4_12R_8C   = ( 8 << 8) | 5,  /**< 64 MBit (4Mx16), B=4, R=12, C=8 */
    EMC_DYNAMIC_ORG_512Kx32BITx4_11R_8C = ( 8 << 8) | 6,  /**< 64 MBit (2Mx32), B=4, R=11, C=8 */
    EMC_DYNAMIC_ORG_4Mx8BITx4_12R_10C   = (10 << 8) | 8,  /**< 128 MBit (16Mx8), B=4, R=12, C=10 */
    EMC_DYNAMIC_ORG_2Mx16BITx4_12R_9C   = ( 9 << 8) | 9,  /**< 128 MBit (8Mx16), B=4, R=12, C=9 */
    EMC_DYNAMIC_ORG_1Mx32BITx4_12R_8C   = ( 8 << 8) | 10, /**< 128 MBit (4Mx32), B=4, R=12, C=8 */
    EMC_DYNAMIC_ORG_8Mx8BITx4_13R_10C   = (10 << 8) | 12, /**< 256 MBit (32Mx8), B=4, R=13, C=10 */
    EMC_DYNAMIC_ORG_4Mx16BITx4_13R_9C   = ( 9 << 8) | 13, /**< 256 MBit (16Mx16), B=4, R=13, C=9 */
    EMC_DYNAMIC_ORG_2Mx32BITx4_13R_8C   = ( 8 << 8) | 14, /**< 256 MBit (8Mx32), B=4, R=13, C=8 */
    EMC_DYNAMIC_ORG_16Mx8BITx4_13R_11C  = (11 << 8) | 16, /**< 512 MBit (64Mx8), B=4, R=13, C=11 */
    EMC_DYNAMIC_ORG_8Mx16BITx4_13R_10C  = (10 << 8) | 17, /**< 512 MBit (32Mx16), B=4, R=13, C=10 */
} EMC_DynamicOrganization;


/** Config options for a dynamic memory device. */
typedef struct EMC_ConfigDynamicDevice {
    EMC_DynamicDevice device;               /**< Selects a device (DYCS0, ...) */
    LPCLIB_Switch lowPower;                 /**< DISABLE: normal SDRAM; ENABLE: Low-power SDRAM */
    EMC_RasDelay ras;                       /**< RAS delay cycles */
    EMC_CasDelay cas;                       /**< CAS delay cycles */
    EMC_DynamicBusWidth busWidth;           /**< Bus width (16/32) */
    EMC_DynamicOrganization org;            /**< SDRAM organization (size, rows, columns) */
} EMC_ConfigDynamicDevice;


typedef enum EMC_StaticDevice {
    EMC_CS0 = 0,                            /**< Static chip select 0 (CS0) */
    EMC_CS1 = 1,                            /**< Static chip select 1 (CS1) */
    EMC_CS2 = 2,                            /**< Static chip select 2 (CS2) */
    EMC_CS3 = 3,                            /**< Static chip select 3 (CS3) */
} EMC_StaticDevice;

typedef enum EMC_StaticBusWidth {
    EMC_STATIC_BUS_8 = 0,                   /**< 8 data bits */
    EMC_STATIC_BUS_16 = 1,                  /**< 16 data bits */
    EMC_STATIC_BUS_32 = 2,                  /**< 32 data bits */
} EMC_StaticBusWidth;

typedef enum EMC_StaticMultipleDevices {
    EMC_STATIC_SINGLE_DEVICE = 0x80,        /**< A single memory device (8/16/32 bits wide) */
    EMC_STATIC_MULTI_DEVICE = 0,            /**< Two or four devices (2x8/2x16/4x8) */
} EMC_StaticMultipleDevices;

/** Config options for a static memory device. */
struct EMC_ConfigStaticDevice {
    EMC_StaticDevice device;                /**< Selects a device (CS0, ...) */
    EMC_StaticBusWidth busWidth;            /**< Bus width (8/16/32) */
    EMC_StaticMultipleDevices multi;        /**< Single device or multiple devices */
    uint8_t writePulseWidth;                /**< Width of write pulse (WE/BLSx) in CCLK cycles */
    uint8_t writePulseDelay;                /**< Delay address/data -> WE start in CCLK cycles */
    uint8_t readCycleWidth;                 /**< Width of read cycle (address/CS -> data)
                                             *   in CCLK cycles
                                             */
    uint8_t readCycleWidthPage;             /**< Width of read cycle in page mode
                                             *   (address/CS -> data) in CCLK cycles
                                             */
    uint8_t readOeDelay;                    /**< Delay CS -> OE in CCLK cycles */
};


/** Descriptor to specify the configuration in a call to \ref EMC_ioctl. */
typedef struct EMC_Config {
    EMC_Opcode opcode;                      /**< Config action opcode */

    union {
        struct EMC_ConfigDynamicGlobals dynTimings; /**< Config global dynamic memory timings */
        struct EMC_ConfigDynamicDevice dynDevice;   /**< Config an SDRAM device */
        struct EMC_ConfigStaticDevice staticDevice; /**< Config a static memory device */
    };
} EMC_Config;

/** Config list terminator. */
#define EMC_CONFIG_END \
    {.opcode = EMC_OPCODE_INVALID}


/** @} EMC Types, enums, macros */



/** \defgroup EMC_Public_Functions EMC API Functions
 *  @{
 */


/** Open the EMC interface for use.
 *
 *  Enable the peripheral clock to the EMC block, then enable
 *  the block.
 *
 *  \param[out] pHandle Handle to be used in future API calls to the EMC module.
 *  \retval LPCLIB_SUCCESS Success. \ref pHandle contains a valid handle.
 *  \retval LPCLIB_BUSY Handle already in use
 */
LPCLIB_Result EMC_open (EMC_Handle *pHandle);


#if LPCLIB_EMC_CLOSE
/** Close the EMC interface.
 *
 *  Power down the EMC block.
 *
 *  \param[in] pHandle EMC handle as obtained by \ref EMC_open.
 *  \retval LPCLIB_SUCCESS
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle
 */
LPCLIB_Result EMC_close (EMC_Handle *pHandle);
#endif


/** Configure the EMC block.
 *
 *  Pass a configuration command to the EMC block. Configuration options
 *  include setting the memory timings and bus width.
 *
 *  \param[in] handle EMC handle as obtained by \ref EMC_open.
 *  \param[in] pConfig Pointer to a configuration descriptor of type
 *             \ref EMC_Config.
 */
void EMC_ioctl (EMC_Handle handle, const EMC_Config *pConfig);

/** @} EMC API Functions */


#endif /* #if LPCLIB_EMC */

/** @} EMC */

#endif /* #ifndef __LPC17XX_EMC_H__ */

