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
 *  \brief IOCON driver interface.
 *
 *  This file contains the API for the IOCON block.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17XX_IOCON_H__
#define __LPC17XX_IOCON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** \defgroup IOCON
 *  \ingroup API
 *  @{
 */

#include "lpc17xx_libconfig.h"

#include "lpc17xx_clkpwr.h"


/** \defgroup IOCON_Public_Types IOCON Types, enums, macros
 *  @{
 */


enum IOCON_PinType {
    IOCON_PINTYPE_D = (1u << 28),           /**< Normal digital pin */
    IOCON_PINTYPE_A = (2u << 28),
    IOCON_PINTYPE_U = (3u << 28),
    IOCON_PINTYPE_I = (4u << 28),
    IOCON_PINTYPE_W = (5u << 28),
};


typedef enum IOCON_PinName {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    PIN_P0_0    =   0 | IOCON_PINTYPE_D,
    PIN_P0_1    =   1 | IOCON_PINTYPE_D,
    PIN_P0_2    =   2 | IOCON_PINTYPE_D,
    PIN_P0_3    =   3 | IOCON_PINTYPE_D,
    PIN_P0_4    =   4 | IOCON_PINTYPE_D,
    PIN_P0_5    =   5 | IOCON_PINTYPE_D,
    PIN_P0_6    =   6 | IOCON_PINTYPE_D,
    PIN_P0_7    =   7 | IOCON_PINTYPE_D,
    PIN_P0_8    =   8 | IOCON_PINTYPE_D,
    PIN_P0_9    =   9 | IOCON_PINTYPE_D,
    PIN_P0_10   =  10 | IOCON_PINTYPE_D,
    PIN_P0_11   =  11 | IOCON_PINTYPE_D,
    PIN_P0_15   =  15 | IOCON_PINTYPE_D,
    PIN_P0_16   =  16 | IOCON_PINTYPE_D,
    PIN_P0_17   =  17 | IOCON_PINTYPE_D,
    PIN_P0_18   =  18 | IOCON_PINTYPE_D,
    PIN_P0_19   =  19 | IOCON_PINTYPE_D,
    PIN_P0_20   =  20 | IOCON_PINTYPE_D,
    PIN_P0_21   =  21 | IOCON_PINTYPE_D,
    PIN_P0_22   =  22 | IOCON_PINTYPE_D,
    PIN_P0_23   =  23 | IOCON_PINTYPE_D,
    PIN_P0_24   =  24 | IOCON_PINTYPE_D,
    PIN_P0_25   =  25 | IOCON_PINTYPE_D,
    PIN_P0_26   =  26 | IOCON_PINTYPE_D,
    PIN_P0_27   =  27 | IOCON_PINTYPE_I,
    PIN_P0_28   =  28 | IOCON_PINTYPE_I,
    PIN_P0_29   =  29 | IOCON_PINTYPE_D,
    PIN_P0_30   =  30 | IOCON_PINTYPE_D,
    PIN_P1_0    =  32 | IOCON_PINTYPE_D,
    PIN_P1_1    =  33 | IOCON_PINTYPE_D,
    PIN_P1_4    =  36 | IOCON_PINTYPE_D,
    PIN_P1_8    =  40 | IOCON_PINTYPE_D,
    PIN_P1_9    =  41 | IOCON_PINTYPE_D,
    PIN_P1_10   =  42 | IOCON_PINTYPE_D,
    PIN_P1_14   =  46 | IOCON_PINTYPE_D,
    PIN_P1_15   =  47 | IOCON_PINTYPE_D,
    PIN_P1_16   =  48 | IOCON_PINTYPE_D,
    PIN_P1_17   =  49 | IOCON_PINTYPE_D,
    PIN_P1_18   =  50 | IOCON_PINTYPE_D,
    PIN_P1_19   =  51 | IOCON_PINTYPE_D,
    PIN_P1_20   =  52 | IOCON_PINTYPE_D,
    PIN_P1_21   =  53 | IOCON_PINTYPE_D,
    PIN_P1_22   =  54 | IOCON_PINTYPE_D,
    PIN_P1_23   =  55 | IOCON_PINTYPE_D,
    PIN_P1_24   =  56 | IOCON_PINTYPE_D,
    PIN_P1_25   =  57 | IOCON_PINTYPE_D,
    PIN_P1_26   =  58 | IOCON_PINTYPE_D,
    PIN_P1_27   =  59 | IOCON_PINTYPE_D,
    PIN_P1_28   =  60 | IOCON_PINTYPE_D,
    PIN_P1_29   =  61 | IOCON_PINTYPE_D,
    PIN_P1_30   =  62 | IOCON_PINTYPE_D,
    PIN_P1_31   =  63 | IOCON_PINTYPE_D,
    PIN_P2_0    =  64 | IOCON_PINTYPE_D,
    PIN_P2_1    =  65 | IOCON_PINTYPE_D,
    PIN_P2_2    =  66 | IOCON_PINTYPE_D,
    PIN_P2_3    =  67 | IOCON_PINTYPE_D,
    PIN_P2_4    =  68 | IOCON_PINTYPE_D,
    PIN_P2_5    =  69 | IOCON_PINTYPE_D,
    PIN_P2_6    =  70 | IOCON_PINTYPE_D,
    PIN_P2_7    =  71 | IOCON_PINTYPE_D,
    PIN_P2_8    =  72 | IOCON_PINTYPE_D,
    PIN_P2_9    =  73 | IOCON_PINTYPE_D,
    PIN_P2_10   =  74 | IOCON_PINTYPE_D,
    PIN_P2_11   =  75 | IOCON_PINTYPE_D,
    PIN_P2_12   =  76 | IOCON_PINTYPE_D,
    PIN_P2_13   =  77 | IOCON_PINTYPE_D,
    PIN_P3_25   = 121 | IOCON_PINTYPE_D,
    PIN_P3_26   = 122 | IOCON_PINTYPE_D,
    PIN_P4_28   = 156 | IOCON_PINTYPE_D,
    PIN_P4_29   = 157 | IOCON_PINTYPE_D,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    PIN_P0_0    =   0 | IOCON_PINTYPE_D,
    PIN_P0_1    =   1 | IOCON_PINTYPE_D,
    PIN_P0_2    =   2 | IOCON_PINTYPE_D,
    PIN_P0_3    =   3 | IOCON_PINTYPE_D,
    PIN_P0_4    =   4 | IOCON_PINTYPE_D,
    PIN_P0_5    =   5 | IOCON_PINTYPE_D,
    PIN_P0_6    =   6 | IOCON_PINTYPE_D,
    PIN_P0_7    =   7 | IOCON_PINTYPE_W,
    PIN_P0_8    =   8 | IOCON_PINTYPE_W,
    PIN_P0_9    =   9 | IOCON_PINTYPE_W,
    PIN_P0_10   =  10 | IOCON_PINTYPE_D,
    PIN_P0_11   =  11 | IOCON_PINTYPE_D,
    PIN_P0_12   =  12 | IOCON_PINTYPE_A,
    PIN_P0_13   =  13 | IOCON_PINTYPE_A,
    PIN_P0_14   =  14 | IOCON_PINTYPE_D,
    PIN_P0_15   =  15 | IOCON_PINTYPE_D,
    PIN_P0_16   =  16 | IOCON_PINTYPE_D,
    PIN_P0_17   =  17 | IOCON_PINTYPE_D,
    PIN_P0_18   =  18 | IOCON_PINTYPE_D,
    PIN_P0_19   =  19 | IOCON_PINTYPE_D,
    PIN_P0_20   =  20 | IOCON_PINTYPE_D,
    PIN_P0_21   =  21 | IOCON_PINTYPE_D,
    PIN_P0_22   =  22 | IOCON_PINTYPE_D,
    PIN_P0_23   =  23 | IOCON_PINTYPE_A,
    PIN_P0_24   =  24 | IOCON_PINTYPE_A,
    PIN_P0_25   =  25 | IOCON_PINTYPE_A,
    PIN_P0_26   =  26 | IOCON_PINTYPE_A,
    PIN_P0_27   =  27 | IOCON_PINTYPE_I,
    PIN_P0_28   =  28 | IOCON_PINTYPE_I,
    PIN_P0_29   =  29 | IOCON_PINTYPE_U,
    PIN_P0_30   =  30 | IOCON_PINTYPE_U,
    PIN_P0_31   =  31 | IOCON_PINTYPE_U,
    PIN_P1_0    =  32 | IOCON_PINTYPE_D,
    PIN_P1_1    =  33 | IOCON_PINTYPE_D,
    PIN_P1_2    =  34 | IOCON_PINTYPE_D,
    PIN_P1_3    =  35 | IOCON_PINTYPE_D,
    PIN_P1_4    =  36 | IOCON_PINTYPE_D,
    PIN_P1_5    =  37 | IOCON_PINTYPE_D,
    PIN_P1_6    =  38 | IOCON_PINTYPE_D,
    PIN_P1_7    =  39 | IOCON_PINTYPE_D,
    PIN_P1_8    =  40 | IOCON_PINTYPE_D,
    PIN_P1_9    =  41 | IOCON_PINTYPE_D,
    PIN_P1_10   =  42 | IOCON_PINTYPE_D,
    PIN_P1_11   =  43 | IOCON_PINTYPE_D,
    PIN_P1_12   =  44 | IOCON_PINTYPE_D,
    PIN_P1_13   =  45 | IOCON_PINTYPE_D,
    PIN_P1_14   =  46 | IOCON_PINTYPE_D,
    PIN_P1_15   =  47 | IOCON_PINTYPE_D,
    PIN_P1_16   =  48 | IOCON_PINTYPE_D,
    PIN_P1_17   =  49 | IOCON_PINTYPE_D,
    PIN_P1_18   =  50 | IOCON_PINTYPE_D,
    PIN_P1_19   =  51 | IOCON_PINTYPE_D,
    PIN_P1_20   =  52 | IOCON_PINTYPE_D,
    PIN_P1_21   =  53 | IOCON_PINTYPE_D,
    PIN_P1_22   =  54 | IOCON_PINTYPE_D,
    PIN_P1_23   =  55 | IOCON_PINTYPE_D,
    PIN_P1_24   =  56 | IOCON_PINTYPE_D,
    PIN_P1_25   =  57 | IOCON_PINTYPE_D,
    PIN_P1_26   =  58 | IOCON_PINTYPE_D,
    PIN_P1_27   =  59 | IOCON_PINTYPE_D,
    PIN_P1_28   =  60 | IOCON_PINTYPE_D,
    PIN_P1_29   =  61 | IOCON_PINTYPE_D,
    PIN_P1_30   =  62 | IOCON_PINTYPE_A,
    PIN_P1_31   =  63 | IOCON_PINTYPE_A,
    PIN_P2_0    =  64 | IOCON_PINTYPE_D,
    PIN_P2_1    =  65 | IOCON_PINTYPE_D,
    PIN_P2_2    =  66 | IOCON_PINTYPE_D,
    PIN_P2_3    =  67 | IOCON_PINTYPE_D,
    PIN_P2_4    =  68 | IOCON_PINTYPE_D,
    PIN_P2_5    =  69 | IOCON_PINTYPE_D,
    PIN_P2_6    =  70 | IOCON_PINTYPE_D,
    PIN_P2_7    =  71 | IOCON_PINTYPE_D,
    PIN_P2_8    =  72 | IOCON_PINTYPE_D,
    PIN_P2_9    =  73 | IOCON_PINTYPE_D,
    PIN_P2_10   =  74 | IOCON_PINTYPE_D,
    PIN_P2_11   =  75 | IOCON_PINTYPE_D,
    PIN_P2_12   =  76 | IOCON_PINTYPE_D,
    PIN_P2_13   =  77 | IOCON_PINTYPE_D,
    PIN_P2_14   =  78 | IOCON_PINTYPE_D,
    PIN_P2_15   =  79 | IOCON_PINTYPE_D,
    PIN_P2_16   =  80 | IOCON_PINTYPE_D,
    PIN_P2_17   =  81 | IOCON_PINTYPE_D,
    PIN_P2_18   =  82 | IOCON_PINTYPE_D,
    PIN_P2_19   =  83 | IOCON_PINTYPE_D,
    PIN_P2_20   =  84 | IOCON_PINTYPE_D,
    PIN_P2_21   =  85 | IOCON_PINTYPE_D,
    PIN_P2_22   =  86 | IOCON_PINTYPE_D,
    PIN_P2_23   =  87 | IOCON_PINTYPE_D,
    PIN_P2_24   =  88 | IOCON_PINTYPE_D,
    PIN_P2_25   =  89 | IOCON_PINTYPE_D,
    PIN_P2_26   =  90 | IOCON_PINTYPE_D,
    PIN_P2_27   =  91 | IOCON_PINTYPE_D,
    PIN_P2_28   =  92 | IOCON_PINTYPE_D,
    PIN_P2_29   =  93 | IOCON_PINTYPE_D,
    PIN_P2_30   =  94 | IOCON_PINTYPE_D,
    PIN_P2_31   =  95 | IOCON_PINTYPE_D,
    PIN_P3_0    =  96 | IOCON_PINTYPE_D,
    PIN_P3_1    =  97 | IOCON_PINTYPE_D,
    PIN_P3_2    =  98 | IOCON_PINTYPE_D,
    PIN_P3_3    =  99 | IOCON_PINTYPE_D,
    PIN_P3_4    = 100 | IOCON_PINTYPE_D,
    PIN_P3_5    = 101 | IOCON_PINTYPE_D,
    PIN_P3_6    = 102 | IOCON_PINTYPE_D,
    PIN_P3_7    = 103 | IOCON_PINTYPE_D,
    PIN_P3_8    = 104 | IOCON_PINTYPE_D,
    PIN_P3_9    = 105 | IOCON_PINTYPE_D,
    PIN_P3_10   = 106 | IOCON_PINTYPE_D,
    PIN_P3_11   = 107 | IOCON_PINTYPE_D,
    PIN_P3_12   = 108 | IOCON_PINTYPE_D,
    PIN_P3_13   = 109 | IOCON_PINTYPE_D,
    PIN_P3_14   = 110 | IOCON_PINTYPE_D,
    PIN_P3_15   = 111 | IOCON_PINTYPE_D,
    PIN_P3_16   = 112 | IOCON_PINTYPE_D,
    PIN_P3_17   = 113 | IOCON_PINTYPE_D,
    PIN_P3_18   = 114 | IOCON_PINTYPE_D,
    PIN_P3_19   = 115 | IOCON_PINTYPE_D,
    PIN_P3_20   = 116 | IOCON_PINTYPE_D,
    PIN_P3_21   = 117 | IOCON_PINTYPE_D,
    PIN_P3_22   = 118 | IOCON_PINTYPE_D,
    PIN_P3_23   = 119 | IOCON_PINTYPE_D,
    PIN_P3_24   = 120 | IOCON_PINTYPE_D,
    PIN_P3_25   = 121 | IOCON_PINTYPE_D,
    PIN_P3_26   = 122 | IOCON_PINTYPE_D,
    PIN_P3_27   = 123 | IOCON_PINTYPE_D,
    PIN_P3_28   = 124 | IOCON_PINTYPE_D,
    PIN_P3_29   = 125 | IOCON_PINTYPE_D,
    PIN_P3_30   = 126 | IOCON_PINTYPE_D,
    PIN_P3_31   = 127 | IOCON_PINTYPE_D,
    PIN_P4_0    = 128 | IOCON_PINTYPE_D,
    PIN_P4_1    = 129 | IOCON_PINTYPE_D,
    PIN_P4_2    = 130 | IOCON_PINTYPE_D,
    PIN_P4_3    = 131 | IOCON_PINTYPE_D,
    PIN_P4_4    = 132 | IOCON_PINTYPE_D,
    PIN_P4_5    = 133 | IOCON_PINTYPE_D,
    PIN_P4_6    = 134 | IOCON_PINTYPE_D,
    PIN_P4_7    = 135 | IOCON_PINTYPE_D,
    PIN_P4_8    = 136 | IOCON_PINTYPE_D,
    PIN_P4_9    = 137 | IOCON_PINTYPE_D,
    PIN_P4_10   = 138 | IOCON_PINTYPE_D,
    PIN_P4_11   = 139 | IOCON_PINTYPE_D,
    PIN_P4_12   = 140 | IOCON_PINTYPE_D,
    PIN_P4_13   = 141 | IOCON_PINTYPE_D,
    PIN_P4_14   = 142 | IOCON_PINTYPE_D,
    PIN_P4_15   = 143 | IOCON_PINTYPE_D,
    PIN_P4_16   = 144 | IOCON_PINTYPE_D,
    PIN_P4_17   = 145 | IOCON_PINTYPE_D,
    PIN_P4_18   = 146 | IOCON_PINTYPE_D,
    PIN_P4_19   = 147 | IOCON_PINTYPE_D,
    PIN_P4_20   = 148 | IOCON_PINTYPE_D,
    PIN_P4_21   = 149 | IOCON_PINTYPE_D,
    PIN_P4_22   = 150 | IOCON_PINTYPE_D,
    PIN_P4_23   = 151 | IOCON_PINTYPE_D,
    PIN_P4_24   = 152 | IOCON_PINTYPE_D,
    PIN_P4_25   = 153 | IOCON_PINTYPE_D,
    PIN_P4_26   = 154 | IOCON_PINTYPE_D,
    PIN_P4_27   = 155 | IOCON_PINTYPE_D,
    PIN_P4_28   = 156 | IOCON_PINTYPE_D,
    PIN_P4_29   = 157 | IOCON_PINTYPE_D,
    PIN_P4_30   = 158 | IOCON_PINTYPE_D,
    PIN_P4_31   = 159 | IOCON_PINTYPE_D,
    PIN_P5_0    = 160 | IOCON_PINTYPE_D,
    PIN_P5_1    = 161 | IOCON_PINTYPE_D,
    PIN_P5_2    = 162 | IOCON_PINTYPE_I,
    PIN_P5_3    = 163 | IOCON_PINTYPE_I,
    PIN_P5_4    = 164 | IOCON_PINTYPE_D,
#endif
} IOCON_PinName;


typedef enum IOCON_Function {
    PIN_FUNCTION_0 = 0,         /**< Function selector 0 */
    PIN_FUNCTION_1 = 1,         /**< Function selector 1 */
    PIN_FUNCTION_2 = 2,         /**< Function selector 2 */
    PIN_FUNCTION_3 = 3,         /**< Function selector 3 */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    PIN_FUNCTION_4 = 4,         /**< Function selector 4 */
    PIN_FUNCTION_5 = 5,         /**< Function selector 5 */
    PIN_FUNCTION_6 = 6,         /**< Function selector 6 */
    PIN_FUNCTION_7 = 7,         /**< Function selector 7 */
#endif
} IOCON_Function;


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
typedef enum IOCON_Mode {
    PIN_PULL_NONE = 2,          /**< No pull-up or pull-down resistor */
    PIN_PULL_UP   = 0,          /**< Internal pull-up resistor */
    PIN_PULL_DOWN = 3,          /**< Internal pull-down resistor */
    PIN_PULL_REPEATER = 1,      /**< Keep last actively driven input level */
} IOCON_Mode;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
typedef enum IOCON_Mode {
    PIN_PULL_NONE = 0,          /**< No pull-up or pull-down resistor */
    PIN_PULL_UP   = 2,          /**< Internal pull-up resistor */
    PIN_PULL_DOWN = 1,          /**< Internal pull-down resistor */
    PIN_PULL_REPEATER = 3,      /**< Keep last actively driven input level */
} IOCON_Mode;
#endif


typedef enum IOCON_OpenDrain {
    PIN_OPENDRAIN_OFF = 0,      /**< Open-drain disabled */
    PIN_OPENDRAIN_ON  = 1,      /**< Open-drain enabled */
} IOCON_OpenDrain;


typedef enum IOCON_SlewRate {
    PIN_SLEWRATE_NORMAL = 0,    /**< Standard mode */
    PIN_SLEWRATE_FAST = 1,      /**< Fast mode */
} IOCON_SlewRate;


typedef enum IOCON_Polarity {
    PIN_INPUT_NOT_INVERTED = 0, /**< Input not inverted */
    PIN_INPUT_INVERTED = 1,     /**< Input inverted */
} IOCON_Polarity;


typedef enum IOCON_Hysteresis {
    PIN_HYSTERESIS_OFF = 0,     /**< Hysteresis disabled */
    PIN_HYSTERESIS_ON  = 1,     /**< Hysteresis enabled */
} IOCON_Hysteresis;

typedef enum IOCON_AdMode {
    PIN_ADMODE_ANALOG = 0,      /**< Analog mode */
    PIN_ADMODE_DIGITAL = 1,     /**< Digital mode */
} IOCON_AdMode;

typedef enum IOCON_GlitchFilter {
    PIN_FILTER_OFF = 1,         /**< Glich filter disabled */
    PIN_FILTER_ON = 0,          /**< Glitch filter enabled */
} IOCON_GlitchFilter;

typedef enum IOCON_DacEn {
    PIN_DAC_OFF = 0,            /**< DAC disabled */
    PIN_DAC_ON = 1,             /**< DAC enabled (P0.26 only) */
} IOCON_DacEn;

typedef enum IOCON_I2cMode {
    PIN_I2CMODE_NORMAL = 0,     /**< Normal and fast mode */
    PIN_I2CMODE_FMPLUS = 1,     /**< Fast mode + */
} IOCON_I2cMode;

typedef enum IOCON_DriveStrength {
    PIN_HIDRIVE_4MA = 0,        /**< Pin can drive 4 mA */
    PIN_HIDRIVE_20MA = 1,       /**< Pin can drive 20 mA */
} IOCON_DriveStrength;



typedef uint32_t IOCON_PinConfig;


typedef struct IOCON_PinConfigList {
    IOCON_PinName pin;
    IOCON_PinConfig config;
} IOCON_PinConfigList;


/** @} IOCON Types enums, macros */


/** \defgroup IOCON_Public_Functions IOCON API Functions
 *  @{
 */


/** Initialize the IOCON module.
 */
static void IOCON_open (void);



/** Configure a pin.
 *
 *  \param[in] pin Selects the pin to be configured
 *  \param[in] config Provides the configuration (as obtained by PINSEL_MakeConfig())
 */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
static
#endif
void IOCON_configurePin (IOCON_PinName pin, IOCON_PinConfig config);



/** Return the selected function of a pin.
 *
 *  \param[in] pin Selects the pin to be configured
 */
static IOCON_Function IOCON_getPinFunction (IOCON_PinName pin);



/** Check if an error occured during any of the pin configurations done so far.
 *
 *  \retval LPCLIB_SUCCESS No problems occurred.
 *  \retval LPCLIB_ILLEGAL_PARAMETER At least one mismatch between pin and
 *          makeConfigX function found
 */
static LPCLIB_Result IOCON_checkErrors (void);


/** @} IOCON API Functions */


extern LPCLIB_Result __IOCON_configError;


__FORCEINLINE(void IOCON_open (void))
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_deassertPeripheralReset(CLKPWR_RESET_IOCON);
#endif

    __IOCON_configError = LPCLIB_SUCCESS;
}

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigD (
        IOCON_Function function,
        IOCON_Mode mode,
        IOCON_OpenDrain openDrain))
{
    return (IOCON_PinConfig)(
        (function << 0)   |
        (mode << 3)       |
        (openDrain << 10) |
        IOCON_PINTYPE_D
        );
}
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigD (
        IOCON_Function function,
        IOCON_Mode mode,
        IOCON_Hysteresis hysteresis,
        IOCON_Polarity polarity,
        IOCON_SlewRate slew,
        IOCON_OpenDrain openDrain))
{
    return (IOCON_PinConfig)(
        (function << 0)   |
        (mode << 3)       |
        (hysteresis << 5) |
        (polarity << 6)   |
        (slew << 9)       |
        (openDrain << 10) |
        IOCON_PINTYPE_D
        );
}
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigA (
        IOCON_Function function,
        IOCON_Mode mode,
        IOCON_Polarity polarity,
        IOCON_AdMode admode,
        IOCON_GlitchFilter filter,
        IOCON_OpenDrain openDrain,
        IOCON_DacEn dacen))
{
    return (IOCON_PinConfig)(
        (function << 0)   |
        (mode << 3)       |
        (polarity << 6)   |
        (admode << 7)     |
        (filter << 8)     |
        (openDrain << 10) |
        (dacen << 16)     |
        IOCON_PINTYPE_A
        );
}
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigU (
        IOCON_Function function))
{
    return (IOCON_PinConfig)(
        (function << 0)   |
        IOCON_PINTYPE_U
        );
}
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigI (
        IOCON_Function func,
        IOCON_I2cMode hs,
        IOCON_DriveStrength hidrive))
{
    return (IOCON_PinConfig)(
        (func << 0)       |
        (hs << 8)         |
        (hidrive << 9)    |
        IOCON_PINTYPE_I
        );
}
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigI (
        IOCON_Function func,
        IOCON_Polarity invert,
        IOCON_I2cMode hs,
        IOCON_DriveStrength hidrive))
{
    return (IOCON_PinConfig)(
        (func << 0)       |
        (invert << 6)     |
        (hs << 8)         |
        (hidrive << 9)    |
        IOCON_PINTYPE_I
        );
}
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigW (
        IOCON_Function func,
        IOCON_Mode mode,
        IOCON_Hysteresis hys,
        IOCON_Polarity inv,
        IOCON_GlitchFilter filter,
        IOCON_SlewRate slew,
        IOCON_OpenDrain od))
{
    return (IOCON_PinConfig)(
        (func << 0)       |
        (mode << 3)       |
        (hys << 5)        |
        (inv << 6)        |
        (1u << 7)         |
        (filter << 8)     |
        (slew << 9)       |
        (od << 10)        |
        IOCON_PINTYPE_W
        );
}
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(void IOCON_configurePin (IOCON_PinName pin, IOCON_PinConfig config))
{
    if ((pin >> 28) != (config >> 28)) {    /* Type encoded into the upper 4 bits */
        __IOCON_configError = LPCLIB_ILLEGAL_PARAMETER;
        return;
    }

    ((volatile uint32_t *)LPC_IOCON)[pin & 0x0FFFFFFF] = (uint32_t)config & 0x0FFFFFFF;
}
#endif


__FORCEINLINE(IOCON_Function IOCON_getPinFunction (IOCON_PinName pin))
{
    /* For all pin types, function is encoded in the LSB's. */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    return (IOCON_Function)(((volatile uint32_t *)LPC_PINCON)[pin & 0x0FFFFFFF] & 0x03);
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    return (IOCON_Function)(((volatile uint32_t *)LPC_IOCON)[pin & 0x0FFFFFFF] & 0x07);
#endif
}



__FORCEINLINE(LPCLIB_Result IOCON_checkErrors (void))
{
    return __IOCON_configError;
}


/** @} IOCON API Functions */

/** @} IOCON */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __LPC17xx_IOCON_H__ */

