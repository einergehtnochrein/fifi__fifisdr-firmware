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
 *  \brief LCD driver interface.
 *  This file defines all interface objects needed to use the LCD driver.
 *
 *  \author NXP Semiconductors
 */

#ifndef __LPC17XX_LCD_H__
#define __LPC17XX_LCD_H__

/** \defgroup LCD
 *  \ingroup API
 *  @{
 */


#include "lpc17xx_libconfig.h"
#include "lpclib_types.h"


/** \defgroup LCD_Public_Types LCD Types, enums, macros
 *  @{
 */


typedef enum LCD_Name {
    LCD0 = 0,                           /**< The one and only LCD */
} LCD_Name;


/** Handle for an LCD block */
typedef struct LCD_Context *LCD_Handle;


/** Opcodes to specify the configuration command in a call to \ref LCD_ioctl. */
typedef enum LCD_Opcode {
    LCD_OPCODE_INVALID = 0,                 /**< (List terminator) */
    LCD_OPCODE_SET_MODE,                    /**< Config action: Set mode */
    LCD_OPCODE_SET_TIMINGS,                 /**< Set timings and mode */
    LCD_OPCODE_SET_POWER,                   /**< Set power state */
    LCD_OPCODE_SET_CALLBACK,                /**< Set callback handler */
} LCD_Opcode;


/** LCD signal polarity. */
typedef enum LCD_Polarity {
    LCD_POLARITY_NON_INVERTED = 0,          /**< high pulse */
    LCD_POLARITY_INVERTED = 1,              /**< low pulse */
    LCD_POLARITY_ACTIVE_HIGH = 0,           /**< Alias of LCD_POLARITY_NON_INVERTED */
    LCD_POLARITY_ACTIVE_LOW = 1,            /**< Alias of LCD_POLARITY_INVERTED */
    LCD_POLARITY_RISING_EDGE = 0,           /**< Alias of LCD_POLARITY_NON_INVERTED */
    LCD_POLARITY_FALLING_EDGE = 1,          /**< Alias of LCD_POLARITY_INVERTED */
} LCD_Polarity;


/** LCD bits per pixel. */
typedef enum LCD_BitsPerPixel {
    LCD_BPP_1 = 0,                          /**< monochrome (2 colors) */
    LCD_BPP_2 = 1,                          /**< 2 bits/pixel (4 colors) */
    LCD_BPP_4 = 2,                          /**< 4 bits/pixel (16 colors) */
    LCD_BPP_8 = 3,                          /**< 8 bits/pixel (256 colors) */
    LCD_BPP_12 = 7,                         /**< 12 bits/pixel (4:4:4 mode) (4096 colors) */
    LCD_BPP_16 = 4,                         /**< 16 bits/pixel (65536 colors) */
    LCD_BPP_16_565 = 6,                     /**< 16 bits/pixel (5:6:5 mode) (65536 colors) */
    LCD_BPP_24 = 5,                         /**< 24 bits/pixel (16M colors) */
} LCD_BitsPerPixel;


/** Timing configuration. */
typedef struct LCD_ConfigTimings {
    uint32_t pixelFrequency;                /**< Pixel clock [Hz] */
    uint32_t externalClockFrequency;        /**< Frequency [Hz] of internal clock (or 0 if internal) */
    uint16_t sizeX;                         /**< Number of visible pixels per line */
    uint16_t sizeY;                         /**< Number of visible lines */
    uint8_t frontPorchH;                    /**< Horizontal front porch [clocks] */
    uint8_t syncWidthH;                     /**< Horizontal sync pulse width [clocks] */
    uint8_t backPorchH;                     /**< Horizontal back porch [clocks] */
    uint8_t frontPorchV;                    /**< Vertical front porch [lines] */
    uint8_t syncWidthV;                     /**< Vertical sync pulse width [lines] */
    uint8_t backPorchV;                     /**< Vertical back porch [lines] */
    LCD_Polarity polarityHSYNC;             /**< HSYNC polarity */
    LCD_Polarity polarityVSYNC;             /**< VSYNC polarity */
    LCD_Polarity polarityClock;             /**< Pixel clock polarity */
    LCD_Polarity polarityENAB;              /**< DENAB polarity */
} LCD_ConfigTimings;


/** Color mode. */
typedef enum LCD_ColorFormat {
    LCD_COLORFORMAT_RGB = 0,                /**< Normal RGB output */
    LCD_COLORFORMAT_BGR = 1,                /**< Red and blue swapped */
} LCD_ColorFormat;

/** Panel type (STN or TFT). */
typedef enum LCD_PanelType {
    LCD_PANELTYPE_STN_COLOR = 0,            /**< STN, color */
    LCD_PANELTYPE_STN_MONOCHROME = 1,       /**< STN, mono */
    LCD_PANELTYPE_TFT = 2,                  /**< TFT */
} LCD_PanelType;

/** Mode configuration. */
typedef struct LCD_ConfigMode {
    LCD_PanelType panelType;                /**< STN/TFT */
    LCD_ColorFormat colorFormat;            /**< RGB or BGR */
    LCD_BitsPerPixel bitsPerPixel;          /**< Color depth */
} LCD_ConfigMode;

/** Callback configuration. */
struct LCD_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};

    
/** Descriptor to specify the configuration in a call to \ref LCD_Ioctl. */
typedef struct LCD_Config {
    LCD_Opcode opcode;                      /**< Config action opcode */

    union {
        struct LCD_ConfigTimings timings;   /**< Timings, color mode, etc. */
        LPCLIB_Switch powered;              /**< Enable/Disable the LCD */
        struct LCD_ConfigMode mode;         /**< Color mode, etc. */
        struct LCD_ConfigCallback callback; /**< Callback handler */
    };
} LCD_Config;

/** Config list terminator. */
#define LCD_CONFIG_END \
    {.opcode = LCD_OPCODE_INVALID}


/** Event types generated by LCD driver. */
typedef enum LCD_CallbackEvent {
    LCD_EVENT_FRAME,                        /**< New frame started */
    LCD_EVENT_LINE_MATCH,                   /**< Vertical Compare interrupt */
    LCD_EVENT_CURSOR,                       /**< Cursor refreshed */
    LCD_EVENT_ERROR_UNDERRUN,               /**< FIFO underrun */
    LCD_EVENT_ERROR_BUS,                    /**< AHB bus error */
} LCD_CallbackEvent;


/** @} LCD Types, enums, macros */

/** \defgroup LCD_Public_Functions LCD API Functions
 *  @{
 */


/** Prepare the use of the LCD block.
 */
LPCLIB_Result LCD_open (LCD_Name name, LCD_Handle *pHandle);


/** Close the LCD interface.
 *
 *  \param[in] pHandle Device handle
 */
void LCD_close (LCD_Handle *pHandle);


/** Set LCD options.
 *
 *  \param[in] pConfig Config descriptor
 */
void LCD_ioctl (LCD_Handle handle, const LCD_Config *pConfig);


/** Get LCD dimension (visible area).
 *
 *  \param[in] handle Device handle
 *  \param[out] pSizeX Pointer to X size
 *  \param[out] pSizeY Pointer to Y size
 */
LPCLIB_Result LCD_getDimension (LCD_Handle handle, uint32_t *pSizeX, uint32_t *pSizeY);


/** @} LCD API Functions */

/** @} LCD */

#endif /* #ifndef __LPC17XX_LCD_H__ */

