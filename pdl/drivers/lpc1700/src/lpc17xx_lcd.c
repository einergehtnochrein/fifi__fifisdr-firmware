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

#include "lpc17xx_libconfig.h"

#if LPCLIB_LCD

/** \file
 *  \brief LCD driver implementation.
 *
 *  This file contains the driver code for the LCD peripheral.
 *
 *  \author NXP Semiconductors
 */




#include "lpc17xx_lcd.h"
#include "lpc17xx_clkpwr.h"


/** Field definition for hardware register LCD_CFG. */
LPCLIB_DefineRegBit(LCD_CFG_CLKDIV,             0,  5);

/** Field definition for hardware register LCD_TIMH. */
LPCLIB_DefineRegBit(LCD_TIMH_PPL,               2,  6);
LPCLIB_DefineRegBit(LCD_TIMH_HSW,               8,  8);
LPCLIB_DefineRegBit(LCD_TIMH_HFP,               16, 8);
LPCLIB_DefineRegBit(LCD_TIMH_HBP,               24, 8);

/** Field definition for hardware register LCD_TIMV. */
LPCLIB_DefineRegBit(LCD_TIMV_LPP,               0,  10);
LPCLIB_DefineRegBit(LCD_TIMV_VSW,               10, 6);
LPCLIB_DefineRegBit(LCD_TIMV_VFP,               16, 8);
LPCLIB_DefineRegBit(LCD_TIMV_VBP,               24, 8);

/** Field definition for hardware register LCD_POL. */
LPCLIB_DefineRegBit(LCD_POL_PCD_LO,             0,  5);
LPCLIB_DefineRegBit(LCD_POL_CLKSEL,             5,  1);
LPCLIB_DefineRegBit(LCD_POL_ACB,                6,  5);
LPCLIB_DefineRegBit(LCD_POL_IVS,                11, 1);
LPCLIB_DefineRegBit(LCD_POL_IHS,                12, 1);
LPCLIB_DefineRegBit(LCD_POL_IPC,                13, 1);
LPCLIB_DefineRegBit(LCD_POL_IOE,                14, 1);
LPCLIB_DefineRegBit(LCD_POL_CPL,                16, 10);
LPCLIB_DefineRegBit(LCD_POL_BCD,                26, 1);
LPCLIB_DefineRegBit(LCD_POL_PCD_HI,             27, 5);

/** (Selected) Field definitions for hardware register LCD_LE. */
LPCLIB_DefineRegBit(LCD_LE_LED,                 0,  7);
LPCLIB_DefineRegBit(LCD_LE_LEE,                 16, 1);

/** (Selected) Field definitions for hardware register LCD_CTRL. */
LPCLIB_DefineRegBit(LCD_CTRL_LCDEN,             0,  1);
LPCLIB_DefineRegBit(LCD_CTRL_LCDBPP,            1,  3);
LPCLIB_DefineRegBit(LCD_CTRL_LCDBW_LCDTFT,      4,  2);
LPCLIB_DefineRegBit(LCD_CTRL_LCDMONO8,          6,  1);
LPCLIB_DefineRegBit(LCD_CTRL_LCDDUAL,           7,  1);
LPCLIB_DefineRegBit(LCD_CTRL_BGR,               8,  1);
LPCLIB_DefineRegBit(LCD_CTRL_BEBO,              9,  1);
LPCLIB_DefineRegBit(LCD_CTRL_BEPO,              10, 1);
LPCLIB_DefineRegBit(LCD_CTRL_LCDPWR,            11, 1);
LPCLIB_DefineRegBit(LCD_CTRL_LCDVCOMP,          12, 2);
LPCLIB_DefineRegBit(LCD_CTRL_WATERMARK,         16, 1);

/** Field definition for hardware register LCD_INTMSK. */
LPCLIB_DefineRegBit(LCD_INTMSK_FUFIM,           1,  1);
LPCLIB_DefineRegBit(LCD_INTMSK_LNBUIM,          2,  1);
LPCLIB_DefineRegBit(LCD_INTMSK_VCOMPIM,         3,  1);
LPCLIB_DefineRegBit(LCD_INTMSK_BERIM,           4,  1);

/** Field definition for hardware register LCD_INTRAW. */
LPCLIB_DefineRegBit(LCD_INTRAW_FUFRIS,          1,  1);
LPCLIB_DefineRegBit(LCD_INTRAW_LNBURIS,         2,  1);
LPCLIB_DefineRegBit(LCD_INTRAW_VCOMPRIS,        3,  1);
LPCLIB_DefineRegBit(LCD_INTRAW_BERRIS,          4,  1);

/** Field definition for hardware register LCD_INTSTAT. */
LPCLIB_DefineRegBit(LCD_INTSTAT_FUFMIS,         1,  1);
LPCLIB_DefineRegBit(LCD_INTSTAT_LNBUMIS,        2,  1);
LPCLIB_DefineRegBit(LCD_INTSTAT_VCOMPMIS,       3,  1);
LPCLIB_DefineRegBit(LCD_INTSTAT_BERMIS,         4,  1);

/** Field definition for hardware register LCD_INTCLR. */
LPCLIB_DefineRegBit(LCD_INTCLR_FUFIC,           1,  1);
LPCLIB_DefineRegBit(LCD_INTCLR_LNBUIC,          2,  1);
LPCLIB_DefineRegBit(LCD_INTCLR_VCOMPIC,         3,  1);
LPCLIB_DefineRegBit(LCD_INTCLR_BERIC,           4,  1);

/** Field definition for hardware register LCD_PAL. */
LPCLIB_DefineRegBit(LCD_PAL_R0,                 0,  5);
LPCLIB_DefineRegBit(LCD_PAL_G0,                 5,  5);
LPCLIB_DefineRegBit(LCD_PAL_B0,                 10, 5);
LPCLIB_DefineRegBit(LCD_PAL_I0,                 15, 1);
LPCLIB_DefineRegBit(LCD_PAL_R1,                 16, 5);
LPCLIB_DefineRegBit(LCD_PAL_G1,                 21, 5);
LPCLIB_DefineRegBit(LCD_PAL_B1,                 26, 5);
LPCLIB_DefineRegBit(LCD_PAL_I1,                 31, 1);

/** Field definition for hardware register CRSR_CTRL. */
LPCLIB_DefineRegBit(LCD_CRSR_CTRL_CRSRON,       0,  1);
LPCLIB_DefineRegBit(LCD_CRSR_CTRL_CRSRNUM,      4,  2);

/** Field definition for hardware register CRSR_CFG. */
LPCLIB_DefineRegBit(LCD_CRSR_CFG_CRSRSIZE,      0,  1);
LPCLIB_DefineRegBit(LCD_CRSR_CFG_FRAMESYNC,     1,  1);

/** Field definition for hardware register CRSR_PAL0. */
LPCLIB_DefineRegBit(LCD_CRSR_PAL0_RED,          0,  8);
LPCLIB_DefineRegBit(LCD_CRSR_PAL0_GREEN,        8,  8);
LPCLIB_DefineRegBit(LCD_CRSR_PAL0_BLUE,         16, 8);

/** Field definition for hardware register CRSR_PAL1. */
LPCLIB_DefineRegBit(LCD_CRSR_PAL1_RED,          0,  8);
LPCLIB_DefineRegBit(LCD_CRSR_PAL1_GREEN,        8,  8);
LPCLIB_DefineRegBit(LCD_CRSR_PAL1_BLUE,         16, 8);

/** Field definition for hardware register CRSR_XY. */
LPCLIB_DefineRegBit(LCD_CRSR_XY_CRSRX,          0,  10);
LPCLIB_DefineRegBit(LCD_CRSR_XY_CRSRY,          16, 10);

/** Field definition for hardware register CRSR_CLIP. */
LPCLIB_DefineRegBit(LCD_CRSR_CLIP_CRSRCLIPX,    0,  6);
LPCLIB_DefineRegBit(LCD_CRSR_CLIP_CRSRCLIPY,    8,  6);

/** Field definition for hardware register CRSR_INTMSK. */
LPCLIB_DefineRegBit(LCD_CRSR_INTMSK_CRSRIM,     0,  1);

/** Field definition for hardware register CRSR_INTCLR. */
LPCLIB_DefineRegBit(LCD_CRSR_INTCLR_CRSRIC,     0,  1);

/** Field definition for hardware register CRSR_INTRAW. */
LPCLIB_DefineRegBit(LCD_CRSR_INTRAW_CRSRRIS,    0,  1);

/** Field definition for hardware register CRSR_INTSTAT. */
LPCLIB_DefineRegBit(LCD_CRSR_INTSTAT_CRSRMIS,   0,  1);


/** Local context of LCD. */
static struct LCD_Context {
    LCD_Name name;                          /**< LCD identifier */
    LPCLIB_Switch inUse;                    /**< Set if interface open */
    LPCLIB_Callback callback;
} lcdContext;



/* Open the LCD interface. */
LPCLIB_Result LCD_open (LCD_Name name, LCD_Handle *pHandle)
{
    (void) name;
    LCD_Handle handle = &lcdContext;
    int i;

    CLKPWR_deassertPeripheralReset(CLKPWR_RESET_LCD);   /* Release reset */
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_LCD);         /* Enable peripheral clock */

    /* Cannot open an LCD twice */
    if (!handle->inUse) {
        for (i = 0; i < 128; i++) {
            LPC_LCD->PAL[i] = 0;                        /* Clear palette */
        }

        LPC_LCD->UPBASE = 0xA0000000;                   /* Safe default base address TODO */

        handle->inUse = LPCLIB_YES;
        *pHandle = &lcdContext;                         /* Return handle */

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;
}



/* Close the LCD interface. */
void LCD_close (LCD_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

//TODO disable interrupts

    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_LCD);        /* Disable peripheral clock */

    (*pHandle)->callback = NULL;
    (*pHandle)->inUse = LPCLIB_NO;

    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Configure the LCD block. */
void LCD_ioctl (LCD_Handle handle, const LCD_Config *pConfig)
{
    uint32_t divider;
    int bypass;
    int extClock;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    while (pConfig->opcode != LCD_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case LCD_OPCODE_SET_MODE:
            LPC_LCD->CTRL = 0
                | (0 << LCD_CTRL_LCDEN_Pos)                             /* Disable */
                | (pConfig->mode.bitsPerPixel << LCD_CTRL_LCDBPP_Pos)   /* #bits per pixel */
                | (pConfig->mode.panelType << LCD_CTRL_LCDBW_LCDTFT_Pos)/* Panel type (STN/TFT) */
                | (pConfig->mode.colorFormat << LCD_CTRL_BGR_Pos)       /* RGB/BGR */
                | (0 << LCD_CTRL_BEBO_Pos)                              /* Fix: Little endian byte order */
                | (0 << LCD_CTRL_BEPO_Pos)                              /* Fix: Little endian pixel order */
                | (0 << LCD_CTRL_LCDPWR_Pos)                            /* Power off */
                ;
            break;

        case LCD_OPCODE_SET_TIMINGS:
            LPC_LCD->TIMH =
                ((pConfig->timings.sizeX / 16 - 1)  << 2)   |
                ((pConfig->timings.syncWidthH - 1)  << 8)   |
                ((pConfig->timings.frontPorchH - 1) << 16)  |
                ((pConfig->timings.backPorchH - 1)  << 24);

            LPC_LCD->TIMV =
                ((pConfig->timings.sizeY - 1)       << 0)   |
                ((pConfig->timings.syncWidthV - 1)  << 10)  |
                (pConfig->timings.frontPorchV       << 16)  |
                (pConfig->timings.backPorchV        << 24);

    //         CLKPWR_connectBaseClock(CLKPWR_BASECLOCK_LCD, CLKPWR_SOURCE_PLL1); //TODO

    //TODO Improve calculation of best divider (include bypass)
            /* Set the divider such that the resulting pixel clock doesn't exceed
             * the requested clock frequency.
             * The assumption is that a lower refresh rate will not cause problems, while
             * an excess refresh rate may cause bus contention.
             *
             * NOTE: The "pixelFrequency+1" ensures that we do not select a lower frequency
             *       if the bus frequency is an integer multiple of the pixel frequency.
             */
            divider = CLKPWR_getBusClock(CLKPWR_CLOCK_LCD) / (pConfig->timings.pixelFrequency + 1) + 1;
            if (divider >= 2) {
                divider -= 2;                           /* Hardware adds 2 to the divider */
            }
            bypass = 0;                //TODO
            extClock = pConfig->timings.externalClockFrequency ? 1 : 0;
            LPC_LCD->POL =
                ((divider & 0x1F)                   << 0)   |
                (extClock                           << 5)   |
                (0                                  << 6)   |
                (pConfig->timings.polarityVSYNC     << 11)  |
                (pConfig->timings.polarityHSYNC     << 12)  |
                (pConfig->timings.polarityClock     << 13)  |
                (pConfig->timings.polarityENAB      << 14)  |
                ((pConfig->timings.sizeX - 1)       << 16)  |
                (bypass                             << 26)  |
                ((divider >> 5)                     << 27);
            break;

        case LCD_OPCODE_SET_POWER:
            if (pConfig->powered) {
                LPC_LCD->CTRL |= LCD_CTRL_LCDEN_Msk | LCD_CTRL_LCDPWR_Msk;
                LPC_LCD->INTMSK = LCD_INTMSK_LNBUIM_Msk;    /* Enable selected interrupts TODO */
            }
            else {
                LPC_LCD->INTMSK = 0;                    /* No more interrupts */
                LPC_LCD->CTRL &= ~(LCD_CTRL_LCDEN_Msk | LCD_CTRL_LCDPWR_Msk);
            }
            break;

        case LCD_OPCODE_SET_CALLBACK:
            if (pConfig->callback.pOldCallback) {       /* Return current callback if requested */
                *(pConfig->callback.pOldCallback) = handle->callback;
            }
            handle->callback = pConfig->callback.callback;
            break;

        case LCD_OPCODE_INVALID:
            /* ignore */
            break;
        }

        ++pConfig;
    }
}



/* Get dimension (visible area) */
LPCLIB_Result LCD_getDimension (LCD_Handle handle, uint32_t *pSizeX, uint32_t *pSizeY)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (pSizeX) {
        *pSizeX = ((LPC_LCD->POL & LCD_POL_CPL_Msk) >> LCD_POL_CPL_Pos) + 1;
    }
    if (pSizeY) {
        *pSizeY = ((LPC_LCD->TIMV & LCD_TIMV_LPP_Msk) >> LCD_TIMV_LPP_Pos) + 1;
    }

    return LPCLIB_SUCCESS;
}



/** LCD interrupt handler.
 */
void LCD_IRQHandler (void)
{
    LPCLIB_Event event;
    LCD_Handle const handle = &lcdContext;
    uint32_t status;

    event.id = LPCLIB_EVENTID_LCD;

    /* Check interrupt sources. Combine cursor and other interrupts.
     * Combining works because the events have unique bit positions in both registers!
     */
    status = (LPC_LCD->INTSTAT & (LCD_INTSTAT_FUFMIS_Msk |
                                  LCD_INTSTAT_LNBUMIS_Msk |
                                  LCD_INTSTAT_VCOMPMIS_Msk |
                                  LCD_INTSTAT_BERMIS_Msk))
           | (LPC_LCD->CRSR_INTSTAT & LCD_CRSR_INTSTAT_CRSRMIS_Msk);
    if (status != 0) {
        if (status & LCD_INTSTAT_LNBUMIS_Msk) {
            event.opcode = LCD_EVENT_FRAME;
            LPC_LCD->INTCLR = LCD_INTCLR_LNBUIC_Msk;
        }
        else if (status & LCD_INTSTAT_VCOMPMIS_Msk) {
            event.opcode = LCD_EVENT_LINE_MATCH;
            LPC_LCD->INTCLR = LCD_INTCLR_VCOMPIC_Msk;
        }
        else if (status & LCD_CRSR_INTSTAT_CRSRMIS_Msk) {
            event.opcode = LCD_EVENT_CURSOR;
            LPC_LCD->CRSR_INTCLR = status & LCD_CRSR_INTCLR_CRSRIC_Msk;
        }
        else if (status & LCD_INTSTAT_FUFMIS_Msk) {
            event.opcode = LCD_EVENT_ERROR_UNDERRUN;
            LPC_LCD->INTCLR = LCD_INTCLR_FUFIC_Msk;
        }
        else if (status & LCD_INTSTAT_BERMIS_Msk) {
            event.opcode = LCD_EVENT_ERROR_BUS;
            LPC_LCD->INTCLR = LCD_INTCLR_BERIC_Msk;
        }

        if (handle->callback) {
            handle->callback(event);
        }
    }
}

/** @} */

/** @} addtogroup LCD */

#endif  /* #ifdef LPCLIB_LCD */

