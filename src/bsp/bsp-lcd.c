/* Copyright (c) 2013, DF9DQ
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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "st7565r.h"

#include "bsp-fifisdr.h"
#include "bsp-io.h"
#include "bsp-lcd.h"
#include "params.h"


extern const uint8_t font_7seg[10][12*3];



static struct {
    uint8_t frame[(128 * 64) / 8];
} lcd;


/** Arbitrary identifiers to be used in SSP slave select mechanism. */
enum {
    BSP_SSEL_LCD,
};



/** Event handler for SSP1 slave select demuxing. */
static void BSP_handleSsp1Select (LPCLIB_Event event)
{
    switch (event.opcode) {
    case SSP_EVENT_ASSERT_CHIPSELECT:
        switch (event.channel) {
        case BSP_SSEL_LCD:
            GPIO_writeBit(GPIO_ROLF1_GUI_SSEL, 0);
            /* Here the parameter is the value of the command/data select line */
            GPIO_writeBit(GPIO_ROLF1_GUI_A0, (int)event.parameter);
            break;
        }
        break;

    case SSP_EVENT_DEASSERT_CHIPSELECT:
        switch (event.channel) {
        case BSP_SSEL_LCD:
            GPIO_writeBit(GPIO_ROLF1_GUI_SSEL, 1);
            break;
        }
        break;

    default:
        break;
    }
}


SSP_Handle sspLcd;
ST7565R_Handle lcdDev;


static const SSP_DeviceSelect lcdSelect = {
    .callback = BSP_handleSsp1Select,
    .channel = BSP_SSEL_LCD,
};

static const ST7565R_Config lcdConfig[] = {
    {.opcode = ST7565R_OPCODE_INIT, },

    ST7565R_CONFIG_END
};


LPCLIB_Result BSP_openLcd (void)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    if (BSP_getBoardType() == BSP_BOARD_ROLF1) {
        GPIO_writeBit(GPIO_ROLF1_GUI_RESET, 1);         /* Deassert RESET */
        SSP_open(SSP1, &sspLcd);
        ST7565R_open(sspLcd, &lcdSelect, &lcdDev);
        result = ST7565R_ioctl(lcdDev, lcdConfig);
        GPIO_writeBit(GPIO_ROLF1_GUI_LIGHT, 1);         /* Backlight on */
    }

    return result;
}


LPCLIB_Result BSP_closeLcd (void)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    if (BSP_getBoardType() == BSP_BOARD_ROLF1) {
        GPIO_writeBit(GPIO_ROLF1_GUI_LIGHT, 0);         /* Backlight off */
        ST7565R_close(&lcdDev);
        SSP_close(&sspLcd);
        GPIO_writeBit(GPIO_ROLF1_GUI_RESET, 0);         /* Assert RESET */
    }

    return result;
}



void BSP_show7seg (int x, int y, int c)
{
    int i;
(void)y; //TODO

    if (c < 10) {
        for (i = 0; i < 12; i++) {
            lcd.frame[512+0*128+x+i] = font_7seg[c][0*12+i];
        }
        for (i = 0; i < 12; i++) {
            lcd.frame[512+1*128+x+i] = font_7seg[c][1*12+i];
        }
        for (i = 0; i < 12; i++) {
            lcd.frame[512+2*128+x+i] = font_7seg[c][2*12+i];
        }
        ST7565R_writeFullScreen(lcdDev, lcd.frame);
    }
}

