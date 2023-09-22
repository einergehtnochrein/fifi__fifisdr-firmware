/* Copyright (c) 2011, NXP Semiconductors
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
 *  LPC17xx Library Header for Applications.
 *  This file must be included in all application source files.
 *  There is no need to include individual peripheral driver headers files.
 */


#ifndef __LPCLIB_H
#define __LPCLIB_H


#include "lpc17xx_libconfig.h"

#if LPCLIB_ADC
    #include "lpc17xx_adc.h"
#endif
#if LPCLIB_CAN
//    #include "lpc17xx_can.h"
#endif
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_crc.h"
#include "lpc17xx_dac.h"
#if LPCLIB_DMA
    #include "lpc17xx_dma.h"
#endif
#if LPCLIB_EMC
    #include "lpc17xx_emc.h"
#endif
#include "lpc17xx_gpio.h"
#if LPCLIB_I2C
    #include "lpc17xx_i2c.h"
#endif
#if LPCLIB_I2S
    #include "lpc17xx_i2s.h"
#endif
#include "lpc17xx_iap.h"
#include "lpc17xx_iocon.h"
#if LPCLIB_LCD
    #include "lpc17xx_lcd.h"
#endif
#if LPCLIB_MCI
    #include "lpc17xx_mci.h"
#endif
//#include "lpc17xx_romhandler.h"
#include "lpc17xx_rtc.h"
#if LPCLIB_SSP
    #include "lpc17xx_ssp.h"
#endif
#if LPCLIB_TIMER
    #include "lpc17xx_timer.h"
#endif
#if LPCLIB_UART
    #include "lpc17xx_uart.h"
#endif
#if LPCLIB_USB
    #include "lpc17xx_usb.h"
#endif


#endif /* __LPCLIB_H */

/** @} */

