/* Copyright (c) 2010-2011, DF9DQ
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


#include "lpclib.h"

#include "usbdevice.h"
#include "usbclass.h"

#include "bsp-fifisdr.h"
#include "params.h"
#include "softrock.h"
#include "task-sys.h"


uint32_t softrockDebugInfo[8];


/* Handle request for the Softrock emulation. */
LPCLIB_Result softrock_handleRequests (
    USBDEV_Handle handle,
    const struct USB_Class *pClass,
    USB_SetupPacket *pSetup,
    USB_Buffer *pBuffer)
{
    (void) handle;
    (void) pClass;

    bool new_params;
    LPCLIB_Result result = LPCLIB_SUCCESS;
    LPCLIB_Event event;
    uint32_t temp;
    union {
        uint8_t *b;
        uint16_t *w;
        uint32_t *d;
    } p = {.b = pBuffer->data};
    int length = 0;
    int i;


    /* Filter: Accept VENDOR requests to the DEVICE(!) */
    if ((pSetup->Recipient != USB_REQUEST_RECIPIENT_DEVICE) ||
        (pSetup->Type != USB_REQUEST_TYPE_VENDOR)) {

        return LPCLIB_UNDEFINED;
    }

    event.id = LPCLIB_EVENTID_APPLICATION;
    new_params = false;

    switch (pSetup->bRequest) {
    case 0x00:  /* Get firmware version */
        p.w[0] = (17 << 8) | (1 << 0);
        length = 2;
        break;

    case 0x01:  /* Dummy (Set DDR) */
        break;

    case 0x02:  /* Dummy (Get PIN) */
        p.b[0] = 0
               | (0 << 5)               /* CW1 */
               | (0 << 4)               /* PTT */
               | (1 << 3)               /* 3.3V indicator */
               | (0 << 1)               /* CW2 */
               ;
        length = 1;
        break;

    case 0x03:  /* Dummy (Get Port) */
        p.b[0] = 0;
        length = 1;
        break;

    case 0x04:  /* Dummy (Set Port) */
        break;

    case 0x15:  /* Dummy (Set I/O) */
        p.b[0] = 0;
        p.b[1] = 0;
        length = 2;
        break;

    case 0x16:  /* Dummy (Get I/O) */
        p.w[0] = 0;
        length = 2;
        break;

    case 0x17:  /* Filter */
        /* First filter bank */
        if (pSetup->wIndex <= 255) {
            /* Set a filter trip point? */
            if (pSetup->wIndex <= 2) {
                /* Setting filter trip points is not supported! */
                ;
            }

            /* Return preselector band edges as crossover points.
             * This is correct only if preselector bands form a contiguous frequency range.
             * For the default settings, this assumption is valid.
             */
            for (i = 0; i < 15; i++) {
                /* Convert from 11.21 to 11.5 format */
                p.w[i] = g_params.presel_freq[i][1] >> 16;
            }
            length = 15 * 2;
        }
        /* Second filter bank (TX lowpass, not used) */
        else {
            p.w[0] = 0;
            length = 2;
        }
        break;

    case 0x18:  /* Set RX Band Pass Filter Address (ignored) */
        if (pSetup->wLength == 100) {
            for (i = 0; i < 100; i++) {
                p.b[i] = 0; /* No idea what the data should be... */
            }
            length = 100;
        }
        else {
            result = LPCLIB_ILLEGAL_PARAMETER;
        }
        break;

    case 0x19:  /* Read RX Band Pass Filter */
        p.b[0] = 0;
        p.b[1] = 1;
        p.b[2] = 2;
        p.b[3] = 3;
        length = 4;
        break;

    case 0x1B:  /* Dummy */
        for (i = 0; i < 10; i++) {
            p.b[i] = 0;
        }
        length = 10;
        break;

    case 0x20:  /* Access to a (virtual) Si570 register */
        /* Response length should be 1 (flag that indicates error) */
        if (pSetup->wLength == 1) {
            event.opcode = FIFISDR_EVENT_SET_SINGLE_VIRTUAL_REGISTER;
            event.channel = pSetup->wValue / 256;
            event.parameter = (void *)((uint32_t)pSetup->wIndex % 256);
            SYS_submitJob(event);

            p.b[0] = 0;     /* No error */
            length = 1;
        }
        else {
            result = LPCLIB_ILLEGAL_PARAMETER;
        }
        break;

    case 0x30:  /* Set frequency by register values */
        event.opcode = FIFISDR_EVENT_SET_VIRTUAL_REGISTERS;
        event.parameter = pBuffer->data;    //TODO pass by value!
        SYS_submitJob(event);
        break;

    case 0x31:  /* Set subtract/multiply values */
        g_params.freqsubtract1121 = ((uint32_t *)pBuffer->data)[0];
        g_params.freqmultiply = ((uint32_t *)pBuffer->data)[1];
        event.opcode = FIFISDR_EVENT_UPDATE_VCO;
        SYS_submitJob(event);
        new_params = true;
        break;

    case 0x32:  /* Set oscillator frequency by value */
        temp = p.d[0];          /* Get frequency (format 11.21) */
        event.opcode = FIFISDR_EVENT_SET_FREQUENCY;
        event.parameter = (void *)temp;
        SYS_submitJob(event);
        break;

    case 0x33:  /* Set XTAL frequency */
        g_params.freqxtal = p.d[0];
        event.opcode = FIFISDR_EVENT_UPDATE_VCO;
        SYS_submitJob(event);
        new_params = true;
        break;

    case 0x34:  /* Set startup frequency */
        g_params.freqstartup = p.d[0];
        new_params = true;
        break;

    case 0x35:  /* Set smooth tune */
        g_params.smoothtune = p.w[0];
        new_params = true;
        break;

    case 0x39:  /* Frequency subtract/multiply */
        p.d[0] = g_params.freqsubtract1121;
        p.d[1] = g_params.freqmultiply;
        length = 8;
        break;

    case 0x3A:  /* Read current frequency */
        event.opcode = FIFISDR_EVENT_GET_FREQUENCY;
        event.parameter = pBuffer;
        SYS_submitJob(event);
        result = LPCLIB_BUSY;
        break;

    case 0x3B:  /* Read smooth tune */
        p.w[0] = g_params.smoothtune;
        length = 2;
        break;

    case 0x3C:  /* Read startup frequency */
        p.d[0] = g_params.freqstartup;
        length = 4;
        break;

    case 0x3D:  /* Read XTAL frequency */
        p.d[0] = g_params.freqxtal;
        length = 4;
        break;

    case 0x3F:  /* Read (virtual) Registers */
        event.opcode = FIFISDR_EVENT_GET_VIRTUAL_REGISTERS;
        event.parameter = pBuffer;
        SYS_submitJob(event);
        result = LPCLIB_BUSY;
        break;

    case 0x40:  /* I2C errors */
        p.b[0] = 0;
        length = 1;
        break;

    case 0x41:  /* Set I2C address (ignored) */
        p.b[0] = si570Address;
        length = 1;
        break;

    case 0x42:  /* Dummy (Get CPU Temperature) */
        p.w[0] = 16 * 25;               /* 25°C */
        length = 2;
        break;

    case 0x43:  /* Dummy (Get/Set USB ID) */
        p.b[0] = '0';
        length = 1;
        break;

    case 0x44:  /* Si570 Speed Grade and DCO range */
        /* No support for changing the parameters! */

        /* Report status */
        p.w[0] = 4850;                  /* DCO minimum frequency [MHz] */
        p.w[1] = 5670;                  /* DCO maximum frequency [MHz] */
        p.b[4] = 3;                     /* Grade C (A=1, B=2, C=3, C+=4 */
        p.b[5] = 0x87;                  /* M freeze for RFREQ change, register set starts at 7 */
        length = 6;
        break;

    case 0x50:  /* Set PTT line */
        event.opcode = FIFISDR_EVENT_SET_PTT;
        event.parameter = (void *)((uint32_t)pSetup->wValue);
        SYS_submitJob(event);
        /* No inputs keys supported yet. */
        p.b[0] = 0x22;
        length = 1;
        break;

    case 0x51:  /* Read CW key level */
        /* No inputs keys supported yet. */
        p.b[0] = 0x22;
        length = 1;
        break;

    case 0x61:  /* Dummy (Mobo read analog inputs) */
        switch (pSetup->wIndex) {
        case 0:                         /* Current */
            p.w[0] = 0;
            break;
        case 1:                         /* VSWR1 */
            p.w[0] = 0;
            break;
        case 2:                         /* VSWR2 */
            p.w[0] = 0;
            break;
        case 3:                         /* Voltage */
            p.w[0] = 13830;             /* 3.3V */
            break;
        case 4:                         /* Temperature */
            p.w[0] = 25 * 256;          /* 25°C */
            break;
        default:
            p.w[0] = 0;
            break;
        }
        length = 2;
        break;

    case 0xAB:    /* FiFi-SDR specific commands (read) */
        switch (pSetup->wIndex) {
        case 0:
            /* Return the SVN build number. */
            p.d[0] = SVNMAXREV;         /* This macro is defiuned in GCC command line (Makefile) */
            length = 4;
            break;
        case 1:
            /* Return a string describing the firmware version. */
            pBuffer->data = SVNNAME;
            length = sizeof(SVNNAME);
            break;
        case 3:
            p.d[0] = g_params.freq_3rd_harmonic;
            length = 4;
            break;
        case 5:
            p.d[0] = g_params.freq_5th_harmonic;
            length = 4;
            break;
        case 6:
            p.d[0] = g_params.presel_mode;
            length = 4;
            break;
        case 7:
            if (pSetup->wValue <= 15) {
                p.d[0] = g_params.presel_freq[pSetup->wValue][0];
                p.d[1] = g_params.presel_freq[pSetup->wValue][1];
                p.b[8] = g_params.presel_pattern[pSetup->wValue];
                length = 9;
            }
            else {
                result = LPCLIB_ILLEGAL_PARAMETER;
            }
            break;
        case 10:
            /* Read real Si570 registers */
            event.opcode = FIFISDR_EVENT_GET_REAL_REGISTERS;
            event.parameter = pBuffer;
            SYS_submitJob(event);
            result = LPCLIB_BUSY;
            break;
        case 11:
            p.d[0] = g_params.virtual_vco_factor;
            length = 4;
            break;
        case 12:
            event.opcode = FIFISDR_EVENT_GET_FACTORY_STARTUP;
            event.parameter = pBuffer;
            SYS_submitJob(event);
            result = LPCLIB_BUSY;
            break;
        case 14:
            /* Get demodulator volume (in %) */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_VOLUME;
            event.parameter = pBuffer->data;
            SYS_submitJob(event);
            length = 2;
            break;
        case 15:
            /* Get demodulator mode (LSB/USB/...) */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_MODE;
            event.parameter = pBuffer->data;
            SYS_submitJob(event);
            length = 1;
        break;
        case 16:
            /* Get demodulator bandwidth (Hz) */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_BANDWIDTH;
            event.parameter = pBuffer->data;
            SYS_submitJob(event);
            length = 4;
            break;
        case 17:
            /* Get S-Meter */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_RSSI;
            event.parameter = pBuffer->data;
            SYS_submitJob(event);
            length = 4;
            break;
        case 18:
            /* Get FM center */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_FMCENTER;
            event.parameter = pBuffer->data;
            SYS_submitJob(event);
            length = 4;
            break;
        case 19:
            /* Get preamp (ADC) volume */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_PREAMP;
            event.parameter = pBuffer->data;
            SYS_submitJob(event);
            if (p.w[0] == 0) {
                p.b[0] = 1;         /* 0 dB */
            }
            else {
                p.b[0] = 0;         /* -6 dB */
            }
            length = 1;
            break;
        case 20:
            /* Get squelch control */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_SQUELCH;
            event.parameter = pBuffer->data;
            p.b[0] = 0;
            SYS_submitJob(event);
            length = 1;
            break;
        case 21:
            /* Get AGC mode */
            event.opcode = FIFISDR_EVENT_DEMOD_GET_AGCTEMPLATE;
            event.parameter = pBuffer->data;
            p.b[0] = 0;
            SYS_submitJob(event);
            length = 1;
            break;
        case 0xFFFF:
            /* Debug data */
            for (i = 0; i < 8; i++) {
                p.d[i] = softrockDebugInfo[i];
            }
            length = 32;
            break;
        default:
            result = LPCLIB_ILLEGAL_PARAMETER;
            break;
        }
        break;

    case 0xAC:    /* FiFi-SDR specific commands (write) */
        switch (pSetup->wIndex) {
        case 3:
            g_params.freq_3rd_harmonic = p.d[0];
            new_params = true;
            break;
        case 5:
            g_params.freq_5th_harmonic = p.d[0];
            new_params = true;
            break;
        case 6:
            g_params.presel_mode = p.d[0];
            new_params = true;
            break;
        case 7:
            if (pSetup->wValue <= 15) {
                g_params.presel_freq[pSetup->wValue][0] = p.d[0];
                g_params.presel_freq[pSetup->wValue][1] = p.d[1];
                g_params.presel_pattern[pSetup->wValue] = p.b[8];
                if (pSetup->wValue == 15) {
                    new_params = true;
                }
            }
            break;
        case 11:
            g_params.virtual_vco_factor = p.d[0];
            new_params = true;
            break;
        case 14:
            /* Set demodulator volume (in %) */
            event.opcode = FIFISDR_EVENT_DEMOD_SET_VOLUME;
            event.parameter = (void *)((uint32_t) p.w[0]);
            SYS_submitJob(event);
            break;
        case 15:
            /* Set demodulator mode (LSB/USB/...) */
            event.opcode = FIFISDR_EVENT_DEMOD_SET_MODE;
            event.parameter = (void *)((uint32_t) p.b[0]);
            SYS_submitJob(event);
            break;
        case 16:
            /* Set demodulator bandwidth (Hz) */
            event.opcode = FIFISDR_EVENT_DEMOD_SET_BANDWIDTH;
            event.parameter = (void *)((uint32_t) p.d[0]);
            SYS_submitJob(event);
            break;
        case 19:
            /* Set preamp (ADC) volume */
            event.opcode = FIFISDR_EVENT_DEMOD_SET_PREAMP;
            event.parameter = (void *)((uint32_t) p.b[0]);
            SYS_submitJob(event);
            break;
        case 20:
            /* Set squelch control */
            event.opcode = FIFISDR_EVENT_DEMOD_SET_SQUELCH;
            event.parameter = (void *)((uint32_t) p.b[0]);
            SYS_submitJob(event);
            break;
        case 21:
            /* Set AGC template */
            event.opcode = FIFISDR_EVENT_DEMOD_SET_AGCTEMPLATE;
            event.parameter = (void *)((uint32_t) p.b[0]);
            SYS_submitJob(event);
            break;
        }
        break;

    default:
        result = LPCLIB_ILLEGAL_PARAMETER;
        break;
    }

    /* Send response now if ready */
    if (result == LPCLIB_BUSY) {
        /* Response will be sent later. This is a success! */
        result = LPCLIB_SUCCESS;
    }
    else {
        /* Return length of reply. Truncate to requested size. */
        if (length > pSetup->wLength) {
            length = pSetup->wLength;
        }
        pBuffer->maxSize = length;
        USBDEV_submitSetupResponse(handle,
                                   (result == LPCLIB_SUCCESS) ? pBuffer : NULL);
    }

    /* Store new parameters */
    if (new_params) {
        event.opcode = FIFISDR_EVENT_SAVE_PARAMS;
        SYS_submitJob(event);
    }

    return result;
}

