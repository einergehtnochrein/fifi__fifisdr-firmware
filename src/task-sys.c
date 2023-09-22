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


#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "system_LPC17xx.h"
#include "bsp-fifisdr.h"
#include "bsp-touch.h"

#include "task-gui.h"
#include "task-sys.h"
#include "si570.h"
#include "usbuser.h"
#include "params.h"


/** \file
 *  \brief System management task for FiFi-SDR.
 *
 *  The system management task is the heart of the SDR control code. Its responsibility
 *  includes:
 *  - Interface between the Softrock layer and the Si570+CPLD+Preselector.
 *  - Power management
 *
 *  \author DF9DQ
 */



#define SYS_QUEUE_LENGTH                        (10)


typedef struct {
    uint8_t opcode;
    LPCLIB_Event event;
    union {
        uint8_t si570Regs[6];
    };
} SYS_Message;


/** Message opcodes for SYS task. */
enum {
    SYS_OPCODE_EVENT,
    SYS_OPCODE_INIT_VCO,
};



struct {
    osMailQId queue;                                    /**< Task message queue */
    uint32_t rxFrequency1121;                           /**< Current RX frequency */
    _Bool si570Initialized;                             /**< Set if Si570 set to a new frequency
                                                         *   after startup.
                                                         */
    _Bool swapIQGlobal;                                 /**< Swap I/Q channels */
    _Bool sleeping;                                     /**< Low-power mode activated */
    LPCLIB_Callback callback;                           /**< Callback for system events */
} sysContext;




/** Set the preselector.
 *
 *  Based on the frequency, one of the four possible preselector outputs
 *  is activated.
 *
 *  \param[in] frequency1121 Frequency (format 11.21)
 */
static void SYS_setPreselector (uint32_t frequency1121)
{
    uint32_t freq_11_5;
    int i;
    uint32_t search_range;
    BSP_PreselectorSetting setting;


    /* Convert to 11.5 format */
    freq_11_5 = (frequency1121 >> 16);

    setting.frequency1121 = frequency1121;
    setting.pattern = 0;

    if (g_params.presel_mode == 0) {
        /* Select one of the prescaler outputs */
        if (!sysContext.sleeping) {
            if (freq_11_5 < g_params.freqrxtrip[0]) {
                setting.pattern = (1u << 1);
            }
            else if (freq_11_5 < g_params.freqrxtrip[1]) {
                setting.pattern = (1u << 2);
            }
            else if (freq_11_5 < g_params.freqrxtrip[2]) {
                setting.pattern = (1u << 0);
            }
            else {
                setting.pattern = (1u << 3);
            }
        }
    }

    if ((g_params.presel_mode == 1) || (g_params.presel_mode == 2) || (g_params.presel_mode == 3)) {
        if (!sysContext.sleeping) {
            search_range = 0xFFFFFFFF;

            /* Search for the smallest frequency range that contains the current frequency */
            for (i = 0; i < 16; i++) {
                if ((frequency1121 >= g_params.presel_freq[i][0]) &&
                    (frequency1121 <= g_params.presel_freq[i][1])) {
                    if (g_params.presel_freq[i][1] - g_params.presel_freq[i][0] < search_range) {
                        /* Smallest fit so far */
                        search_range = g_params.presel_freq[i][1] - g_params.presel_freq[i][0];
                        setting.pattern = g_params.presel_pattern[i];
                    }
                }
            }
        }
    }

    /* Set outputs */
    BSP_setPreselector(&setting);
}



/** Set the required prescaler (CPLD).
 *
 *  Based on the frequency, one of the four possible prescalers
 *  (1, 4, 16, 64) is selected.
 *
 *  \param[in] rxFrequency1121 Frequency (format 11.21)
 *  \param[out] pPrescaler The prescaler value is returned here
 *  \param[in] update Set the prescaler outputs if ENABLEd
 */
static void SYS_setPrescaler (uint32_t rxFrequency1121, uint32_t *pPrescaler, LPCLIB_Switch update)
{
    uint32_t prescaler;
    int swapIQ;


    /* Determine what prescaler is needed now */
    if (rxFrequency1121 < _11_21(4 * 0.2)) {
        prescaler = 64;
    }
    else if (rxFrequency1121 < _11_21(4 * 0.8)) {
        prescaler = 16;
    }
    else if (rxFrequency1121 < _11_21(4 * 2.5)) {
        prescaler = 4;
    }
    else {
        prescaler = 1;
    }

    /* Return prescaler */
    if (pPrescaler) {
        *pPrescaler = prescaler;
    }

    /* Update CPLD control lines if enabled */
    if (update) {
        /* Select prescaler in CPLD */
        BSP_setPrescaler(prescaler);

        /* Swap I/Q ? */
        swapIQ = sysContext.swapIQGlobal;
        if ((rxFrequency1121 >= g_params.freq_3rd_harmonic) &&
            (rxFrequency1121 < g_params.freq_5th_harmonic)) {
            swapIQ ^= 1;        /* Swap I and Q channels in this frequency range */
        }

        BSP_setIqSwap(swapIQ);
    }
}



/** Set a new RX frequency.
 *
 *  Adjust Si570, prescaler, and preselector for a requested RX frequency. Takes care of
 *  special RX options, like 3rd and 5th harmonic reception.
 *
 *  \param[in] rxFrequency1121 Desired RX frequency.
 */
static void SYS_setRxFrequency (uint32_t rxFrequency1121)
{
    uint32_t prescaler;
    uint32_t freq1121;
    uint64_t frequencyHz;
    LPCLIB_Event event;


    /* Find out what prescaler value we need (but do not yet adjust it, because the attempt to
     * set the required frequency in the Si570 might fail!)
     */
    SYS_setPrescaler(rxFrequency1121, &prescaler, DISABLE);
    freq1121 = prescaler * rxFrequency1121;             /* This is the required Si570 frequency */

    /* VHF frequencies: Use 3rd/5th harmonic */
    if (rxFrequency1121 >= g_params.freq_5th_harmonic) {
        freq1121 = freq1121 / 5;
    }
    else if (rxFrequency1121 >= g_params.freq_3rd_harmonic) {
        freq1121 = freq1121 / 3;
    }

    /* See if we can set the Si570 to the calculated frequency */
    if (SI570_write(vco, freq1121) == LPCLIB_SUCCESS) {
        /* Now we can set the CPLD to the new prescaler ratio. */
        SYS_setPrescaler(rxFrequency1121, NULL, ENABLE);

        /* Set the preselector */
        SYS_setPreselector(rxFrequency1121);

        sysContext.rxFrequency1121 = rxFrequency1121;
    }

    /* Inform subscribers about new frequency */
    frequencyHz = (rxFrequency1121 + g_params.freqsubtract1121) / g_params.virtual_vco_factor;
    frequencyHz = ((frequencyHz * 2048000000ll) >> 32) + 5; /* From 11.21 to Hz, snap to 10 Hz grid */
    if (sysContext.callback) {
        LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
        event.opcode = FIFISDR_EVENT_FREQUENCY_HZ;
        event.parameter = (void *)((uint32_t)frequencyHz);
        sysContext.callback(event);
    }
}



static const SI570_Config si570FlushSmoothtune[] = {
    {.opcode = SI570_OPCODE_FLUSH_SMOOTHTUNE, },
};



/** Leave low-power mode */
static void SYS_wakeup (void)
{
    sysContext.sleeping = false;

    /* Restore GPIO state. */
    BSP_wakeup();

    /* Update Si570 and preselector */
    SI570_ioctl(vco, si570FlushSmoothtune);
    SYS_setRxFrequency(sysContext.rxFrequency1121);

    /* Allow audio task activity */
    FIFIAUDIO_setPowerState(ENABLE);
}



/** Enter low-power mode */
static void SYS_sleep (void)
{
    uint32_t OldSystemCoreClock;


    sysContext.sleeping = true;

    /* Stop audio task activity */
    FIFIAUDIO_setPowerState(DISABLE);

    /* Set preselector to low-power mode */
    SYS_setPreselector(sysContext.rxFrequency1121);

    /* All GPIO pins to lowest power mode. */
    BSP_prepareSleep();

    /* TODO: We should probably prevent any interrupt other than USBActivity from
     *       preventing us entering power down state.
     *       Seems to work without in this application...
     *       NOTE: Tasks must have the privilege to do so!
     */


    /* Remember system clock frequency */
    OldSystemCoreClock = SystemCoreClock;

    /* Erratum PLL0.1
     * Entering Power Down mode doesn't disconnect and disable PLL0 automatically.
     * Instead we have to do that manually here.
     */
    CLKPWR_disconnectPLL0();

    /* Enter Power Down mode, and wait for wake-up by USB activity. */
    //NOTE: Requires privileges! That's why RTX must run with privileged tasks for now
    CLKPWR_enterPowerSaving(CLKPWR_POWERSAVING_POWERDOWN);

    /* Re-enable the PLL */
    CLKPWR_setCpuClock(OldSystemCoreClock);

    /* Do everything a USB resume would do. Note that a resume does not always occur! */
    SYS_wakeup();
}



/* Submit a job for the system handler. */
void SYS_submitJob (LPCLIB_Event event)
{
    SYS_Message *pMessage = NULL;
    USB_Buffer *pBuffer;


    if (!sysContext.queue) {
        return;
    }

    if (event.id == LPCLIB_EVENTID_APPLICATION) {
        /* Handle things that can be done immediately without too much effort. */
        switch (event.opcode) {
        case FIFISDR_EVENT_SET_PTT:
            if (g_params.presel_mode == 3) {
                BSP_setPtt((((uint32_t)event.parameter) & 1) != 0);
            }
            return;

        case FIFISDR_EVENT_GET_FREQUENCY:
            pBuffer = (USB_Buffer *)event.parameter;
            ((uint32_t *)pBuffer->data)[0] = sysContext.rxFrequency1121 + g_params.freqsubtract1121;
            pBuffer->maxSize = 4;
            USBDEV_submitSetupResponse(usbdev, pBuffer);
            return;

        default:
            break;
        }

        /* All remaining event require task action. */
        pMessage = osMailAlloc(sysContext.queue, 0);
        if (pMessage) {
            pMessage->opcode = SYS_OPCODE_EVENT;
            pMessage->event = event;
        }

        switch (event.opcode) {
        case FIFISDR_EVENT_SET_IQ_SWAP:
            /* Select I/Q swap (depends on I/Q sample size; 32 bit format swaps I/Q channels!) */
            sysContext.swapIQGlobal = (uint32_t)event.parameter == 32;

            /* Set frequency (updates swap control) */
            event.opcode = FIFISDR_EVENT_UPDATE_VCO;
            break;

        case FIFISDR_EVENT_SET_VIRTUAL_REGISTERS:
            /* Make a copy of the registers which have been passed by reference. */
            if (pMessage) {
                memcpy(pMessage->si570Regs, event.parameter, 6);
            }
            break;

        case FIFISDR_EVENT_DEMOD_SET_MODE:
        case FIFISDR_EVENT_DEMOD_GET_MODE:
        case FIFISDR_EVENT_DEMOD_SET_BANDWIDTH:
        case FIFISDR_EVENT_DEMOD_GET_BANDWIDTH:
        case FIFISDR_EVENT_DEMOD_SET_VOLUME:
        case FIFISDR_EVENT_DEMOD_GET_VOLUME:
        case FIFISDR_EVENT_DEMOD_SET_SQUELCH:
        case FIFISDR_EVENT_DEMOD_GET_SQUELCH:
        case FIFISDR_EVENT_DEMOD_SET_PREAMP:
        case FIFISDR_EVENT_DEMOD_GET_PREAMP:
        case FIFISDR_EVENT_DEMOD_SET_AGCTEMPLATE:
        case FIFISDR_EVENT_DEMOD_GET_AGCTEMPLATE:
        case FIFISDR_EVENT_DEMOD_GET_RSSI:
        case FIFISDR_EVENT_DEMOD_GET_FMCENTER:
            event.id = LPCLIB_EVENTID_APPLICATION;
            FIFIAUDIO_handleEventDemod(event);
            if (sysContext.callback) {
                sysContext.callback(event);
            }
            break;

        default:
            break;
        }
    }

    /* Defer more complicated jobs to the SYS handler task. */
    if (pMessage) {
        osMailPut(sysContext.queue, pMessage);
    }
}



/* Install a callback to become informed about system events. */
void SYS_installCallback (struct SYS_ConfigCallback configCallback)
{
//TODO: critical section!
    if (configCallback.pOldCallback) {
        *(configCallback.pOldCallback) = sysContext.callback;
    }
    sysContext.callback = configCallback.callback;
}



osMailQDef(sysQueue, SYS_QUEUE_LENGTH, SYS_Message);


void SYS_task (const void *pArgs)
{
    (void) pArgs;
    SYS_Message *pMessage;
    SI570_Config configVco;
    USB_Buffer *pBuffer;
    osEvent event;


    sysContext.queue = osMailCreate(osMailQ(sysQueue), NULL);

    /* Update the VCO frequency to the startup frequency (adjusts XTAL as well) */
    sysContext.rxFrequency1121 = g_params.freqstartup;
    pMessage = osMailAlloc(sysContext.queue, 0);
    if (pMessage != NULL) {
        pMessage->opcode = SYS_OPCODE_EVENT;
        pMessage->event.opcode = FIFISDR_EVENT_UPDATE_VCO;
        osMailPut(sysContext.queue, pMessage);
    }
    pMessage = osMailAlloc(sysContext.queue, 0);
    if (pMessage != NULL) {
        pMessage->opcode = SYS_OPCODE_INIT_VCO;
        osMailPut(sysContext.queue, pMessage);
    }

    while (1) {
        /* Is there a new message? */
        event = osMailGet(sysContext.queue, osWaitForever);
        if (event.status == osEventMail) {
            pMessage = (SYS_Message *)event.value.p;

            switch (pMessage->opcode) {
            case SYS_OPCODE_INIT_VCO:       /* Do some initialization of Si570 */
                /* Set smooth tune value from config parameters */
                configVco.opcode = SI570_OPCODE_SET_SMOOTHTUNE;
                configVco.smoothtune = g_params.smoothtune;
                SI570_ioctl(vco, &configVco);
                break;

            case SYS_OPCODE_EVENT:
                switch(pMessage->event.opcode) {
                case FIFISDR_EVENT_USB_SUSPEND:
                    SYS_sleep();
                    break;

                case FIFISDR_EVENT_USB_RESUME:
                    SYS_wakeup();
                    break;

                case FIFISDR_EVENT_SET_FREQUENCY:
                    SYS_setRxFrequency((uint32_t)pMessage->event.parameter - g_params.freqsubtract1121);
                    sysContext.si570Initialized = true;
                    break;

                case FIFISDR_EVENT_GET_REAL_REGISTERS:
                    pBuffer = (USB_Buffer *)pMessage->event.parameter;
                    configVco.opcode = SI570_OPCODE_GET_REGISTERS;
                    configVco.pRegs = pBuffer->data;
                    if (SI570_ioctl(vco, &configVco) == LPCLIB_SUCCESS) {
                        pBuffer->maxSize = 6;
                        USBDEV_submitSetupResponse(usbdev, pBuffer);
                    }
                    break;

                case FIFISDR_EVENT_GET_VIRTUAL_REGISTERS:
                    pBuffer = (USB_Buffer *)pMessage->event.parameter;
                    if (sysContext.si570Initialized) {
                        SI570_calcVirtualRegisters(
                                vco,
                                sysContext.rxFrequency1121,
                                pBuffer->data);
                    }
                    else {
                        configVco.opcode = SI570_OPCODE_GET_FACTORY_STARTUP_REGISTERS;
                        configVco.pRegs = pBuffer->data;
                        SI570_ioctl(vco, &configVco);
                    }
                    pBuffer->maxSize = 6;
                    USBDEV_submitSetupResponse(usbdev, pBuffer);
                    break;

                case FIFISDR_EVENT_SET_VIRTUAL_REGISTERS:
                    break;

                case FIFISDR_EVENT_GET_FACTORY_STARTUP:
                    pBuffer = (USB_Buffer *)pMessage->event.parameter;
                    configVco.opcode = SI570_OPCODE_GET_FACTORY_STARTUP_REGISTERS;
                    configVco.pRegs = pBuffer->data;
                    SI570_ioctl(vco, &configVco);
                    pBuffer->maxSize = 6;
                    USBDEV_submitSetupResponse(usbdev, pBuffer);
                    break;

                case FIFISDR_EVENT_UPDATE_VCO:
                    /* Update XTAL frequency, in case it has changed. */
                    configVco.opcode = SI570_OPCODE_SET_XTAL_FREQUENCY;
                    configVco.frequency824 = g_params.freqxtal;
                    SI570_ioctl(vco, &configVco);

                    /* Set frequency. This accounts for any changes in parameters */
                    SYS_setRxFrequency(sysContext.rxFrequency1121);
                    sysContext.si570Initialized = true;
                    break;

                case FIFISDR_EVENT_SET_SINGLE_VIRTUAL_REGISTER:
                    /* This is used by the CFGSR PC application to force reload of
                     * factory startup frequency of the Si570 during calibration.
                     * Therefore we are only interested in the special case of
                     * setting bit 0 in register 135.
                     * After setting bit 135.0, and before updating the Si570 frequency again,
                     * reading the virtual registers will return the factory startup registers.
                     */
                    if ((pMessage->event.channel == 135) && ((uint32_t)pMessage->event.parameter & 1)) {
                        sysContext.si570Initialized = false;
                    }
                    break;

                case FIFISDR_EVENT_SAVE_PARAMS:
                    paramsWrite();
                    break;

                case FIFISDR_EVENT_TOUCH:
                    //TODO
                    break;

                case FIFISDR_EVENT_TOUCH_NEED_SERVICE:
                    BSP_TOUCH_worker(pMessage->event);
                    break;
                }
                break;
            }

            osMailFree(sysContext.queue, pMessage);
        }
    }
}


