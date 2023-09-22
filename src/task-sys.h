
#ifndef __TASK_SYS_H
#define __TASK_SYS_H

#include "lpclib.h"


/** Opcodes for application events. */
enum {
    /* Interface to oscillator (Si570) and CPLD for frequency control. */
    FIFISDR_EVENT_SET_FREQUENCY,            /**< Set new frequency (from Softrock) */
    FIFISDR_EVENT_GET_FREQUENCY,            /**< Get frequency */
    FIFISDR_EVENT_GET_REAL_REGISTERS,       /**< Ask for real registers of Si570 */
    FIFISDR_EVENT_SET_VIRTUAL_REGISTERS,    /**< Set virtual Si570 registers */
    FIFISDR_EVENT_GET_VIRTUAL_REGISTERS,    /**< Ask for virtual registers of Si570 */
    FIFISDR_EVENT_GET_FACTORY_STARTUP,      /**< Ask for factory startup registers */
    FIFISDR_EVENT_UPDATE_VCO,               /**< Update VCO (after parameter change) */
    FIFISDR_EVENT_SET_SINGLE_VIRTUAL_REGISTER,  /**< Set a single register of the
                                                 *   emulated Si570.
                                                 */
    FIFISDR_EVENT_SAVE_PARAMS,              /** Save parameter to flash */
    FIFISDR_EVENT_SET_PTT,                  /**< Set the PTT line */

    FIFISDR_EVENT_FREQUENCY_HZ,             /**< Inform about frequency (Hz) */

    FIFISDR_EVENT_DEMOD_SET_MODE,           /**< Set mode of demodulator */
    FIFISDR_EVENT_DEMOD_GET_MODE,           /**< Get mode of demodulator */
    FIFISDR_EVENT_DEMOD_SET_BANDWIDTH,      /**< Set filter bandwidth of demodulator */
    FIFISDR_EVENT_DEMOD_GET_BANDWIDTH,      /**< Get filter bandwidth of demodulator */
    FIFISDR_EVENT_DEMOD_SET_VOLUME,         /**< Set audio volume */
    FIFISDR_EVENT_DEMOD_GET_VOLUME,         /**< Get audio volume */
    FIFISDR_EVENT_DEMOD_SET_SQUELCH,        /**< Set Squelch control */
    FIFISDR_EVENT_DEMOD_GET_SQUELCH,        /**< Get Squelch control */
    FIFISDR_EVENT_DEMOD_SET_PREAMP,         /**< Set preamp (ADC) volume */
    FIFISDR_EVENT_DEMOD_GET_PREAMP,         /**< Get preamp (ADC) volume */
    FIFISDR_EVENT_DEMOD_SET_AGCTEMPLATE,    /**< Set AGC template */
    FIFISDR_EVENT_DEMOD_GET_AGCTEMPLATE,    /**< Get AGC template */
    FIFISDR_EVENT_DEMOD_GET_RSSI,           /**< Get RSSI */
    FIFISDR_EVENT_DEMOD_GET_FMCENTER,       /**< Get FM center deviation */

    FIFISDR_EVENT_SET_IQ_SWAP,              /**< Enable/disable I/Q swap */

    FIFISDR_EVENT_USB_SUSPEND,              /**< USB suspended */
    FIFISDR_EVENT_USB_RESUME,               /**< USB resumed */

    FIFISDR_EVENT_TOUCH,                    /**< Touch event */
    FIFISDR_EVENT_TOUCH_NEED_SERVICE,       /**< Service request by touch pad driver */
};


struct SYS_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};



/** Submit a job for the system handler. */
void SYS_submitJob (LPCLIB_Event event);

/** Install a callback to become informed about system events. */
void SYS_installCallback (struct SYS_ConfigCallback configCallback);


/** System management task. */
void SYS_task (const void *pArgs);

#endif
