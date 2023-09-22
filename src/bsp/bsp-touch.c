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

#include "bsp-fifisdr.h"
#include "bsp-io.h"
#include "bsp-touch.h"

#include "task-sys.h"


//TODO combine with LCD driver
#define LCD_XSIZE                           128
#define LCD_YSIZE                           64


#define TOUCH_SAMPLE_INTERVAL               20


enum {
    TOUCH_SERVICE_FIRST_TOUCH,
    TOUCH_SERVICE_TIMER,
    TOUCH_SERVICE_ADC,
};


/* State machine */
typedef enum {
    TOUCH_STATE_IDLE,                       /**< Waiting for touch event */
    TOUCH_STATE_SCAN_X,                     /**< Scanning x coordinate */
    TOUCH_STATE_SCAN_Y,                     /**< Scanning y coordinate */
    TOUCH_STATE_CHECK_PEN,                  /**< Check pen up/down */
} _TouchState;


/* Pin mode for the four touch pins */
typedef enum {
    TOUCH_PINMODE_IDLE,                     /**< Waiting for touch event */
    TOUCH_PINMODE_MEASURE_X,                /**< Prepare for X measurement */
    TOUCH_PINMODE_MEASURE_Y,                /**< Prepare for Y measurement */
} _TouchPinMode;


/* Pen state (up/down and coordinate) */
typedef struct {
    int x;
    int y;
    int pen;
} _TOUCH_PenState;


#define NORMAL_PIN(func) \
    IOCON_makeConfigD(PIN_FUNCTION_##func, PIN_PULL_NONE, PIN_OPENDRAIN_OFF)
#define PULLUP_PIN(func) \
    IOCON_makeConfigD(PIN_FUNCTION_##func, PIN_PULL_UP, PIN_OPENDRAIN_OFF)



/** Identifiers for OS timers. */
enum {
    TOUCH_TIMERMAGIC_SAMPLE,
};


static struct {
    osTimerId sampleTick;
    int xRaw, yRaw;
    _TOUCH_PenState penCurrent;
    _TOUCH_PenState penLast;
    _TouchState state;
} touch;



static void _BSP_TOUCH_eventHandler (LPCLIB_Event event);
static void _BSP_TOUCH_setPinMode (_TouchPinMode pinMode);
void _BSP_TOUCH_enterState (_TouchState state);


static const ADC_Config adcInstallHandler[] = {
    {.opcode = ADC_OPCODE_SET_CALLBACK,
        {.callback = {
            .pOldCallback = NULL,
            .callback = _BSP_TOUCH_eventHandler, }}},

    ADC_CONFIG_END
};


static const ADC_Config adcConvertX[] = {
    {.opcode = ADC_OPCODE_SET_CHANNELMODE,
        {.channel = {
            .index = ADC_CH6,
            .active = DISABLE, }}},

    {.opcode = ADC_OPCODE_SET_CHANNELMODE,
        {.channel = {
            .index = ADC_CH7,
            .active = LPCLIB_YES,
            .sendEvent = LPCLIB_YES, }}},

    {.opcode = ADC_OPCODE_SET_STARTMODE,
        {.startMode = ADC_START_NOW, }},

    ADC_CONFIG_END
};


static const ADC_Config adcConvertY[] = {
    {.opcode = ADC_OPCODE_SET_CHANNELMODE,
        {.channel = {
            .index = ADC_CH7,
            .active = DISABLE, }}},

    {.opcode = ADC_OPCODE_SET_CHANNELMODE,
        {.channel = {
            .index = ADC_CH6,
            .active = LPCLIB_YES,
            .sendEvent = LPCLIB_YES, }}},

    {.opcode = ADC_OPCODE_SET_STARTMODE,
        {.startMode = ADC_START_NOW, }},

    ADC_CONFIG_END
};


static const GPIO_Config configEnableTouchInterrupt[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = GPIO_ROLF1_TOUCH_BOTTOM,
            .enable = ENABLE,
            .mode = GPIO_INT_FALLING_EDGE,
            .callback = _BSP_TOUCH_eventHandler, }}},

    GPIO_CONFIG_END
};


static const GPIO_Config configDisableTouchInterrupt[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = GPIO_ROLF1_TOUCH_BOTTOM,
            .enable = DISABLE,
            .mode = GPIO_INT_FALLING_EDGE,
            .callback = NULL, }}},

    GPIO_CONFIG_END
};



static void _BSP_TOUCH_eventHandler (LPCLIB_Event event)
{
    switch (event.id) {
    case LPCLIB_EVENTID_GPIO:
        GPIO_ioctl(configDisableTouchInterrupt);        //TODO: thread-safe?
        event.id = LPCLIB_EVENTID_APPLICATION;
        event.opcode = FIFISDR_EVENT_TOUCH_NEED_SERVICE;
        event.block = TOUCH_SERVICE_FIRST_TOUCH;
        SYS_submitJob(event);
        break;

    case LPCLIB_EVENTID_ADC:
        event.id = LPCLIB_EVENTID_APPLICATION;
        event.opcode = FIFISDR_EVENT_TOUCH_NEED_SERVICE;
        event.block = TOUCH_SERVICE_ADC;
        SYS_submitJob(event);
        break;

    default:
        /* Nothing to do */
        break;
    }
}



static void _BSP_TOUCH_osalCallback (void const *pArgument)
{
    LPCLIB_Event event;
    (void) pArgument;


    /* Report sample timer event to GUI task */
    event.id = LPCLIB_EVENTID_APPLICATION;
    event.opcode = FIFISDR_EVENT_TOUCH_NEED_SERVICE;
    event.block = TOUCH_SERVICE_TIMER;
    SYS_submitJob(event);
}



static void _BSP_TOUCH_setPinMode (_TouchPinMode pinMode)
{
    switch (pinMode) {
    case TOUCH_PINMODE_IDLE:
        /* Reconfigure the pins for GPIO mode.
         * LEFT: Output 0
         * RIGHT: Output 0
         * BOTTOM: Input with pull-up
         * TOP: Input without pull-up/pull-down
         */
        IOCON_configurePin(PIN_ROLF1_TOUCH_LEFT, NORMAL_PIN(0));
        IOCON_configurePin(PIN_ROLF1_TOUCH_RIGHT, NORMAL_PIN(0));
        IOCON_configurePin(PIN_ROLF1_TOUCH_BOTTOM, PULLUP_PIN(0));
        IOCON_configurePin(PIN_ROLF1_TOUCH_TOP, NORMAL_PIN(0));
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_LEFT, ENABLE);
        GPIO_writeBit(GPIO_ROLF1_TOUCH_LEFT, 0);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_RIGHT, ENABLE);
        GPIO_writeBit(GPIO_ROLF1_TOUCH_RIGHT, 0);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_BOTTOM, DISABLE);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_TOP, DISABLE);
        break;

    case TOUCH_PINMODE_MEASURE_X:
        /* X: Voltage gradient from left to right. Y: Measure.
         * LEFT: Output 0
         * RIGHT: Output 1
         * BOTTOM: ADC input without pull-up
         * TOP: Input without pull-up/pull-down
         */
        IOCON_configurePin(PIN_ROLF1_TOUCH_LEFT, NORMAL_PIN(0));
        IOCON_configurePin(PIN_ROLF1_TOUCH_RIGHT, NORMAL_PIN(0));
        IOCON_configurePin(PIN_ROLF1_TOUCH_BOTTOM, NORMAL_PIN(2));
        IOCON_configurePin(PIN_ROLF1_TOUCH_TOP, NORMAL_PIN(0));
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_LEFT, ENABLE);
        GPIO_writeBit(GPIO_ROLF1_TOUCH_LEFT, 0);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_RIGHT, ENABLE);
        GPIO_writeBit(GPIO_ROLF1_TOUCH_RIGHT, 1);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_BOTTOM, DISABLE);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_TOP, DISABLE);
        break;

    case TOUCH_PINMODE_MEASURE_Y:
        /* X: Measure.  Y: Voltage gradient from top to bottom.
         * LEFT: ADC input
         * RIGHT: Input without pull-up/pull-down
         * BOTTOM: Output 1
         * TOP: Output 0
         */
        IOCON_configurePin(PIN_ROLF1_TOUCH_LEFT, NORMAL_PIN(2));
        IOCON_configurePin(PIN_ROLF1_TOUCH_RIGHT, NORMAL_PIN(0));
        IOCON_configurePin(PIN_ROLF1_TOUCH_BOTTOM, NORMAL_PIN(0));
        IOCON_configurePin(PIN_ROLF1_TOUCH_TOP, NORMAL_PIN(0));
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_LEFT, DISABLE);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_RIGHT, DISABLE);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_BOTTOM, ENABLE);
        GPIO_writeBit(GPIO_ROLF1_TOUCH_BOTTOM, 1);
        GPIO_setDirBit(GPIO_ROLF1_TOUCH_TOP, ENABLE);
        GPIO_writeBit(GPIO_ROLF1_TOUCH_TOP, 0);
        break;
    }
}



osTimerDef(sampleTickDef, _BSP_TOUCH_osalCallback);


LPCLIB_Result BSP_TOUCH_open (void)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    if (BSP_getBoardType() == BSP_BOARD_ROLF1) {
        touch.sampleTick =                              /* Create touch sample timer */
            osTimerCreate(osTimer(sampleTickDef), osTimerOnce, (void *)TOUCH_TIMERMAGIC_SAMPLE);

        ADC_ioctl(adc, adcInstallHandler);              /* Install handler for ADC events */

        _BSP_TOUCH_enterState(TOUCH_STATE_IDLE);        /* Wait for touch event */
    }

    return result;
}



/* Send out a touch event if pen state has changed */
static void _BSP_TOUCH_reportPenState (void)
{
    LPCLIB_Event event;


    if ((touch.penCurrent.pen != touch.penLast.pen) ||
        (touch.penCurrent.pen &&
           ((touch.penCurrent.x   != touch.penLast.x  ) ||
            (touch.penCurrent.y   != touch.penLast.y  )))) {

        touch.penLast = touch.penCurrent;

        event.id = LPCLIB_EVENTID_APPLICATION;
        event.opcode = FIFISDR_EVENT_TOUCH;
        event.channel = touch.penLast.pen;
        event.parameter = (void *)((touch.penLast.x << 16) | (touch.penLast.y << 0));
        SYS_submitJob(event);
    }
}



/* Enter new state */
void _BSP_TOUCH_enterState (_TouchState state)
{
    touch.state = state;

    switch (state) {
    case TOUCH_STATE_IDLE:
        /* Prepare to detect touch event */
        _BSP_TOUCH_setPinMode(TOUCH_PINMODE_IDLE);

        /* Configure and enable touch interrupt (GPIO interrupt @ BOTTOM) */
        GPIO_ioctl(configEnableTouchInterrupt);
        break;

    case TOUCH_STATE_SCAN_X:
        /* Start measurement of x coordinate */
        _BSP_TOUCH_setPinMode(TOUCH_PINMODE_MEASURE_X);
        ADC_ioctl(adc, adcConvertX);
        break;

    case TOUCH_STATE_SCAN_Y:
        /* Start measurement of y coordinate */
        _BSP_TOUCH_setPinMode(TOUCH_PINMODE_MEASURE_Y);
        ADC_ioctl(adc, adcConvertY);
        break;

    case TOUCH_STATE_CHECK_PEN:
        _BSP_TOUCH_setPinMode(TOUCH_PINMODE_IDLE);
        osTimerStart(touch.sampleTick, TOUCH_SAMPLE_INTERVAL);
        break;
    }
}



static const int _touchTransform[2][3] = {
    {    0,  2597, -17},
    { 2097,     0, -32},
};



/* Handle events on process level. */
void BSP_TOUCH_worker (LPCLIB_Event event)
{
    if ((event.id != LPCLIB_EVENTID_APPLICATION) || (event.opcode != FIFISDR_EVENT_TOUCH_NEED_SERVICE)) {
        /* Nothing to do */
        return;
    }

    switch (touch.state) {
    case TOUCH_STATE_IDLE:
        if (event.block == TOUCH_SERVICE_FIRST_TOUCH) {
            _BSP_TOUCH_enterState(TOUCH_STATE_SCAN_X);
        }
        break;

    case TOUCH_STATE_SCAN_X:
        if (event.block == TOUCH_SERVICE_ADC) {
            touch.xRaw = (uint32_t)event.parameter;
            _BSP_TOUCH_enterState(TOUCH_STATE_SCAN_Y);
        }
        else {
            /* Unexpected event */
            _BSP_TOUCH_enterState(TOUCH_STATE_IDLE);
        }
        break;

    case TOUCH_STATE_SCAN_Y:
        if (event.block == TOUCH_SERVICE_ADC) {
            touch.yRaw = (uint32_t)event.parameter;
            _BSP_TOUCH_enterState(TOUCH_STATE_CHECK_PEN);
        }
        else {
            /* Unexpected event */
            _BSP_TOUCH_enterState(TOUCH_STATE_IDLE);
        }
        break;

    case TOUCH_STATE_CHECK_PEN:
        if (event.block == TOUCH_SERVICE_TIMER) {
            touch.penCurrent.pen = GPIO_readBit(GPIO_ROLF1_TOUCH_BOTTOM) == 0;
            touch.penCurrent.x = (touch.xRaw * _touchTransform[0][0] + touch.yRaw * _touchTransform[0][1]) / 65536;
            touch.penCurrent.x += _touchTransform[0][2];
            touch.penCurrent.y = (touch.xRaw * _touchTransform[1][0] + touch.yRaw * _touchTransform[1][1]) / 65536;
            touch.penCurrent.y += _touchTransform[1][2];
            _BSP_TOUCH_reportPenState();

            if (touch.penCurrent.pen == 0) {
                _BSP_TOUCH_enterState(TOUCH_STATE_IDLE);
            }
            else {
                _BSP_TOUCH_enterState(TOUCH_STATE_SCAN_X);
            }
        }
        break;
    }
}

