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


/** \addtogroup GPIO
 *  @{
 */

#include "lpc17xx_clkpwr.h"
#include "lpc17xx_gpio.h"



/** GPIO module context. */
static struct GPIO_Context {
    osMutexId accessMutex;
#if LPCLIB_GPIO_INTERRUPTS
    LPCLIB_Callback pinCallbacks[2];
#endif
} gpioContext;

osMutexDef(gpioAccessMutexDef);



void GPIO_open (void)
{
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_GPIO);
    gpioContext.accessMutex = osMutexCreate(osMutex(gpioAccessMutexDef));
}


void GPIO_setDir32 (GPIO_Port32 port, uint32_t value, uint32_t mask)
{
    if (osMutexWait(gpioContext.accessMutex, osWaitForever) == osOK) {
        __gpio[port & 3]->FIODIR32 = (__gpio[port & 3]->FIODIR32 & ~(~value & mask)) | (value & mask);

        osMutexRelease(gpioContext.accessMutex);
    }
}

void GPIO_setDirBit (GPIO_Pin pin, LPCLIB_Switch outputEnable)
{
    if (osMutexWait(gpioContext.accessMutex, osWaitForever) == osOK) {
        __gpio[(pin >> 0) & 7]->FIODIR32 = (__gpio[(pin >> 0) & 7]->FIODIR32 & ~(1u << ((pin >> 3) & 0x1F))) | (outputEnable << ((pin >> 3) & 0x1F));

        osMutexRelease(gpioContext.accessMutex);
    }
}


/* Set GPIO options. */
void GPIO_ioctl (const GPIO_Config *pConfig)
{
#if LPCLIB_GPIO_INTERRUPTS
    int port;
    uint32_t mask;
#endif


    switch (pConfig->opcode) {
#if LPCLIB_GPIO_INTERRUPTS
    case GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT:
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
        port = (pConfig->pinInterrupt.pin >> 0) & 3;
        mask = 1u << ((pConfig->pinInterrupt.pin >> 3) & 0x1F);

        if (port == 0) {
            if (pConfig->pinInterrupt.enable) {
                switch (pConfig->pinInterrupt.mode) {
                case GPIO_INT_FALLING_EDGE:
                    LPC_GPIOINT->IO0IntEnR &= ~mask;
                    LPC_GPIOINT->IO0IntEnF |=  mask;
                    break;
                case GPIO_INT_RISING_EDGE:
                    LPC_GPIOINT->IO0IntEnF &= ~mask;
                    LPC_GPIOINT->IO0IntEnR |=  mask;
                    break;
                case GPIO_INT_BOTH_EDGES:
                    LPC_GPIOINT->IO0IntEnF |=  mask;
                    LPC_GPIOINT->IO0IntEnR |=  mask;
                    break;
                }
            }
            else {
                LPC_GPIOINT->IO0IntEnF &= ~mask;
                LPC_GPIOINT->IO0IntEnR &= ~mask;
            }
        }

        if (port == 2) {
            if (pConfig->pinInterrupt.enable) {
                switch (pConfig->pinInterrupt.mode) {
                case GPIO_INT_FALLING_EDGE:
                    LPC_GPIOINT->IO2IntEnR &= ~mask;
                    LPC_GPIOINT->IO2IntEnF |=  mask;
                    break;
                case GPIO_INT_RISING_EDGE:
                    LPC_GPIOINT->IO2IntEnF &= ~mask;
                    LPC_GPIOINT->IO2IntEnR |=  mask;
                    break;
                case GPIO_INT_BOTH_EDGES:
                    LPC_GPIOINT->IO2IntEnF |=  mask;
                    LPC_GPIOINT->IO2IntEnR |=  mask;
                    break;
                }
            }
            else {
                LPC_GPIOINT->IO2IntEnF &= ~mask;
                LPC_GPIOINT->IO2IntEnR &= ~mask;
            }
        }

        if (port == 0) {
            gpioContext.pinCallbacks[0] = pConfig->pinInterrupt.callback;
        }
        if (port == 2) {
            gpioContext.pinCallbacks[1] = pConfig->pinInterrupt.callback;
        }
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
        port = (pConfig->pinInterrupt.pin >> 0) & 3;
        mask = 1u << ((pConfig->pinInterrupt.pin >> 3) & 0x1F);

        if (port == 0) {
            if (pConfig->pinInterrupt.enable) {
                switch (pConfig->pinInterrupt.mode) {
                case GPIO_INT_FALLING_EDGE:
                    LPC_GPIOINT->IO0IntEnR &= ~mask;
                    LPC_GPIOINT->IO0IntEnF |=  mask;
                    break;
                case GPIO_INT_RISING_EDGE:
                    LPC_GPIOINT->IO0IntEnF &= ~mask;
                    LPC_GPIOINT->IO0IntEnR |=  mask;
                    break;
                case GPIO_INT_BOTH_EDGES:
                    LPC_GPIOINT->IO0IntEnF |=  mask;
                    LPC_GPIOINT->IO0IntEnR |=  mask;
                    break;
                }
            }
            else {
                LPC_GPIOINT->IO0IntEnF &= ~mask;
                LPC_GPIOINT->IO0IntEnR &= ~mask;
            }
        }

        if (port == 2) {
            if (pConfig->pinInterrupt.enable) {
                switch (pConfig->pinInterrupt.mode) {
                case GPIO_INT_FALLING_EDGE:
                    LPC_GPIOINT->IO2IntEnR &= ~mask;
                    LPC_GPIOINT->IO2IntEnF |=  mask;
                    break;
                case GPIO_INT_RISING_EDGE:
                    LPC_GPIOINT->IO2IntEnF &= ~mask;
                    LPC_GPIOINT->IO2IntEnR |=  mask;
                    break;
                case GPIO_INT_BOTH_EDGES:
                    LPC_GPIOINT->IO2IntEnF |=  mask;
                    LPC_GPIOINT->IO2IntEnR |=  mask;
                    break;
                }
            }
            else {
                LPC_GPIOINT->IO2IntEnF &= ~mask;
                LPC_GPIOINT->IO2IntEnR &= ~mask;
            }
        }

        if (port == 0) {
            gpioContext.pinCallbacks[0] = pConfig->pinInterrupt.callback;
        }
        if (port == 2) {
            gpioContext.pinCallbacks[1] = pConfig->pinInterrupt.callback;
        }
#endif
        break;
#endif

    default:
        break;
    }
}



#if LPCLIB_GPIO_INTERRUPTS

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
//TODO Check for External interrupt!
void EINT3_IRQHandler (void)
#elif LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
void GPIO_IRQHandler (void)
#endif
{
    LPCLIB_Event event;
    int index = 0;
    int channel;
    GPIO_InterruptMode mode;

    /* Determine interrupt source */
    channel = __CLZ(LPC_GPIOINT->IO0IntStatR);
    mode = GPIO_INT_RISING_EDGE;
    if (channel == 32) {
        channel = __CLZ(LPC_GPIOINT->IO0IntStatF);
        mode = GPIO_INT_FALLING_EDGE;
        if (channel == 32) {
            index = 1;
            channel = __CLZ(LPC_GPIOINT->IO2IntStatR);
            mode = GPIO_INT_RISING_EDGE;
            if (channel == 32) {
                channel = __CLZ(LPC_GPIOINT->IO2IntStatF);
                mode = GPIO_INT_FALLING_EDGE;
            }
        }
    }

    if (channel == 32) {
        LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
        LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
    }
    else {
        channel = 31 - channel;

        if (gpioContext.pinCallbacks[index]) {
            event.id = LPCLIB_EVENTID_GPIO;
            event.block = (index == 0) ? 0 : 2;
            event.channel = (channel << 3) | (event.block << 0);    /* Format: enum GPIO_Pin */
            event.parameter = (void *)mode;
            gpioContext.pinCallbacks[index](event);
        }

        if (index == 0) {
            LPC_GPIOINT->IO0IntClr = (1u << channel);
        }
        else {
            LPC_GPIOINT->IO2IntClr = (1u << channel);
        }
    }
}
#endif

/** @} */

