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



#include <stdint.h>

#include "lpclib.h"
#include "i2cdev.h"
#include "si570.h"


/** Local device context. */
typedef struct SI570_Context {
    I2C_Handle bus;                         /**< Bus to which device is connected. */
    uint8_t slaveAddress;                   /**< Slave address on I2C bus. */

    uint32_t currentFrequency1121;          /**< Current frequency (11.21). */
    uint32_t anchorFrequency;               /**< Si570 anchor frequency. Real frequency of Si570.
                                             *   May differ from the frequency of the host
                                             *   by the automatically selected prescaler.
                                             */
    uint8_t anchorHsDiv;                    /**< Value of HS_DIV at anchor frequency */
    uint8_t anchorN1;                       /**< Value of N1 at anchor frequency */
    uint8_t regs[6];                        /**< Current content of real Si570 registers 7...12 */
    uint8_t factoryDefaultRegs[6];          /**< Content of Si570 registers after power-up
                                             *   (factory defaults)
                                             */
    uint32_t freqXtal824;                   /**< Crystal frequency (calibration!) (8.24) */
    uint32_t smoothtune;                    /**< Smoothtune range (ppm) */
} SI570_Context;


/** Register names. */
enum SI570_RegisterNames {
    SI570_REG_HSDIV_N1 = 7,
    SI570_REG_N1_RFREQ4 = 8,
    SI570_REG_RFREQ3 = 9,
    SI570_REG_RFREQ2 = 10,
    SI570_REG_RFREQ1 = 11,
    SI570_REG_RFREQ0 = 12,
    SI570_REG_RESET_FREEZE = 135,
    SI570_REG_FREEZEDCO = 137,
};

LPCLIB_DefineRegBit(SI570_RESET_FREEZE_RECALL,  0,  1);
LPCLIB_DefineRegBit(SI570_RESET_FREEZE_FREEZEM, 4,  1);
LPCLIB_DefineRegBit(SI570_RESET_FREEZE_NEWFREQ, 6,  1);
LPCLIB_DefineRegBit(SI570_RESET_FREEZE_RST_REG, 7,  1);

LPCLIB_DefineRegBit(SI570_FREEZEDCO_FREEZEDCO,  4,  1);



static SI570_Context globalContext; //TODO: Must allocate memory dynamically!!



/** Return the optimum values for HS_DIV and N1.
 *
 *  For power reasons it is desirable to have the lowest possible DCO
 *  frequency within its operating range of 4850...5670 MHz. For the same
 *  reason, HS_DIV should be as large as possible.
 *
 *  \param[in] freq Frequency (format 11.21)
 *  \param[out] hsdiv Pointer to HS_DIV value (4, 5, 6, 7, 9, 11)
 *  \param[out] n1 Pointer to N1 value (1...128)
 *  \param[in] check_limits Check limitations of real Si570 device
 *
 *  \retval true ok
 *  \retval false freq not allowed
 */
static _Bool SI570_findHsdivN1 (uint32_t freq, uint32_t *hsdiv, uint32_t *n1, bool check_limits)
{
    const uint8_t hsdiv_val[6] = {11, 9, 7, 6, 5, 4};
    int i, n;


#define DCO_MIN _43_21(4850.0)
#define DCO_MAX _43_21(5670.0)

    /* Check limits of real device. Ignore for virtual device. */
    if (check_limits) {
        /* Handle the special frequency bands first.
        * 970 MHz <= freq <= 1134 MHz       --> HS_DIV = 5, N1 = 1
        * 1213 MHz <= freq <= 1417.5 MHz    --> HS_DIV = 4, N1 = 1
        */
        if ((_11_21(970.0) <= freq) && (freq <= _11_21(1134.0))) {
            *hsdiv = 5;
            *n1 = 1;
            return true;
        }

        if ((_11_21(1213.0) <= freq) && (freq <= _11_21(1417.5))) {
            *hsdiv = 5;
            *n1 = 1;
            return true;
        }

        /* Check valid frequency range */
        if ((freq < _11_21(10.0)) || (freq > SI570_MAX_FREQUENCY))
            return false;
    }

    /* Check with all six possible HS_DIV values. Start with the large values,
     * and stop at the first match.
     */
    for (i = 0; i <= 5; i++) {
        /* Check with all possible N1 dividers. */
        for (n = 1; n <= 128; n++) {
            if ((n % 2) && (n > 1))                     /* Only 1 or even number allowed! */
                continue;

            if( (n * hsdiv_val[i]) * (uint64_t)freq >= DCO_MIN) {
                if((n * hsdiv_val[i]) * (uint64_t)freq <= DCO_MAX) {
                    *hsdiv = hsdiv_val[i];
                    *n1 = n;

                    return true;                        /* Got it! */
                }
            }
        }
    }

    /* There is no divider setting that allows the DCO to run within its operating limits. */

    return false;
}



/** Read one or more consecutive registers from Si570.
 *
 *  \param handle Device handle
 *  \param firstReg Number of first register
 *  \param lastReg Number of last register
 *  \param pValue Points to buffer that will receive register content
 *  \retval LPCLIB_SUCCESS ok
 */
static LPCLIB_Result SI570_readRegs (SI570_Handle handle,
                                     enum SI570_RegisterNames firstReg,
                                     enum SI570_RegisterNames lastReg,
                                     uint8_t *pValue)
{
    *pValue = 0;
    return I2CDEV_writeAndRead(handle->bus,
                               handle->slaveAddress,
                               1, &firstReg,
                               1 + (lastReg - firstReg),
                               pValue);
}



/** Write to one or more Si570 registers.
 *
 *  \param handle Device handle
 *  \param firstReg Number of first register
 *  \param lastReg Number of last register
 *  \param pValue Points to buffer that hold new register content
 *  \retval LPCLIB_SUCCESS ok
 */
static LPCLIB_Result SI570_writeRegs (SI570_Handle handle,
                                     enum SI570_RegisterNames firstReg,
                                     enum SI570_RegisterNames lastReg,
                                     const uint8_t *pValue)
{
    uint8_t buffer [7];
    uint8_t numRegs;
    int i;


    numRegs = 1 + (lastReg - firstReg);
    if (numRegs > 6) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    buffer [0] = firstReg;
    for (i = 0; i < numRegs; i++) {
        buffer[1 + i] = pValue[i];
    }

    return I2CDEV_write(handle->bus, handle->slaveAddress, 1 + 1 + (lastReg - firstReg), buffer);
}



/** Write to a single Si570 register.
 *
 *  \param handle Device handle
 *  \param theReg Number of register
 *  \param value new register content
 *  \retval LPCLIB_SUCCESS ok
 */
static LPCLIB_Result SI570_writeReg (SI570_Handle handle,
                                    enum SI570_RegisterNames theReg,
                                    uint8_t value)
{
    return SI570_writeRegs(handle, theReg, theReg, &value);
}



/** Set a new frequency in Si570.
 *
 *  Calculates the required register settings for the Si570.
 *
 *  \param[in] handle Device handle
 *  \param[in] virtualFrequency Frequency (format 11.21)
 *  \retval true ok
 *  \retval false error, or illegal parameter
 */
static _Bool SI570_setToNewFrequency (SI570_Handle handle, uint32_t frequency1121)
{
    uint32_t hs_div;
    uint32_t n1;
    uint64_t rfreq;
    int32_t delta_f;
    uint8_t txbuf[6];
    uint8_t regValue;
    bool smooth;


    /* Find out if a combination of post dividers exists for the requested *real* frequency. */
    if (!SI570_findHsdivN1(frequency1121, &hs_div, &n1, true))
        return false;

    handle->currentFrequency1121 = frequency1121;

    /* RFREQ */
    rfreq = frequency1121;
    rfreq <<= 23;
    rfreq *= hs_div * n1;
    rfreq /= (handle->freqXtal824 >> 8);  //TODO Auflösung!

    /* Prepare message for I2C bus */
    txbuf[0] = (((hs_div-4) & 0x07) << 5) | (((n1-1) >> 2) & 0x1F); // Reg 7
    txbuf[1] = (((n1-1) & 0x03) << 6) | ((rfreq >> 32) & 0x3F);     // Reg 8
    txbuf[2] = ((rfreq >> 24) & 0xFF);                              // Reg 9
    txbuf[3] = ((rfreq >> 16) & 0xFF);                              // Reg 10
    txbuf[4] = ((rfreq >> 8) & 0xFF);                               // Reg 11
    txbuf[5] = (rfreq & 0xFF);                                      // Reg 12

    /* Check if we can use "smooth tuning" */
    delta_f = frequency1121 - (int32_t)handle->anchorFrequency;
    if (delta_f < 0) {
        delta_f = -delta_f;
    }
    /* Note: 'ppm' requires to divide by 1000000. It's easier to divide
     *       by 1048576 (shift 20 bits) though. This reduces the range for
     *       smooth tuning by 5%.
     */
    smooth =
        delta_f <= ((handle->smoothtune * (int64_t)handle->anchorFrequency) >> 20);
    /* Smooth tuning is only possible if HS_DIV and N1 do not change. */
    smooth = smooth && (hs_div == handle->anchorHsDiv) && (n1 == handle->anchorN1);

    /* Smooth tuning requires to update the RFREQ registers only.
     * A larger frequency step requires to freeze the oscillator first. In this
     * case the frequency becomes the new anchor.
     */
    if (smooth) {
        SI570_readRegs(handle, SI570_REG_RESET_FREEZE, SI570_REG_RESET_FREEZE, &regValue);
        regValue |= SI570_RESET_FREEZE_FREEZEM_Msk;
        SI570_writeReg(handle, SI570_REG_RESET_FREEZE, regValue);   /* Freeze M = 1 */
        SI570_writeRegs(handle, SI570_REG_N1_RFREQ4, SI570_REG_RFREQ0, &txbuf[1]);
        SI570_readRegs(handle, SI570_REG_RESET_FREEZE, SI570_REG_RESET_FREEZE, &regValue);
        regValue &= ~SI570_RESET_FREEZE_FREEZEM_Msk;
        SI570_writeReg(handle, SI570_REG_RESET_FREEZE, regValue);   /* Freeze M = 0 */
    }
    else {
        SI570_readRegs(handle, SI570_REG_FREEZEDCO, SI570_REG_FREEZEDCO, &regValue);
        regValue |= SI570_FREEZEDCO_FREEZEDCO_Msk;
        SI570_writeReg(handle, SI570_REG_FREEZEDCO, regValue);      /* Freeze */
        SI570_writeRegs(handle, SI570_REG_HSDIV_N1, SI570_REG_RFREQ0, &txbuf[0]);
        SI570_readRegs(handle, SI570_REG_FREEZEDCO, SI570_REG_FREEZEDCO, &regValue);
        regValue &= ~SI570_FREEZEDCO_FREEZEDCO_Msk;
        SI570_writeReg(handle, SI570_REG_FREEZEDCO, regValue);      /* Unfreeze */
        SI570_writeReg(handle, SI570_REG_RESET_FREEZE, SI570_RESET_FREEZE_NEWFREQ_Msk);

        handle->anchorFrequency = frequency1121;
        handle->anchorHsDiv = hs_div;
        handle->anchorN1 = n1;
    }

    /* Read back current registers */
    SI570_readRegs(handle, SI570_REG_HSDIV_N1, SI570_REG_RFREQ0, &handle->regs[0]);

    return true;
}



/* Get access to the device. */
LPCLIB_Result SI570_open (I2C_Handle bus, const uint8_t address, SI570_Handle *pHandle)
{
    SI570_Context *pContext = &globalContext; //TODO!


    pContext->bus = bus;
    pContext->slaveAddress = address;
    pContext->currentFrequency1121 = 0;                 /* This forces a full register load */
                                                        /* on first frequency set */
    pContext->anchorFrequency = 0;
    pContext->anchorHsDiv = 0;
    pContext->anchorN1 = 0;
    pContext->freqXtal824 = _8_24(114.285);             /* The nominal XTAL frequency */
    pContext->smoothtune = 3500;                        /* Max possible smooth tune range */

    *pHandle = pContext;

    /* Reset device to factory defaults (uses the RECALL mechanism) */
    SI570_writeReg(pContext, SI570_REG_RESET_FREEZE, SI570_RESET_FREEZE_RECALL_Msk);

    /* Read factory defaults */
    SI570_readRegs(pContext, SI570_REG_HSDIV_N1, SI570_REG_RFREQ0, &pContext->factoryDefaultRegs[0]);
    SI570_readRegs(pContext, SI570_REG_HSDIV_N1, SI570_REG_RFREQ0, &pContext->regs[0]);

    return LPCLIB_SUCCESS;
}



/* Close the device. */
LPCLIB_Result SI570_close (SI570_Handle *pHandle)
{
    if (pHandle == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    (*pHandle) = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Configure the device. */
LPCLIB_Result SI570_ioctl (SI570_Handle handle, const SI570_Config *pConfig)
{
    int i;

    switch (pConfig->opcode) {
    case SI570_OPCODE_SET_FREQUENCY:
        SI570_setToNewFrequency(handle, pConfig->frequency1121);
        break;

    case SI570_OPCODE_GET_REGISTERS:
        if (SI570_readRegs(handle, SI570_REG_HSDIV_N1, SI570_REG_RFREQ0, &handle->regs[0]) != LPCLIB_SUCCESS) {
            return LPCLIB_ERROR;
        }
        for (i = 0; i < 6; i++) {
            (*pConfig->pRegs)[i] = handle->regs[i];
        }
        break;

    case SI570_OPCODE_GET_FACTORY_STARTUP_REGISTERS:
        for (i = 0; i < 6; i++) {
            (*pConfig->pRegs)[i] = handle->factoryDefaultRegs[i];
        }
        break;

    case SI570_OPCODE_SET_XTAL_FREQUENCY:
        handle->freqXtal824 = pConfig->frequency824;
        SI570_setToNewFrequency(handle, handle->currentFrequency1121);
        break;

    case SI570_OPCODE_SET_SMOOTHTUNE:
        handle->smoothtune = pConfig->smoothtune;
        break;

    case SI570_OPCODE_FLUSH_SMOOTHTUNE:
        /* Forget anchor frequency, and force full register rewrite for next tune */
        handle->anchorFrequency = 0;
        handle->anchorHsDiv = 0;
        handle->anchorN1 = 0;
        break;
    }

    return LPCLIB_SUCCESS;
}



/* Set a new frequency. */
LPCLIB_Result SI570_write (SI570_Handle handle, uint32_t frequency1121)
{
    if (!SI570_setToNewFrequency(handle, frequency1121)) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    return LPCLIB_SUCCESS;
}



/* Read current frequency. */
LPCLIB_Result SI570_read (SI570_Handle handle, uint32_t *pFrequency1121)
{
    *pFrequency1121 = handle->currentFrequency1121;

    return LPCLIB_SUCCESS;
}



/* Return register set for a given frequency (debug function). */
void SI570_calcVirtualRegisters (SI570_Handle handle,
                                 uint32_t frequency1121,
                                 uint8_t (*pRegs)[6])
{
    uint32_t hs_div = 11;
    uint32_t n1 = 128;
    uint64_t rfreq;


    /* Calculate HS_DIV and N1.
     * NOTE: We ignore the result of the function call, and calculate a result even
     *       if the desired frequency is too low to produce a valid DCO frequency.
     */
    SI570_findHsdivN1(frequency1121, &hs_div, &n1, false);

    /* Calculate multiplier for virtual registers */
    rfreq = frequency1121;
    rfreq <<= 23;
    rfreq *= hs_div * n1;
    rfreq /= (handle->freqXtal824 >> 8);  //TODO Auflösung!

    /* Build the virtual register set */
    (*pRegs)[0] = (((hs_div-4) & 7) << 5) | (((n1-1) >> 2) & 0x1F); /* Reg 7 */
    (*pRegs)[1] = (((n1-1) & 0x03) << 6) | ((rfreq >> 32) & 0x3F);  /* Reg 8 */
    (*pRegs)[2] = ((rfreq >> 24) & 0xFF);                           /* Reg 9 */
    (*pRegs)[3] = ((rfreq >> 16) & 0xFF);                           /* Reg 10 */
    (*pRegs)[4] = ((rfreq >> 8) & 0xFF);                            /* Reg 11 */
    (*pRegs)[5] = (rfreq & 0xFF);                                   /* Reg 12 */
}

