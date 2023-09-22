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

#if LPCLIB_EMC

/** \file
 *  \brief EMC driver implementation.
 *
 *  This file contains the driver code for the EMC peripheral.
 *
 *  \author NXP Semiconductors
 */


#include "lpc17xx_emc.h"
#include "lpc17xx_clkpwr.h"


/** Field definition for hardware register EMCControl. */
LPCLIB_DefineRegBit(EMC_CONTROL_ENABLE,         0,  1);
LPCLIB_DefineRegBit(EMC_CONTROL_E,              0,  1);
LPCLIB_DefineRegBit(EMC_CONTROL_ADDRESS_MIRROR, 1,  1);
LPCLIB_DefineRegBit(EMC_CONTROL_M,              1,  1);
LPCLIB_DefineRegBit(EMC_CONTROL_LOW_POWER,      2,  1);
LPCLIB_DefineRegBit(EMC_CONTROL_L,              2,  1);

/** Field definition for hardware register EMCCLKDLY. */
LPCLIB_DefineRegBit(EMC_DLYCTL_CMDDLY,          0,  5);
LPCLIB_DefineRegBit(EMC_DLYCTL_FBCLKDLY,        8,  5);
LPCLIB_DefineRegBit(EMC_DLYCTL_CLKOUT0DLY,      16, 5);
LPCLIB_DefineRegBit(EMC_DLYCTL_CLKOUT1DLY,      24, 5);


/** Field definition for hardware register EMCDynamicControl. */
enum {
    EMC_DYNCONTROL_CE_NORMAL = (0u << 0),
    EMC_DYNCONTROL_CE_CONTINUOUS = (1u << 0),
    EMC_DYNCONTROL_CS_NORMAL = (0u << 1),
    EMC_DYNCONTROL_CS_CONTINUOUS = (1u << 1),
    EMC_DYNCONTROL_SR_DISABLE = (0u << 2),
    EMC_DYNCONTROL_SR_ENABLE = (1u << 2),
    EMC_DYNCONTROL_MMC_ENABLE = (0u << 5),
    EMC_DYNCONTROL_MMC_DISABLE = (1u << 5),
    EMC_DYNCONTROL_I_NORMAL = (0u << 7),
    EMC_DYNCONTROL_I_MODE = (1u << 7),
    EMC_DYNCONTROL_I_PALL = (2u << 7),
    EMC_DYNCONTROL_I_NOP = (3u << 7),
    EMC_DYNCONTROL_DP_NORMAL = (0u << 13),
    EMC_DYNCONTROL_DP_DEEPSLEEP = (1u << 13),
};

/** Field definition for hardware register EMCDynamicReadConfig. */
enum {
    EMC_DYNREADCONFIG_CLOCKOUT_DELAYED = 0,
    EMC_DYNREADCONFIG_COMMAND_DELAYED = 1,
    EMC_DYNREADCONFIG_COMMAND_DELAYED_PLUS1 = 2,
    EMC_DYNREADCONFIG_COMMAND_DELAYED_PLUS2 = 3,
};


/* Base address of dynamic memory bank n */
#define EMC_SDRAM_BASE_ADDRESS(n)           ((0x0A + n) << 28)


/** Local device context. */
static struct EMC_Context {
    _Bool halfRate;                         /**< EMC runs on half the CCLK frequency */
} emcContext;



/* Open the EMC. */
LPCLIB_Result EMC_open (EMC_Handle *pHandle)
{
    uint32_t emcClock;

    if (*pHandle) {
        return LPCLIB_BUSY;
    }

    CLKPWR_deassertPeripheralReset(CLKPWR_RESET_EMC);   /* Release EMC reset */
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_EMC);         /* Enable EMC clock */

    /* Check whether we can use full CCLK speed */
    emcContext.halfRate = false;
    emcClock = CLKPWR_getBusClock(CLKPWR_CLOCK_CPU);
    if (emcClock > 80000000ul) {
        emcContext.halfRate = true;
        emcClock /= 2;
        CLKPWR_setDivider(CLKPWR_DIVIDER_EMC, CLKPWR_RATIO_2);
    }
    else {
        CLKPWR_setDivider(CLKPWR_DIVIDER_EMC, CLKPWR_RATIO_1);
    }

    LPC_EMC->Control = EMC_CONTROL_ENABLE_Msk;          /* Disable address mirror, enable EMC */

    /* Auto select SDRAM read strategy, depending on bus clock frequency */
    if (emcClock <= 48000000ul) {
        LPC_SC->EMCDLYCTL =
                (31u << EMC_DLYCTL_CLKOUT1DLY_Pos)
              | (31u << EMC_DLYCTL_CLKOUT0DLY_Pos)
              | (15u << EMC_DLYCTL_FBCLKDLY_Pos)
              | ( 0  << EMC_DLYCTL_CMDDLY_Pos);
        LPC_EMC->DynamicReadConfig = EMC_DYNREADCONFIG_CLOCKOUT_DELAYED;
    }
    else {
        LPC_SC->EMCDLYCTL =
                ( 0  << EMC_DLYCTL_CLKOUT1DLY_Pos)
              | ( 0  << EMC_DLYCTL_CLKOUT0DLY_Pos)
              | (20u << EMC_DLYCTL_FBCLKDLY_Pos)
              | (16u << EMC_DLYCTL_CMDDLY_Pos);
        LPC_EMC->DynamicReadConfig = EMC_DYNREADCONFIG_COMMAND_DELAYED;
    }
    *pHandle = &emcContext;                             /* Return handle */

    return LPCLIB_SUCCESS;
}



#if LPCLIB_EMC_CLOSE
/* Close the EMC. */
LPCLIB_Result EMC_close (EMC_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPC_EMC->Control = 0;                               /* Disable the EMC */
    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_EMC);        /* Disable the clock */

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}
#endif


/* Configure the EMC block. */
void EMC_ioctl (EMC_Handle handle, const EMC_Config *pConfig)
{
    LPC_EMC_TypeDef * const emc = LPC_EMC;
    int n;
    int i;
    uint32_t temp;
    int modeReg;
    int offset;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    while (pConfig->opcode != EMC_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case EMC_OPCODE_DYNAMIC_GLOBALS:                    /* Dynamic memory timings (all devices) */
            emc->DynamicRefresh = pConfig->dynTimings.refreshCycles;
            emc->DynamicRP = pConfig->dynTimings.trp;
            emc->DynamicRAS = pConfig->dynTimings.tras;
            emc->DynamicSREX = pConfig->dynTimings.tsrex;
            emc->DynamicAPR = pConfig->dynTimings.tapr;
            emc->DynamicDAL = pConfig->dynTimings.tdal;
            emc->DynamicWR = pConfig->dynTimings.twr;
            emc->DynamicRC = pConfig->dynTimings.trc;
            emc->DynamicRFC = pConfig->dynTimings.trfc;
            emc->DynamicXSR = pConfig->dynTimings.txsr;
            emc->DynamicRRD = pConfig->dynTimings.trrd;
            emc->DynamicMRD = pConfig->dynTimings.tmrd;
            break;

        case EMC_OPCODE_DYNAMIC_DEVICE:                     /* Configure an SDRAM device */
            n = (int)(pConfig->dynDevice.device);
            emc->D[n].DynamicRasCas = (pConfig->dynDevice.ras << 0) |
                                      (pConfig->dynDevice.cas << 8);
            emc->D[n].DynamicConfig =
                (pConfig->dynDevice.busWidth << 14) |
                (pConfig->dynDevice.lowPower << 12) |
                ((pConfig->dynDevice.org & 0xFF) << 7) |
                (pConfig->dynDevice.lowPower << 3);
            emc->DynamicControl =
                EMC_DYNCONTROL_I_NOP |
                EMC_DYNCONTROL_CS_CONTINUOUS |
                EMC_DYNCONTROL_CE_CONTINUOUS;
            for (i = 0; i < 10000; i++);     //TODO (min. 100us)

            emc->DynamicControl =
                EMC_DYNCONTROL_I_PALL |
                EMC_DYNCONTROL_CS_CONTINUOUS |
                EMC_DYNCONTROL_CE_CONTINUOUS;
            temp = emc->DynamicRefresh;
            emc->DynamicRefresh = 1;
            for (i = 0; i < 10000; i++);       //TODO
            emc->DynamicRefresh = temp;

            emc->DynamicControl =
                EMC_DYNCONTROL_I_MODE |
                EMC_DYNCONTROL_CS_CONTINUOUS |
                EMC_DYNCONTROL_CE_CONTINUOUS;

            /* Determine content of SDRAM's mode register.
             * Default: 32-bit bus: 4 read cycles/burst.
             * Use same CAS delay as programmed in EMC.
             */
            modeReg = 0x02 | ((pConfig->dynDevice.cas - 0) << 4);
            if (pConfig->dynDevice.busWidth == EMC_DYNAMIC_BUS_16) {
                modeReg |= 1;                       /* 16-bit bus: 8 read cycles/burst */
            }

            /* Determine the dummy read address for mode register programming. */
            offset = 1 + ((pConfig->dynDevice.org >> 8) & 0x0F);
            if (pConfig->dynDevice.busWidth == EMC_DYNAMIC_BUS_32) {
                ++offset;                           /* 32-bit bus: one extra shift */
            }
            if (!pConfig->dynDevice.lowPower) {
                ++offset;                           /* Low power mode, 2 banks: one extra shift */
                if (pConfig->dynDevice.org & (7u << 2)) {
                    ++offset;                       /* Another shift for 4 banks */
                }
            }
            i = *((volatile uint32_t *)(EMC_SDRAM_BASE_ADDRESS(n) + (modeReg << offset)));  /* Set mode register! */

            emc->DynamicControl =
                EMC_DYNCONTROL_I_NORMAL |
                EMC_DYNCONTROL_CS_NORMAL |
                EMC_DYNCONTROL_CE_NORMAL;
            emc->D[n].DynamicConfig |= (1u << 19);      /* Enable buffering */
            break;

        case EMC_OPCODE_STATIC_DEVICE:
            emc->S[pConfig->staticDevice.device].StaticConfig =
                pConfig->staticDevice.busWidth |
                pConfig->staticDevice.multi;

            temp = pConfig->staticDevice.writePulseDelay;
            if (temp == 0) {
                temp = 1;
            }
            emc->S[pConfig->staticDevice.device].StaticWaitWen = temp - 1;

            temp = temp + pConfig->staticDevice.writePulseWidth;
            emc->S[pConfig->staticDevice.device].StaticWaitWr = temp - 2;

            temp = pConfig->staticDevice.readCycleWidth;
            if (temp == 0) {
                temp = 1;
            }
            emc->S[pConfig->staticDevice.device].StaticWaitRd = temp - 1;
            temp = pConfig->staticDevice.readCycleWidthPage;
            if (temp == 0) {
                temp = 1;
            }
            emc->S[pConfig->staticDevice.device].StaticWaitPage = temp - 1;
            emc->S[pConfig->staticDevice.device].StaticWaitOen =
                pConfig->staticDevice.readOeDelay;
            break;

        case EMC_OPCODE_INVALID:
            /* ignore */
            break;
        }

        ++pConfig;
    }
}


/** @} */

/** @} addtogroup EMC */

#endif  /* #ifdef LPCLIB_EMC */

