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
 *  \brief CLKPWR driver implementation.
 *
 *  This file contains the implementation of the CLKPWR block driver.
 *
 *  \author NXP Semiconductors
 */

/** \addtogroup CLKPWR
 *  @{
 */

#include "lpc17xx_libconfig.h"
#include "lpc17xx_clkpwr.h"


LPCLIB_DefineRegBit(SC_SCS_OSCRANGE,            4,  1);
LPCLIB_DefineRegBit(SC_SCS_OSCEN,               5,  1);
LPCLIB_DefineRegBit(SC_SCS_OSCSTAT,             6,  1);

LPCLIB_DefineRegBit(SC_PCON_PM0,                0,  1);
LPCLIB_DefineRegBit(SC_PCON_PM1,                1,  1);
LPCLIB_DefineRegBit(SC_PCON_BODRPM,             2,  1);
LPCLIB_DefineRegBit(SC_PCON_BOGD,               3,  1);
LPCLIB_DefineRegBit(SC_PCON_BORD,               4,  1);
LPCLIB_DefineRegBit(SC_PCON_SMFLAG,             8,  1);
LPCLIB_DefineRegBit(SC_PCON_DSFLAG,             9,  1);
LPCLIB_DefineRegBit(SC_PCON_PDFLAG,             10, 1);
LPCLIB_DefineRegBit(SC_PCON_DPDFLAG,            11, 1);

LPCLIB_DefineRegBit(SC_CCLKSEL_CCLKDIV,         0,  5);
LPCLIB_DefineRegBit(SC_CCLKSEL_CCLKSEL,         8,  1);

LPCLIB_DefineRegBit(SC_EMCCLKSEL_EMCDIV,        0,  1);

LPCLIB_DefineRegBit(SC_FLASHCFG_FLASHTIM,       12, 4);

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
LPCLIB_DefineRegBit(SC_PLL0CON_PLLE0,           0,  1);
LPCLIB_DefineRegBit(SC_PLL0CON_PLLC0,           1,  1);

LPCLIB_DefineRegBit(SC_PLL0CFG_MSEL0,           0,  15);
LPCLIB_DefineRegBit(SC_PLL0CFG_NSEL0,           16, 8);

LPCLIB_DefineRegBit(SC_PLL0STAT_MSEL0,          0,  15);
LPCLIB_DefineRegBit(SC_PLL0STAT_NSEL0,          16, 8);
LPCLIB_DefineRegBit(SC_PLL0STAT_PLLE0_STAT,     24, 1);
LPCLIB_DefineRegBit(SC_PLL0STAT_PLLC0_STAT,     25, 1);
LPCLIB_DefineRegBit(SC_PLL0STAT_PLOCK0,         26, 1);
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
LPCLIB_DefineRegBit(SC_PLL0CON_PLLE,            0,  1);

LPCLIB_DefineRegBit(SC_PLL0CFG_MSEL,            0,  5);
LPCLIB_DefineRegBit(SC_PLL0CFG_PSEL,            5,  2);

LPCLIB_DefineRegBit(SC_PLL0STAT_MSEL,           0,  5);
LPCLIB_DefineRegBit(SC_PLL0STAT_PSEL,           5,  2);
LPCLIB_DefineRegBit(SC_PLL0STAT_PLLE_STAT,      8,  1);
LPCLIB_DefineRegBit(SC_PLL0STAT_PLOCK,          10, 1);

LPCLIB_DefineRegBit(SC_PLL1STAT_MSEL,           0,  5);
LPCLIB_DefineRegBit(SC_PLL1STAT_PSEL,           5,  2);
LPCLIB_DefineRegBit(SC_PLL1STAT_PLLE_STAT,      8,  1);
LPCLIB_DefineRegBit(SC_PLL1STAT_PLOCK,          10, 1);

LPCLIB_DefineRegBit(SC_PBOOST_BOOST,            0,  2);

enum {
    SC_PBOOST_BOOST_OFF = 0,
    SC_PBOOST_BOOST_ON = 3,
};
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
#define IRC_FREQUENCY                       (4000000ul)
#define PLL_M_MIN                           (6)
#define PLL_M_MAX                           (512)
#define PLL_N_MIN                           (1)
#define PLL_N_MAX                           (31)
#define PLL_F_MIN                           (288000ul)
#define PLL_F_MAX                           (520000ul)
#define CCLK_DIV_MAX                        (256)
#define CCLK_MAX                            (120000000ul)
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
#define IRC_FREQUENCY                       (12000000ul)
#define PLL_M_MIN                           (1)
#define PLL_M_MAX                           (31)
#define PLL_N_MIN                           (0)
#define PLL_N_MAX                           (3)
#define PLL_F_MIN                           (156000ul)
#define PLL_F_MAX                           (320000ul)
#define CCLK_DIV_MAX                        (31)
#define CCLK_MAX                            (120000000ul)
#endif



/** \addtogroup CLKPWR_Public_Functions
 *  @{
 */


/** Return the clock frequency (in Hz) of an internal bus.
 *
 *  \param[in] bus The bus to be queried
 */
uint32_t CLKPWR_getBusClock (CLKPWR_Clock clock)
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    const uint32_t pclkselRatios[4] = {4, 1, 2, 8};
    uint32_t divider = 1;


    if (clock != CLKPWR_CLOCK_CPU) {
        divider = (LPC_SC->PCLKSEL[clock / 16] >> (2 * (clock % 16))) & 0x03;
        divider = pclkselRatios[divider];

        if ((clock == CLKPWR_CLOCK_CAN1) ||
            (clock == CLKPWR_CLOCK_CAN2) ||
            (clock == CLKPWR_CLOCK_ACF)) {
            if (divider == 8) {
                divider = 6;
            }
        }
    }

    return (SystemCoreClock / divider);
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    uint32_t divider = 1;
    uint32_t multiplier = 1;


    if (clock == CLKPWR_CLOCK_EMC) {
        divider = 1u << ((LPC_SC->EMCCLKSEL & SC_EMCCLKSEL_EMCDIV_Msk) >> SC_EMCCLKSEL_EMCDIV_Pos);
    }
    else if ((clock != CLKPWR_CLOCK_CPU) && (clock != CLKPWR_CLOCK_I2S)) {
        multiplier = (LPC_SC->CCLKSEL & SC_CCLKSEL_CCLKDIV_Msk) >> SC_CCLKSEL_CCLKDIV_Pos;
        divider = LPC_SC->PCLKSEL & 0x1F;
    }

    return ((SystemCoreClock * multiplier) / divider);
#endif
}



/* Reset all peripherals. */
void CLKPWR_resetAllPeripherals (void)
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    LPC_SC->RSTCON0 = 0xFFFEFFFF;
    LPC_SC->RSTCON1 = 0x00000007;
    LPC_SC->RSTCON0 = 0;
    LPC_SC->RSTCON1 = 0;
#endif
}



/** Set a clock divider.
 */
LPCLIB_Result CLKPWR_setDivider (CLKPWR_Divider divider, CLKPWR_ClockRatio value)
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    uint32_t offset = 2 * (divider % 16);
    LPC_SC->PCLKSEL[divider / 16] = (LPC_SC->PCLKSEL[divider / 16] & ~(3u << offset)) | (value << offset);
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    if (divider == CLKPWR_DIVIDER_EMC) {
        if (value == CLKPWR_RATIO_1) {
            LPC_SC->EMCCLKSEL = (0 << SC_EMCCLKSEL_EMCDIV_Pos);
            return LPCLIB_SUCCESS;
        }
        else if (value == CLKPWR_RATIO_2) {
            LPC_SC->EMCCLKSEL = (1 << SC_EMCCLKSEL_EMCDIV_Pos);
            return LPCLIB_SUCCESS;
        }

        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (divider != CLKPWR_DIVIDER_PCLK) {
        return LPCLIB_ILLEGAL_PARAMETER;                /* Must use global PCLK selector */
    }

    LPC_SC->PCLKSEL = value;
#endif

    return LPCLIB_SUCCESS;
}



/** Enables the crystal oscillator.
 */
static LPCLIB_Result CLKPWR_enableMainOscillator (LPCLIB_Switch activate)
{
    if (activate) {
        if (!(LPC_SC->SCS & SC_SCS_OSCEN_Msk)) {                /* Already running? */
            if (LPCLIB_CLKPWR_XTAL_FREQUENCY < 16000000ul) {    /* XTAL below 16 MHz? */
                LPC_SC->SCS &= ~(SC_SCS_OSCRANGE_Msk);          /* Yes. Use range 1-20 MHz */
            }
            else {
                LPC_SC->SCS |= SC_SCS_OSCRANGE_Msk;             /* No. Use range 15-25 MHz */
            }
            LPC_SC->SCS |= SC_SCS_OSCEN_Msk;                    /* Start now */

            while (!(LPC_SC->SCS & SC_SCS_OSCSTAT_Msk))         /* Wait until it's stable */
                ;
        }
    }
    else {
//TODO do not deactivate if it is current clock source
        LPC_SC->SCS &= ~SC_SCS_OSCEN_Msk;                       /* Stop */
    }

    return LPCLIB_SUCCESS;
}



/** Attempts to set the CPU clock to the desired frequency.
 *
 *  Sets the CPU clock to the frequency which is closest to the requested
 *  value. Assumes that the PLL input clock has been selected before.
 *
 *  \param[in] targetCpuFrequency Desired CPU frequency (Hz)
 *  \param[out] newCpuFrequency CPU frequency (Hz) after the call
 *  \return ...
 */   //TODO TODO TODO... :-)
LPCLIB_Result CLKPWR_setCpuClock (uint32_t targetCpuFrequency)
{
    uint32_t inClock;
    uint32_t m, n, div;
    uint32_t mopt, nopt, divopt, enableopt;
    uint32_t f, fpll;
    int32_t delta, maxdelta;
    int flashCycles;


    /* Anything to do at all? */
    if (targetCpuFrequency != SystemCoreClock) {
        /* If the PLL is enabled or connected, disconnect and disable it now. */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
        if (LPC_SC->PLL0CON & (1 << 1)) {
            LPC_SC->PLL0CON  = 1;                           /* Disconnect */
            LPC_SC->PLL0FEED = 0xAA;
            LPC_SC->PLL0FEED = 0x55;
        }
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
        LPC_SC->CCLKSEL =                                   /* Switch over to system clock */
                (0u << SC_CCLKSEL_CCLKSEL_Pos)
              | (1u << SC_CCLKSEL_CCLKDIV_Pos);

        LPC_SC->PBOOST = (SC_PBOOST_BOOST_OFF << SC_PBOOST_BOOST_Pos);  /* Disable power boost (f <= 100 MHz) */
#endif

        LPC_SC->PLL0CON  = 0;                               /* Disable */
        LPC_SC->PLL0FEED = 0xAA;
        LPC_SC->PLL0FEED = 0x55;

        /* Determine PLL clock input */
#if LPCLIB_CLKPWR_PLL_INPUT_CLOCK == CLKPWR_OSCILLATOR_SYSTEM
        inClock = LPCLIB_CLKPWR_XTAL_FREQUENCY;             /* Crystal oscillator */
        CLKPWR_enableMainOscillator(ENABLE);
        LPC_SC->CLKSRCSEL = 0x01;
#else
        inClock = IRC_FREQUENCY;                            /* IRC oscillator */
        LPC_SC->CLKSRCSEL = 0x00;
#endif
        SystemCoreClock = inClock;

        /* Find optimum settings */
        maxdelta = 0x7FFFFFFF;
        enableopt = 0;
        divopt = 1;
        mopt = 6;
        nopt = 1;
        for (div = 1; div <= CCLK_DIV_MAX; div++) {
            f = inClock / div;
            delta = f - targetCpuFrequency;
            if (delta < 0) {
                delta = -delta;
            }

            if (delta < maxdelta) {
                maxdelta = delta;
                divopt = div;
                SystemCoreClock = f;
            }
        }

        for (m = PLL_M_MIN; m <= PLL_M_MAX; m++) {
            for (n = PLL_N_MIN; n <= PLL_N_MAX; n++) {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
                fpll = ((inClock / 1000ul) * 2 * m) / n;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
                fpll = (inClock / 1000ul) * m * (2u << n);
#endif
                if ((fpll >= PLL_F_MIN) && (fpll <= PLL_F_MAX)) {
                    for (div = 1; div <= CCLK_DIV_MAX; div++) {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
                        f = (1000ul * fpll) / div;
                        if (div % 2) {
                            continue;                   /* Odd dividers not allowed */
                        }
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
                        /* Do not allow more than CCLK_MAX before the CCLK divider. */
                        if (inClock * m > CCLK_MAX) {
                            continue;
                        }
                        f = (inClock * m) / div;
#endif
                        delta = f - targetCpuFrequency;
                        if (delta < 0) {
                            delta = -delta;
                        }
                        if (delta < maxdelta) {
                            maxdelta = delta;
                            enableopt = 1;
                            mopt = m;
                            nopt = n;
                            divopt = div;
                            SystemCoreClock = f;
                        }

                        if (f < targetCpuFrequency) {
                            break;                  /* Stop here. Larger dividers won't fit! */
                        }
                    }
                }
            }
        }

        if (enableopt) {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
            LPC_SC->PLL0CFG =
                    ((nopt - 1) << SC_PLL0CFG_NSEL0_Pos)
                  | ((mopt - 1) << SC_PLL0CFG_MSEL0_Pos);
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
            LPC_SC->PLL0CFG =
                    (nopt << SC_PLL0CFG_PSEL_Pos)
                  | ((mopt - 1) << SC_PLL0CFG_MSEL_Pos);
#endif
            LPC_SC->PLL0FEED = 0xAA;
            LPC_SC->PLL0FEED = 0x55;
        }

        /* Clock diver to select CCLK */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
        LPC_SC->CCLKCFG = divopt - 1;
#endif

        if (enableopt) {
            /* Enable the Main PLL, and wait until it locks. Then connect it. */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
            LPC_SC->PLL0CON   = SC_PLL0CON_PLLE0_Msk;
            LPC_SC->PLL0FEED  = 0xAA;
            LPC_SC->PLL0FEED  = 0x55;

            while (!(LPC_SC->PLL0STAT & SC_PLL0STAT_PLOCK0_Msk))
            {
            }

            if (targetCpuFrequency <= 20000000ul) {
                flashCycles = 1;
            }
            else if (targetCpuFrequency <= 40000000ul) {
                flashCycles = 2;
            }
            else if (targetCpuFrequency <= 60000000ul) {
                flashCycles = 3;
            }
            else if (targetCpuFrequency <= 80000000ul) {
                flashCycles = 4;
            }
            else if (targetCpuFrequency <= 100000000ul) {
                flashCycles = 5;
            }
            else {
                flashCycles = 6;
            }
            LPC_SC->FLASHCFG = ((flashCycles - 1) << SC_FLASHCFG_FLASHTIM_Pos) | 0x3A;

            LPC_SC->PLL0CON   = SC_PLL0CON_PLLE0_Msk | SC_PLL0CON_PLLC0_Msk;
            LPC_SC->PLL0FEED  = 0xAA;
            LPC_SC->PLL0FEED  = 0x55;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
            LPC_SC->PLL0CON   = SC_PLL0CON_PLLE_Msk;
            LPC_SC->PLL0FEED  = 0xAA;
            LPC_SC->PLL0FEED  = 0x55;

            while (!(LPC_SC->PLL0STAT & SC_PLL0STAT_PLOCK_Msk))
            {
            }

            if (targetCpuFrequency <= 20000000ul) {
                flashCycles = 1;
            }
            else if (targetCpuFrequency <= 40000000ul) {
                flashCycles = 2;
            }
            else if (targetCpuFrequency <= 60000000ul) {
                flashCycles = 3;
            }
            else if (targetCpuFrequency <= 80000000ul) {
                flashCycles = 4;
            }
            else if (targetCpuFrequency <= 100000000ul) {
                flashCycles = 5;
            }
            else {
                flashCycles = 6;

                /* Enable power boost (f > 100 MHz) */
                LPC_SC->PBOOST = (SC_PBOOST_BOOST_ON << SC_PBOOST_BOOST_Pos);
            }
            LPC_SC->FLASHCFG = ((flashCycles - 1) << SC_FLASHCFG_FLASHTIM_Pos) | 0x3A;

            LPC_SC->CCLKSEL = (1u << SC_CCLKSEL_CCLKSEL_Pos) | divopt;
#endif
        }
    }

    return LPCLIB_SUCCESS;
}



#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
/* Disconnect the PLL0. */
LPCLIB_Result CLKPWR_disconnectPLL0 (void)
{
    if (LPC_SC->PLL0STAT & SC_PLL0STAT_PLLC0_STAT_Msk) {
        LPC_SC->PLL0CON &= ~SC_PLL0CON_PLLC0_Msk;       /* Disconnect, but leave enabled */
        LPC_SC->PLL0FEED = 0xAA;
        LPC_SC->PLL0FEED = 0x55;
        while (LPC_SC->PLL0STAT & SC_PLL0STAT_PLLC0_STAT_Msk)
            ;

        LPC_SC->PLL0CON = 0;                            /* Disable */
        LPC_SC->PLL0FEED = 0xAA;
        LPC_SC->PLL0FEED = 0x55;

        // TODO: Calculate SystemCoreClock by looking at actual register settings!
    #if LPCLIB_CLKPWR_PLL_INPUT_CLOCK == CLKPWR_OSCILLATOR_SYSTEM
        SystemCoreClock = LPCLIB_CLKPWR_XTAL_FREQUENCY;
    #else
        SystemCoreClock = IRC_FREQUENCY;
    #endif
    }

    return LPCLIB_SUCCESS;
}
#endif


/* Sets up the USB clock using the USB PLL. */
LPCLIB_Result CLKPWR_setUsbClock (void)
{
    //TODO TODO TODO TODO TODO
    uint32_t m;

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    if (LPC_SC->PLL1CON & (1u << 1)) {
        LPC_SC->PLL1CON  = 1;                           /* Disconnect */
        LPC_SC->PLL1FEED = 0xAA;
        LPC_SC->PLL1FEED = 0x55;
    }
#endif

    LPC_SC->PLL1CON = 0;                                /* Disable */
    LPC_SC->PLL1FEED = 0xAA;
    LPC_SC->PLL1FEED = 0x55;

#if LPCLIB_CLKPWR_USB_INPUT_CLOCK == CLKPWR_USBCLOCK_PLL1
    CLKPWR_enableMainOscillator(ENABLE);

    m = 48000000ul / LPCLIB_CLKPWR_XTAL_FREQUENCY;
    LPC_SC->PLL1CFG = (1u << 5) | ((m - 1) << 0);
    LPC_SC->PLL1FEED = 0xAA;
    LPC_SC->PLL1FEED = 0x55;

    LPC_SC->PLL1CON = 1;                                /* Enable */
    LPC_SC->PLL1FEED = 0xAA;
    LPC_SC->PLL1FEED = 0x55;

    while (!(LPC_SC->PLL1STAT & (1u << 10)))            /* Wait until locked */
        ;

  #if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    LPC_SC->PLL1CON = 3;                                /* Connect */
    LPC_SC->PLL1FEED = 0xAA;
    LPC_SC->PLL1FEED = 0x55;
  #endif

  #if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    LPC_SC->USBCLKSEL = (2u << 8) | (1u << 0);          /* Divide by one from alternate PLL (PLL1) */
  #endif

#else
    m = SystemCoreClock / 48000000ul;

  #if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    LPC_SC->USBCLKCFG = m - 1;                          /* Divide by m down to 48 MHz */
  #endif
  #if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    LPC_SC->USBCLKSEL = (1u << 8) | (m << 0);           /* Divide by m from main PLL (PLL0) */
  #endif

#endif

    return LPCLIB_SUCCESS;
}



/* Enter a power-saving mode. */
void CLKPWR_enterPowerSaving (CLKPWR_PowerSavingMode mode)
{
    uint32_t oldPCON;
    
    /* Clear power mode and indicators */
    oldPCON = LPC_SC->PCON
            & ~(SC_PCON_PM0_Msk | SC_PCON_PM1_Msk)
            & ~(SC_PCON_SMFLAG_Msk | SC_PCON_DSFLAG_Msk | SC_PCON_PDFLAG_Msk | SC_PCON_DPDFLAG_Msk);

    /* SLEEPDEEP bit required for anything but SLEEP mode */
    if (mode & (1u << 16)) {
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    }
    else {
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }

    /* Set the mode... */
    LPC_SC->PCON = oldPCON | (mode & 0x0FFFF);

    /* ...and enter it. Wait for wake-up. */
    __WFI();

    /* Make sure any subsequent WFI will simply enter SLEEP */
    LPC_SC->PCON &= ~(SC_PCON_PM0_Msk | SC_PCON_PM1_Msk);
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}


/** @} */

/** @} CLKPWR */


