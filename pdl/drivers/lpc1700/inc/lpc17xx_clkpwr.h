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
 *  \brief CLKPWR driver interface.
 *  This file defines all interface objects needed to use the CLKPWR driver.
 *
 *  \author NXP Semiconductors
 */


#ifndef __LPC17xx_CLKPWR_H__
#define __LPC17xx_CLKPWR_H__

/** \defgroup CLKPWR
 *  \ingroup API
 *  @{
 */

#include "lpc17xx_libconfig.h"

#include "lpclib_types.h"


/** \defgroup CLKPWR_Public_Types CLKPWR Types, enums, macros
 *  @{
 */

#define CLKPWR_OSCILLATOR_IRC       1
#define CLKPWR_OSCILLATOR_SYSTEM    2


typedef enum CLKPWR_Clockswitch {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    CLKPWR_CLOCKSWITCH_TIM0 = 1,
    CLKPWR_CLOCKSWITCH_TIM1 = 2,
    CLKPWR_CLOCKSWITCH_UART0 = 3,
    CLKPWR_CLOCKSWITCH_UART1 = 4,
    CLKPWR_CLOCKSWITCH_PWM1 = 6,
    CLKPWR_CLOCKSWITCH_I2C0 = 7,
    CLKPWR_CLOCKSWITCH_SPI = 8,
    CLKPWR_CLOCKSWITCH_RTC = 9,
    CLKPWR_CLOCKSWITCH_SSP1 = 10,
    CLKPWR_CLOCKSWITCH_ADC = 12,
    CLKPWR_CLOCKSWITCH_CAN1 = 13,
    CLKPWR_CLOCKSWITCH_CAN2 = 14,
    CLKPWR_CLOCKSWITCH_GPIO = 15,
    CLKPWR_CLOCKSWITCH_RIT = 16,
    CLKPWR_CLOCKSWITCH_MCPWM = 17,
    CLKPWR_CLOCKSWITCH_QEI = 18,
    CLKPWR_CLOCKSWITCH_I2C1 = 19,
    CLKPWR_CLOCKSWITCH_SSP0 = 21,
    CLKPWR_CLOCKSWITCH_TIM2 = 22,
    CLKPWR_CLOCKSWITCH_TIM3 = 23,
    CLKPWR_CLOCKSWITCH_UART2 = 24,
    CLKPWR_CLOCKSWITCH_UART3 = 25,
    CLKPWR_CLOCKSWITCH_I2C2 = 26,
    CLKPWR_CLOCKSWITCH_I2S = 27,
    CLKPWR_CLOCKSWITCH_GPDMA = 29,
    CLKPWR_CLOCKSWITCH_ENET = 30,
    CLKPWR_CLOCKSWITCH_USB = 31,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_CLOCKSWITCH_LCD = 0,
    CLKPWR_CLOCKSWITCH_TIM0 = 1,
    CLKPWR_CLOCKSWITCH_TIM1 = 2,
    CLKPWR_CLOCKSWITCH_UART0 = 3,
    CLKPWR_CLOCKSWITCH_UART1 = 4,
    CLKPWR_CLOCKSWITCH_PWM0 = 5,
    CLKPWR_CLOCKSWITCH_PWM1 = 6,
    CLKPWR_CLOCKSWITCH_I2C0 = 7,
    CLKPWR_CLOCKSWITCH_UART4 = 8,
    CLKPWR_CLOCKSWITCH_RTC = 9,
    CLKPWR_CLOCKSWITCH_SSP1 = 10,
    CLKPWR_CLOCKSWITCH_EMC = 11,
    CLKPWR_CLOCKSWITCH_ADC = 12,
    CLKPWR_CLOCKSWITCH_CAN1 = 13,
    CLKPWR_CLOCKSWITCH_CAN2 = 14,
    CLKPWR_CLOCKSWITCH_GPIO = 15,
    CLKPWR_CLOCKSWITCH_MCPWM = 17,
    CLKPWR_CLOCKSWITCH_QEI = 18,
    CLKPWR_CLOCKSWITCH_I2C1 = 19,
    CLKPWR_CLOCKSWITCH_SSP2 = 20,
    CLKPWR_CLOCKSWITCH_SSP0 = 21,
    CLKPWR_CLOCKSWITCH_TIM2 = 22,
    CLKPWR_CLOCKSWITCH_TIM3 = 23,
    CLKPWR_CLOCKSWITCH_UART2 = 24,
    CLKPWR_CLOCKSWITCH_UART3 = 25,
    CLKPWR_CLOCKSWITCH_I2C2 = 26,
    CLKPWR_CLOCKSWITCH_I2S = 27,
    CLKPWR_CLOCKSWITCH_SDC = 28,
    CLKPWR_CLOCKSWITCH_GPDMA = 29,
    CLKPWR_CLOCKSWITCH_ENET = 30,
    CLKPWR_CLOCKSWITCH_USB = 31,
#endif
} CLKPWR_Clockswitch;

typedef enum CLKPWR_Divider {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    CLKPWR_DIVIDER_WDT = 0,
    CLKPWR_DIVIDER_TIMER0 = 1,
    CLKPWR_DIVIDER_TIMER1 = 2,
    CLKPWR_DIVIDER_UART0 = 3,
    CLKPWR_DIVIDER_UART1 = 4,
    CLKPWR_DIVIDER_PWM1 = 6,
    CLKPWR_DIVIDER_I2C0 = 7,
    CLKPWR_DIVIDER_SPI = 8,
    CLKPWR_DIVIDER_SSP1 = 10,
    CLKPWR_DIVIDER_DAC = 11,
    CLKPWR_DIVIDER_ADC = 12,
    CLKPWR_DIVIDER_CAN1 = 13,
    CLKPWR_DIVIDER_CAN2 = 14,
    CLKPWR_DIVIDER_ACF = 15,
    CLKPWR_DIVIDER_QEI = 16,
    CLKPWR_DIVIDER_GPIOINT = 17,
    CLKPWR_DIVIDER_PCB = 18,
    CLKPWR_DIVIDER_I2C1 = 19,
    CLKPWR_DIVIDER_SSP0 = 21,
    CLKPWR_DIVIDER_TIMER2 = 22,
    CLKPWR_DIVIDER_TIMER3 = 23,
    CLKPWR_DIVIDER_UART2 = 24,
    CLKPWR_DIVIDER_UART3 = 25,
    CLKPWR_DIVIDER_I2C2 = 26,
    CLKPWR_DIVIDER_I2S = 27,
    CLKPWR_DIVIDER_RIT = 29,
    CLKPWR_DIVIDER_SYSCON = 30,
    CLKPWR_DIVIDER_MC = 31,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_DIVIDER_PCLK = 32,               /**< common peripheral clock divider */
    CLKPWR_DIVIDER_EMC = 33,                /**< EMC clock divider (1 or 2) */
#endif
} CLKPWR_Divider;


typedef enum CLKPWR_ClockRatio {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    CLKPWR_RATIO_1 = 1,
    CLKPWR_RATIO_2 = 2,
    CLKPWR_RATIO_4 = 0,
    CLKPWR_RATIO_8 = 3,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_RATIO_0 = 0,
    CLKPWR_RATIO_1 = 1,
    CLKPWR_RATIO_2 = 2,
    CLKPWR_RATIO_3 = 3,
    CLKPWR_RATIO_4 = 4,
    CLKPWR_RATIO_5 = 5,
    CLKPWR_RATIO_6 = 6,
    CLKPWR_RATIO_7 = 7,
    CLKPWR_RATIO_8 = 8,
    CLKPWR_RATIO_9 = 9,
    CLKPWR_RATIO_10 = 10,
    CLKPWR_RATIO_11 = 11,
    CLKPWR_RATIO_12 = 12,
    CLKPWR_RATIO_13 = 13,
    CLKPWR_RATIO_14 = 14,
    CLKPWR_RATIO_15 = 15,
    CLKPWR_RATIO_16 = 16,
    CLKPWR_RATIO_17 = 17,
    CLKPWR_RATIO_18 = 18,
    CLKPWR_RATIO_19 = 19,
    CLKPWR_RATIO_20 = 20,
    CLKPWR_RATIO_21 = 21,
    CLKPWR_RATIO_22 = 22,
    CLKPWR_RATIO_23 = 23,
    CLKPWR_RATIO_24 = 24,
    CLKPWR_RATIO_25 = 25,
    CLKPWR_RATIO_26 = 26,
    CLKPWR_RATIO_27 = 27,
    CLKPWR_RATIO_28 = 28,
    CLKPWR_RATIO_29 = 29,
    CLKPWR_RATIO_30 = 30,
    CLKPWR_RATIO_31 = 31,
#endif
} CLKPWR_ClockRatio;


typedef enum CLKPWR_Clock {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    CLKPWR_CLOCK_WDT = 0,
    CLKPWR_CLOCK_TIMER0 = 1,
    CLKPWR_CLOCK_TIMER1 = 2,
    CLKPWR_CLOCK_UART0 = 3,
    CLKPWR_CLOCK_UART1 = 4,
    CLKPWR_CLOCK_PWM1 = 6,
    CLKPWR_CLOCK_I2C0 = 7,
    CLKPWR_CLOCK_SPI = 8,
    CLKPWR_CLOCK_SSP1 = 10,
    CLKPWR_CLOCK_DAC = 11,
    CLKPWR_CLOCK_ADC = 12,
    CLKPWR_CLOCK_CAN1 = 13,
    CLKPWR_CLOCK_CAN2 = 14,
    CLKPWR_CLOCK_ACF = 15,
    CLKPWR_CLOCK_QEI = 16,
    CLKPWR_CLOCK_GPIOINT = 17,
    CLKPWR_CLOCK_PCB = 18,
    CLKPWR_CLOCK_I2C1 = 19,
    CLKPWR_CLOCK_SSP0 = 21,
    CLKPWR_CLOCK_TIMER2 = 22,
    CLKPWR_CLOCK_TIMER3 = 23,
    CLKPWR_CLOCK_UART2 = 24,
    CLKPWR_CLOCK_UART3 = 25,
    CLKPWR_CLOCK_I2C2 = 26,
    CLKPWR_CLOCK_I2S = 27,
    CLKPWR_CLOCK_RIT = 29,
    CLKPWR_CLOCK_SYSCON = 30,
    CLKPWR_CLOCK_MC = 31,

    CLKPWR_CLOCK_CPU = 111,                 /* Virtual bit number only! */
    CLKPWR_CLOCK_LCD = CLKPWR_CLOCK_CPU,    /* Virtual bit number only! */
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    /* No register indices. Just enumerate clocks. */
    CLKPWR_CLOCK_ADC,
    CLKPWR_CLOCK_CAN1,
    CLKPWR_CLOCK_CAN2,
    CLKPWR_CLOCK_PWM1,
    CLKPWR_CLOCK_DAC,
    CLKPWR_CLOCK_EMC,
    CLKPWR_CLOCK_GPIOINT,
    CLKPWR_CLOCK_PCB,
    CLKPWR_CLOCK_I2C0,
    CLKPWR_CLOCK_I2C1,
    CLKPWR_CLOCK_I2C2,
    CLKPWR_CLOCK_I2S,
    CLKPWR_CLOCK_SDC,
    CLKPWR_CLOCK_GPDMA,
    CLKPWR_CLOCK_SYSCON,
    CLKPWR_CLOCK_MC,
    CLKPWR_CLOCK_QEI,
    CLKPWR_CLOCK_SSP0,
    CLKPWR_CLOCK_SSP1,
    CLKPWR_CLOCK_SSP2,
    CLKPWR_CLOCK_TIMER0,
    CLKPWR_CLOCK_TIMER1,
    CLKPWR_CLOCK_TIMER2,
    CLKPWR_CLOCK_TIMER3,
    CLKPWR_CLOCK_UART0,
    CLKPWR_CLOCK_UART1,
    CLKPWR_CLOCK_UART2,
    CLKPWR_CLOCK_UART3,
    CLKPWR_CLOCK_UART4,
    CLKPWR_CLOCK_WDT,

    CLKPWR_CLOCK_CPU,
    CLKPWR_CLOCK_LCD = CLKPWR_CLOCK_CPU,
    CLKPWR_CLOCK_EEPROM = CLKPWR_CLOCK_CPU,
#endif
} CLKPWR_Clock;

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
typedef enum CLKPWR_Reset {
    CLKPWR_RESET_LCD = 0,
    CLKPWR_RESET_TIM0 = 1,
    CLKPWR_RESET_TIM1 = 2,
    CLKPWR_RESET_UART0 = 3,
    CLKPWR_RESET_UART1 = 4,
    CLKPWR_RESET_PWM0 = 5,
    CLKPWR_RESET_PWM1 = 6,
    CLKPWR_RESET_I2C0 = 7,
    CLKPWR_RESET_UART4 = 8,
    CLKPWR_RESET_RTC = 9,
    CLKPWR_RESET_SSP1 = 10,
    CLKPWR_RESET_EMC = 11,
    CLKPWR_RESET_ADC = 12,
    CLKPWR_RESET_CAN1 = 13,
    CLKPWR_RESET_CAN2 = 14,
    CLKPWR_RESET_GPIO = 15,
    CLKPWR_RESET_MCPWM = 17,
    CLKPWR_RESET_QEI = 18,
    CLKPWR_RESET_I2C1 = 19,
    CLKPWR_RESET_SSP2 = 20,
    CLKPWR_RESET_SSP0 = 21,
    CLKPWR_RESET_TIM2 = 22,
    CLKPWR_RESET_TIM3 = 23,
    CLKPWR_RESET_UART2 = 24,
    CLKPWR_RESET_UART3 = 25,
    CLKPWR_RESET_I2C2 = 26,
    CLKPWR_RESET_I2S = 27,
    CLKPWR_RESET_SDC = 28,
    CLKPWR_RESET_GPDMA = 29,
    CLKPWR_RESET_ENET = 30,
    CLKPWR_RESET_USB = 31,
    CLKPWR_RESET_IOCON = 32 + 0,
    CLKPWR_RESET_DAC = 32 + 1,
    CLKPWR_RESET_CANACC = 32 + 2,
} CLKPWR_Reset;
#endif

typedef enum CLKPWR_UnitPower {
    CLKPWR_UNITPOWER___DUMMY__,
} CLKPWR_UnitPower;


typedef enum CLKPWR_PowerSavingMode {
    CLKPWR_POWERSAVING_SLEEP            = (0u << 16) | (1u <<  8) | 0,
    CLKPWR_POWERSAVING_DEEPSLEEP        = (1u << 16) | (1u <<  9) | 0,
    CLKPWR_POWERSAVING_POWERDOWN        = (1u << 16) | (1u << 10) | 1,
    CLKPWR_POWERSAVING_DEEPPOWERDOWN    = (1u << 16) | (1u << 11) | 3,
} CLKPWR_PowerSavingMode;



/** @} CLKPWR Types, enums, macros */


/** \defgroup CLKPWR_Public_Functions CLKPWR API Functions
 *  @{
 */


/** Return the clock frequency of an internal bus.
 *
 *  \param[in] bus The bus to be queried
 *  \return Bus clock in Hz
 */
uint32_t CLKPWR_getBusClock (CLKPWR_Clock clock);


/** Enable a clock signal.
 *
 *  \param[in] clock Clock switch selector
 */
static void CLKPWR_enableClock (CLKPWR_Clockswitch clock);


/** Disable a clock signal.
 *
 *  \param[in] clock Clock switch selector
 */
static void CLKPWR_disableClock (CLKPWR_Clockswitch clock);


/** Power up a functional unit.
 *
 *  \param[in] unit Unit selector
 */
static void CLKPWR_powerUpUnit (CLKPWR_UnitPower unit);


/** Power down a functional unit.
 *
 *  \param[in] unit Unit selector
 */
static void CLKPWR_powerDownUnit (CLKPWR_UnitPower unit);


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
/** Assert the reset signal of the selected peripheral block.
 *
 *  \param[in] peripheral Peripheral block selector
 */
static void CLKPWR_assertPeripheralReset (CLKPWR_Reset peripheral);
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
/** Deassert the reset signal of the selected peripheral block.
 *
 *  \param[in] peripheral Peripheral block selector
 */
static void CLKPWR_deassertPeripheralReset (CLKPWR_Reset peripheral);
#endif


/** Reset all peripherals.
 *
 *  Apply a reset to all peripherals. This can be useful to do at system startup in a
 *  debug session. In this case the peripherals may be in an undefined state, already
 *  fully or partially initialized from a previous run of the code.
 */
void CLKPWR_resetAllPeripherals (void);


/** Set a clock divider.
 *
 *  \param[in] divider Clock divider selector
 *  \param[in] value Division ratio (1/2/4/8)
 *  \retval LPCLIB_SUCCESS Divider was set as requested.
 *  \retval LPCLIB_ILLEGAL_PARAMETER Divider cannot be set
 *          LPC178x: Use 'CLKPWR_DIVIDER_PCLK' to set the global
 *                   peripheral clock common to all peripherals.)
 *                   Use 'CLKPWR_DIVIDER_EMC' to set the CCLK vs EMC clock ratio.
 *          LPC17xx: You cannot use CLKPWR_DIVIDER_PCLK or CLKPWR_DIVIDER_EMC here.
 *                   Use individual peripheral clock selectors.
 */
LPCLIB_Result CLKPWR_setDivider (CLKPWR_Divider divider, CLKPWR_ClockRatio value);


/** Attempts to set the CPU clock to the desired frequency.
 *
 *  Sets the CPU clock to the frequency which is closest to the requested
 *  value. Assumes that the PLL input clock has been selected before.
 *
 *  \param[in] targetCpuFrequency Desired CPU frequency (Hz)
 *  \return ...
 */
LPCLIB_Result CLKPWR_setCpuClock (uint32_t targetCpuFrequency);


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
/** Disconnect the PLL0.
 *
 *  Disconnect the CPU from PLL0, and let it run from the input clock.
 *  This function has been implemented to support a workaround for erratum PLL0.1
 *
 *  \return ...
 */
LPCLIB_Result CLKPWR_disconnectPLL0 (void);
#endif



/** Sets up the USB clock using the USB PLL.
 *
 *  Only works if the crystal frequency is an integer fraction of 48 MHz.
 *
 *  \param[in] targetCpuFrequency Desired CPU frequency (kHz)
 *  \param[out] newCpuFrequency CPU frequency (kHz) after the call
 *  \return ...
 */
LPCLIB_Result CLKPWR_setUsbClock (void);



/** Enter a power-saving mode.
 *
 *  Enters the selected power-saving mode, and does not return before a qualified
 *  wake-up event occurs.
 *  Note 1: You must prepare a suitable wake-up source before entering this function.
 *          Failure to do so may cause the system to stay in power-saving mode forever.
 *  Note 2: This function uses the WFI instruction, not WFE.
 *
 *  \param[in] mode
 */
void CLKPWR_enterPowerSaving (CLKPWR_PowerSavingMode mode);



__FORCEINLINE(void CLKPWR_powerUpUnit (CLKPWR_UnitPower unit))
{
    (void) unit;
}

__FORCEINLINE(void CLKPWR_powerDownUnit (CLKPWR_UnitPower unit))
{
    (void) unit;
}

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(void CLKPWR_assertPeripheralReset (CLKPWR_Reset peripheral))
{
    /* Note: The parameter "peripheral" can have values from 0...63.
     *       Yet the bitband access works since the registers RSTCON0 and RSTCON1
     *       are on consecutive addresses.
     */
    LPCLIB_BITBAND(&LPC_SC->RSTCON, (int)peripheral) = 1;
}
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
__FORCEINLINE(void CLKPWR_deassertPeripheralReset (const CLKPWR_Reset peripheral))
{
    /* Note: The parameter "peripheral" can have values from 0...63.
     *       Yet the bitband access works since the registers RSTCON0 and RSTCON1
     *       are on consecutive addresses.
     */
    LPCLIB_BITBAND(&LPC_SC->RSTCON, (int)peripheral) = 0;
}
#endif



__FORCEINLINE(void CLKPWR_enableClock (CLKPWR_Clockswitch clock))
{
    LPCLIB_BITBAND(&LPC_SC->PCONP, (int)clock) = 1;
}



__FORCEINLINE(void CLKPWR_disableClock (CLKPWR_Clockswitch clock))
{
    LPCLIB_BITBAND(&LPC_SC->PCONP, (int)clock) = 0;
}


/** @} CLKPWR API Functions */

/** @} CLKPWR */

#endif /* #ifndef __LPC17XX_CLKPWR_H__ */

