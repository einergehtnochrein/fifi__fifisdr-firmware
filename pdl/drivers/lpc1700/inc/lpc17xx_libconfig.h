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
 *  LPC17xx Library Configuration.
 *  This file defines all compile time options that enable (disable) and/or
 *  parameterize features of the peripheral driver library.
 */


/** \defgroup LPC_LibConfig Library Configuration
 *  \ingroup LPC1700_PDL
 *  \{
 */


#ifndef LPCLIB_CONFIG_H
#define LPCLIB_CONFIG_H


/* Application config options */
#include "lpclib_config.h"

/* OS header included here, because it's needed by most drivers */
#include "cmsis_os.h"


/** Device family.
 *  LPCLIB_FAMILY_17XX:
 *  LPCLIB_FAMILY_178X:
 */
#define LPCLIB_FAMILY_LPC17XX   1
#define LPCLIB_FAMILY_LPC178X   2
#ifndef LPCLIB_FAMILY
#define LPCLIB_FAMILY                                   LPCLIB_FAMILY_LPC178X
#endif


/** Support GPIO interrupts.
 */
#ifndef LPCLIB_GPIO_INTERRUPTS
#define LPCLIB_GPIO_INTERRUPTS                          0
#endif



/** Crystal frequency (in Hz).
 *  Set to 0 if no crystal is used. In this case no attempt
 *  is made to start the main oscillator.
 */
#ifndef LPCLIB_CLKPWR_XTAL_FREQUENCY
#define LPCLIB_CLKPWR_XTAL_FREQUENCY                    12000000ul
#endif

/** PLL input clock source.
 *  This must be one of CLKPWR_OSCILLATOR_IRC or CLKPWR_OSCILLATOR_SYSTEM.
 *  (See enum type \ref CLKPWR_Oscillator).
 */
#ifndef LPCLIB_CLKPWR_PLL_INPUT_CLOCK
#define LPCLIB_CLKPWR_PLL_INPUT_CLOCK                   CLKPWR_OSCILLATOR_SYSTEM
#endif




/** Include TIMER driver. */
#ifndef LPCLIB_TIMER
#define LPCLIB_TIMER                                    1
#endif

/** Simple timer objects.
 *  If enabled, one of the timers must be assigned to it
 *  by the \ref LPCLIB_TIMER_SIMPLE_TIMER macro.
 */
#define LPCLIB_TIMER_USE_SIMPLE_TIMERS                  0

/** Select the hardware timer to be used as the simple timer base.
 *  0=TMR16_0, 1=TMR16_1, 2=TMR32_0, 3=TMR32_1
 */
#define LPCLIB_TIMER_SIMPLE_TIMER                       3

/** Number of objects supported by the simple timer.
 */
#define LPCLIB_TIMER_NUM_SIMPLE_TIMERS                  10



/** Include UART driver. */
#ifndef LPCLIB_UART
#define LPCLIB_UART                                     1
#endif

/** Support modem control signals. */
#ifndef LPCLIB_UART_MODEM_CONTROL
#define LPCLIB_UART_MODEM_CONTROL                       0
#endif

/** Support RS485 mode. */
#ifndef LPCLIB_UART_RS485
#define LPCLIB_UART_RS485                               0
#endif

/** Support for auto-bauding. */
#ifndef LPCLIB_UART_AUTOBAUDING
#define LPCLIB_UART_AUTOBAUDING                         0
#endif

/** Support handling of overflow, framing and parity errors.
 *  These error conditions are ignored if this option is disabled.
 */
#define LPCLIB_UART_ERROR_HANDLING                      0

/** Support static setting of fractional divider
 *  (when divider ratio is known a priori)
 */
#define LPCLIB_UART_FRACTIONAL_STATIC                   0

/** Support dynamic setting of fractional divider
 *  (when divider ratio must be calculated by target)
 */
#define LPCLIB_UART_FRACTIONAL_DYNAMIC                  0

/** Size of RX buffer(s) (in bytes) */
#ifndef LPCLIB_UART_RX_BUFFER_SIZE
#define LPCLIB_UART_RX_BUFFER_SIZE                      10
#endif

/** Size of TX buffer(s) (in bytes) */
#ifndef LPCLIB_UART_TX_BUFFER_SIZE
#define LPCLIB_UART_TX_BUFFER_SIZE                      10
#endif

/** Blocking UART_Write() in case buffer is full */
#define LPCLIB_UART_WRITE_BLOCKING                      1



/** Include I2C driver */
#ifndef LPCLIB_I2C
#define LPCLIB_I2C                                      0
#endif

/** Default bus clock speed (in Hz) */
#ifndef LPCLIB_I2C_DEFAULT_BUSCLOCK
#define LPCLIB_I2C_DEFAULT_BUSCLOCK                     100000ul
#endif

/** Support master mode */
#ifndef LPCLIB_I2C_MASTER
#define LPCLIB_I2C_MASTER                               1
#endif

/** Support I2C slave mode */
#ifndef LPCLIB_I2C_SLAVE
#define LPCLIB_I2C_SLAVE                                0
#endif

/** Number of supported slave addresses (set to either 1 or 4) */
#ifndef LPCLIB_I2C_NUM_SLAVE_ADDRESSES
#define LPCLIB_I2C_NUM_SLAVE_ADDRESSES                  4
#endif

/** Default slave address (2*0x01...2*0x78, 0x00=off) */
#define LPCLIB_I2C_SLAVE_DEFAULT_ADDRESS                (0x00)
/** Mask for default slave address (0xFE=all address bits relevant) */
#define LPCLIB_I2C_SLAVE_DEFAULT_MASK                   (0x00)

/** Support I2C bit-banging via GPIO's */
#ifndef LPCLIB_I2CEMU
#define LPCLIB_I2CEMU                                   0
#endif



/** Include SSP driver */
#ifndef LPCLIB_SSP
#define LPCLIB_SSP                                      0
#endif

/** Support SSP master mode. */
#ifndef LPCLIB_SSP_MASTER
#define LPCLIB_SSP_MASTER                               1
#endif

/** Support SSP slave mode. */
#ifndef LPCLIB_SSP_SLAVE
#define LPCLIB_SSP_SLAVE                                0
#endif

/** SSP async mode (default is sync mode) */
#ifndef LPCLIB_SSP_ASYNC
#define LPCLIB_SSP_ASYNC                                0
#endif



/** Include ADC driver */
#ifndef LPCLIB_ADC
#define LPCLIB_ADC                                      0
#endif



/** Include CAN driver */
#ifndef LPCLIB_CAN
#define LPCLIB_CAN                                      0
#endif

/** Default CAN bus bitrate (bit/s) */
#define LPCLIB_CAN_DEFAULT_BITRATE                      (100000ul)



/** Include USB driver */
#ifndef LPCLIB_USB
#define LPCLIB_USB                                      0
#endif



/** Include DMA driver */
#ifndef LPCLIB_DMA
#define LPCLIB_DMA                                      0
#endif



/** Include EMC driver */
#ifndef LPCLIB_EMC
#define LPCLIB_EMC                                      0
#endif

/** Include EMC shutdown function */
#ifndef LPCLIB_EMC_CLOSE
#define LPCLIB_EMC_CLOSE                                0
#endif


/** Include MCI driver */
#ifndef LPCLIB_MCI
#define LPCLIB_MCI                                      0
#endif


/** Include I2S driver */
#ifndef LPCLIB_I2S
#define LPCLIB_I2S                                      0
#endif

/** DMA support for I2S driver */
#ifndef LPCLIB_I2S_DMA
#define LPCLIB_I2S_DMA                                  0
#endif


/** Include USB (device) driver */
#ifndef LPCLIB_USB
#define LPCLIB_USB                                      0
#endif

/** Use DMA for non-control USB endpoints. */
#ifndef LPCLIB_USB_DMA
#define LPCLIB_USB_DMA                                  1
#endif

/** USB input clock source.
 *  This must be one of CLKPWR_USBCLOCK_PLL1 or CLKPWR_USBCLOCK_CCLK.
 */
#ifndef LPCLIB_CLKPWR_USB_INPUT_CLOCK
#define LPCLIB_CLKPWR_USB_INPUT_CLOCK                   CLKPWR_USBCLOCK_PLL1
#endif


/** Include LCD (TFT) driver */
#ifndef LPCLIB_LCD
#define LPCLIB_LCD                                      0
#endif


/** Use an RTOS.
 *  If this is set, you must provide the OSAL layer (and the selected OS).
 */
#ifndef LPCLIB_RTOS
#define LPCLIB_RTOS                                     0
#endif


/*******************  Automatic settings  *****************/

/* Include the correct CMSIS-compatible device header.
 */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
#include "LPC17xx.h"
#include "system_LPC17xx.h"

void SVC_Handler (void);
void PendSV_Handler (void);
void SysTick_Handler (void);

void WDT_IRQHandler (void);
void TIMER0_IRQHandler (void);
void TIMER1_IRQHandler (void);
void TIMER2_IRQHandler (void);
void TIMER3_IRQHandler (void);
void UART0_IRQHandler (void);
void UART1_IRQHandler (void);
void UART2_IRQHandler (void);
void UART3_IRQHandler (void);
void PWM1_IRQHandler (void);
void I2C0_IRQHandler (void);
void I2C1_IRQHandler (void);
void I2C2_IRQHandler (void);
void SSP0_IRQHandler (void);
void SSP1_IRQHandler (void);
void PLL0_IRQHandler (void);
void RTC_IRQHandler (void);
void EINT0_IRQHandler (void);
void EINT1_IRQHandler (void);
void EINT2_IRQHandler (void);
void EINT3_IRQHandler (void);
void ADC_IRQHandler (void);
void BOD_IRQHandler (void);
void USB_IRQHandler (void);
void CAN_IRQHandler (void);
void DMA_IRQHandler (void);
void I2S_IRQHandler (void);
void ENET_IRQHandler (void);
void MCI_IRQHandler (void);
void MCPWM_IRQHandler (void);
void QEI_IRQHandler (void);
void PLL1_IRQHandler (void);
void USBActivity_IRQHandler (void);
void CANActivity_IRQHandler (void);

#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
#include "LPC178x.h"
#include "system_LPC178x.h"

void SVC_Handler (void);
void PendSV_Handler (void);
void SysTick_Handler (void);

void WDT_IRQHandler (void);
void TIMER0_IRQHandler (void);
void TIMER1_IRQHandler (void);
void TIMER2_IRQHandler (void);
void TIMER3_IRQHandler (void);
void UART0_IRQHandler (void);
void UART1_IRQHandler (void);
void UART2_IRQHandler (void);
void UART3_IRQHandler (void);
void PWM1_IRQHandler (void);
void I2C0_IRQHandler (void);
void I2C1_IRQHandler (void);
void I2C2_IRQHandler (void);
void SSP0_IRQHandler (void);
void SSP1_IRQHandler (void);
void PLL0_IRQHandler (void);
void RTC_IRQHandler (void);
void EINT0_IRQHandler (void);
void EINT1_IRQHandler (void);
void EINT2_IRQHandler (void);
void EINT3_IRQHandler (void);
void ADC_IRQHandler (void);
void BOD_IRQHandler (void);
void USB_IRQHandler (void);
void CAN_IRQHandler (void);
void DMA_IRQHandler (void);
void I2S_IRQHandler (void);
void Ethernet_IRQHandler (void);
void MCI_IRQHandler (void);
void MCPWM_IRQHandler (void);
void QEI_IRQHandler (void);
void PLL1_IRQHandler (void);
void USBActivity_IRQHandler (void);
void CANActivity_IRQHandler (void);
void UART4_IRQHandler (void);
void SSP2_IRQHandler (void);
void LCD_IRQHandler (void);
void GPIO_IRQHandler (void);
void PWM0_IRQHandler (void);
void EEPROM_IRQHandler (void);

#endif


/*******************  Check the config parameters  *****************/


#endif /* LPCLIB_CONFIG_H */

/** @} */

