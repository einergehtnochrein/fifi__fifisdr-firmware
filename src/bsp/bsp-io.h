#ifndef __BSP_IO_H
#define __BSP_IO_H

#include "lpclib.h"


/* GPIO functions */
#define GPIO_KAI2_LED                       GPIO_1_18
#define GPIO_KAI2_ENAREG_3V3                GPIO_0_22
#define GPIO_KAI2_ENAREG_4V0                GPIO_1_19
#define GPIO_KAI2_DIV0                      GPIO_1_9
#define GPIO_KAI2_DIV1                      GPIO_1_8
#define GPIO_KAI2_SWAPIQ                    GPIO_1_4
#define GPIO_KAI2_SI570_OE                  GPIO_1_10
#define GPIO_KAI2_UDA1361_PWDN              GPIO_4_28
#define GPIO_KAI2_PRESELECT1                GPIO_2_5
#define GPIO_KAI2_PRESELECT2                GPIO_2_6
#define GPIO_KAI2_PRESELECT3                GPIO_2_7
#define GPIO_KAI2_PRESELECT4                GPIO_2_8
#define GPIO_KAI2_CPLD_TCK                  GPIO_2_0
#define GPIO_KAI2_CPLD_TDI                  GPIO_2_1
#define GPIO_KAI2_CPLD_TDO                  GPIO_2_2
#define GPIO_KAI2_CPLD_TMS                  GPIO_0_9

#define GPIO_KAI3_LED                       GPIO_1_18
#define GPIO_KAI3_ENAREG_3V3                GPIO_0_22
#define GPIO_KAI3_ENAREG_4V0                GPIO_1_19
#define GPIO_KAI3_DIV0                      GPIO_1_9
#define GPIO_KAI3_DIV1                      GPIO_1_8
#define GPIO_KAI3_SWAPIQ                    GPIO_1_4
#define GPIO_KAI3_SI570_OE                  GPIO_1_10
#define GPIO_KAI3_RESET_AD1974              GPIO_4_28
#define GPIO_KAI3_PRESELECT1                GPIO_2_5
#define GPIO_KAI3_PRESELECT2                GPIO_2_6
#define GPIO_KAI3_PRESELECT3                GPIO_2_7
#define GPIO_KAI3_PRESELECT4                GPIO_2_8
#define GPIO_KAI3_CPLD_TCK                  GPIO_2_0
#define GPIO_KAI3_CPLD_TDI                  GPIO_2_1
#define GPIO_KAI3_CPLD_TDO                  GPIO_2_2
#define GPIO_KAI3_CPLD_TMS                  GPIO_0_9
#define GPIO_KAI3_CIN                       GPIO_0_18
#define GPIO_KAI3_COUT                      GPIO_0_17
#define GPIO_KAI3_CLATCH                    GPIO_0_16
#define GPIO_KAI3_CCLK                      GPIO_0_15
#define GPIO_KAI3_EXT0                      GPIO_2_3
#define GPIO_KAI3_EXT1                      GPIO_2_4

#define GPIO_ROLF1_LED                      GPIO_1_18
#define GPIO_ROLF1_ENAREG1                  GPIO_2_6
#define GPIO_ROLF1_ENAREG2                  GPIO_2_7
#define GPIO_ROLF1_TEILER1                  GPIO_2_4
#define GPIO_ROLF1_TEILER2                  GPIO_2_5
#define GPIO_ROLF1_SI570_OE                 GPIO_0_11
#define GPIO_ROLF1_UDA1361_PWDN             GPIO_4_28
#define GPIO_ROLF1_GUI_LIGHT                GPIO_0_25
#define GPIO_ROLF1_GUI_RESET                GPIO_1_1
#define GPIO_ROLF1_GUI_SSEL                 GPIO_1_0
#define GPIO_ROLF1_GUI_A0                   GPIO_1_4
#define GPIO_ROLF1_TOUCH_TOP                GPIO_1_9
#define PIN_ROLF1_TOUCH_TOP                 PIN_P1_9
#define GPIO_ROLF1_TOUCH_BOTTOM             GPIO_0_2
#define PIN_ROLF1_TOUCH_BOTTOM              PIN_P0_2
#define GPIO_ROLF1_TOUCH_LEFT               GPIO_0_3
#define PIN_ROLF1_TOUCH_LEFT                PIN_P0_3
#define GPIO_ROLF1_TOUCH_RIGHT              GPIO_1_10
#define PIN_ROLF1_TOUCH_RIGHT               PIN_P1_10

#define GPIO_ROLF2_LED                      GPIO_0_2
#define GPIO_ROLF2_ENAREG1                  GPIO_0_11
#define GPIO_ROLF2_ENAREG2                  GPIO_1_26
#define GPIO_ROLF2_TEILER1                  GPIO_2_6
#define GPIO_ROLF2_TEILER2                  GPIO_2_7
#define GPIO_ROLF2_SI570_OE                 GPIO_0_10
#define GPIO_ROLF2_PRESELECT1               GPIO_1_25
#define GPIO_ROLF2_PRESELECT3               GPIO_1_18
#define GPIO_ROLF2_PRESELECT4               GPIO_1_22
#define GPIO_ROLF2_PRESELECT5               GPIO_0_22
#define GPIO_ROLF2_PLL1                     GPIO_2_8
#define GPIO_ROLF2_PLL2                     GPIO_0_16
#define GPIO_ROLF2_RESET_AD1974             GPIO_1_8
#define GPIO_ROLF2_CCLK                     GPIO_1_20
#define GPIO_ROLF2_CIN                      GPIO_1_24
#define GPIO_ROLF2_CLATCH                   GPIO_2_2
#define GPIO_ROLF2_INH7046                  GPIO_2_0

#define GPIO_ROLF3_LED1                     GPIO_1_18
#define GPIO_ROLF3_LED2                     GPIO_1_26
#define GPIO_ROLF3_ENAREG_3V3               GPIO_1_29
#define GPIO_ROLF3_ENAREG_4V0               GPIO_0_2
#define GPIO_ROLF3_DIV0                     GPIO_0_15
#define GPIO_ROLF3_DIV1                     GPIO_0_16
#define GPIO_ROLF3_SWAPIQ                   GPIO_2_8
#define GPIO_ROLF3_SI570_OE                 GPIO_1_25
#define GPIO_ROLF3_RESET_AD1974             GPIO_1_0
#define GPIO_ROLF3_PRESELECT1               GPIO_1_22
#define GPIO_ROLF3_PRESELECT2               GPIO_1_20
#define GPIO_ROLF3_PRESELECT3               GPIO_1_19
#define GPIO_ROLF3_PRESELECT4               GPIO_0_10
#define GPIO_ROLF3_CPLD_TCK                 GPIO_0_17
#define GPIO_ROLF3_CPLD_TDI                 GPIO_0_22
#define GPIO_ROLF3_CPLD_TDO                 GPIO_2_7
#define GPIO_ROLF3_CPLD_TMS                 GPIO_0_18
#define GPIO_ROLF3_CIN                      GPIO_0_9
#define GPIO_ROLF3_COUT                     GPIO_4_28
#define GPIO_ROLF3_CLATCH                   GPIO_2_0
#define GPIO_ROLF3_CCLK                     GPIO_2_1
#define GPIO_ROLF3_EXT0                     GPIO_1_28
#define GPIO_ROLF3_EXT1                     GPIO_0_26

#define GPIO_TEST_LED                       GPIO_1_28
#define GPIO_TEST_ENAREG1                   GPIO_1_29   /* not used */
#define GPIO_TEST_ENAREG2                   GPIO_1_31   /* not used */
#define GPIO_TEST_TEILER1                   GPIO_1_31   /* not used */
#define GPIO_TEST_TEILER2                   GPIO_1_31   /* not used */
#define GPIO_TEST_SI570_OE                  GPIO_1_31   /* not used */
#define GPIO_TEST_UDA1361_PWDN              GPIO_1_31   /* not used */
/* P2.2...P2.6 used for ETM Trace! */


#endif


