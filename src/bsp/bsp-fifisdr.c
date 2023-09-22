
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"

#include "bsp-fifisdr.h"
#include "bsp-io.h"
#include "bsp-lcd.h"
#include "params.h"


#if defined(__ICCARM__)
    #define __stdout __iar_Stdout
#endif

#if defined(__GNUC__)
    #define __stdout (*stdout)
    #define _Handle _file
#endif

#if !defined(__ICCARM__)
struct __FILE {
    uint8_t _Handle;                                    /**< Dummy! */
};
#endif

volatile int32_t ITM_RxBuffer = ITM_RXBUFFER_EMPTY;     /**< Declared in core_cm3.h */


I2C_Handle i2c_bus;
uint8_t si570Address;
SI570_Handle vco;

static BSP_BoardType boardType = BSP_BOARD_UNDEFINED;

const I2C_Config i2c_bitrate400k_config[] = {
    {.opcode = I2C_OPCODE_SET_BITRATE,
        {.bitrate = 400000ul, }},

    I2C_CONFIG_END
};



DMA_Handle gpdma;
#if LPCLIB_USB
USB_Handle usbHardware;
#endif
ADC_Handle adc;
DAC_Handle dac;


/** Probe all I2C addresses to find the Si570.
 *
 *  The Si570 is the only device on the bus. The first ACK response must be the Si570.
 *  Return address=0 if no response is received for any of the probed addresses.
 *
 *  \param[in] handle Bus handle
 *  \retval 0 No Si570 found
 *  \retval !=0 Address of Si570
 */
static uint8_t BSP_probeSi570 (I2C_Handle handle)
{
    uint8_t address;

    /* Probe valid address range.
     * NOTE: The Si570 may use addresses below 0x08 that are not allowed by the I2C spec!
     */
    for (address = 0x01; address < 0x78; address++) {
        if (I2C_probe(handle, address) == LPCLIB_SUCCESS) {
            return address;                             /* Device found! */
        }
    }

    return 0;                                           /* Bad luck */
}



/** DMA driver memory allocation handler. */
static void BSP_dmaMemoryAllocation (LPCLIB_Event event)
{
    uint32_t size;

    switch (event.opcode) {
    case DMA_EVENT_MEMALLOC:
        size = *((uint32_t *)event.parameter);
//TODO !!!!!
#if defined(RTOS_FREERTOS)
        *((void **)event.parameter) = pvPortMalloc(size);
#else
        *((void **)event.parameter) = malloc(size);
#endif
        break;

    case DMA_EVENT_MEMFREE:
//TODO !!!!!
#if defined(RTOS_FREERTOS)
        vPortFree(event.parameter);
#else
        free(event.parameter);
#endif
        break;

    default:
        event.parameter = NULL;
        break;
    }
}

static const DMA_Config dmaInstallHandler = {
    .opcode = DMA_OPCODE_SET_SYSTEM_CALLBACK,
    {.callback = BSP_dmaMemoryAllocation, },
};



#define NORMAL_PIN(func) \
    IOCON_makeConfigD(PIN_FUNCTION_##func, PIN_PULL_NONE, PIN_OPENDRAIN_OFF)
#define PULLUP_PIN(func) \
    IOCON_makeConfigD(PIN_FUNCTION_##func, PIN_PULL_UP, PIN_OPENDRAIN_OFF)
#define PULLDOWN_PIN(func) \
    IOCON_makeConfigD(PIN_FUNCTION_##func, PIN_PULL_DOWN, PIN_OPENDRAIN_OFF)
#define OPENDRAIN_PIN(func) \
    IOCON_makeConfigD(PIN_FUNCTION_##func, PIN_PULL_NONE, PIN_OPENDRAIN_ON)


/** Select pin multiplexers (and the electrical characteristics). */
static void BSP_initPins (void)
{
    IOCON_open();
    GPIO_open();

    switch (boardType) {
    case BSP_BOARD_KAI2:
        IOCON_configurePin(PIN_P0_0,  OPENDRAIN_PIN(3));    /* SDA1 */
        IOCON_configurePin(PIN_P0_1,  OPENDRAIN_PIN(3));    /* SCL1 */
        IOCON_configurePin(PIN_P0_6,  IOCON_makeConfigD(    /* I2SRX_SDA */
                        PIN_FUNCTION_1, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));
        IOCON_configurePin(PIN_P0_7,  NORMAL_PIN(1));       /* I2STX_CLK */
        IOCON_configurePin(PIN_P0_8,  NORMAL_PIN(1));       /* I2STX_WS */
        IOCON_configurePin(PIN_P0_9,  NORMAL_PIN(0));       /* CPLD_TMS */
        IOCON_configurePin(PIN_P0_22, NORMAL_PIN(0));       /* GPIO (ENAREG1) */
        IOCON_configurePin(PIN_P0_29, NORMAL_PIN(1));       /* USB_D+ */
        IOCON_configurePin(PIN_P0_30, NORMAL_PIN(1));       /* USB_D- */
        IOCON_configurePin(PIN_P1_4,  NORMAL_PIN(0));       /* GPIO (TEILER3) */
        IOCON_configurePin(PIN_P1_8,  NORMAL_PIN(0));       /* GPIO (TEILER2) */
        IOCON_configurePin(PIN_P1_9,  NORMAL_PIN(0));       /* GPIO (TEILER1) */
        IOCON_configurePin(PIN_P1_10, NORMAL_PIN(0));       /* GPIO (SI570_OE) */
        IOCON_configurePin(PIN_P1_18, NORMAL_PIN(0));       /* GPIO (LED) */
        IOCON_configurePin(PIN_P1_19, NORMAL_PIN(0));       /* GPIO (ENAREG2) */
        IOCON_configurePin(PIN_P1_30, NORMAL_PIN(2));       /* VBUS */
        IOCON_configurePin(PIN_P2_0,  NORMAL_PIN(0));       /* CPLD_TCK */
        IOCON_configurePin(PIN_P2_1,  NORMAL_PIN(0));       /* CPLD_TDI */
        IOCON_configurePin(PIN_P2_2,  PULLDOWN_PIN(0));     /* CPLD_TDO */
        IOCON_configurePin(PIN_P2_5,  NORMAL_PIN(0));       /* GPIO (PRESELECT1) */
        IOCON_configurePin(PIN_P2_6,  NORMAL_PIN(0));       /* GPIO (PRESELECT2) */
        IOCON_configurePin(PIN_P2_7,  NORMAL_PIN(0));       /* GPIO (PRESELECT3) */
        IOCON_configurePin(PIN_P2_8,  NORMAL_PIN(0));       /* GPIO (PRESELECT4) */
        IOCON_configurePin(PIN_P2_9,  NORMAL_PIN(1));       /* USB_CONNECT */
        IOCON_configurePin(PIN_P4_28, NORMAL_PIN(0));       /* GPIO (UDA1361_PWDN) */
        IOCON_configurePin(PIN_P4_29, NORMAL_PIN(1));       /* TX_MCLK */

        if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
            while (1);  //DEBUGGING ONLY
        }

        GPIO_setDirBit(GPIO_KAI2_LED, ENABLE);              GPIO_writeBit(GPIO_KAI2_LED, 1);
        GPIO_setDirBit(GPIO_KAI2_ENAREG_3V3, ENABLE);       GPIO_writeBit(GPIO_KAI2_ENAREG_3V3, 1);
        GPIO_setDirBit(GPIO_KAI2_ENAREG_4V0, ENABLE);       GPIO_writeBit(GPIO_KAI2_ENAREG_4V0, 1);
        GPIO_setDirBit(GPIO_KAI2_UDA1361_PWDN, ENABLE);     GPIO_writeBit(GPIO_KAI2_UDA1361_PWDN, 0);
        GPIO_setDirBit(GPIO_KAI2_DIV0, ENABLE);
        GPIO_setDirBit(GPIO_KAI2_DIV1, ENABLE);
        GPIO_setDirBit(GPIO_KAI2_SWAPIQ, ENABLE);
        GPIO_setDirBit(GPIO_KAI2_PRESELECT1, ENABLE);
        GPIO_setDirBit(GPIO_KAI2_PRESELECT2, ENABLE);
        GPIO_setDirBit(GPIO_KAI2_PRESELECT3, ENABLE);
        GPIO_setDirBit(GPIO_KAI2_PRESELECT4, ENABLE);       GPIO_writeBit(GPIO_KAI2_PRESELECT4, 0);
        GPIO_setDirBit(GPIO_KAI2_SI570_OE, ENABLE);         GPIO_writeBit(GPIO_KAI2_SI570_OE, 1);
        GPIO_setDirBit(GPIO_KAI2_CPLD_TCK, ENABLE);         GPIO_writeBit(GPIO_KAI2_CPLD_TCK, 0);
        GPIO_setDirBit(GPIO_KAI2_CPLD_TDI, ENABLE);         GPIO_writeBit(GPIO_KAI2_CPLD_TDI, 0);
        GPIO_setDirBit(GPIO_KAI2_CPLD_TMS, ENABLE);         GPIO_writeBit(GPIO_KAI2_CPLD_TMS, 0);
        break;

    case BSP_BOARD_KAI3:
        IOCON_configurePin(PIN_P0_0,  OPENDRAIN_PIN(3));    /* SDA1 */
        IOCON_configurePin(PIN_P0_1,  OPENDRAIN_PIN(3));    /* SCL1 */
        IOCON_configurePin(PIN_P0_6,  IOCON_makeConfigD(    /* I2SRX_SDA */
                        PIN_FUNCTION_1, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));
        IOCON_configurePin(PIN_P0_7,  NORMAL_PIN(1));       /* I2STX_CLK */
        IOCON_configurePin(PIN_P0_8,  NORMAL_PIN(1));       /* I2STX_WS */
        IOCON_configurePin(PIN_P0_9,  NORMAL_PIN(0));       /* CPLD_TMS */
        IOCON_configurePin(PIN_P0_15, NORMAL_PIN(0));       /* GPIO (CCLK) */
        IOCON_configurePin(PIN_P0_16, NORMAL_PIN(0));       /* GPIO (CLATCH) */
        IOCON_configurePin(PIN_P0_17, IOCON_makeConfigD(    /* GPIO (COUT) */
                        PIN_FUNCTION_0, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));
        IOCON_configurePin(PIN_P0_18, NORMAL_PIN(0));       /* GPIO (CIN) */
        IOCON_configurePin(PIN_P0_22, NORMAL_PIN(0));       /* GPIO (ENAREG1) */
        IOCON_configurePin(PIN_P0_29, NORMAL_PIN(1));       /* USB_D+ */
        IOCON_configurePin(PIN_P0_30, NORMAL_PIN(1));       /* USB_D- */
        IOCON_configurePin(PIN_P1_4,  NORMAL_PIN(0));       /* GPIO (TEILER3) */
        IOCON_configurePin(PIN_P1_8,  NORMAL_PIN(0));       /* GPIO (TEILER2) */
        IOCON_configurePin(PIN_P1_9,  NORMAL_PIN(0));       /* GPIO (TEILER1) */
        IOCON_configurePin(PIN_P1_10, NORMAL_PIN(0));       /* GPIO (SI570_OE) */
        IOCON_configurePin(PIN_P1_18, NORMAL_PIN(0));       /* GPIO (LED) */
        IOCON_configurePin(PIN_P1_19, NORMAL_PIN(0));       /* GPIO (ENAREG2) */
        IOCON_configurePin(PIN_P1_30, NORMAL_PIN(2));       /* VBUS */
        IOCON_configurePin(PIN_P2_0,  NORMAL_PIN(0));       /* CPLD_TCK */
        IOCON_configurePin(PIN_P2_1,  NORMAL_PIN(0));       /* CPLD_TDI */
        IOCON_configurePin(PIN_P2_2,  PULLDOWN_PIN(0));     /* CPLD_TDO */
        IOCON_configurePin(PIN_P2_5,  NORMAL_PIN(0));       /* GPIO (PRESELECT1) */
        IOCON_configurePin(PIN_P2_6,  NORMAL_PIN(0));       /* GPIO (PRESELECT2) */
        IOCON_configurePin(PIN_P2_7,  NORMAL_PIN(0));       /* GPIO (PRESELECT3) */
        IOCON_configurePin(PIN_P2_8,  NORMAL_PIN(0));       /* GPIO (PRESELECT4) */
        IOCON_configurePin(PIN_P2_9,  NORMAL_PIN(1));       /* USB_CONNECT */
        IOCON_configurePin(PIN_P4_28, NORMAL_PIN(0));       /* GPIO (RESET_AD1974) */
        IOCON_configurePin(PIN_P4_29, NORMAL_PIN(1));       /* TX_MCLK */

        if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
            while (1);  //DEBUGGING ONLY
        }

        GPIO_setDirBit(GPIO_KAI3_LED, ENABLE);              GPIO_writeBit(GPIO_KAI3_LED, 1);
        GPIO_setDirBit(GPIO_KAI3_ENAREG_3V3, ENABLE);       GPIO_writeBit(GPIO_KAI3_ENAREG_3V3, 1);
        GPIO_setDirBit(GPIO_KAI3_ENAREG_4V0, ENABLE);       GPIO_writeBit(GPIO_KAI3_ENAREG_4V0, 1);
        GPIO_setDirBit(GPIO_KAI3_DIV0, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_DIV1, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_SWAPIQ, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_PRESELECT1, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_PRESELECT2, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_PRESELECT3, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_PRESELECT4, ENABLE);       GPIO_writeBit(GPIO_KAI3_PRESELECT4, 0);
        GPIO_setDirBit(GPIO_KAI3_SI570_OE, ENABLE);         GPIO_writeBit(GPIO_KAI3_SI570_OE, 1);
        GPIO_setDirBit(GPIO_KAI3_CPLD_TCK, ENABLE);         GPIO_writeBit(GPIO_KAI3_CPLD_TCK, 0);
        GPIO_setDirBit(GPIO_KAI3_CPLD_TDI, ENABLE);         GPIO_writeBit(GPIO_KAI3_CPLD_TDI, 0);
        GPIO_setDirBit(GPIO_KAI3_CPLD_TMS, ENABLE);         GPIO_writeBit(GPIO_KAI3_CPLD_TMS, 0);
        GPIO_setDirBit(GPIO_KAI3_CIN, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_CCLK, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_RESET_AD1974, ENABLE);
        GPIO_setDirBit(GPIO_KAI3_CLATCH, ENABLE);           GPIO_writeBit(GPIO_KAI3_CLATCH, 1);
        break;

    case BSP_BOARD_ROLF1:
        IOCON_configurePin(PIN_P0_0,  OPENDRAIN_PIN(3));    /* SDA1 */
        IOCON_configurePin(PIN_P0_1,  OPENDRAIN_PIN(3));    /* SCL1 */
        IOCON_configurePin(PIN_P0_6,  IOCON_makeConfigD(    /* I2SRX_SDA */
                        PIN_FUNCTION_1, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));
        IOCON_configurePin(PIN_P0_7,  NORMAL_PIN(1));       /* I2STX_CLK */
        IOCON_configurePin(PIN_P0_8,  NORMAL_PIN(1));       /* I2STX_WS */
        IOCON_configurePin(PIN_P0_9,  NORMAL_PIN(2));       /* MOSI1 */
        IOCON_configurePin(PIN_P0_11, NORMAL_PIN(0));       /* GPIO (SI570_OE) */
        IOCON_configurePin(PIN_P0_25, NORMAL_PIN(0));       /* GPIO (GUI_LIGHT) */
        IOCON_configurePin(PIN_P0_26, NORMAL_PIN(2));       /* AOUT */
        IOCON_configurePin(PIN_P0_29, NORMAL_PIN(1));       /* USB_D+ */
        IOCON_configurePin(PIN_P0_30, NORMAL_PIN(1));       /* USB_D- */
        IOCON_configurePin(PIN_P1_0,  NORMAL_PIN(0));       /* GPIO (GUI_SSEL) */
        IOCON_configurePin(PIN_P1_1,  NORMAL_PIN(0));       /* GPIO (GUI_RESET) */
        IOCON_configurePin(PIN_P1_4,  NORMAL_PIN(0));       /* GPIO (GUI_A0) */
        IOCON_configurePin(PIN_P1_18, NORMAL_PIN(0));       /* GPIO (LED) */
        IOCON_configurePin(PIN_P1_30, NORMAL_PIN(2));       /* VBUS */
        IOCON_configurePin(PIN_P1_31, NORMAL_PIN(2));       /* SCK1 */
        IOCON_configurePin(PIN_P2_4,  NORMAL_PIN(0));       /* GPIO (TEILER1) */
        IOCON_configurePin(PIN_P2_5,  NORMAL_PIN(0));       /* GPIO (TEILER2) */
        IOCON_configurePin(PIN_P2_6,  NORMAL_PIN(0));       /* GPIO (ENAREG1) */
        IOCON_configurePin(PIN_P2_7,  NORMAL_PIN(0));       /* GPIO (ENAREG2) */
        IOCON_configurePin(PIN_P2_9,  NORMAL_PIN(1));       /* USB_CONNECT */
        IOCON_configurePin(PIN_P4_28, NORMAL_PIN(0));       /* GPIO (UDA1361_PWDN) */
        IOCON_configurePin(PIN_P4_29, NORMAL_PIN(1));       /* TX_MCLK */

        if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
            while (1);  //DEBUGGING ONLY
        }

        GPIO_setDirBit(GPIO_ROLF1_LED, ENABLE);             GPIO_writeBit(GPIO_ROLF1_LED, 0);
        GPIO_setDirBit(GPIO_ROLF1_ENAREG1, ENABLE);         GPIO_writeBit(GPIO_ROLF1_ENAREG1, 1);
        GPIO_setDirBit(GPIO_ROLF1_ENAREG2, ENABLE);         GPIO_writeBit(GPIO_ROLF1_ENAREG2, 1);
        GPIO_setDirBit(GPIO_ROLF1_UDA1361_PWDN, ENABLE);    GPIO_writeBit(GPIO_ROLF1_UDA1361_PWDN, 0);
        GPIO_setDirBit(GPIO_ROLF1_TEILER1, ENABLE);
        GPIO_setDirBit(GPIO_ROLF1_TEILER2, ENABLE);
        GPIO_setDirBit(GPIO_ROLF1_SI570_OE, ENABLE);        GPIO_writeBit(GPIO_ROLF1_SI570_OE, 1);
        GPIO_setDirBit(GPIO_ROLF1_GUI_LIGHT, ENABLE);       GPIO_writeBit(GPIO_ROLF1_GUI_LIGHT, 0);
        GPIO_setDirBit(GPIO_ROLF1_GUI_RESET, ENABLE);       GPIO_writeBit(GPIO_ROLF1_GUI_RESET, 0);
        GPIO_setDirBit(GPIO_ROLF1_GUI_A0, ENABLE);
        GPIO_setDirBit(GPIO_ROLF1_GUI_SSEL, ENABLE);        GPIO_writeBit(GPIO_ROLF1_GUI_SSEL, 1);
        break;

    case BSP_BOARD_ROLF2:
        IOCON_configurePin(PIN_P0_0,  OPENDRAIN_PIN(3));    /* SDA1 */
        IOCON_configurePin(PIN_P0_1,  OPENDRAIN_PIN(3));    /* SCL1 */
        IOCON_configurePin(PIN_P0_2,  NORMAL_PIN(0));       /* GPIO (LED) */
        IOCON_configurePin(PIN_P0_6,  IOCON_makeConfigD(    /* I2SRX_SDA */
                        PIN_FUNCTION_1, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));
        IOCON_configurePin(PIN_P0_7,  NORMAL_PIN(1));       /* I2STX_CLK */
        IOCON_configurePin(PIN_P0_8,  NORMAL_PIN(1));       /* I2STX_WS */
        IOCON_configurePin(PIN_P0_10, NORMAL_PIN(0));       /* GPIO (SI570_OE) */
        IOCON_configurePin(PIN_P0_11, NORMAL_PIN(0));       /* GPIO (ENAREG1) */
        IOCON_configurePin(PIN_P0_16, NORMAL_PIN(0));       /* GPIO (PLL2) */
        IOCON_configurePin(PIN_P0_22, NORMAL_PIN(0));       /* GPIO (PRESEL5) */
        IOCON_configurePin(PIN_P0_29, NORMAL_PIN(1));       /* USB_D+ */
        IOCON_configurePin(PIN_P0_30, NORMAL_PIN(1));       /* USB_D- */
        IOCON_configurePin(PIN_P1_8,  NORMAL_PIN(0));       /* GPIO (RESET_AD1974) */
        IOCON_configurePin(PIN_P1_18, NORMAL_PIN(0));       /* GPIO (PRESEL3) */
        IOCON_configurePin(PIN_P1_20, NORMAL_PIN(0));       /* GPIO (CCLK) */
        IOCON_configurePin(PIN_P1_22, NORMAL_PIN(0));       /* GPIO (PRESEL4) */
        IOCON_configurePin(PIN_P1_24, NORMAL_PIN(0));       /* GPIO (CIN) */
        IOCON_configurePin(PIN_P1_25, NORMAL_PIN(0));       /* GPIO (PRESEL1) */
        IOCON_configurePin(PIN_P1_26, NORMAL_PIN(0));       /* GPIO (ENAREG2) */
        IOCON_configurePin(PIN_P1_30, NORMAL_PIN(2));       /* VBUS */
        IOCON_configurePin(PIN_P2_0,  NORMAL_PIN(0));       /* GPIO (INH7046) */
        IOCON_configurePin(PIN_P2_1,  NORMAL_PIN(1));       /* GPIO (PWM1.2) */
        IOCON_configurePin(PIN_P2_2,  NORMAL_PIN(0));       /* GPIO (CLATCH) */
        IOCON_configurePin(PIN_P2_6,  NORMAL_PIN(0));       /* GPIO (TEILER1) */
        IOCON_configurePin(PIN_P2_7,  NORMAL_PIN(0));       /* GPIO (TEILER2) */
        IOCON_configurePin(PIN_P2_8,  NORMAL_PIN(0));       /* GPIO (PLL1) */
        IOCON_configurePin(PIN_P2_9,  NORMAL_PIN(1));       /* USB_CONNECT */

        if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
            while (1);  //DEBUGGING ONLY
        }

        GPIO_setDirBit(GPIO_ROLF2_LED, ENABLE);             GPIO_writeBit(GPIO_ROLF2_LED, 1);
        GPIO_setDirBit(GPIO_ROLF2_ENAREG1, ENABLE);         GPIO_writeBit(GPIO_ROLF2_ENAREG1, 1);
        GPIO_setDirBit(GPIO_ROLF2_ENAREG2, ENABLE);         GPIO_writeBit(GPIO_ROLF2_ENAREG2, 1);
        GPIO_setDirBit(GPIO_ROLF2_PRESELECT1, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_PRESELECT3, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_PRESELECT4, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_PRESELECT5, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_PLL1, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_PLL2, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_CIN, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_CCLK, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_RESET_AD1974, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_CLATCH, ENABLE);          GPIO_writeBit(GPIO_ROLF2_CLATCH, 1);
        GPIO_setDirBit(GPIO_ROLF2_INH7046, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_TEILER1, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_TEILER2, ENABLE);
        GPIO_setDirBit(GPIO_ROLF2_SI570_OE, ENABLE);        GPIO_writeBit(GPIO_ROLF2_SI570_OE, 1);
        break;

    case BSP_BOARD_ROLF3:
        IOCON_configurePin(PIN_P0_0,  OPENDRAIN_PIN(3));    /* SDA1 */
        IOCON_configurePin(PIN_P0_1,  OPENDRAIN_PIN(3));    /* SCL1 */
        IOCON_configurePin(PIN_P0_2,  NORMAL_PIN(0));       /* GPIO (ENAREG2) */
        IOCON_configurePin(PIN_P0_6,  IOCON_makeConfigD(    /* I2SRX_SDA */
                        PIN_FUNCTION_1, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));
        IOCON_configurePin(PIN_P0_7,  NORMAL_PIN(1));       /* I2STX_CLK */
        IOCON_configurePin(PIN_P0_8,  NORMAL_PIN(1));       /* I2STX_WS */
        IOCON_configurePin(PIN_P0_9,  NORMAL_PIN(0));       /* GPIO (CIN) */
        IOCON_configurePin(PIN_P0_10, NORMAL_PIN(0));       /* GPIO (PRESELECT4) */
        IOCON_configurePin(PIN_P0_11, NORMAL_PIN(1));       /* RXD2 (for non-standard preselector mode) */
        IOCON_configurePin(PIN_P0_15, NORMAL_PIN(0));       /* GPIO (DIV0) */
        IOCON_configurePin(PIN_P0_16, NORMAL_PIN(0));       /* GPIO (DIV1) */
        IOCON_configurePin(PIN_P0_17, NORMAL_PIN(0));       /* CPLD_TCK */
        IOCON_configurePin(PIN_P0_18, NORMAL_PIN(0));       /* CPLD_TMS */
        IOCON_configurePin(PIN_P0_22, NORMAL_PIN(0));       /* CPLD_TDI */
        IOCON_configurePin(PIN_P0_29, NORMAL_PIN(1));       /* USB_D+ */
        IOCON_configurePin(PIN_P0_30, NORMAL_PIN(1));       /* USB_D- */
        IOCON_configurePin(PIN_P1_0,  NORMAL_PIN(0));       /* GPIO (RESET_AD1974) */
        IOCON_configurePin(PIN_P1_18, NORMAL_PIN(0));       /* GPIO (LED1) */
        IOCON_configurePin(PIN_P1_19, NORMAL_PIN(0));       /* GPIO (PRESELECT3) */
        IOCON_configurePin(PIN_P1_20, NORMAL_PIN(0));       /* GPIO (PRESELECT2) */
        IOCON_configurePin(PIN_P1_22, NORMAL_PIN(0));       /* GPIO (PRESELECT1) */
        IOCON_configurePin(PIN_P1_25, NORMAL_PIN(0));       /* GPIO (SI570_OE) */
        IOCON_configurePin(PIN_P1_26, NORMAL_PIN(0));       /* GPIO (LED2) */
        IOCON_configurePin(PIN_P1_29, NORMAL_PIN(0));       /* GPIO (ENAREG1) */
        IOCON_configurePin(PIN_P1_30, NORMAL_PIN(2));       /* VBUS */
        IOCON_configurePin(PIN_P2_0,  NORMAL_PIN(0));       /* GPIO (CLATCH) */
        IOCON_configurePin(PIN_P2_1,  NORMAL_PIN(0));       /* GPIO (CCLK) */
        IOCON_configurePin(PIN_P2_2,  PULLDOWN_PIN(0));     /* TRACE */
        IOCON_configurePin(PIN_P2_3,  PULLDOWN_PIN(0));     /* TRACE */
        IOCON_configurePin(PIN_P2_4,  PULLDOWN_PIN(0));     /* TRACE */
        IOCON_configurePin(PIN_P2_5,  PULLDOWN_PIN(0));     /* TRACE */
        IOCON_configurePin(PIN_P2_6,  PULLDOWN_PIN(0));     /* TRACE */
        IOCON_configurePin(PIN_P2_7,  PULLDOWN_PIN(0));     /* CPLD_TDO */
        IOCON_configurePin(PIN_P2_8,  NORMAL_PIN(0));       /* GPIO (SWAPIQ) */
        IOCON_configurePin(PIN_P2_9,  NORMAL_PIN(1));       /* USB_CONNECT */
        IOCON_configurePin(PIN_P4_28, IOCON_makeConfigD(    /* GPIO (COUT) */
                        PIN_FUNCTION_0, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));

        if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
            while (1);  //DEBUGGING ONLY
        }

        GPIO_setDirBit(GPIO_ROLF3_LED1, ENABLE);            GPIO_writeBit(GPIO_ROLF3_LED1, 0);
        GPIO_setDirBit(GPIO_ROLF3_LED2, ENABLE);            GPIO_writeBit(GPIO_ROLF3_LED2, 0);
        GPIO_setDirBit(GPIO_ROLF3_ENAREG_3V3, ENABLE);      GPIO_writeBit(GPIO_ROLF3_ENAREG_3V3,1);
        GPIO_setDirBit(GPIO_ROLF3_ENAREG_4V0, ENABLE);      GPIO_writeBit(GPIO_ROLF3_ENAREG_4V0,1);
        GPIO_setDirBit(GPIO_ROLF3_DIV0, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_DIV1, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_SWAPIQ, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_PRESELECT1, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_PRESELECT2, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_PRESELECT3, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_PRESELECT4, ENABLE);      GPIO_writeBit(GPIO_ROLF3_PRESELECT4, 0);
        GPIO_setDirBit(GPIO_ROLF3_SI570_OE, ENABLE);        GPIO_writeBit(GPIO_ROLF3_SI570_OE, 1);
        GPIO_setDirBit(GPIO_ROLF3_CPLD_TCK, ENABLE);        GPIO_writeBit(GPIO_ROLF3_CPLD_TCK, 0);
        GPIO_setDirBit(GPIO_ROLF3_CPLD_TDI, ENABLE);        GPIO_writeBit(GPIO_ROLF3_CPLD_TDI, 0);
        GPIO_setDirBit(GPIO_ROLF3_CPLD_TMS, ENABLE);        GPIO_writeBit(GPIO_ROLF3_CPLD_TMS, 0);
        GPIO_setDirBit(GPIO_ROLF3_CIN, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_CCLK, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_RESET_AD1974, ENABLE);
        GPIO_setDirBit(GPIO_ROLF3_CLATCH, ENABLE);          GPIO_writeBit(GPIO_ROLF3_CLATCH, 1);
        break;

    case BSP_BOARD_TEST:
        IOCON_configurePin(PIN_P0_6,  IOCON_makeConfigD(    /* I2SRX_SDA */
                        PIN_FUNCTION_1, PIN_PULL_REPEATER, PIN_OPENDRAIN_OFF));
        IOCON_configurePin(PIN_P0_7,  NORMAL_PIN(1));       /* I2STX_CLK */
        IOCON_configurePin(PIN_P0_8,  NORMAL_PIN(1));       /* I2STX_WS */
        IOCON_configurePin(PIN_P0_19, PULLUP_PIN(3));       /* SDA1 */
        IOCON_configurePin(PIN_P0_20, PULLUP_PIN(3));       /* SCL1 */
        IOCON_configurePin(PIN_P0_26, NORMAL_PIN(2));       /* AOUT */
        IOCON_configurePin(PIN_P0_29, NORMAL_PIN(1));       /* USB_D+ */
        IOCON_configurePin(PIN_P0_30, NORMAL_PIN(1));       /* USB_D- */
        IOCON_configurePin(PIN_P1_28, NORMAL_PIN(0));       /* GPIO (LED) */
        IOCON_configurePin(PIN_P1_29, NORMAL_PIN(0));       /* GPIO (ENAREG1) */
        IOCON_configurePin(PIN_P1_30, NORMAL_PIN(2));       /* VBUS */
        IOCON_configurePin(PIN_P1_31, NORMAL_PIN(0));       /* GPIO (ENAREG2) */
        IOCON_configurePin(PIN_P2_2,  NORMAL_PIN(0));       /* GPIO (TEILER1) */
        IOCON_configurePin(PIN_P2_3,  NORMAL_PIN(0));       /* GPIO (TEILER2) */
        IOCON_configurePin(PIN_P2_5,  NORMAL_PIN(0));       /* GPIO (UDA1361_PWDN) */
        IOCON_configurePin(PIN_P2_9,  NORMAL_PIN(1));       /* USB_CONNECT */
        IOCON_configurePin(PIN_P4_29, NORMAL_PIN(1));       /* TX_MCLK */

        if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
            while (1);  //DEBUGGING ONLY
        }

        GPIO_setDirBit(GPIO_TEST_LED, ENABLE);              GPIO_writeBit(GPIO_TEST_LED, 0);
        GPIO_setDirBit(GPIO_TEST_ENAREG1, ENABLE);          GPIO_writeBit(GPIO_TEST_ENAREG1, 1);
        GPIO_setDirBit(GPIO_TEST_ENAREG2, ENABLE);          GPIO_writeBit(GPIO_TEST_ENAREG2, 1);
        GPIO_setDirBit(GPIO_TEST_UDA1361_PWDN, ENABLE);     GPIO_writeBit(GPIO_TEST_UDA1361_PWDN, 0);
        GPIO_setDirBit(GPIO_TEST_TEILER1, ENABLE);
        GPIO_setDirBit(GPIO_TEST_TEILER2, ENABLE);
        break;

    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }

//    IOCON_close();
}



/** Detect board version. */
static void _BSP_detectBoard (void)
{
    boardType = BSP_BOARD_UNDEFINED;

    /* Newer versions have a dedicated port to detect the hardware. */
    if (GPIO_readBit(GPIO_1_24) == 0) {                 /* Pull-down for version detect */
        boardType = BSP_BOARD_KAI3;
        IOCON_configurePin(PIN_P1_24, PULLDOWN_PIN(0)); /* Avoid pull-up current */
    }
    else if (GPIO_readBit(GPIO_1_23) == 0) {            /* Pull-down for version detect */
        boardType = BSP_BOARD_ROLF3;
        IOCON_configurePin(PIN_P1_23, PULLDOWN_PIN(0)); /* Avoid pull-up current */
    }
    else {
        if (GPIO_readBit(GPIO_0_25) == 0) {             /* Transistor for backlight control.
                                                         * Open (internal pull-up) on normal boards.
                                                         */
            boardType = BSP_BOARD_ROLF1;
        }
        else {
            if (GPIO_readBit(GPIO_1_22) == 0) {         /* Transistor in preselector.
                                                         * Open (internal pull-up) on normal boards.
                                                         */
                boardType = BSP_BOARD_ROLF2;
            }
            else {
                boardType = BSP_BOARD_KAI2;
            }
        }
    }
}



/* Return board type. */
BSP_BoardType BSP_getBoardType (void)
{
    return boardType;
}



void BSP_init (void)
{
    _BSP_detectBoard();
    BSP_initPins();

    DMA_open(DMA0, &gpdma);
    DMA_ioctl(gpdma, &dmaInstallHandler);

    I2C_open(I2C1, &i2c_bus);
    I2C_ioctl(i2c_bus, i2c_bitrate400k_config);
    si570Address = BSP_probeSi570(i2c_bus);             /* Identify slave address of Si570 */
    SI570_open(i2c_bus, si570Address, &vco);

    ADC_open(ADC0, &adc);

    CLKPWR_setUsbClock();
    USB_open(USB0, &usbHardware);

    BSP_openLcd();                                      /* Not on all boards */

    BSP_setLed(ENABLE);                                 /* LED on: Indicate end of initialization */
}



/* Configure preselector for new frequency */
void BSP_setPreselector (BSP_PreselectorSetting *setting)
{
    switch (boardType) {
    case BSP_BOARD_KAI2:
        GPIO_writeBit(GPIO_KAI2_PRESELECT1, (setting->pattern >> 0) & 1);
        GPIO_writeBit(GPIO_KAI2_PRESELECT2, (setting->pattern >> 1) & 1);
        GPIO_writeBit(GPIO_KAI2_PRESELECT3, (setting->pattern >> 2) & 1);
        GPIO_writeBit(GPIO_KAI2_PRESELECT4, (setting->pattern >> 3) & 1);
        break;

    case BSP_BOARD_KAI3:
        GPIO_writeBit(GPIO_KAI3_PRESELECT1, (setting->pattern >> 0) & 1);
        GPIO_writeBit(GPIO_KAI3_PRESELECT2, (setting->pattern >> 1) & 1);
        GPIO_writeBit(GPIO_KAI3_PRESELECT3, (setting->pattern >> 2) & 1);
        GPIO_writeBit(GPIO_KAI3_PRESELECT4, (setting->pattern >> 3) & 1);
        break;

    case BSP_BOARD_ROLF1:
        /* No preselector */
        break;

    case BSP_BOARD_ROLF2:
        GPIO_writeBit(GPIO_ROLF2_PRESELECT1, (setting->pattern >> 0) & 1);
        GPIO_writeBit(GPIO_ROLF2_PRESELECT3, (setting->pattern >> 1) & 1);
        GPIO_writeBit(GPIO_ROLF2_PRESELECT4, (setting->pattern >> 2) & 1);
        GPIO_writeBit(GPIO_ROLF2_PRESELECT5, (setting->pattern >> 3) & 1);
        break;

    case BSP_BOARD_ROLF3:
        GPIO_writeBit(GPIO_ROLF3_PRESELECT1, (setting->pattern >> 0) & 1);
        GPIO_writeBit(GPIO_ROLF3_PRESELECT2, (setting->pattern >> 1) & 1);
        GPIO_writeBit(GPIO_ROLF3_PRESELECT3, (setting->pattern >> 2) & 1);
        GPIO_writeBit(GPIO_ROLF3_PRESELECT4, (setting->pattern >> 3) & 1);
        break;

    case BSP_BOARD_TEST:
        /* No preselector */
        break;

    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }
}



/* Set value of prescaler in CPLD */
void BSP_setPrescaler (int prescaler)
{
    int div0 = 0, div1 = 0;


    switch (prescaler) {
    case 1:     div0 = 0; div1 = 0; break;
    case 4:     div0 = 1; div1 = 0; break;
    case 16:    div0 = 0; div1 = 1; break;
    case 64:    div0 = 1; div1 = 1; break;
    }

    switch (boardType) {
    case BSP_BOARD_KAI2:
        GPIO_writeBit(GPIO_KAI2_DIV1, div1);
        GPIO_writeBit(GPIO_KAI2_DIV0, div0);
        break;
    case BSP_BOARD_KAI3:
        GPIO_writeBit(GPIO_KAI3_DIV1, div1);
        GPIO_writeBit(GPIO_KAI3_DIV0, div0);
        break;
    case BSP_BOARD_ROLF1:
        GPIO_writeBit(GPIO_ROLF1_TEILER2, div1);
        GPIO_writeBit(GPIO_ROLF1_TEILER1, div0);
        break;
    case BSP_BOARD_ROLF2:
        GPIO_writeBit(GPIO_ROLF2_TEILER2, div1);
        GPIO_writeBit(GPIO_ROLF2_TEILER1, div0);
        break;
    case BSP_BOARD_ROLF3:
        GPIO_writeBit(GPIO_ROLF3_DIV1, div1);
        GPIO_writeBit(GPIO_ROLF3_DIV0, div0);
        break;
    case BSP_BOARD_TEST:
        GPIO_writeBit(GPIO_TEST_TEILER2, div1);
        GPIO_writeBit(GPIO_TEST_TEILER1, div0);
        break;
    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }
}



/** Control I/Q swap. */
void BSP_setIqSwap (int swapIQ)
{
    switch (boardType) {
    case BSP_BOARD_KAI2:
        GPIO_writeBit(GPIO_KAI2_SWAPIQ, swapIQ ? 1 : 0);
        break;

    case BSP_BOARD_KAI3:
        GPIO_writeBit(GPIO_KAI3_SWAPIQ, swapIQ ? 1 : 0);
        break;

    case BSP_BOARD_ROLF3:
        GPIO_writeBit(GPIO_ROLF3_SWAPIQ, swapIQ ? 1 : 0);
        break;

    default:
        break;
    }
}



/** Control PTT output. */
void BSP_setPtt (int on)
{
    switch (boardType) {
    case BSP_BOARD_KAI2:
        if (g_params.presel_mode == 3) {
            GPIO_writeBit(GPIO_KAI2_PRESELECT4, on ? 0 : 1);
        }
        break;

    case BSP_BOARD_KAI3:
        if (g_params.presel_mode == 3) {
            GPIO_writeBit(GPIO_KAI3_PRESELECT4, on ? 0 : 1);
        }
        break;

    case BSP_BOARD_ROLF3:
        if (g_params.presel_mode == 3) {
            GPIO_writeBit(GPIO_ROLF3_PRESELECT4, on ? 0 : 1);
        }
        break;

    default:
        break;
    }
}



/* Control the LED */
void BSP_setLed (LPCLIB_Switch on)
{
    switch (boardType) {
    case BSP_BOARD_KAI2:
        GPIO_writeBit(GPIO_KAI2_LED, on ? 0 : 1);
        break;

    case BSP_BOARD_KAI3:
        GPIO_writeBit(GPIO_KAI3_LED, on ? 0 : 1);
        break;

    case BSP_BOARD_ROLF1:
        GPIO_writeBit(GPIO_ROLF1_LED, on ? 1 : 0);
        break;

    case BSP_BOARD_ROLF2:
        GPIO_writeBit(GPIO_ROLF2_LED, on ? 0 : 1);
        break;

    case BSP_BOARD_ROLF3:
        GPIO_writeBit(GPIO_ROLF3_LED2, on ? 1 : 0);
        break;

    case BSP_BOARD_TEST:
        GPIO_writeBit(GPIO_TEST_LED, on ? 1 : 0);
        break;

    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }
}



/* Port preparation for sleep mode */
void BSP_prepareSleep (void)
{
    BSP_closeLcd();
    BSP_setLed(DISABLE);
    
    switch (boardType) {
    case BSP_BOARD_KAI2:
        GPIO_writeBit(GPIO_KAI2_SI570_OE, 0);           /* Prevent leakage via Si570 into 3V3_SWI */
        GPIO_writeBit(GPIO_KAI2_ENAREG_3V3, 0);         /* Power line 3V3_SWI off */
        GPIO_writeBit(GPIO_KAI2_ENAREG_4V0, 0);         /* Power line 4V_SWI off */
        GPIO_writeBit(GPIO_KAI2_DIV0, 0);               /* Prevent leakage via CPLD into 3V3_SWI */
        GPIO_writeBit(GPIO_KAI2_DIV1, 0);               /* Prevent leakage via CPLD into 3V3_SWI */
        GPIO_writeBit(GPIO_KAI2_SWAPIQ, 0);             /* Prevent leakage via CPLD into 3V3_SWI */
        break;

    case BSP_BOARD_KAI3:
        GPIO_writeBit(GPIO_KAI3_RESET_AD1974, 0);       /* Codec Reset */
        GPIO_writeBit(GPIO_KAI3_CIN, 0);                /* Codec SPI */
        GPIO_writeBit(GPIO_KAI3_CLATCH, 0);
        IOCON_configurePin(PIN_P0_17, PULLDOWN_PIN(0));
        GPIO_writeBit(GPIO_KAI3_SI570_OE, 0);           /* Prevent leakage via Si570 into 3V3_SWI */
        GPIO_writeBit(GPIO_KAI3_ENAREG_3V3, 0);         /* Power line 3V3_SWI off */
        GPIO_writeBit(GPIO_KAI3_ENAREG_4V0, 0);         /* Power line 4V_SWI off */
        GPIO_writeBit(GPIO_KAI3_DIV0, 0);               /* Prevent leakage via CPLD into 3V3_SWI */
        GPIO_writeBit(GPIO_KAI3_DIV1, 0);               /* Prevent leakage via CPLD into 3V3_SWI */
        GPIO_writeBit(GPIO_KAI3_SWAPIQ, 0);             /* Prevent leakage via CPLD into 3V3_SWI */
        IOCON_configurePin(PIN_P0_7, PULLDOWN_PIN(0));  /* I2STX_CLK --> GPIO */
        IOCON_configurePin(PIN_P0_8, PULLDOWN_PIN(0));  /* I2STX_WS --> GPIO */
        IOCON_configurePin(PIN_P0_6, PULLDOWN_PIN(0));  /* I2SRX_SDA --> GPIO */
        break;

    case BSP_BOARD_ROLF1:
        GPIO_writeBit(GPIO_ROLF1_SI570_OE, 0);
        GPIO_writeBit(GPIO_ROLF1_ENAREG1, 0);
        GPIO_writeBit(GPIO_ROLF1_ENAREG2, 0);
        break;

    case BSP_BOARD_ROLF2:
        GPIO_writeBit(GPIO_ROLF2_SI570_OE, 0);
        GPIO_writeBit(GPIO_ROLF2_ENAREG1, 0);
        GPIO_writeBit(GPIO_ROLF2_ENAREG2, 0);
        break;

    case BSP_BOARD_ROLF3:
        GPIO_writeBit(GPIO_ROLF3_RESET_AD1974, 0);      /* Codec Reset */
        GPIO_writeBit(GPIO_ROLF3_CIN, 0);               /* Codec SPI */
        GPIO_writeBit(GPIO_ROLF3_CLATCH, 0);
        IOCON_configurePin(PIN_P4_28, PULLDOWN_PIN(0));
        GPIO_writeBit(GPIO_ROLF3_SI570_OE, 0);          /* Prevent leakage via Si570 into 3V3_SWI */
        GPIO_writeBit(GPIO_ROLF3_LED2, 0);              /* LED off */
        GPIO_writeBit(GPIO_ROLF3_ENAREG_3V3, 0);        /* Power line 3V3_SWI off */
        GPIO_writeBit(GPIO_ROLF3_ENAREG_4V0, 0);        /* Power line 4V_SWI off */
        GPIO_writeBit(GPIO_ROLF3_DIV0, 0);              /* Prevent leakage via CPLD into 3V3_SWI */
        GPIO_writeBit(GPIO_ROLF3_DIV1, 0);              /* Prevent leakage via CPLD into 3V3_SWI */
        GPIO_writeBit(GPIO_ROLF3_SWAPIQ, 0);            /* Prevent leakage via CPLD into 3V3_SWI */
GPIO_writeBit(GPIO_ROLF3_PRESELECT1, 0);
GPIO_writeBit(GPIO_ROLF3_PRESELECT2, 0);
GPIO_writeBit(GPIO_ROLF3_PRESELECT3, 0);
GPIO_writeBit(GPIO_ROLF3_PRESELECT4, 0);
        IOCON_configurePin(PIN_P0_7, PULLDOWN_PIN(0));  /* I2STX_CLK --> GPIO */
        IOCON_configurePin(PIN_P0_8, PULLDOWN_PIN(0));  /* I2STX_WS --> GPIO */
        IOCON_configurePin(PIN_P0_6, PULLDOWN_PIN(0));  /* I2SRX_SDA --> GPIO */
        break;

    case BSP_BOARD_TEST:
        GPIO_writeBit(GPIO_TEST_ENAREG1, 0);
        GPIO_writeBit(GPIO_TEST_ENAREG2, 0);
        break;

    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }
}



/* Port initialization after wakeup */
void BSP_wakeup (void)
{
    BSP_initPins();
    osDelay(20);                                        /* Wait for Si570 power-up */
    BSP_openLcd();

    switch (boardType) {
    case BSP_BOARD_KAI2:
        GPIO_writeBit(GPIO_KAI2_SI570_OE, 1);
        break;

    case BSP_BOARD_KAI3:
        GPIO_writeBit(GPIO_KAI3_SI570_OE, 1);
        break;

    case BSP_BOARD_ROLF1:
        GPIO_writeBit(GPIO_ROLF1_SI570_OE, 1);
        break;

    case BSP_BOARD_ROLF2:
        GPIO_writeBit(GPIO_ROLF2_SI570_OE, 1);
        break;

    case BSP_BOARD_ROLF3:
        GPIO_writeBit(GPIO_ROLF3_SI570_OE, 1);
        break;

    case BSP_BOARD_TEST:
        break;

    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }
}



/* To be called as the first step in SystemInit(). */
void BSP_systemInit (void)
{
    uint32_t n;
    uint32_t numIrqLines;


    /* Check if debugger connected */
//    if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
//        inDebugMode = true;
//    }

    /* Allow user mode code to pend interrupts via NVIC->STIR */
    SCB->CCR |= SCB_CCR_USERSETMPEND_Msk;

    /* Individual exception handlers for Usage Fault, Bus Fault, MemManage Fault */
    SCB->SHCSR |= 0
               | SCB_SHCSR_USGFAULTENA_Msk
               | SCB_SHCSR_BUSFAULTENA_Msk
               | SCB_SHCSR_MEMFAULTENA_Msk
               ;
    /* Priority of peripheral interrupts depends on the application, and therefore the drivers
     * library does not set the priority.
     * After reset, all interrupts have highest priority, and this can lead to nasty failures
     * if your application uses an RTOS. Therefore it is a good idea to preset all interrupt
     * priorities to a safe default (the lowest priority). You may afterwards raise the
     * priority of individual interrupts as required by the application.
     */
    numIrqLines = 32u * (1u + (uint32_t)((SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) >> SCnSCB_ICTR_INTLINESNUM_Pos));
    if (numIrqLines > 240u) {
        numIrqLines = 240u;
    }
    for (n = 0; n < numIrqLines; n++) {
        NVIC_SetPriority((IRQn_Type)n, (1u << __NVIC_PRIO_BITS) - 1);
    }

    SystemCoreClock = 0;
}


/********** Standard Library I/O **********/

#if (defined(__GNUC__))

#include <sys/stat.h>

extern uint32_t __heap_start;


int _fstat (int fd, struct stat *st);
int _fstat (int fd, struct stat *st)
{
    (void) fd;
    (void) st;

    return -1;
}


int _read (int fd, char *ptr, int len);
int _read (int fd, char *ptr, int len)
{
    (void) fd;

    int bytesUnRead = len;
    int32_t c;

    c = ITM_ReceiveChar();
    if (c >= 0) {
        *ptr = c;
        --bytesUnRead;
    }

    return len - bytesUnRead;
}


int _write (int fd, const char *ptr, int len);
int _write (int fd, const char *ptr, int len)
{
    (void) fd;

    int bytesUnWritten = -1;

#if 0
    int n = 0;
    while (++n < 10000) {
        if (*((volatile uint8_t *)0x20083800) == 0) {
            if (len > 252) {
                len = 252; //TODO
            }
            memcpy((void *)0x20083804, ptr, len);
            *((volatile uint8_t *)0x20083800) = len;

            break;
        }
    }

    bytesUnWritten = 0;
#else
    int i;
    char c;

    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr);

        if (*ptr == '\n') {
            c = '\r';
            ITM_SendChar(c);
        }

        ++ptr;
    }
    bytesUnWritten = len - i;
#endif

    return len - bytesUnWritten;
}


/* This one gets linked in if we use printf-stadarg.c */
void _outbyte (int c);
void _outbyte (int c)
{
    _write(0, (const char *)&c, 1);
}


int _close (int fd);
int _close (int fd)
{
    (void) fd;

    return 0;
}


int _lseek (int fd, int ptr, int dir);
int _lseek (int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;

    return 0;
}


int _isatty (int fd);
int _isatty (int fd)
{
    return (fd <= 2) ? 1 : 0;   /* one of stdin, stdout, stderr */
}


caddr_t _sbrk (int incr);
caddr_t _sbrk (int incr)
{
    static caddr_t heap = NULL;
    caddr_t prev_heap;

    if (heap == NULL) {
        heap = (caddr_t)&__heap_start;
    }

    prev_heap = heap;
    heap += incr;

    return prev_heap;
}


#else


FILE __stdout;
FILE __stdin;


int fputc (int ch, FILE *f)
{
    (void) f;

    if (ch == '\n') {
        ch = '\r';
    }
    ITM_SendChar(ch);

    return ch;
}


#if defined(__ICCARM__)
int fgetc (FILE *f)
#else
int getc (FILE *f)
#endif
{
    int c;

    c = ITM_ReceiveChar();
    if (c >= 0) {
        return c;
    }

    return EOF;
}

int ferror (FILE *f)
{
    return EOF;
}
#endif

