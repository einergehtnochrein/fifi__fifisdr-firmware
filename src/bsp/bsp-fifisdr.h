#ifndef __BSP_H
#define __BSP_H

#include "lpclib.h"


/* Board type */
typedef enum BSP_BoardType {
    BSP_BOARD_UNDEFINED = 0,
    BSP_BOARD_KAI2,
    BSP_BOARD_KAI3,
    BSP_BOARD_ROLF1,
    BSP_BOARD_ROLF2,
    BSP_BOARD_ROLF3,
    BSP_BOARD_TEST,
} BSP_BoardType;



/** Preselector setting */
typedef struct {
    uint32_t frequency1121;
    uint32_t pattern;
} BSP_PreselectorSetting;



#include "si570.h"

/* I2C devices on FiFi-SDR board:
 *
 * Si570 oscillator
 */
extern uint8_t si570Address;

extern I2C_Handle i2c_bus;
extern SI570_Handle vco;

extern DMA_Handle gpdma;
extern USB_Handle usbHardware;

extern ADC_Handle adc;
extern DAC_Handle dac;

void BSP_init (void);

/** To be called as the first step in SystemInit(). */
void BSP_systemInit (void);

/** Return board type. */
BSP_BoardType BSP_getBoardType (void);

/** Configure preselector for new frequency */
void BSP_setPreselector (BSP_PreselectorSetting *setting);

/** Set value of prescaler in CPLD */
void BSP_setPrescaler (int prescaler);

/** Control I/Q swap. */
void BSP_setIqSwap (int swapIQ);

/** Control PTT output. */
void BSP_setPtt (int on);

/** Control the LED */
void BSP_setLed (LPCLIB_Switch on);

/** Port preparation for sleep mode */
void BSP_prepareSleep (void);

/** Port initialization after wakeup */
void BSP_wakeup (void);

/** Set codec gain. */
void BSP_setCodecVolume (int16_t volume, uint32_t sampleRate, int use32bitFormat);

/** Volume value that indicates a request for power down */
#define BSP_VOLUME_POWERDOWN                (-9999)

/** Set up the clock tree and mode of the I2S interface.
 *
 *  \param[in] i2s I2S block handle
 *  \param[in] sampleRate Sample rate in Hz
 *  \param[in] use32bitFormat Flag: True if using 32 bit samples
 *  \retval LPCLIB_SUCCESS ok
 */
LPCLIB_Result BSP_setupI2S (I2S_Handle i2s, uint32_t sampleRate, int use32bitFormat);

/** Enable/disable the DAC for audio output. */
void BSP_openDac (const DAC_Job *pJob);
void BSP_closeDac (void);

#endif


