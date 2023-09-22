
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"

#include "bsp-fifisdr.h"
#include "bsp-io.h"
#include "params.h"



#define ad1974_delay(n) do{ for(del=0;del<n;del++); }while(0)

static void BSP_sendToAD1974 (uint8_t value)
{
    volatile uint32_t del;
    int i;
    GPIO_Pin cclk, cin;



    switch (BSP_getBoardType()) {
    case BSP_BOARD_KAI3:
        cclk = GPIO_KAI3_CCLK;
        cin = GPIO_KAI3_CIN;
        break;

    case BSP_BOARD_ROLF2:
        cclk = GPIO_ROLF2_CCLK;
        cin = GPIO_ROLF2_CIN;
        break;

    case BSP_BOARD_ROLF3:
        cclk = GPIO_ROLF3_CCLK;
        cin = GPIO_ROLF3_CIN;
        break;

    default:
        return;
    }


    ad1974_delay(3);

    for (i = 0; i < 8; i++) {                           /* Send eight bits */
        GPIO_writeBit(cclk, 0);                         /* Clock = 0 */
        GPIO_writeBit(cin, ((value >> 7) & 1));         /* Bit */
        value <<= 1;
        ad1974_delay(3);
        GPIO_writeBit(cclk, 1);                         /* Clock = 1 */
        ad1974_delay(3);
    }
    GPIO_writeBit(cclk, 0);                             /* Clock = 0 */

    ad1974_delay(3);
}

static void BSP_writeAD1974Register (uint8_t reg, uint8_t value)
{
    GPIO_Pin clatch;



    switch (BSP_getBoardType()) {
    case BSP_BOARD_KAI3:
        clatch = GPIO_KAI3_CLATCH;
        break;

    case BSP_BOARD_ROLF2:
        clatch = GPIO_ROLF2_CLATCH;
        break;

    case BSP_BOARD_ROLF3:
        clatch = GPIO_ROLF3_CLATCH;
        break;

    default:
        return;
    }

    GPIO_writeBit(clatch, 0);               /* chip select */
    BSP_sendToAD1974(0x08);                 /* Address */
    BSP_sendToAD1974(reg);
    BSP_sendToAD1974(value);
    GPIO_writeBit(clatch, 1);               /* chip select */
}



/* Set codec gain. */
void BSP_setCodecVolume (int16_t volume, uint32_t sampleRate, int use32bitFormat)
{
    GPIO_Pin reset_ad1974;
    uint8_t pllsetting_ad1974;
    GPIO_Pin uda1361_pwdn;
    BSP_BoardType boardType = BSP_getBoardType();
    (void) use32bitFormat;


    if ((boardType == BSP_BOARD_KAI3) || (boardType == BSP_BOARD_ROLF2) || (boardType == BSP_BOARD_ROLF3)) {
        switch (BSP_getBoardType()) {
        case BSP_BOARD_KAI3:
            reset_ad1974 = GPIO_KAI3_RESET_AD1974;
            pllsetting_ad1974 = 0x98;                       /* ADC active, PLL from MCLKI */
            break;

        case BSP_BOARD_ROLF2:
            reset_ad1974 = GPIO_ROLF2_RESET_AD1974;
            pllsetting_ad1974 = 0xD8;                       /* ADC active, PLL from WS */
            break;

        case BSP_BOARD_ROLF3:
            reset_ad1974 = GPIO_ROLF3_RESET_AD1974;
            pllsetting_ad1974 = 0xD8;                       /* ADC active, PLL from WS */
            break;

        default:
            return;
        }

        if (volume == BSP_VOLUME_POWERDOWN) {
            GPIO_writeBit(reset_ad1974, 0);                 /* Codec in reset */
        }
        else {
            GPIO_writeBit(reset_ad1974, 1);                 /* Release from reset */

            if (sampleRate == 48000ul) {                    /* Set rate (48k/96k/192k). Enable high-pass */
                BSP_writeAD1974Register(14, 0x32);
            }
            else if (sampleRate == 96000ul) {
                BSP_writeAD1974Register(14, 0x72);
            }
            else if (sampleRate == 192000ul) {
                BSP_writeAD1974Register(14, 0xB2);
            }

            BSP_writeAD1974Register(15, 0x00);              /* Stereo, 24 bits, 1 BCLK SDATA delay */
            BSP_writeAD1974Register(16, 0x00);              /* WS, BCLK: Slave interface. 64 clocks per frame */
            BSP_writeAD1974Register(0, pllsetting_ad1974);
            BSP_writeAD1974Register(1, 0x00);               /* ADC + AUXPORT from PLL */
        }
    }
    else {
        switch (BSP_getBoardType()) {
        case BSP_BOARD_KAI2:
            uda1361_pwdn = GPIO_KAI2_UDA1361_PWDN;
            break;

        case BSP_BOARD_ROLF1:
            uda1361_pwdn = GPIO_ROLF1_UDA1361_PWDN;
            break;

        default:
            return;
        }

        if (volume == 0) {
            /* 0 dB volume in audio function */
            GPIO_writeBit(uda1361_pwdn, 1);
            GPIO_setDirBit(uda1361_pwdn, ENABLE);           /* UDA1361_PWON=1 --> 6 dB */
        }
        else if (volume == BSP_VOLUME_POWERDOWN) {
            GPIO_writeBit(uda1361_pwdn, 0);                 /* UDA1361 PWDN=0 --> power down */
            GPIO_setDirBit(uda1361_pwdn, ENABLE);
        }
        else {
            /* -6 dB volume in audio function */
            GPIO_setDirBit(uda1361_pwdn, DISABLE);          /* UDA1361_PWON=floating --> 0 dB */
        }
    }
}



static const I2S_Config i2sPortMode16_32clocksPerFrame[] = {
    {.opcode = I2S_OPCODE_SET_MODE,
     .channel = I2S_CHANNEL_RX,
     {.modeAndSize = {
         .bitsPerChannel = 16,
         .size = I2S_WORDSIZE_16BIT_STEREO,
         .mode = I2S_MODE_SLAVE_4WIRE_SHARED_CLOCK,}}},

    {.opcode = I2S_OPCODE_SET_MODE,
     .channel = I2S_CHANNEL_TX,
     {.modeAndSize = {
         .bitsPerChannel = 16,
         .size = I2S_WORDSIZE_16BIT_STEREO,
         .mode = I2S_MODE_MASTER_MCLK_OUT, }}},

    I2S_CONFIG_END
};

static const I2S_Config i2sPortMode16_64clocksPerFrame[] = {
    {.opcode = I2S_OPCODE_SET_MODE,
     .channel = I2S_CHANNEL_RX,
     {.modeAndSize = {
         .bitsPerChannel = 32,
         .size = I2S_WORDSIZE_16BIT_STEREO,
         .mode = I2S_MODE_SLAVE_4WIRE_SHARED_CLOCK,}}},

    {.opcode = I2S_OPCODE_SET_MODE,
     .channel = I2S_CHANNEL_TX,
     {.modeAndSize = {
         .bitsPerChannel = 32,
         .size = I2S_WORDSIZE_16BIT_STEREO,
         .mode = I2S_MODE_MASTER_MCLK_OUT, }}},

    I2S_CONFIG_END
};

static const I2S_Config i2sPortMode32[] = {
    {.opcode = I2S_OPCODE_SET_MODE,
     .channel = I2S_CHANNEL_RX,
     {.modeAndSize = {
         .bitsPerChannel = 32,
         .size = I2S_WORDSIZE_32BIT_STEREO,
         .mode = I2S_MODE_SLAVE_4WIRE_SHARED_CLOCK,}}},

    {.opcode = I2S_OPCODE_SET_MODE,
     .channel = I2S_CHANNEL_TX,
     {.modeAndSize = {
         .bitsPerChannel = 32,
         .size = I2S_WORDSIZE_32BIT_STEREO,
         .mode = I2S_MODE_MASTER_MCLK_OUT, }}},

    I2S_CONFIG_END
};


/* UDA1361, 48 kHz, 16 bits */
static const struct I2S_Config i2sConfigDividers_UDA1361_48k_16b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 8, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 8, }}},

    I2S_CONFIG_END
};

/* UDA1361, 48 kHz, 32 bits */
static const struct I2S_Config i2sConfigDividers_UDA1361_48k_32b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 4, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 4, }}},

    I2S_CONFIG_END
};

/* UDA1361, 96 kHz, 16 bits */
static const struct I2S_Config i2sConfigDividers_UDA1361_96k_16b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 65,
            .bitClockDivider = 8, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 65,
            .bitClockDivider = 8, }}},

    I2S_CONFIG_END
};

/* UDA1361, 96 kHz, 32 bits */
static const struct I2S_Config i2sConfigDividers_UDA1361_96k_32b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 65,
            .bitClockDivider = 4, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 65,
            .bitClockDivider = 4, }}},

    I2S_CONFIG_END
};

/* AD1974, 48 kHz, 16 bits */
static const struct I2S_Config i2sConfigDividers_AD1974_48k_16b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 4, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 4, }}},

    I2S_CONFIG_END
};

/* AD1974, 48 kHz, 32 bits */
static const struct I2S_Config i2sConfigDividers_AD1974_48k_32b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 4, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 4, }}},

    I2S_CONFIG_END
};

/* AD1974, 96 kHz, 16 bits */
static const struct I2S_Config i2sConfigDividers_AD1974_96k_16b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 2, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 2, }}},

    I2S_CONFIG_END
};

/* AD1974, 96 kHz, 32 bits */
static const struct I2S_Config i2sConfigDividers_AD1974_96k_32b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 2, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 2, }}},

    I2S_CONFIG_END
};

/* AD1974, 192 kHz, 16 bits */
static const struct I2S_Config i2sConfigDividers_AD1974_192k_16b[] = {
    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_RX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 1, }}},

    {.opcode = I2S_OPCODE_SET_DIVIDERS,
        .channel = I2S_CHANNEL_TX,
        {.dividers = {
            .fracNominator = 64,
            .fracDenominator = 130,
            .bitClockDivider = 1, }}},

    I2S_CONFIG_END
};



/* Set the clock tree and mode of the I2S interface. */
LPCLIB_Result BSP_setupI2S (I2S_Handle i2s, uint32_t sampleRate, int use32bitFormat)
{
    BSP_BoardType boardType = BSP_getBoardType();


    /* Operating mode of I2S interface. */
    switch (boardType) {
    case BSP_BOARD_KAI2:
    case BSP_BOARD_ROLF1:
        I2S_ioctl(i2s, use32bitFormat ?
                    i2sPortMode32 : i2sPortMode16_32clocksPerFrame);
        break;

    case BSP_BOARD_KAI3:
    case BSP_BOARD_ROLF2:
    case BSP_BOARD_ROLF3:
        I2S_ioctl(i2s, use32bitFormat ?
                    i2sPortMode32 : i2sPortMode16_64clocksPerFrame);
        break;

    case BSP_BOARD_TEST:
    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }

    /* The MCLK frequency depends on the codec.
     * UDA1361 needs to have MCLK scaled with fs: MCLK = 256 * fs
     *   48 kHz: MCLK = 12.288 MHz
     *   96 kHz: MCLK = 24.576 MHz
     * AD1974 in MCLK mode needs a fixed MCLK of 256 * 48 kHz = 12.288 MHz
     */

    switch (boardType) {
    case BSP_BOARD_KAI2:
    case BSP_BOARD_ROLF1:
        if (sampleRate == 48000ul) {
            I2S_ioctl(i2s, use32bitFormat ?
                           i2sConfigDividers_UDA1361_48k_32b : i2sConfigDividers_UDA1361_48k_16b);
        }
        else if (sampleRate == 96000ul) {
            I2S_ioctl(i2s, use32bitFormat ?
                           i2sConfigDividers_UDA1361_96k_32b : i2sConfigDividers_UDA1361_96k_16b);
        }
        else {
            return LPCLIB_ILLEGAL_PARAMETER;
        }
        break;

    case BSP_BOARD_KAI3:
    case BSP_BOARD_ROLF2:
    case BSP_BOARD_ROLF3:
        if (sampleRate == 48000ul) {
            I2S_ioctl(i2s, use32bitFormat ?
                           i2sConfigDividers_AD1974_48k_32b : i2sConfigDividers_AD1974_48k_16b);
        }
        else if (sampleRate == 96000ul) {
            I2S_ioctl(i2s, use32bitFormat ?
                           i2sConfigDividers_AD1974_96k_32b : i2sConfigDividers_AD1974_96k_16b);
        }
        else if (sampleRate == 192000ul) {
            if (use32bitFormat) {
                return LPCLIB_ILLEGAL_PARAMETER;
            }

            I2S_ioctl(i2s, i2sConfigDividers_AD1974_192k_16b);
        }
        else {
            return LPCLIB_ILLEGAL_PARAMETER;
        }
        break;

    case BSP_BOARD_TEST:
    case BSP_BOARD_UNDEFINED:
        /* Nothing to do */
        break;
    }

    return LPCLIB_SUCCESS;
}



static const struct DAC_Config dacConfig[] = {
    {.opcode = DAC_OPCODE_SET_SAMPLERATE,
        {.sampleRateHz = 192000ul, }},

    {.opcode = DAC_OPCODE_SET_CALLBACK,
        {.callback = {
            .callback = NULL,
            .pOldCallback = NULL, }}},

    DAC_CONFIG_END
};



static DMA_ChannelHandle dacDma;


void BSP_openDac (const DAC_Job *pJob)
{
    BSP_BoardType boardType = BSP_getBoardType();

    switch (boardType) {
    case BSP_BOARD_ROLF1:
        DMA_acquireChannel(gpdma, &dacDma);
        DAC_open(&dac);
        DAC_ioctl(dac, dacConfig);
        DAC_submitJob(dac, pJob, dacDma);
        break;

    default:
        /* Nothing to do */
        break;
    }
}



void BSP_closeDac (void)
{
    BSP_BoardType boardType = BSP_getBoardType();

    switch (boardType) {
    case BSP_BOARD_ROLF1:
        DAC_close(&dac);
        DMA_releaseChannel(&dacDma);
        break;

    default:
        /* Nothing to do */
        break;
    }
}
