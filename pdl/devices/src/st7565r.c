
#include <stdint.h>
#include <string.h>

#include "lpclib.h"

#include "st7565r.h"


/** Local device context. */
typedef struct ST7565R_Context {
    SSP_Handle bus;                         /**< Bus to which device is connected. */
    const SSP_DeviceSelect *pDeviceSelect;  /**< SSEL demuxing. */
    uint16_t sizeX;                         /**< Frame buffer size x */
    uint16_t sizeY;                         /**< Frame buffer size y */
} ST7565R_Context;


static ST7565R_Context globalContext; //TODO: Must allocate memory dynamically!!


const SSP_Config st7565rDevice[] = {
    {.opcode = SSP_OPCODE_SET_FORMAT,
        {.format = {.bits = SSP_BITS_8,
                    .clockFormat = SSP_CLOCK_MODE3,
                    .frameFormat = SSP_FRAMEFORMAT_SPI, }}},
    {.opcode = SSP_OPCODE_SET_BITRATE,
        {.bitrate = 10000000ul, }},

    SSP_CONFIG_END
};


/** Initialization sequence. */
static const uint8_t st7565rInitSequence[] = {
#if 0
    0xA0,
    0xAE,
    0xC8,
    0xA2,
    0x2F,
    0x21,
    0x81,
    0x20,

    0xA4,
    0xAF,
#else
    0x40,
    0xA1,
    0xC0,
    0xA6,
    0xA2,
    0x2F,
    0xF8,
    0x00,
    0x27,
    0x81,
    0x16,

//    0xAC, 0x00,
    0xAD, 0x01,

    0xAF,
#endif
};



/* Write a command or data string */
LPCLIB_Result ST7565R_writeGeneric (ST7565R_Handle handle,
                                    int numBytes,
                                    const void *pBuffer,
                                    enum ST7565R_CommandDataSelect select)
{
    SSP_Job job;
    SSP_JobPhase phase[1];


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    job.pDeviceSelect = handle->pDeviceSelect;
    job.pConfig = st7565rDevice;
    job.firstPhase = SSP_makePhase(&phase[0], pBuffer, NULL, numBytes);
    job.extraParameter = (void *)select;                /* 0 --> Command, 1 --> Data */

    if (SSP_submitJob(handle->bus, &job) != LPCLIB_SUCCESS) {
        return LPCLIB_ERROR;
    }

    return LPCLIB_SUCCESS;
}


/* Open access to device */
LPCLIB_Result ST7565R_open (SSP_Handle bus, const SSP_DeviceSelect *pDeviceSelect, ST7565R_Handle *pHandle)
{
    ST7565R_Context *pContext = &globalContext; //TODO


    pContext->bus = bus;
    pContext->pDeviceSelect = pDeviceSelect;

    *pHandle = pContext;
pContext->sizeX = 128;
pContext->sizeY = 64;
    return LPCLIB_SUCCESS;
}



/* Close device */
LPCLIB_Result ST7565R_close (ST7565R_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Configure the device */
LPCLIB_Result ST7565R_ioctl (ST7565R_Handle handle, const ST7565R_Config *pConfig)
{
    while (pConfig->opcode != ST7565R_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case ST7565R_OPCODE_INIT:
            return ST7565R_writeCommand(handle,
                                        sizeof(st7565rInitSequence),
                                        st7565rInitSequence);

        case ST7565R_OPCODE_CLEAR_SCREEN:
            //TODO
            break;

        case ST7565R_OPCODE_INVALID:
            /* Nothing to do */
            break;
        }

        ++pConfig;                                      /* More config's follow */
    }

    return LPCLIB_SUCCESS;
}



/* Write complete frame buffer */
LPCLIB_Result ST7565R_writeFullScreen (ST7565R_Handle handle, const uint8_t *frameBuffer)
{
    uint8_t data[3];
    LPCLIB_Result result = LPCLIB_SUCCESS;
    int page;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Loop over all 'pages' (8 rows each) */
    for (page = 0; page < (1 + ((handle->sizeY - 1) / 8)); page++) {
        /* Set write position to start of row. */
        data[0] = 0x10;
        data[1] = 0x00;
        data[2] = 0xB0 + page;
        result = ST7565R_writeCommand(handle, 3, data);

        /* Send one row of data */
        if (result == LPCLIB_SUCCESS) {
            result = ST7565R_writeData(handle,
                                       handle->sizeX,
                                       &frameBuffer[page * handle->sizeX]);
        }

        /* Stop here if something went wrong */
        if (result != LPCLIB_SUCCESS) {
            break;
        }
    }

    return result;
}

