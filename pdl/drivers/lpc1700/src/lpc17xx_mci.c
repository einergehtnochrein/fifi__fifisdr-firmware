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

#if LPCLIB_MCI

/** \file
 *  \brief MCI driver implementation.
 *
 *  This file contains the driver code for the MCI peripheral.
 *
 *  \author NXP Semiconductors
 */


#include "lpc17xx_mci.h"
#include "lpc17xx_clkpwr.h"
#include "lpc17xx_dma.h"


#if !LPCLIB_DMA
#define DMA_ChannelHandle struct DMA_ChannelContext *
#endif

/** Card clock frequency (MCICLK) in case of interrupt driven transfer. */
#define MCICLK_INTERRUPT_MODE                   1000000ul


/** Field definition for hardware register MCIPWR. */
LPCLIB_DefineRegBit(MCI_PWR_CTRL,               0,  2);
LPCLIB_DefineRegBit(MCI_PWR_OPENDRAIN,          6,  1);
LPCLIB_DefineRegBit(MCI_PWR_ROD,                7,  1);

enum {
    MCI_PWR_CTRL_OFF = 0,
    MCI_PWR_CTRL_UP = 2,
    MCI_PWR_CTRL_ON = 3,
};
enum {
    MCI_PWR_OPENDRAIN_OFF = 0,
    MCI_PWR_OPENDRAIN_ON = 1,
};
enum {
    MCI_PWR_ROD_OFF = 0,
    MCI_PWR_ROD_ON = 1,
};

/** Field definition for hardware register MCIClock. */
LPCLIB_DefineRegBit(MCI_CLOCK_CLKDIV,           0,  8);
LPCLIB_DefineRegBit(MCI_CLOCK_ENABLE,           8,  1);
LPCLIB_DefineRegBit(MCI_CLOCK_PWRSAVE,          9,  1);
LPCLIB_DefineRegBit(MCI_CLOCK_BYPASS,           10, 1);
LPCLIB_DefineRegBit(MCI_CLOCK_WIDEBUS,          11, 1);

/** Field definition for hardware register MCICommand. */
LPCLIB_DefineRegBit(MCI_COMMAND_CMDINDEX,       0,  6);
LPCLIB_DefineRegBit(MCI_COMMAND_RESPONSE,       6,  1);
LPCLIB_DefineRegBit(MCI_COMMAND_LONGRSP,        7,  1);
LPCLIB_DefineRegBit(MCI_COMMAND_INTERRUPT,      8,  1);
LPCLIB_DefineRegBit(MCI_COMMAND_PENDING,        9,  1);
LPCLIB_DefineRegBit(MCI_COMMAND_ENABLE,         10, 1);

/** Field definition for hardware register MCIRespCommand. */
LPCLIB_DefineRegBit(MCI_RESPCOMMAND_RESPCMD,    0,  6);

/** Field definition for hardware register MCIDataLength. */
LPCLIB_DefineRegBit(MCI_DATALENGTH_DATALENGTH,  0,  16);

/** Field definition for hardware register MCIDataCtrl. */
LPCLIB_DefineRegBit(MCI_DATACTRL_ENABLE,        0,  1);
LPCLIB_DefineRegBit(MCI_DATACTRL_DIRECTION,     1,  1);
LPCLIB_DefineRegBit(MCI_DATACTRL_MODE,          2,  1);
LPCLIB_DefineRegBit(MCI_DATACTRL_DMAENABLE,     3,  1);
LPCLIB_DefineRegBit(MCI_DATACTRL_BLOCKSIZE,     4,  4);

enum {
    MCI_DATACTRL_BLOCKSIZE_512 = 9,
};

/** Field definition for hardware register MCIDataCnt. */
LPCLIB_DefineRegBit(MCI_DATACNT_DATACOUNT,      0,  16);

/** Field definition for hardware register MCIStatus. */
LPCLIB_DefineRegBit(MCI_STATUS_CMDCRCFAIL,      0,  1);
LPCLIB_DefineRegBit(MCI_STATUS_DATACRCFAIL,     1,  1);
LPCLIB_DefineRegBit(MCI_STATUS_CMDTIMEOUT,      2,  1);
LPCLIB_DefineRegBit(MCI_STATUS_DATATIMEOUT,     3,  1);
LPCLIB_DefineRegBit(MCI_STATUS_TXUNDERRUN,      4,  1);
LPCLIB_DefineRegBit(MCI_STATUS_RXOVERRUN,       5,  1);
LPCLIB_DefineRegBit(MCI_STATUS_CMDRESPOND,      6,  1);
LPCLIB_DefineRegBit(MCI_STATUS_CMDSENT,         7,  1);
LPCLIB_DefineRegBit(MCI_STATUS_DATAEND,         8,  1);
LPCLIB_DefineRegBit(MCI_STATUS_STARTBITERR,     9,  1);
LPCLIB_DefineRegBit(MCI_STATUS_DATABLOCKEND,    10, 1);
LPCLIB_DefineRegBit(MCI_STATUS_CMDACTIVE,       11, 1);
LPCLIB_DefineRegBit(MCI_STATUS_TXACTIVE,        12, 1);
LPCLIB_DefineRegBit(MCI_STATUS_RXACTIVE,        13, 1);
LPCLIB_DefineRegBit(MCI_STATUS_TXFIFOHALFEMPTY, 14, 1);
LPCLIB_DefineRegBit(MCI_STATUS_RXFIFOHALFFULL,  15, 1);
LPCLIB_DefineRegBit(MCI_STATUS_TXFIFOFULL,      16, 1);
LPCLIB_DefineRegBit(MCI_STATUS_RXFIFOFULL,      17, 1);
LPCLIB_DefineRegBit(MCI_STATUS_TXFIFOEMPTY,     18, 1);
LPCLIB_DefineRegBit(MCI_STATUS_RXFIFOEMPTY,     19, 1);
LPCLIB_DefineRegBit(MCI_STATUS_TXDATAAVLBL,     20, 1);
LPCLIB_DefineRegBit(MCI_STATUS_RXDATAAVLBL,     21, 1);

/** Field definition for hardware register MCIClear. */
LPCLIB_DefineRegBit(MCI_CLEAR_CMDCRCFAILCLR,    0,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_DATACRCFAILCLR,   1,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_CMDTIMEOUTCLR,    2,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_DATATIMEOUTCLR,   3,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_TXUNDERRUNCLR,    4,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_RXOVERRUNCLR,     5,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_CMDRESPENDCLR,    6,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_CMDSENTCLR,       7,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_DATAENDCLR,       8,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_STARTBITERRCLR,   9,  1);
LPCLIB_DefineRegBit(MCI_CLEAR_DATABLOCKENDCLR,  10, 1);

/* Mask that includes all MCI_CLEAR bits. */
#define MCI_CLEAR_Msk ( 0               \
      | MCI_CLEAR_CMDCRCFAILCLR_Msk     \
      | MCI_CLEAR_DATACRCFAILCLR_Msk    \
      | MCI_CLEAR_CMDTIMEOUTCLR_Msk     \
      | MCI_CLEAR_DATATIMEOUTCLR_Msk    \
      | MCI_CLEAR_TXUNDERRUNCLR_Msk     \
      | MCI_CLEAR_RXOVERRUNCLR_Msk      \
      | MCI_CLEAR_CMDRESPENDCLR_Msk     \
      | MCI_CLEAR_CMDSENTCLR_Msk        \
      | MCI_CLEAR_DATAENDCLR_Msk        \
      | MCI_CLEAR_STARTBITERRCLR_Msk    \
      | MCI_CLEAR_DATABLOCKENDCLR_Msk   \
      )


/** Field definition for hardware register MCIMask0. */
LPCLIB_DefineRegBit(MCI_MASK0_CMDCRCFAIL,       0,  1);
LPCLIB_DefineRegBit(MCI_MASK0_DATACRCFAIL,      1,  1);
LPCLIB_DefineRegBit(MCI_MASK0_CMDTIMEOUT,       2,  1);
LPCLIB_DefineRegBit(MCI_MASK0_DATATIMEOUT,      3,  1);
LPCLIB_DefineRegBit(MCI_MASK0_TXUNDERRUN,       4,  1);
LPCLIB_DefineRegBit(MCI_MASK0_RXOVERRUN,        5,  1);
LPCLIB_DefineRegBit(MCI_MASK0_CMDRESPOND,       6,  1);
LPCLIB_DefineRegBit(MCI_MASK0_CMDSENT,          7,  1);
LPCLIB_DefineRegBit(MCI_MASK0_DATAEND,          8,  1);
LPCLIB_DefineRegBit(MCI_MASK0_STARTBITERR,      9,  1);
LPCLIB_DefineRegBit(MCI_MASK0_DATABLOCKEND,     10, 1);
LPCLIB_DefineRegBit(MCI_MASK0_CMDACTIVE,        11, 1);
LPCLIB_DefineRegBit(MCI_MASK0_TXACTIVE,         12, 1);
LPCLIB_DefineRegBit(MCI_MASK0_RXACTIVE,         13, 1);
LPCLIB_DefineRegBit(MCI_MASK0_TXFIFOHALFEMPTY,  14, 1);
LPCLIB_DefineRegBit(MCI_MASK0_RXFIFOHALFFULL,   15, 1);
LPCLIB_DefineRegBit(MCI_MASK0_TXFIFOFULL,       16, 1);
LPCLIB_DefineRegBit(MCI_MASK0_RXFIFOFULL,       17, 1);
LPCLIB_DefineRegBit(MCI_MASK0_TXFIFOEMPTY,      18, 1);
LPCLIB_DefineRegBit(MCI_MASK0_RXFIFOEMPTY,      19, 1);
LPCLIB_DefineRegBit(MCI_MASK0_TXDATAAVLBL,      20, 1);
LPCLIB_DefineRegBit(MCI_MASK0_RXDATAAVLBL,      21, 1);

/** Field definition for hardware register MCIFifoCnt. */
LPCLIB_DefineRegBit(MCI_FIFOCNT_DATACOUNT,      0,  15);


/** Command codes for SD Card */
typedef enum {
    MCI_CMD0__GO_IDLE_STATE             = 0,
    MCI_CMD1__SEND_OP_COND              = 1,
    MCI_CMD2__ALL_SEND_CID              = 2,
    MCI_CMD3__SEND_RELATIVE_ADDR        = 3,
    MCI_CMD4__SET_DSR                   = 4,
    MCI_CMD7__SELECT_DESELECT_CARD      = 7,
    MCI_CMD8__SEND_IF_COND              = 8,
    MCI_CMD9__SEND_CSD                  = 9,
    MCI_CMD10__SEND_CID                 = 10,
    MCI_CMD12__STOP_TRANSMISSION        = 12,
    MCI_CMD13__SEND_STATUS              = 13,
    MCI_CMD15__GO_INACTIVE_STATE        = 15,
    MCI_CMD16__SET_BLOCKLEN             = 16,
    MCI_CMD17__READ_SINGLE_BLOCK        = 17,
    MCI_CMD18__READ_MULTIPLE_BLOCK      = 18,
    MCI_CMD24__WRITE_BLOCK              = 24,
    MCI_CMD25__WRITE_MULTIPLE_BLOCK     = 25,
    MCI_CMD27__PROGRAM_CSD              = 27,
    MCI_CMD32__ERASE_WR_BLK_START       = 32,
    MCI_CMD33__ERASE_WR_BLK_END         = 33,
    MCI_CMD38__ERASE                    = 38,
    MCI_CMD42__LOCK_UNLOCK              = 42,
    MCI_CMD55__APP_CMD                  = 55,
    MCI_ACMD6__SET_BUS_WIDTH            = 6,
    MCI_ACMD13__SD_STATUS               = 13,
    MCI_ACMD22__SEND_NUM_WR_BLOCKS      = 22,
    MCI_ACMD23__SET_WR_BLK_ERASE_COUNT  = 23,
    MCI_ACMD41__SD_SEND_OP_COND         = 41,
    MCI_ACMD42__SET_CLK_CARD_DETECT     = 42,
    MCI_ACMD51__SEND_SCR                = 51,
} MCI_CommandOpcode;


/** Card response type. */
typedef enum {
    MCI_RESPONSE_NONE   = 0,
    MCI_RESPONSE_R1     = MCI_COMMAND_RESPONSE_Msk,
    MCI_RESPONSE_R1b    = MCI_COMMAND_RESPONSE_Msk,
    MCI_RESPONSE_R2     = MCI_COMMAND_RESPONSE_Msk | MCI_COMMAND_LONGRSP_Msk,
    MCI_RESPONSE_R3     = MCI_COMMAND_RESPONSE_Msk,
    MCI_RESPONSE_R6     = MCI_COMMAND_RESPONSE_Msk,
    MCI_RESPONSE_R7     = MCI_COMMAND_RESPONSE_Msk,
} MCI_ResponseType;


/** Flags for card features. */
typedef struct {
    unsigned int allow4Bit : 1;             /**< Supports four data lines */
    unsigned int isSDCard : 1;              /**< Is an SD card (and not MMC) */
    unsigned int supportSDV2 : 1;           /**< Supports SD card spec V2.00 */
    unsigned int isHighCapacity : 1;        /**< Is an SDHC card */
} MCI_Flags;



/** SD card states as described in SD Card official spec. */
typedef enum {
    MCI_STATE_UNKNOWN,                      /**< Unknown state. Use CMD0 to reach IDLE state */
    MCI_STATE_IDLE,                         /**< After power-on or CMD0 */
    MCI_STATE_IDLE_SENT_CMD8,               /**< After sending CMD8 from IDLE state */
    MCI_STATE_IDLE_SENT_ACMD41,             /**< After sending ACMD41 from IDLE state */
    MCI_STATE_READY_SENT_CMD2,              /**< After sending CMD2 in READY state */
    MCI_STATE_INACTIVE,
    MCI_STATE_IDENT_SENT_CMD3,              /**< After sending CMD3 in IDENT state */
    MCI_STATE_STANDBY_SENT_CMD9,            /**< After sending CMD9 in STANDBY state */
    MCI_STATE_STANDBY_SENT_CMD7,            /**< After sending SELECT (CMD7) in STANDBY state */
    MCI_STATE_STANDBY,
    MCI_STATE_TRANSFER_SENT_ACMD6,          /**< After sending ACMD6 when entering TRANSFER state */
    MCI_STATE_TRANSFER,
    MCI_STATE_DATA,
    MCI_STATE_DATA_SENT_CMD12,              /**< After sending CMD12 in DATA state (multiple block read) */
    MCI_STATE_RECEIVE,
    MCI_STATE_RECEIVE_SENT_CMD12,           /**< After sending CMD12 in RCV state (multiple block write) */
    MCI_STATE_PROGRAM,                      /**< Programming operation active */
    MCI_STATE_DISCONNECT,
} MCI_State;


/** CSD register version */
typedef enum MCI_CSDVersion {
    MCI_CSD_VERSION1 = 0,                   /**< CSD structure version 1.x */
    MCI_CSD_VERSION2 = 1,                   /**< CSD structure version 2.0 */
} MCI_CSDVersion;


/** Card Status Register fields */
LPCLIB_DefineRegBit(MCI_CARDSTATUS_READY_FOR_DATA,  8,  1);
LPCLIB_DefineRegBit(MCI_CARDSTATUS_CURRENT_STATE,   9,  4);


/** Device context. */
static struct MCI_Context {
    LPCLIB_Switch inUse;                    /**< Set if interface open */
    MCI_State state;
    MCI_Flags flags;                        /**< Card features and operating flags */
    uint32_t maxClockHz;                    /**< Maximum allowed card clock */
    int numRetriesACMD41;                   /**< Counts the retries of ACMD41 in init phase */
    uint32_t ocr;                           /**< Card's OCR register */
    uint32_t cid[4];                        /**< Card's CID register */
    uint32_t csd[4];                        /**< Card's CSD register */
    uint32_t capacity;                      /**< Storage capacity (in sectors) */
    uint32_t rca;                           /**< Assigned card address (RCA) */
    LPCLIB_Result result;                   /**< Used to pass R/W operation result */

    osMutexId accessMutex;
    osSemaphoreId syncSema;

    LPCLIB_Callback callback;               /**< Event handler */

    const uint32_t *pTx;
    uint32_t *pRx;

    DMA_ChannelHandle dma;                  /**< Handle of DMA channel (both read and write) */
    DMA_Job *pDmaJob;                       /**< DMA job */
} mciContext;


osMutexDef(mciAccessMutexDef);
osSemaphoreDef(mciSyncSemaDef);


/** Helper macros to control the CMD pin open-drain state. */
#define _MCI_enableOpenDrain()                                          \
    do {                                                                \
        LPCLIB_BITBAND(&LPC_MCI->POWER, MCI_PWR_OPENDRAIN_Pos) = 1;     \
    } while (0)

#define _MCI_disableOpenDrain()                                         \
    do {                                                                \
        LPCLIB_BITBAND(&LPC_MCI->POWER, MCI_PWR_OPENDRAIN_Pos) = 0;     \
    } while (0)


/** Helper function to prepare a COMMAND register word. */
static uint32_t _MCI_makeCommandWord (MCI_CommandOpcode command,
                                      MCI_ResponseType response,
                                      uint32_t flags)
{
    return flags | (command << MCI_COMMAND_CMDINDEX_Pos) | response;
}

/** Helper function to extract CSD version number. */
static MCI_CSDVersion _MCI_getCSDVersion (MCI_Handle handle)
{
    return (MCI_CSDVersion)((handle->csd[3] >> 30) & 0x03);
}

/** Helper function to extract card capacity (in sectors) from CSD structure. */
static uint32_t _MCI_getCardCapacity (MCI_Handle handle)
{
    uint32_t numSectors;

    if (_MCI_getCSDVersion(handle) == MCI_CSD_VERSION2) {
        numSectors = 1 + (((handle->csd[2] & 0x3F) << 16) | ((handle->csd[1] >> 16) & 0xFFFF));
    }
    else {
        numSectors =
                (1 + (((handle->csd[2] & 0x3FF) << 2) | ((handle->csd[1] >> 30) & 0x03)))
             *  (1u << (2 + ((handle->csd[1] >> 15) & 0x07)))
             *  (1u << ((handle->csd[2] >> 16) & 0x0F));
        numSectors = numSectors / 512;
    }

    return numSectors;
}



/** Send a command. */
static void MCI_sendCommand (uint32_t command, uint32_t argument, uint32_t intMask)
{
    LPC_MCI->MASK0 = intMask;
    LPC_MCI->ARGUMENT = argument;
    LPC_MCI->COMMAND = command;
}


/** Set the card clock frequency. */
static LPCLIB_Result MCI_setCardClock (MCI_Handle handle, uint32_t frequency)
{
    int divider;


    /* Limit to maximum board frequency */
    if (frequency > handle->maxClockHz) {
        frequency = handle->maxClockHz;
    }
    divider = (CLKPWR_getBusClock(CLKPWR_CLOCK_SDC) - 1) / (2 * frequency) + 1;
    if (frequency <= 400000ul) {
        LPC_MCI->CLOCK =
                (1 << MCI_CLOCK_ENABLE_Pos)
            |   (((divider - 1) << MCI_CLOCK_CLKDIV_Pos) & MCI_CLOCK_CLKDIV_Msk);
    }
    else {
        LPC_MCI->CLOCK =
                (1 << MCI_CLOCK_ENABLE_Pos)
            |   (1 << MCI_CLOCK_PWRSAVE_Pos)
            |   (((divider - 1) << MCI_CLOCK_CLKDIV_Pos) & MCI_CLOCK_CLKDIV_Msk);
    }

    return LPCLIB_SUCCESS;
}


/* Open the MCI block. */
LPCLIB_Result MCI_open (MCI_Name mciNum, MCI_Handle *pHandle)
{
    (void) mciNum;                                      /* The LPC has one MCI block only */
    MCI_Handle handle = &mciContext;

    CLKPWR_assertPeripheralReset(CLKPWR_RESET_SDC);     /* Assert MCI reset */
    CLKPWR_deassertPeripheralReset(CLKPWR_RESET_SDC);   /* Release MCI reset */
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SDC);         /* Enable MCI peripheral clock */

    if (!handle->inUse) {
        handle->accessMutex = osMutexCreate(osMutex(mciAccessMutexDef));
        handle->syncSema = osSemaphoreCreate(osSemaphore(mciSyncSemaDef), 1);

        handle->inUse = LPCLIB_YES;
        *pHandle = handle;                              /* Return handle */

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;
}



/* Close the MCI interface. */
void MCI_close (MCI_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_SDC);        /* Disable MCI peripheral clock */

//TODO    LPCLIB_OSALMutexDelete((*pHandle)->accessMutex);
//TODO    LPCLIB_OSALSemaphoreDelete((*pHandle)->syncSema);

    (*pHandle)->inUse = LPCLIB_NO;
    *pHandle = LPCLIB_INVALID_HANDLE;                   /* Invalidate the handle */
}



/* Configure the MCI driver. */
void MCI_ioctl (MCI_Handle handle, const MCI_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    switch (pConfig->opcode) {
    case MCI_OPCODE_BUS_CONFIG:
        handle->maxClockHz = pConfig->bus.maxClockHz;
        handle->flags.allow4Bit = pConfig->bus.allow4Bit;
        break;

    case MCI_OPCODE_SET_CALLBACK:
        handle->callback = pConfig->callback;
        break;

    case MCI_OPCODE_CONNECT:
        if ((pConfig->connect.isInserted) && (handle->state == MCI_STATE_UNKNOWN)) {
            /* Card inserted! */
            MCI_setCardClock(handle, 400000ul);
            LPC_MCI->POWER =                /* Power UP: Power active, but clock still off */
                    (MCI_PWR_CTRL_UP << MCI_PWR_CTRL_Pos)
                |   (MCI_PWR_OPENDRAIN_ON << MCI_PWR_OPENDRAIN_Pos)
                |   (MCI_PWR_ROD_OFF << MCI_PWR_ROD_Pos);
            osDelay(5);                     /* Must be long enough to guarantee stable power */
            LPC_MCI->POWER =                /* Power ON: Power active, clock active */
                    (MCI_PWR_CTRL_ON << MCI_PWR_CTRL_Pos)
                |   (MCI_PWR_OPENDRAIN_ON << MCI_PWR_OPENDRAIN_Pos)
                |   (MCI_PWR_ROD_ON << MCI_PWR_ROD_Pos);
            osDelay(2);                     /* Must be >= 1 ms to guarantee card initialization */

            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD0__GO_IDLE_STATE,
                                MCI_RESPONSE_NONE,
                                MCI_COMMAND_ENABLE_Msk),
                            0,
                            MCI_MASK0_CMDSENT_Msk);
        }

        if (!(pConfig->connect.isInserted)) {   //TODO this is just a quick hack...
            /* Card removed! */
            LPC_MCI->MASK0 = 0; //no more interrupts

            handle->state = MCI_STATE_UNKNOWN;

            if (handle->dma != LPCLIB_INVALID_HANDLE) {
                DMA_cancelJob(handle->dma);
                handle->dma = LPCLIB_INVALID_HANDLE;
            }

            MCI_setCardClock(handle, 400000ul);

            LPC_MCI->POWER =                /* Power OFF */
                    (MCI_PWR_CTRL_OFF << MCI_PWR_CTRL_Pos)
                |   (MCI_PWR_OPENDRAIN_ON << MCI_PWR_OPENDRAIN_Pos)
                |   (MCI_PWR_ROD_OFF << MCI_PWR_ROD_Pos);
            osDelay(1);                     /* Wait >= 1 ms to ensure power is really off */
        }
        break;

    case MCI_OPCODE_SELECT_CARD:
        if (handle->state == MCI_STATE_STANDBY) {
            handle->state = MCI_STATE_STANDBY_SENT_CMD7;

            _MCI_disableOpenDrain();
            MCI_setCardClock(handle, handle->maxClockHz);

            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD7__SELECT_DESELECT_CARD,
                                MCI_RESPONSE_R1b,
                                MCI_COMMAND_ENABLE_Msk),
                            handle->rca,
                             MCI_MASK0_CMDCRCFAIL_Msk
                           | MCI_MASK0_CMDTIMEOUT_Msk
                           | MCI_MASK0_CMDRESPOND_Msk);
        }
        break;

    default:
        break;
    }
}


/** Callback for DMA events. */
static void MCI_dmaCallback (LPCLIB_Event event)
{
    const MCI_Handle handle = event.parameter;
    LPCLIB_Event mciEvent = {
        .id = LPCLIB_EVENTID_MCI,
        .parameter = 0, };

    if (handle != LPCLIB_INVALID_HANDLE) {
        if (handle->callback) {
            switch (event.opcode) {
            case DMA_EVENT_STOP:
                mciEvent.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
                handle->callback(mciEvent);             /* Inform about end of job */
                break;
            }
        }
    }
}


#include <stdio.h>

/* Read sector(s) from card. */
LPCLIB_Result MCI_read (MCI_Handle handle,
                        void *pData,
                        int firstSector,
                        int numSectors,
                        DMA_ChannelHandle dma)
{
    LPCLIB_Result result = LPCLIB_BUSY;
    DMA_Job *pDmaJob = NULL;

//printf("R: S=%d, n=%d\r\n", firstSector, numSectors);

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (numSectors == 0) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Card must be ready for a transfer */
    if (handle->state != MCI_STATE_TRANSFER) {
        return LPCLIB_NOT_PREPARED;
    }

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        /* Make sure to reset the sync flag */
        osSemaphoreWait(handle->syncSema, 0);

        handle->state = MCI_STATE_DATA;
        handle->pRx = pData;
        handle->result = LPCLIB_SUCCESS;

        /* Prepare DMA transfer if requested. */
        handle->dma = dma;
        if (handle->dma != LPCLIB_INVALID_HANDLE) {     /* DMA transfer */
            /* Ask DMA driver for memory */
            pDmaJob = DMA_allocPhaseMemory(handle->dma, 1);
            if (pDmaJob == NULL) {
                /* Fall back to interrupt mode */
                handle->dma = LPCLIB_INVALID_HANDLE;
            }
            else {
                handle->pDmaJob = pDmaJob;

                /* Prepare DMA job */
                pDmaJob->source = DMA_PERIPHERAL_SDCARD;
                pDmaJob->destination = DMA_PERIPHERAL_MEMORY;
                pDmaJob->flowControl = DMA_FLOW_PER2MEM_SRC;

                DMA_makePhase(
                    pDmaJob->firstPhase,
                    (uint32_t)&LPC_MCI->FIFO,
                    (uint32_t)pData,
                    handle,
                    DMA_makePhaseParameter(
                        0,                  /* Number of transfers determined by MCI block */
                        DMA_BURSTSIZE_8,    /* Source burst size */
                        DMA_BURSTSIZE_4,    /* Destination burst size */
                        DMA_WIDTH_32,       /* Source width */
                        DMA_WIDTH_32,       /* Destination width */
                        DISABLE,            /* Source increment */
                        ENABLE,             /* Destination increment */
                        DISABLE             /* No callback */
                        )
                    );

                pDmaJob->channel = handle->dma;
                pDmaJob->callback = MCI_dmaCallback;
                DMA_submitJob(dma, pDmaJob);
            }
        }

        /* Without DMA the clock frequency must be reduced drastically. */
        if (handle->dma == LPCLIB_INVALID_HANDLE) {
            MCI_setCardClock(handle, MCICLK_INTERRUPT_MODE);
        }

        /* Prepare for data phase. Different card commands apply depending on whether this is
         * a single or multiple sector read.
         */
        LPC_MCI->DATALENGTH = 512 * numSectors;
        LPC_MCI->DATATIMER = 0xFFFFFFFF;    //TODO
        LPC_MCI->DATACTRL =
                (MCI_DATACTRL_BLOCKSIZE_512 << MCI_DATACTRL_BLOCKSIZE_Pos)
              | ((handle->dma != LPCLIB_INVALID_HANDLE) ? MCI_DATACTRL_DMAENABLE_Msk : 0)
              | MCI_DATACTRL_DIRECTION_Msk
              | MCI_DATACTRL_ENABLE_Msk;
        LPC_MCI->ARGUMENT = (handle->flags.isHighCapacity) ? firstSector : 512 * firstSector;

        if (numSectors == 1) {
            LPC_MCI->MASK0 =
                    MCI_MASK0_DATACRCFAIL_Msk
                  | MCI_MASK0_DATATIMEOUT_Msk
                  | MCI_MASK0_DATABLOCKEND_Msk
                  | MCI_MASK0_CMDCRCFAIL_Msk
                  | MCI_MASK0_CMDTIMEOUT_Msk
                  /*| MCI_MASK0_CMDRESPOND_Msk */;
            LPC_MCI->COMMAND = _MCI_makeCommandWord(
                    MCI_CMD17__READ_SINGLE_BLOCK,
                    MCI_RESPONSE_R1,
                    MCI_COMMAND_ENABLE_Msk);
        }
        else {
            LPC_MCI->MASK0 =
                    MCI_MASK0_DATACRCFAIL_Msk
                  | MCI_MASK0_DATATIMEOUT_Msk
//                  | MCI_MASK0_DATABLOCKEND_Msk
                  | MCI_MASK0_DATAEND_Msk
                  | MCI_MASK0_CMDCRCFAIL_Msk
                  | MCI_MASK0_CMDTIMEOUT_Msk
                 /* | MCI_MASK0_CMDRESPOND_Msk*/;
            LPC_MCI->COMMAND = _MCI_makeCommandWord(
                    MCI_CMD18__READ_MULTIPLE_BLOCK,
                    MCI_RESPONSE_R1,
                    MCI_COMMAND_ENABLE_Msk);
        }

        /* Wait for the end of transaction. */
        if (osSemaphoreWait(handle->syncSema, 5000 /*TODO*/) > 0) {
            result = handle->result;
        }
        else {
            result = LPCLIB_ERROR;
        }

//printf(" -> %d\r\n", result);

        /* Return job/phase memory */
        if (handle->dma != LPCLIB_INVALID_HANDLE) {
            DMA_freePhaseMemory(handle->dma, pDmaJob);

            /* TODO
             * Kill this DMA job (if everything went fine, this is not necessary. But in case of an
             * error, the DMA job may not have run at all.
             */
            DMA_cancelJob(handle->dma);
        }

        osMutexRelease(handle->accessMutex);
    }

    return result;
}



/* Write sector(s) to card. */
LPCLIB_Result MCI_write (MCI_Handle handle,
                         const void *pData,
                         int firstSector,
                         int numSectors,
                         DMA_ChannelHandle dma)
{
    LPCLIB_Result result = LPCLIB_BUSY;
    DMA_Job *pDmaJob = NULL;

//printf("W: S=%d, n=%d\r\n", firstSector, numSectors);

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (numSectors == 0) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Card must be ready for a transfer */
    if (handle->state != MCI_STATE_TRANSFER) {
        return LPCLIB_NOT_PREPARED;
    }

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        /* Make sure to reset the sync flag */
        osSemaphoreWait(handle->syncSema, 0);

        handle->state = MCI_STATE_RECEIVE;
        handle->pTx = pData;
        handle->result = LPCLIB_SUCCESS;

        /* Prepare DMA transfer if requested. */
        handle->dma = dma;
        if (handle->dma != LPCLIB_INVALID_HANDLE) {     /* DMA transfer */
            /* Ask DMA driver for memory */
            pDmaJob = DMA_allocPhaseMemory(handle->dma, 1);
            if (pDmaJob == NULL) {
                /* Fall back to interrupt mode */
                handle->dma = LPCLIB_INVALID_HANDLE;
            }
            else {
                handle->pDmaJob = pDmaJob;

                /* Prepare DMA job */
                pDmaJob->source = DMA_PERIPHERAL_MEMORY;
                pDmaJob->destination = DMA_PERIPHERAL_SDCARD;
                pDmaJob->flowControl = DMA_FLOW_MEM2PER_DEST;

                DMA_makePhase(
                    pDmaJob->firstPhase,
                    (uint32_t)pData,
                    (uint32_t)&LPC_MCI->FIFO,
                    handle,
                    DMA_makePhaseParameter(
                        numSectors / 128,   /* Number of words in whole transaction */
                        DMA_BURSTSIZE_4,    /* Source burst size */
                        DMA_BURSTSIZE_8,    /* Destination burst size */
                        DMA_WIDTH_32,       /* Source width */
                        DMA_WIDTH_32,       /* Destination width */
                        ENABLE,             /* Source increment */
                        DISABLE,            /* Destination increment */
                        DISABLE             /* No callback */
                        )
                    );

                pDmaJob->channel = handle->dma;
                pDmaJob->callback = MCI_dmaCallback;
                DMA_submitJob(dma, pDmaJob);
            }
        }

        /* Without DMA the clock frequency must be reduced drastically. */
        if (handle->dma == LPCLIB_INVALID_HANDLE) {
            MCI_setCardClock(handle, MCICLK_INTERRUPT_MODE);
        }

        /* Prepare for data phase. Different card commands apply depending on whether this is
         * a single or multiple sector read.
         */
        LPC_MCI->DATALENGTH = 512 * numSectors;
        LPC_MCI->DATATIMER = 0xFFFFFFFF;    //TODO
        LPC_MCI->DATACTRL =
                (MCI_DATACTRL_BLOCKSIZE_512 << MCI_DATACTRL_BLOCKSIZE_Pos)
              | ((handle->dma != LPCLIB_INVALID_HANDLE) ? MCI_DATACTRL_DMAENABLE_Msk : 0);
        LPC_MCI->ARGUMENT = (handle->flags.isHighCapacity) ? firstSector : 512 * firstSector;

        if (numSectors == 1) {
            LPC_MCI->MASK0 =
                    MCI_MASK0_DATACRCFAIL_Msk
                  | MCI_MASK0_DATATIMEOUT_Msk
                  | MCI_MASK0_DATABLOCKEND_Msk
                  | MCI_MASK0_CMDCRCFAIL_Msk
                  | MCI_MASK0_CMDTIMEOUT_Msk
                  | MCI_MASK0_CMDRESPOND_Msk;
            LPC_MCI->COMMAND = _MCI_makeCommandWord(
                    MCI_CMD24__WRITE_BLOCK,
                    MCI_RESPONSE_R1,
                    MCI_COMMAND_ENABLE_Msk);
        }
        else {
            LPC_MCI->MASK0 =
                    MCI_MASK0_DATACRCFAIL_Msk
                  | MCI_MASK0_DATATIMEOUT_Msk
//                  | MCI_MASK0_DATABLOCKEND_Msk
                  | MCI_MASK0_DATAEND_Msk
                  | MCI_MASK0_CMDCRCFAIL_Msk
                  | MCI_MASK0_CMDTIMEOUT_Msk
                  | MCI_MASK0_CMDRESPOND_Msk;
            LPC_MCI->COMMAND = _MCI_makeCommandWord(
                    MCI_CMD25__WRITE_MULTIPLE_BLOCK,
                    MCI_RESPONSE_R1,
                    MCI_COMMAND_ENABLE_Msk);
        }

        /* Wait for the end of transaction. */
        if (osSemaphoreWait(handle->syncSema, 5000 /*TODO*/) > 0) {
            result = handle->result;
        }
        else {
            result = LPCLIB_ERROR;
        }

        if (handle->dma != LPCLIB_INVALID_HANDLE) {
            /* Return job/phase memory */
            DMA_freePhaseMemory(handle->dma, pDmaJob);

            /* TODO
             * Kill this DMA job (if everything went fine, this is not necessary. But in case of an
             * error, the DMA job may not have run at all.
             */
            DMA_cancelJob(handle->dma);
        }

        osMutexRelease(handle->accessMutex);
    }

    return result;
}



/** Send event to a registered callback */
static void MCI_sendCallback (MCI_Handle handle, LPCLIB_Event *pEvent)
{
    if (handle->callback) {
        handle->callback(*pEvent);
    }
}



/** MCI block interrupt entry (interrupt number \ref MCI_IRQn).
 */
void MCI_IRQHandler (void)
{
    uint32_t status;
    uint32_t cardStatus;
    MCI_Handle handle = &mciContext;
    bool fail;
    bool busy;
    LPCLIB_Event event;


    status = LPC_MCI->STATUS;                           /* Active interrupt requests */
    LPC_MCI->CLEAR = status & MCI_CLEAR_Msk;            /* NOTE: This works because STATUS and CLEAR
                                                         *       registers have corresponding bits
                                                         *       at the same position!
                                                         */

    /* Prepare event in case we need it */
    event.id = LPCLIB_EVENTID_MCI;
    event.opcode = MCI_EVENT_CARD_INVALID;              /* Assume the worst... */

    switch (handle->state) {
    case MCI_STATE_UNKNOWN:                 /* No info about card state available */
        if (status & MCI_STATUS_CMDSENT_Msk) {          /* Has CMD0 been sent? */
            /* Check if card responds to CMD8 */
            handle->state = MCI_STATE_IDLE_SENT_CMD8;
            LPC_MCI->MASK0 = MCI_MASK0_CMDCRCFAIL_Msk
                           | MCI_MASK0_CMDTIMEOUT_Msk
                           | MCI_MASK0_CMDRESPOND_Msk;
            LPC_MCI->ARGUMENT = 0x000001AA;             /* Indicate 2.7...3.6V */
            LPC_MCI->COMMAND = _MCI_makeCommandWord(
                    MCI_CMD8__SEND_IF_COND,
                    MCI_RESPONSE_R7,
                    MCI_COMMAND_ENABLE_Msk);
            handle->flags.supportSDV2 = 0;
        }
        else {
            /* Send CMD0 to bring card into idle state */
            LPC_MCI->MASK0 = MCI_MASK0_CMDSENT_Msk;     /* Possible events */
            LPC_MCI->ARGUMENT = 0;
            LPC_MCI->COMMAND = MCI_CMD0__GO_IDLE_STATE | MCI_COMMAND_ENABLE_Msk;
        }
        break;

    case MCI_STATE_IDLE_SENT_CMD8:          /* Check reply to CMD8.
                                             * A correct response indicates V2 spec compatibility.
                                             * Non-V2 compatible cards do not respond.
                                             */
        /* A card is invalid if its response does not echo our pattern,
         * or if the response has a CRC failure.
         */
        fail =  ((status & MCI_STATUS_CMDRESPOND_Msk) && ((LPC_MCI->RESPONSE0 & 0xFF) != 0xAA))
             || (status & MCI_STATUS_CMDCRCFAIL_Msk);
        if (fail) {
            handle->state = MCI_STATE_INACTIVE;
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        else {
            handle->state = MCI_STATE_IDLE_SENT_ACMD41;

            if (status & MCI_STATUS_CMDRESPOND_Msk) {   /* Successful response? */
                handle->flags.supportSDV2 = 1;          /* V2 compatible */
            }

            /* Send APP_CMD (prepare ACMD41) */
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD55__APP_CMD,
                                MCI_RESPONSE_R1,
                                MCI_COMMAND_ENABLE_Msk),
                            0,
                             MCI_MASK0_CMDCRCFAIL_Msk
                           | MCI_MASK0_CMDTIMEOUT_Msk
                           | MCI_MASK0_CMDRESPOND_Msk);

            handle->numRetriesACMD41 = 0;
        }
        break;

    case MCI_STATE_IDLE_SENT_ACMD41:
        /* Was it CMD55 or a completed ACMD41? */
        if (LPC_MCI->RESPCMD == MCI_CMD55__APP_CMD) {
            /* Send the ACMD41 now */
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_ACMD41__SD_SEND_OP_COND,
                                MCI_RESPONSE_R3,
                                MCI_COMMAND_ENABLE_Msk),
                            handle->flags.supportSDV2 ? 0x40FC0000 : 0x00FC0000,    /* 3.0...3.6 V */
                             MCI_MASK0_CMDCRCFAIL_Msk
                           | MCI_MASK0_CMDTIMEOUT_Msk
                           | MCI_MASK0_CMDRESPOND_Msk);
        }
        else {
            /* A timeout indicates that this is an MMC card. Continue with CMD1 */
            if (status & MCI_STATUS_CMDTIMEOUT_Msk) {
                /* This is not an SD Card */
                handle->flags.isSDCard = 0;
            }
            else {
                /* We come here if we have a response (CRC failure indicates success here!) */
                busy = (LPC_MCI->RESPONSE0 & (1u << 31)) == 0;
                if (!busy) {
                    /* This is an SD Card */
                    handle->flags.isSDCard = 1;

                    /* End of init phase. */
                    handle->ocr = LPC_MCI->RESPONSE0;
                    handle->flags.isHighCapacity = (handle->ocr & (1u << 30)) != 0;

                    /* READY state. Send CMD2 to ask for CID register */
                    MCI_sendCommand(_MCI_makeCommandWord(
                                        MCI_CMD2__ALL_SEND_CID,
                                        MCI_RESPONSE_R2,
                                        MCI_COMMAND_ENABLE_Msk),
                                    0,
                                     MCI_MASK0_CMDCRCFAIL_Msk
                                   | MCI_MASK0_CMDTIMEOUT_Msk
                                   | MCI_MASK0_CMDRESPOND_Msk);
                    handle->state = MCI_STATE_READY_SENT_CMD2;
                }
                else {
                    /* Repeat up to one second */
                    ++handle->numRetriesACMD41;
                    if (handle->numRetriesACMD41 >= 4000) {
                        handle->state = MCI_STATE_INACTIVE;
                        MCI_sendCallback(handle, &event);   /* Inform application */
                    }
                    else {
                        /* Try again */
                        MCI_sendCommand(_MCI_makeCommandWord(
                                            MCI_CMD55__APP_CMD,
                                            MCI_RESPONSE_R1,
                                            MCI_COMMAND_ENABLE_Msk),
                                        0,
                                         MCI_MASK0_CMDCRCFAIL_Msk
                                       | MCI_MASK0_CMDTIMEOUT_Msk
                                       | MCI_MASK0_CMDRESPOND_Msk);
                    }
                }
            }
        }
        break;

    case MCI_STATE_READY_SENT_CMD2:
        if (status & MCI_STATUS_CMDRESPOND_Msk) {       /* Successful response? */
            /* Store the CID register */
            handle->cid[0] = LPC_MCI->RESPONSE3;
            handle->cid[1] = LPC_MCI->RESPONSE2;
            handle->cid[2] = LPC_MCI->RESPONSE1;
            handle->cid[3] = LPC_MCI->RESPONSE0;

            /* IDENT state. Assign a card address (RCA) */
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD3__SEND_RELATIVE_ADDR,
                                MCI_RESPONSE_R6,
                                MCI_COMMAND_ENABLE_Msk),
                            0,
                             MCI_MASK0_CMDCRCFAIL_Msk
                           | MCI_MASK0_CMDTIMEOUT_Msk
                           | MCI_MASK0_CMDRESPOND_Msk);

            handle->state = MCI_STATE_IDENT_SENT_CMD3;
        }
        else {
            handle->state = MCI_STATE_INACTIVE;
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        break;

    case MCI_STATE_IDENT_SENT_CMD3:
        if (status & MCI_STATUS_CMDRESPOND_Msk) {       /* Successful response? */
            /* Extract RCA fromcard response */
            handle->rca = LPC_MCI->RESPONSE0 & 0xFFFF0000;

            /* Entering STANDBY state. Send CMD9 to ask for CSD register */
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD9__SEND_CSD,
                                MCI_RESPONSE_R2,
                                MCI_COMMAND_ENABLE_Msk),
                            handle->rca,
                            MCI_MASK0_CMDCRCFAIL_Msk
                          | MCI_MASK0_CMDTIMEOUT_Msk
                          | MCI_MASK0_CMDRESPOND_Msk);

            handle->state = MCI_STATE_STANDBY_SENT_CMD9;
        }
        else {
            handle->state = MCI_STATE_INACTIVE;
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        break;

    case MCI_STATE_STANDBY_SENT_CMD9:
        if (status & MCI_STATUS_CMDRESPOND_Msk) {       /* Successful response? */
            handle->state = MCI_STATE_STANDBY;

            /* Store the CSD register */
            handle->csd[0] = LPC_MCI->RESPONSE3;
            handle->csd[1] = LPC_MCI->RESPONSE2;
            handle->csd[2] = LPC_MCI->RESPONSE1;
            handle->csd[3] = LPC_MCI->RESPONSE0;

            /* Extract card capacity */
            handle->capacity = _MCI_getCardCapacity(handle);

            event.opcode = MCI_EVENT_CARD_STANDBY;
            event.parameter = (void *)handle->capacity;
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        else {
            handle->state = MCI_STATE_INACTIVE;
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        break;

    case MCI_STATE_STANDBY_SENT_CMD7:
        if (status & MCI_STATUS_CMDRESPOND_Msk) {       /* Successful response? */
            handle->state = MCI_STATE_TRANSFER_SENT_ACMD6;

            /* Send APP_CMD (prepare ACMD6) */
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD55__APP_CMD,
                                MCI_RESPONSE_R1,
                                MCI_COMMAND_ENABLE_Msk),
                            handle->rca,
                             MCI_MASK0_CMDCRCFAIL_Msk
                           | MCI_MASK0_CMDTIMEOUT_Msk
                           | MCI_MASK0_CMDRESPOND_Msk);
        }
        else {
            handle->state = MCI_STATE_INACTIVE;
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD0__GO_IDLE_STATE,
                                MCI_RESPONSE_NONE,
                                MCI_COMMAND_ENABLE_Msk),
                            0,
                            MCI_MASK0_CMDSENT_Msk);
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        break;

    case MCI_STATE_TRANSFER_SENT_ACMD6:
        if (status & MCI_STATUS_CMDRESPOND_Msk) {       /* Successful response? */
            /* Was it CMD55 or a completed ACMD6? */
            if (LPC_MCI->RESPCMD == MCI_CMD55__APP_CMD) {
                /* Send ACMD6 */
                MCI_sendCommand(_MCI_makeCommandWord(
                                    MCI_ACMD6__SET_BUS_WIDTH,
                                    MCI_RESPONSE_R1,
                                    MCI_COMMAND_ENABLE_Msk),
                                handle->flags.allow4Bit ? 2 : 0,
                                MCI_MASK0_CMDCRCFAIL_Msk
                            | MCI_MASK0_CMDTIMEOUT_Msk
                            | MCI_MASK0_CMDRESPOND_Msk);
            }
            else {
                /* Set MCI block data bus width */
                LPCLIB_BITBAND(&LPC_MCI->CLOCK, MCI_CLOCK_WIDEBUS_Pos) =
                        handle->flags.allow4Bit ? 1 : 0;

                handle->state = MCI_STATE_TRANSFER;

                event.opcode = MCI_EVENT_CARD_SELECTED;
                event.parameter = (void *)handle->capacity;
                MCI_sendCallback(handle, &event);           /* Inform application */
            }
        }
        else {
            handle->state = MCI_STATE_INACTIVE;
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD0__GO_IDLE_STATE,
                                MCI_RESPONSE_NONE,
                                MCI_COMMAND_ENABLE_Msk),
                            0,
                            MCI_MASK0_CMDSENT_Msk);
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        break;

    case MCI_STATE_DATA:
        /* Any failure (CRC fail, timeout)? */
        if (status & (
                MCI_STATUS_CMDCRCFAIL_Msk
              | MCI_STATUS_CMDTIMEOUT_Msk
              | MCI_STATUS_DATACRCFAIL_Msk
              | MCI_STATUS_DATATIMEOUT_Msk)) {
            handle->result = LPCLIB_CRC_FAILURE;

            event.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
            osSemaphoreRelease(handle->syncSema);
        }

        /* Successful end of transfer? */
        else if (status & MCI_STATUS_DATAEND_Msk) {
            if (LPC_MCI->RESPCMD == MCI_CMD18__READ_MULTIPLE_BLOCK) {
                /* Send CMD12 (Stop) */
                MCI_sendCommand(_MCI_makeCommandWord(
                                    MCI_CMD12__STOP_TRANSMISSION,
                                    MCI_RESPONSE_R1b,
                                    MCI_COMMAND_ENABLE_Msk),
                                0,
                                MCI_MASK0_CMDCRCFAIL_Msk
                              | MCI_MASK0_CMDTIMEOUT_Msk
                              | MCI_MASK0_CMDRESPOND_Msk);

                handle->state = MCI_STATE_DATA_SENT_CMD12;
            }
            else {
                handle->state = MCI_STATE_TRANSFER;

                /* Single block transfer is now over */
                event.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
                osSemaphoreRelease(handle->syncSema);
            }
        }

        else if (status & MCI_STATUS_CMDRESPOND_Msk) {
        }

        else {
            handle->state = MCI_STATE_TRANSFER;
        }
        break;

    case MCI_STATE_DATA_SENT_CMD12:
        handle->state = MCI_STATE_TRANSFER;

        if (status & MCI_STATUS_CMDRESPOND_Msk) {
            event.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
            osSemaphoreRelease(handle->syncSema);
        }
        else {
            handle->result = LPCLIB_CRC_FAILURE;

            event.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
            osSemaphoreRelease(handle->syncSema);
        }
        break;

    case MCI_STATE_RECEIVE:
        /* Any failure (CRC fail, timeout)? */
        if (status & (
            MCI_STATUS_CMDCRCFAIL_Msk
              | MCI_STATUS_CMDTIMEOUT_Msk
              | MCI_STATUS_DATACRCFAIL_Msk
              | MCI_STATUS_DATATIMEOUT_Msk)) {
            handle->result = LPCLIB_CRC_FAILURE;
            handle->state = MCI_STATE_TRANSFER;

            event.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
            osSemaphoreRelease(handle->syncSema);
        }

        /* Successful end of transfer? */
        else if (status & MCI_STATUS_DATAEND_Msk) {
            if (LPC_MCI->RESPCMD == MCI_CMD25__WRITE_MULTIPLE_BLOCK) {
                handle->state = MCI_STATE_RECEIVE_SENT_CMD12;

                /* Send CMD12 (Stop) */
                MCI_sendCommand(_MCI_makeCommandWord(
                                    MCI_CMD12__STOP_TRANSMISSION,
                                    MCI_RESPONSE_R1b,
                                    MCI_COMMAND_ENABLE_Msk),
                                0,
                                MCI_MASK0_CMDCRCFAIL_Msk
                            | MCI_MASK0_CMDTIMEOUT_Msk
                            | MCI_MASK0_CMDRESPOND_Msk);
            }
            else {
                handle->state = MCI_STATE_PROGRAM;

                /* Send CMD13 (Read status register) */
                MCI_sendCommand(_MCI_makeCommandWord(
                                    MCI_CMD13__SEND_STATUS,
                                    MCI_RESPONSE_R1,
                                    MCI_COMMAND_ENABLE_Msk),
                                handle->rca,
                                MCI_MASK0_CMDCRCFAIL_Msk
                              | MCI_MASK0_CMDTIMEOUT_Msk
                              | MCI_MASK0_CMDRESPOND_Msk);
            }
        }

        else if (status & MCI_STATUS_CMDRESPOND_Msk) {
            /* Enable Data Path State Machine */
            LPCLIB_BITBAND(&LPC_MCI->DATACTRL, MCI_DATACTRL_ENABLE_Pos) = 1;
        }

        else {
            handle->state = MCI_STATE_TRANSFER;
        }
        break;

    case MCI_STATE_RECEIVE_SENT_CMD12:
        if (status & MCI_STATUS_CMDRESPOND_Msk) {
            handle->state = MCI_STATE_PROGRAM;

            /* Send CMD13 (Read status register) */
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD13__SEND_STATUS,
                                MCI_RESPONSE_R1,
                                MCI_COMMAND_ENABLE_Msk),
                            handle->rca,
                            MCI_MASK0_CMDCRCFAIL_Msk
                          | MCI_MASK0_CMDTIMEOUT_Msk
                          | MCI_MASK0_CMDRESPOND_Msk);
        }
        else {
            handle->state = MCI_STATE_TRANSFER;
            handle->result = LPCLIB_CRC_FAILURE;

            event.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
            osSemaphoreRelease(handle->syncSema);
        }
        break;

    case MCI_STATE_PROGRAM:
        if (status & MCI_STATUS_CMDRESPOND_Msk) {
            /* Has the card reached TRANSFER state? */
            cardStatus = LPC_MCI->RESPONSE0;
            if (((cardStatus & MCI_CARDSTATUS_CURRENT_STATE_Msk) >> MCI_CARDSTATUS_CURRENT_STATE_Pos) == 4) {
                handle->state = MCI_STATE_TRANSFER;

                event.opcode = MCI_EVENT_READ_WRITE_COMPLETE;
                osSemaphoreRelease(handle->syncSema);
            }
            else {
                /* Try again TODO loop limit */

                /* Send CMD13 (Read status register) */
                MCI_sendCommand(_MCI_makeCommandWord(
                                    MCI_CMD13__SEND_STATUS,
                                    MCI_RESPONSE_R1,
                                    MCI_COMMAND_ENABLE_Msk),
                                handle->rca,
                                MCI_MASK0_CMDCRCFAIL_Msk
                              | MCI_MASK0_CMDTIMEOUT_Msk
                              | MCI_MASK0_CMDRESPOND_Msk);
            }
        }
        else {
            handle->state = MCI_STATE_INACTIVE;
            MCI_sendCommand(_MCI_makeCommandWord(
                                MCI_CMD0__GO_IDLE_STATE,
                                MCI_RESPONSE_NONE,
                                MCI_COMMAND_ENABLE_Msk),
                            0,
                            MCI_MASK0_CMDSENT_Msk);
            MCI_sendCallback(handle, &event);           /* Inform application */
        }
        break;

    default:
        break;
    }
}


/** @} */

/** @} addtogroup MCI */

#endif  /* #ifdef LPCLIB_MCI */

