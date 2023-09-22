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

#if LPCLIB_I2C

/** \file
 *  \brief I2C driver implementation.
 *
 *  This file contains the driver code for the I2C peripheral.
 *
 *  \author NXP Semiconductors
 */




#include "lpc17xx_i2c.h"
#if LPCLIB_I2CEMU
#include "i2c-bitbang.h"
#endif
#include "lpc17xx_clkpwr.h"


/** Field definition for hardware register I2CxCONSET. */
LPCLIB_DefineRegBit(I2C_CONSET_AA,              2,  1);
LPCLIB_DefineRegBit(I2C_CONSET_SI,              3,  1);
LPCLIB_DefineRegBit(I2C_CONSET_STO,             4,  1);
LPCLIB_DefineRegBit(I2C_CONSET_STA,             5,  1);
LPCLIB_DefineRegBit(I2C_CONSET_I2EN,            6,  1);

/** Field definition for hardware register I2CxSTAT. */
LPCLIB_DefineRegBit(I2C_STAT_STATUS,            3,  5);

/** List of possible status codes. */
enum {
    I2C_STATUS_0x00 = 0,                    /**< Bus error */
    I2C_STATUS_0x08 = 1,                    /**< MT: START condition successfully sent.
                                             *   MR: START condition successfully sent.
                                             */
    I2C_STATUS_0x10 = 2,                    /**< MT: Repeated START successfully sent.
                                             *   MR: Repeated START successfully sent.
                                             */
    I2C_STATUS_0x18 = 3,                    /**< MT: Received ACK after SLA+W */
    I2C_STATUS_0x20 = 4,                    /**< MT: Received NAK after SLA+W */
    I2C_STATUS_0x28 = 5,                    /**< MT: Received ACK after data */
    I2C_STATUS_0x30 = 6,                    /**< MT: Received NAK after data */
    I2C_STATUS_0x38 = 7,                    /**< MT: Arbitration lost while sending SLA+W or data.
                                             *   MR: Arbitration lost while sending NAK.
                                             */
    I2C_STATUS_0x40 = 8,                    /**< MR: Received ACK after SLA+R */
    I2C_STATUS_0x48 = 9,                    /**< MR: Received NAK after SLA+R */
    I2C_STATUS_0x50 = 10,                   /**< MR: Sent ACK after data RX */
    I2C_STATUS_0x58 = 11,                   /**< MR: Sent NAK after data RX */
    I2C_STATUS_0x60 = 12,                   /**< SR: Addressed as slave receiver */
    I2C_STATUS_0x68 = 13,                   /**< MT/MR: Arbitration lost, and addressed as slave. */
    I2C_STATUS_0x70 = 14,                   /**< SR: Addressed as slave receiver with global call address */
    I2C_STATUS_0x78 = 15,                   /**< MT/MR: Arbitration lost, and addressed by global call */
    I2C_STATUS_0x80 = 16,                   /**< SR: Sent ACK after data */
    I2C_STATUS_0x88 = 17,                   /**< SR: Sent NAK after data */
    I2C_STATUS_0x90 = 18,                   /**< SR: Sent ACK after data in (global call) */
    I2C_STATUS_0x98 = 19,                   /**< SR: Sent NAK after data in (global call) */
    I2C_STATUS_0xA0 = 20,                   /**< SR: Received STOP or repeated START */
    I2C_STATUS_0xA8 = 21,                   /**< ST: Addressed as slave transmitter */
    I2C_STATUS_0xB0 = 22,                   /**< MT: Arbitration lost, ...?
                                             *   MR: Arbitration lost, ...?
                                             */
    I2C_STATUS_0xB8 = 23,                   /**< ST: Received ACK after data */
    I2C_STATUS_0xC0 = 24,                   /**< ST: Received NAK after data */
    I2C_STATUS_0xC8 = 25,                   /**< ST: Received ACK after last available data. (Set AA=0) */
    I2C_STATUS_0xF8 = 31,                   /**< No relevant information. */
};

/** Field definition for hardware register I2CxCONCLR. */
LPCLIB_DefineRegBit(I2C_CONCLR_AAC,             2,  1);
LPCLIB_DefineRegBit(I2C_CONCLR_SIC,             3,  1);
LPCLIB_DefineRegBit(I2C_CONCLR_STAC,            5,  1);
LPCLIB_DefineRegBit(I2C_CONCLR_I2ENC,           6,  1);

/** Field definition for hardware register I2CxMMCTRL. */
LPCLIB_DefineRegBit(I2C_MMCTRL_MM_ENA,          0,  1);
LPCLIB_DefineRegBit(I2C_MMCTRL_ENA_SCL,         1,  1);
LPCLIB_DefineRegBit(I2C_MMCTRL_MATCH_ALL,       2,  1);

/** Field definition for hardware register I2CxADRy. */
LPCLIB_DefineRegBit(I2C_ADR_GC,                 0,  1);
LPCLIB_DefineRegBit(I2C_ADR_ADDRESS,            1,  7);

/** Field definition for hardware register I2CxMASKy. */
LPCLIB_DefineRegBit(I2C_MASK_MASK,              1,  7);


/** Local context of I2C busses. */
static struct I2C_Context {
    /* NOTE:
     * The first element MUST be "I2C_Name bus".
     * This structure definition can be done in multiple C source files
     * (I2C GPIO bit-banging), and "bus" must exist in all of these
     * possible implementations at offset 0.
     */
    I2C_Name bus;                           /**< Bus identifier */
    LPCLIB_Switch inUse;                    /**< Set if interface open */
    LPCLIB_Result errorStatus;              /**< Error status returned with end of transaction */
    volatile uint32_t transferred;          /**< Transfer counter. */
    volatile I2C_Job *job;
    volatile I2C_Job *jobOnHold;
    volatile const I2C_JobPhase *phase;
    osMutexId accessMutex;
    osSemaphoreId syncSema;

#if LPCLIB_I2C_SLAVE
    uint8_t addressedAs;                    /**< Actual address by which the device
                                             *   was addressed in slave mode.
                                             */
    LPCLIB_Callback eventHandler;           /* Receives slave/monitor mode events */
  #if (LPCLIB_I2C_NUM_SLAVE_ADDRESSES > 1)
    uint8_t addressUsed[4];                 /* Flag to indicate that address comparator is used */
  #endif
#endif
} i2cContext[I2C_NUM_BUSSES];


//TODO make sure we have enough (and not more) mutexes/syncs!
//TODO must have *globally* unique name...
osMutexDef(i2cAccessMutexDef0);
osMutexDef(i2cAccessMutexDef1);
osMutexDef(i2cAccessMutexDef2);
osSemaphoreDef(i2cSyncSemaDef0);
osSemaphoreDef(i2cSyncSemaDef1);
osSemaphoreDef(i2cSyncSemaDef2);


static osMutexDef_t * const i2cMutexes[I2C_NUM_BUSSES] = {
    osMutex(i2cAccessMutexDef0), osMutex(i2cAccessMutexDef1), osMutex(i2cAccessMutexDef2),
    };
static osSemaphoreDef_t * const i2cSemas[I2C_NUM_BUSSES] = {
    osSemaphore(i2cSyncSemaDef0), osSemaphore(i2cSyncSemaDef1), osSemaphore(i2cSyncSemaDef2),
    };
/** Peripheral bus address of I2C block(s). */
static LPC_I2C_TypeDef * const i2c_ptr[I2C_NUM_BUSSES] = {LPC_I2C0, LPC_I2C1, LPC_I2C2};



#if LPCLIB_I2C_SLAVE
  #if (LPCLIB_I2C_NUM_SLAVE_ADDRESSES > 1)
    /** Peripheral bus address of slave address registers of I2C block(s). */
    static __IO uint32_t * const i2c_slave_addr[I2C_NUM_BUSSES][4] = {
        {
            (uint32_t *)&LPC_I2C0->ADR0,
            (uint32_t *)&LPC_I2C0->ADR1,
            (uint32_t *)&LPC_I2C0->ADR2,
            (uint32_t *)&LPC_I2C0->ADR3,
        },
        {
            (uint32_t *)&LPC_I2C1->ADR0,
            (uint32_t *)&LPC_I2C1->ADR1,
            (uint32_t *)&LPC_I2C1->ADR2,
            (uint32_t *)&LPC_I2C1->ADR3,
        },
        {
            (uint32_t *)&LPC_I2C2->ADR0,
            (uint32_t *)&LPC_I2C2->ADR1,
            (uint32_t *)&LPC_I2C2->ADR2,
            (uint32_t *)&LPC_I2C2->ADR3,
        },
    };
  #else
    extern uint32_t * const i2c_slave_addr[I2C_NUM_BUSSES][4];  /* Dummy declaration! */
  #endif
#endif
static const CLKPWR_Clockswitch i2c_clockSwitch[I2C_NUM_BUSSES] = {CLKPWR_CLOCKSWITCH_I2C0, CLKPWR_CLOCKSWITCH_I2C1, CLKPWR_CLOCKSWITCH_I2C2,};
static const CLKPWR_Clock i2c_clock[I2C_NUM_BUSSES] = {CLKPWR_CLOCK_I2C0, CLKPWR_CLOCK_I2C1, CLKPWR_CLOCK_I2C2,};
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
static const CLKPWR_Reset i2cReset[I2C_NUM_BUSSES] = {CLKPWR_RESET_I2C0, CLKPWR_RESET_I2C1, CLKPWR_RESET_I2C2,};
#endif



/* Open an I2C bus. */
LPCLIB_Result I2C_open (I2C_Name bus, I2C_Handle *pHandle)
{
    LPC_I2C_TypeDef *i2c = i2c_ptr[bus];
    I2C_Handle handle = &i2cContext[bus];
    I2C_Config setBitrate[2];

#if LPCLIB_I2CEMU
    /* Check for bus emulation */
    if (bus > I2C_NUM_BUSSES) {
        return I2CEMU_open(bus, pHandle);
    }
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    CLKPWR_assertPeripheralReset(i2cReset[bus]);        /* Assert I2C reset */
    CLKPWR_deassertPeripheralReset(i2cReset[bus]);      /* Release I2C reset */
#endif
    CLKPWR_enableClock(i2c_clockSwitch[bus]);           /* Enable I2C peripheral clock */

    /* Cannot open an I2C twice */
    if (!handle->inUse) {
        handle->accessMutex = osMutexCreate(i2cMutexes[bus]);
        handle->syncSema = osSemaphoreCreate(i2cSemas[bus], 1);

        /* Default slave address */
    #if LPCLIB_I2C_SLAVE
        i2c->ADR0 = (LPCLIB_I2C_SLAVE_DEFAULT_ADDRESS) << 1;
        i2c->MASK[0] = (LPCLIB_I2C_SLAVE_DEFAULT_MASK) << 1;
    #else
        i2c->ADR0 = 0x00;
        i2c->MASK[0] = 0x00;
    #endif

        handle->bus = bus;
        handle->inUse = LPCLIB_YES;
        *pHandle = handle;                              /* Return handle */
//Do we have to set a default bitrate?
        setBitrate[0].opcode = I2C_OPCODE_SET_BITRATE;
        setBitrate[0].bitrate = LPCLIB_I2C_DEFAULT_BUSCLOCK;
        setBitrate[1].opcode = I2C_OPCODE_INVALID;
        I2C_ioctl(*pHandle, setBitrate);                /* Default bit clock */
    #if LPCLIB_I2C_SLAVE
        i2c->CONSET = I2C_CONSET_I2EN_Msk | I2C_CONSET_AA_Msk;  /* Enable I2C and allow slave mode */
    #else
        i2c->CONSET = I2C_CONSET_I2EN_Msk;              /* Enable I2C */
    #endif

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;

}



/* Close an I2C bus. */
void I2C_close (I2C_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

#if LPCLIB_I2CEMU
    if ((*pHandle)->bus > I2C_NUM_BUSSES) {
        I2CEMU_close(pHandle);
        return;
    }
#endif

    CLKPWR_disableClock(i2c_clockSwitch[(*pHandle)->bus]);  /* Disable I2C peripheral clock */

//    LPCLIB_OSALMutexDelete(i2cContext[(*pHandle)->bus].accessMutex);
//    LPCLIB_OSALSemaphoreDelete(i2cContext[(*pHandle)->bus].syncSema);

    (*pHandle)->inUse = LPCLIB_NO;
    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Configure the I2C block. */
void I2C_ioctl (I2C_Handle handle, const I2C_Config *pConfig)
{
    uint32_t temp;
    LPC_I2C_TypeDef * i2c;


    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

#if LPCLIB_I2CEMU
    if (handle->bus > I2C_NUM_BUSSES) {
        I2CEMU_ioctl(handle, pConfig);
        return;
    }
#endif

    i2c = i2c_ptr[handle->bus];

    while (pConfig->opcode != I2C_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case I2C_OPCODE_SET_BITRATE:
            temp = CLKPWR_getBusClock(i2c_clock[handle->bus]);
            temp /= pConfig->bitrate;
if(temp<4)temp=4;
            i2c->SCLH = temp / 2;
            i2c->SCLL = temp / 2;
            break;

#if LPCLIB_I2C_SLAVE
        case I2C_OPCODE_SET_SLAVE_ADDRESS:
            if (LPCLIB_I2C_NUM_SLAVE_ADDRESSES > 1) {
                if (pConfig->slave.index < 4) {
                    i2c->MASK[pConfig->slave.index] = 0;    /* Mask all bits while changing the address */
                    *(i2c_slave_addr[handle->bus][pConfig->slave.index]) =
                        (pConfig->slave.address << I2C_ADR_ADDRESS_Pos);
                    i2c->MASK[pConfig->slave.index] =
                        (pConfig->slave.mask << I2C_MASK_MASK_Pos);
      #if (LPCLIB_I2C_NUM_SLAVE_ADDRESSES > 1)
                    handle->addressUsed[pConfig->slave.index] = 1;
                    for (temp = 0; temp < 4; temp++) {      /* Deactivate unused address comparators */
                        if (!handle->addressUsed[temp]) {
                            *(i2c_slave_addr[handle->bus][temp]) =
                                pConfig->slave.address << I2C_ADR_ADDRESS_Pos;
                            i2c->MASK[temp] = 0x00;
                        }
                    }
      #endif
                }
            }
            else {
                i2c->MASK[0] = 0;                           /* Mask all bits while changing the address */
                i2c->ADR0 = pConfig->slave.address;
                i2c->MASK[0] = pConfig->slave.mask;
            }
            break;

        case I2C_OPCODE_SET_CALLBACK:
            if (pConfig->callback.pOldCallback) {       /* Return current callback if requested */
                *(pConfig->callback.pOldCallback) = handle->eventHandler;
            }
            handle->eventHandler = pConfig->callback.callback;
            break;
#endif

#if LPCLIB_I2CEMU
        case I2C_OPCODE_EMU_DEFINE_PINS:
            /* Nothing to handle here (only used in bit-banging emulator) */
            break;
#endif

        case I2C_OPCODE_INVALID:
            /* Dummy to suppress compiler warning */
            break;
        }

        ++pConfig;
    }
}



/* Submit a new I2C job to the driver. */
LPCLIB_Result I2C_submitJob (I2C_Handle handle, I2C_Job *pJob)
{
    LPC_I2C_TypeDef * i2c;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

#if LPCLIB_I2CEMU
    /* Check for bus emulation */
    if (handle->bus > I2C_NUM_BUSSES) {
        return I2CEMU_submitJob(handle, pJob);
    }
#endif

    i2c = i2c_ptr[handle->bus];

    /* Get exclusive access */
    if (osMutexWait(handle->accessMutex, osWaitForever) == osOK) {
        /* Make sure to reset the sync flag */
        osSemaphoreWait(handle->syncSema, 0);

        handle->job = pJob;
        handle->errorStatus = LPCLIB_SUCCESS;
    #if LPCLIB_I2C_SLAVE
        i2c->CONSET = I2C_CONSET_AA_Msk;                /* Allow being addressed as slave */
    #else
        i2c->CONCLR = I2C_CONCLR_AAC_Msk;               /* Cannot be addressed as slave */
    #endif
        i2c->CONSET = I2C_CONSET_STA_Msk;               /* Start transmission in master mode */

        if (pJob->callback) {
            /* In async mode we can leave immediately */
            osMutexRelease(handle->accessMutex);
            return LPCLIB_PENDING;
        }

        /* In sync mode we wait for the end of transaction. */
        if (osSemaphoreWait(handle->syncSema, osWaitForever) > 0) {
            osMutexRelease(handle->accessMutex);

            return handle->errorStatus;
        }

        osMutexRelease(handle->accessMutex);

        //TODO: serious error! Restart I2C bus, re-init semaphores
        //TODO: when will semaacquire bring us here??
        return LPCLIB_ERROR;
    }

    return LPCLIB_BUSY;                                 /* Internal error. Couldn't get semaphore */
}



static const I2C_JobPhase dummyProbePhase = {
    .next = NULL,
    .option = I2C_PHASE_SEND,
};


/* Check for the existence of a slave with given address. */
LPCLIB_Result I2C_probe (I2C_Handle handle, uint8_t address)
{
    I2C_Job job = {
        .callback = NULL,                   //TODO: add support for non-rtos!
        .firstPhase = &dummyProbePhase,     /* No data phase, just SLA+W */
        .slaveAddress = address,
    };

    return I2C_submitJob(handle, &job);
}



/** Common (for all I2C blocks) interrupt handler.
 *
 *  \param[in] i2c_num Indicator that selects an I2C interface block
 */
static void I2C_commonIRQHandler (I2C_Name bus)
{
    LPC_I2C_TypeDef * const i2c = i2c_ptr[bus];
    struct I2C_Context *handle = &i2cContext[bus];
    uint32_t status;
    uint16_t current;
    LPCLIB_Event event;
    uint8_t address;
#if LPCLIB_I2C_SLAVE
    uint8_t rxChar;
#endif

    /* Prepare event */
    event.id = LPCLIB_EVENTID_I2C;
    event.block = bus;

    /* Read hardware state machine */
    status = (i2c->STAT & I2C_STAT_STATUS_Msk) >> I2C_STAT_STATUS_Pos;

    /* Action depends on current state.
     * MT=Master transmitter, MR=Master receiver, ST=Slave transmitter, SR=Slave receiver
     */
    switch (status) {
    case I2C_STATUS_0x00:   /* Bus error */
        i2c->CONSET = I2C_CONSET_STO_Msk;               /* STOP is not actually sent */
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;               /* Clear interrupt request */
        break;

#if LPCLIB_I2C_MASTER
    case I2C_STATUS_0x08:   /* MT: START condition successfully sent.
                             * MR: START condition successfully sent.
                             */
        handle->phase = handle->job->firstPhase;
        address = handle->job->slaveAddress << 1;       /* SLA+W */
        if (handle->phase->option == I2C_PHASE_RECEIVE) {
            address |= 1;                               /* SLA+R */
        }
        i2c->DAT = address;                             /* Send SLA+R/SLA+W */
        i2c->CONCLR = I2C_CONCLR_SIC_Msk | I2C_CONCLR_STAC_Msk;
        handle->transferred = 0;                        /* Reset counter */
        break;

    case I2C_STATUS_0x30:   /* MT: Received NAK after data */
        /* This is usually a failure with the exception of the last byte of a
         * transmission, where either ACK or NAK are accepted.
         */
        current = handle->transferred;                  /* Intermediate step to suppress IAR warning */
        if (current < handle->phase->length) {
            /* Early NAK --> failure */
            i2c->CONCLR = I2C_CONCLR_SIC_Msk;
            i2c->CONSET = I2C_CONSET_STO_Msk;           /* Send STOP */

            event.opcode = I2C_EVENT_NOT_ACCEPTED;
            if (handle->job->callback) {
                handle->job->callback(event);
            }
            break;
        }

        /**** NO BREAK. INTENTIONALLY FALL THROUGH TO ACK HANDLING! (0x28) ****/
        // fall through

    case I2C_STATUS_0x18:   /* MT: Received ACK after SLA+W */
    case I2C_STATUS_0x28:   /* MT: Received ACK after data */

        /* Is there more data to send in this phase? */
        current = handle->transferred;                  /* Intermediate step to suppress IAR warning */
        if (current < handle->phase->length) {
            /* Yes. Send next data byte, and update counter. */
            i2c->DAT = handle->phase->txstart[current];
            ++handle->transferred;
            i2c->CONCLR = I2C_CONCLR_SIC_Msk;
            break;
        }

        /**** NO BREAK. INTENTIONALLY FALL THROUGH TO NEXT PHASE HANDLING! (0x58) ****/
        // fall through

    case I2C_STATUS_0x58:   /* MR: Sent NAK after data RX */
        if (status == I2C_STATUS_0x58) {                /* Need to get the last byte */
            current = handle->transferred;              /* Intermediate step to suppress IAR warning */
            handle->phase->rxstart[current] = i2c->DAT;
            ++handle->transferred;
        }

        /* Is there another phase coming? */
        if (handle->phase->next) {
            handle->phase = handle->phase->next;

            /* In state 0x28 there can be a seamless switch to another data block.
             * All other states, and a switch to MR mode, require a repeated START condition.
             */
            if ((status == I2C_STATUS_0x28) && (handle->phase->option == I2C_PHASE_SEND)) {
                i2c->DAT = handle->phase->txstart[0];
                handle->transferred = 1;
            }
            else {
                i2c->CONSET = I2C_CONSET_STA_Msk;       /* Send repeated START */
            }
        }
        else {
            /* End of transaction. Terminate with STOP condition. */
            i2c->CONSET = I2C_CONSET_STO_Msk;           /* Send STOP */

            if (handle->job->callback) {
                event.opcode = I2C_EVENT_SUCCESS;
                handle->job->callback(event);
            }
            else {
                osSemaphoreRelease(handle->syncSema);
            }
        }
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;


    case I2C_STATUS_0x20:   /* MT: Received NAK after SLA+W */
    case I2C_STATUS_0x48:   /* MR: Received NAK after SLA+R */
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        i2c->CONSET = I2C_CONSET_STO_Msk;               /* Send STOP */

        if (handle->job->callback) {
            event.opcode = I2C_EVENT_NO_SUCH_DEVICE;
            handle->job->callback(event);
        }
        else {
            handle->errorStatus = LPCLIB_NO_RESPONSE;
            osSemaphoreRelease(handle->syncSema);
        }
        break;

    case I2C_STATUS_0x10:   /* MT: Repeated START successfully sent.
                             * MR: Repeated START successfully sent.
                             */
        address = handle->job->slaveAddress << 1;       /* SLA+W */
        if (handle->phase->option == I2C_PHASE_RECEIVE) {
            address |= 1;                               /* SLA+R */
        }
        i2c->DAT = address;                             /* Send slave address + inverted R/W */
        i2c->CONCLR = I2C_CONCLR_SIC_Msk | I2C_CONCLR_STAC_Msk;
        handle->transferred = 0;
        break;

    case I2C_STATUS_0x50:   /* MR: Sent ACK after data RX */
        /* Store the received byte, and update counter. */
        current = handle->transferred;                  /* Intermediate step to suppress IAR warning */
        handle->phase->rxstart[current] = i2c->DAT;
        ++handle->transferred;

        /**** NO BREAK. INTENTIONALLY FALL THROUGH TO ACK/NAK SELECTION! (0x40) ****/
        // fall through

    case I2C_STATUS_0x40:   /* MR: Received ACK after SLA+R */
        /* Receive another data byte. Respond with NAK if this is the last byte of
         * the receive phase, respond with ACK otherwise.
         */
        current = handle->transferred;                  /* Intermediate step to suppress IAR warning */
        if ((current + 1) < handle->phase->length) {
            i2c->CONSET = I2C_CONSET_AA_Msk;            /* Respond with ACK */
        }
        else {
            i2c->CONCLR = I2C_CONCLR_AAC_Msk;           /* Respond with NAK */
        }
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0x38:   /* MT: Arbitration lost while sending SLA+W or data.
                             * MR: Arbitration lost while sending NAK.
                             */
        i2c->CONSET = I2C_CONSET_STA_Msk;               /* Retry immediately when bus is free again */
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0x68:   /* MT/MR: Arbitration lost, and addressed as slave. */
    case I2C_STATUS_0x78:   /* MT/MR: Arbitration lost, and addressed by global call */
        // TODO
        break;

    case I2C_STATUS_0xB0:   /* MT: Arbitration lost, ...?
                             * MR: Arbitration lost, ...?
                             */
        // TODO
        break;
#endif

#if LPCLIB_I2C_SLAVE
    case I2C_STATUS_0xA8:   /* ST: Addressed as slave transmitter */
    case I2C_STATUS_0x60:   /* SR: Addressed as slave receiver */
    case I2C_STATUS_0x70:   /* SR: Addressed as slave receiver with global call address */
        handle->addressedAs = i2c->DATA_BUFFER >> 1;    /* Remember by which address we were called */
        handle->jobOnHold = handle->job;                /* Remember a possibly active master job */

        /* Ask the application to provide a slave job descriptor */
        handle->job = NULL;
        handle->transferred = 0;                        /* Reset counter */
        if (status == I2C_STATUS_0xA8) {
            event.opcode = I2C_EVENT_NEED_SLAVE_JOB_TX;
        }
        else {
            event.opcode = I2C_EVENT_NEED_SLAVE_JOB_RX;
        }
        event.channel = handle->addressedAs;            /* This address is SLA+W! */
        event.parameter = &handle->job;                 /* Application MUST provide job pointer here */
        if (handle->eventHandler) {
            handle->eventHandler(event);
        }

        i2c->CONCLR = I2C_CONCLR_AAC_Msk;
        if (handle->job) {                              /* Is there a valid job? */
            handle->phase = handle->job->firstPhase;
            if (handle->phase->length > 0) {            /* Non-empty phase? */
                if (status == I2C_STATUS_0xA8) {
                    i2c->CONSET = I2C_CONSET_AA_Msk;
                    i2c->DAT = handle->phase->txstart[0];
                    handle->transferred = 1;
                }
                else  {
                    if (handle->phase->option == I2C_PHASE_RECEIVE) {   /* Does it start with an RX phase? */
                        i2c->CONSET = I2C_CONSET_AA_Msk;
                    }
                }
            }
        }
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0x80:   /* SR: Sent ACK after data */
    case I2C_STATUS_0x90:   /* SR: Sent ACK after data in (global call) */
        /* Store the received byte, and update counter. */
        current = handle->transferred;                  /* Intermediate step to suppress IAR warning */
        handle->phase->rxstart[current] = i2c->DAT;
        ++handle->transferred;

        /* Receive another data byte. Respond with NAK if this is the last byte of
         * the receive phase, respond with ACK otherwise.
         */
        current = handle->transferred;                  /* Intermediate step to suppress IAR warning */
        if ((current + 1) < handle->phase->length) {
            i2c->CONSET = I2C_CONSET_AA_Msk;            /* Respond with ACK */
        }
        else {
            i2c->CONCLR = I2C_CONCLR_AAC_Msk;           /* Respond with NAK */
        }
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0x88:   /* SR: Sent NAK after data */
    case I2C_STATUS_0x98:   /* SR: Sent NAK after data in (global call) */
        rxChar = i2c->DAT;                              /* Always read character (we might throw it away) */

        if (handle->job) {  //TODO
            current = handle->transferred;              /* Intermediate step to suppress IAR warning */
            handle->phase->rxstart[current] = rxChar;
            ++handle->transferred;

            /* End of transaction. */
            event.opcode = I2C_EVENT_SUCCESS;
            if (handle->job->callback) {
                handle->job->callback(event);
            }
        }
        else {
            /* The SR mode has ended because we didn't have a job for it.
             * No need to send an event for this.
             */
        }

        /* Restart transmission if there was an arbitration loss before */
        if (handle->jobOnHold) {
            handle->job = handle->jobOnHold;
            handle->jobOnHold = NULL;
            i2c->CONSET = I2C_CONSET_STA_Msk;
        }
        i2c->CONSET = I2C_CONSET_AA_Msk;                /* Allow to be addressed as slave again */
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0xA0:   /* SR: Received STOP or repeated START */
        event.opcode = I2C_EVENT_SUCCESS;
        if (handle->eventHandler) {
            handle->eventHandler(event);
        }
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0xB8:   /* ST: Received ACK after data */
        //TODO
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0xC0:   /* ST: Received NAK after data */
        //TODO
        i2c->CONCLR = I2C_CONCLR_SIC_Msk;
        break;

    case I2C_STATUS_0xC8:   /* ST: Received ACK after last available data. (Set AA=0) */
        break;
#endif

    case I2C_STATUS_0xF8:  /* No relevant information. We should never come her,
                            * because the hardware doesn't set SI in this case.
                            */
        break;
    }
}


/** I2C block interrupt entry (interrupt number \ref I2C0_IRQn).
 */
void I2C0_IRQHandler (void)
{
    I2C_commonIRQHandler(I2C0);
}

/** I2C block interrupt entry (interrupt number \ref I2C1_IRQn).
 */
void I2C1_IRQHandler (void)
{
    I2C_commonIRQHandler(I2C1);
}

/** I2C block interrupt entry (interrupt number \ref I2C2_IRQn).
 */
void I2C2_IRQHandler (void)
{
    I2C_commonIRQHandler(I2C2);
}



/** @} */

/** @} addtogroup I2C */

#endif  /* #ifdef LPCLIB_I2C */

