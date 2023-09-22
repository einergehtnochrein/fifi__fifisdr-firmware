
#include <stdint.h>
#include <string.h>

#include "lpclib.h"

#if LPCLIB_I2C

#include "i2cdev.h"



/* Single buffer write operation */
LPCLIB_Result I2CDEV_write (I2C_Handle bus, uint8_t address, int numBytes, const void *txBuffer)
{
    I2C_Job job;
    I2C_JobPhase phase;

    phase.next = NULL;
    phase.option = I2C_PHASE_SEND;
    phase.txstart = txBuffer;
    phase.length = (uint16_t)numBytes;

    job.firstPhase = &phase;
    job.slaveAddress = address;
    job.callback = NULL;

    return I2C_submitJob(bus, &job);
}



/* Single buffer read operation. */
LPCLIB_Result I2CDEV_read (I2C_Handle bus, uint8_t address, int numBytes, void *rxBuffer)
{
    I2C_Job job;
    I2C_JobPhase phase;

    phase.next = NULL;
    phase.option = I2C_PHASE_RECEIVE;
    phase.rxstart = rxBuffer;
    phase.length = (uint16_t)numBytes;

    job.firstPhase = &phase;
    job.slaveAddress = address;
    job.callback = NULL;

    return I2C_submitJob(bus, &job);
}



/* Single buffer write+read operation. */
LPCLIB_Result I2CDEV_writeAndRead (I2C_Handle bus,
                                   uint8_t address,
                                   int numTxBytes,
                                   const void *txBuffer,
                                   int numRxBytes,
                                   void *rxBuffer)
{
    I2C_Job job;
    LPCLIB_Result result;
    I2C_JobPhase pPhase[2];

    pPhase[0].next = &pPhase[1];
    pPhase[0].option = I2C_PHASE_SEND;
    pPhase[0].txstart = txBuffer;
    pPhase[0].length = (uint16_t)numTxBytes;
    pPhase[1].next = NULL;
    pPhase[1].option = I2C_PHASE_RECEIVE;
    pPhase[1].txstart = rxBuffer;
    pPhase[1].length = (uint16_t)numRxBytes;

    job.firstPhase = &pPhase[0];
    job.slaveAddress = address;
    job.callback = NULL;

    result = I2C_submitJob(bus, &job);

    return result;
}

#endif  /* LPCLIB_I2C */
