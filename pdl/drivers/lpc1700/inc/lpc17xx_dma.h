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

/** \defgroup DMA
 *  \ingroup API
 *  @{
 */

#ifndef __LPC17XX_DMA_H__
#define __LPC17XX_DMA_H__

#include "lpc17xx_libconfig.h"

#if LPCLIB_DMA


#include "lpclib_types.h"


/** \defgroup DMA_Public_Types DMA Types, enums, macros
 *  @{
 */


/** Enumerator for DMA.
 */
typedef enum DMA_Name {
    DMA0 = 0,           /**< First DMA block */
    DMA_NUM_BLOCKS      /* In order for this element to reflect the number of DMA blocks,
                         * you mustn't assign an explicit value to any of the other elements
                         * of this enum! (except for the first element which may get assigned to 0).
                         */
} DMA_Name;


/** Handle for a DMA block. */
typedef struct DMA_Context *DMA_Handle;

/** Handle for a DMA channel. */
typedef struct DMA_ChannelContext *DMA_ChannelHandle;


/**< Config descriptor for memory allocation .*/
struct DMA_ConfigMemory {
    void **mem;                             /**< The memory */
    int numPhases;                          /**< Number of transfer phases */
};


typedef enum DMA_Opcode {
    DMA_OPCODE_SET_SYSTEM_CALLBACK,         /**< Config action: Install system callback handler */
} DMA_Opcode;


typedef struct DMA_Config {
    DMA_Opcode opcode;                      /**< Config action opcode */

    union {
        LPCLIB_Callback callback;           /**< Callback handler */
        struct DMA_ConfigMemory memory;     /**< Acquire memory */
    };
} DMA_Config;



typedef enum DMA_CallbackEvent {
    DMA_EVENT_STOP = 0,                     /**< Transaction completed, DMA stopped. */
    DMA_EVENT_PHASE_COMPLETE,               /**< A DMA phase has completed, but not
                                             *   yet the whole transaction.
                                             */
    DMA_EVENT_MEMALLOC,                     /**< Ask for memory allocation */
    DMA_EVENT_MEMFREE,                      /**< Free memory */
    DMA_EVENT_ERROR = -1,                   /**< DMA error (AHB access error) */
} DMA_CallbackEvent;


typedef enum DMA_BurstSize {
    DMA_BURSTSIZE_1 = 0,
    DMA_BURSTSIZE_4 = 1,
    DMA_BURSTSIZE_8 = 2,
    DMA_BURSTSIZE_16 = 3,
    DMA_BURSTSIZE_32 = 4,
    DMA_BURSTSIZE_64 = 5,
    DMA_BURSTSIZE_128 = 6,
    DMA_BURSTSIZE_256 = 7,
} DMA_BurstSize;

typedef enum DMA_TransferWidth {
    DMA_WIDTH_8 = 0,
    DMA_WIDTH_16 = 1,
    DMA_WIDTH_32 = 2,
} DMA_TransferWidth;


typedef enum DMA_Peripheral {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC17XX
    DMA_PERIPHERAL_MEMORY   = 0,
    DMA_PERIPHERAL_SSP0TX   = 0,
    DMA_PERIPHERAL_SSP0RX   = 1,
    DMA_PERIPHERAL_SSP1TX   = 2,
    DMA_PERIPHERAL_SSP1RX   = 3,
    DMA_PERIPHERAL_ADC      = 4,
    DMA_PERIPHERAL_I2S0CH0  = 5,
    DMA_PERIPHERAL_I2S0CH1  = 6,
    DMA_PERIPHERAL_DAC      = 7,
    DMA_PERIPHERAL_UART0TX  = (1 << 15) | (0 << 14) | (0 << 8) |  8,
    DMA_PERIPHERAL_UART0RX  = (1 << 15) | (0 << 14) | (1 << 8) |  9,
    DMA_PERIPHERAL_UART1TX  = (1 << 15) | (0 << 14) | (2 << 8) | 10,
    DMA_PERIPHERAL_UART1RX  = (1 << 15) | (0 << 14) | (3 << 8) | 11,
    DMA_PERIPHERAL_UART2TX  = (1 << 15) | (0 << 14) | (4 << 8) | 12,
    DMA_PERIPHERAL_UART2RX  = (1 << 15) | (0 << 14) | (5 << 8) | 13,
    DMA_PERIPHERAL_UART3TX  = (1 << 15) | (0 << 14) | (6 << 8) | 14,
    DMA_PERIPHERAL_UART3RX  = (1 << 15) | (0 << 14) | (7 << 8) | 15,
    DMA_PERIPHERAL_T0MAT0   = (1 << 15) | (1 << 14) | (0 << 8) |  8,
    DMA_PERIPHERAL_T0MAT1   = (1 << 15) | (1 << 14) | (1 << 8) |  9,
    DMA_PERIPHERAL_T1MAT0   = (1 << 15) | (1 << 14) | (2 << 8) | 10,
    DMA_PERIPHERAL_T1MAT1   = (1 << 15) | (1 << 14) | (3 << 8) | 11,
    DMA_PERIPHERAL_T2MAT0   = (1 << 15) | (1 << 14) | (4 << 8) | 12,
    DMA_PERIPHERAL_T2MAT1   = (1 << 15) | (1 << 14) | (5 << 8) | 13,
    DMA_PERIPHERAL_T3MAT0   = (1 << 15) | (1 << 14) | (6 << 8) | 14,
    DMA_PERIPHERAL_T3MAT1   = (1 << 15) | (1 << 14) | (7 << 8) | 15,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC178X
    DMA_PERIPHERAL_MEMORY   = (1 << 15) | (0 << 14) | ( 0 << 8) | 0,
    DMA_PERIPHERAL_SDCARD   = (1 << 15) | (0 << 14) | ( 1 << 8) | 1,
    DMA_PERIPHERAL_SSP0TX   = (1 << 15) | (0 << 14) | ( 2 << 8) | 2,
    DMA_PERIPHERAL_SSP0RX   = (1 << 15) | (0 << 14) | ( 3 << 8) | 3,
    DMA_PERIPHERAL_SSP1TX   = (1 << 15) | (0 << 14) | ( 4 << 8) | 4,
    DMA_PERIPHERAL_SSP1RX   = (1 << 15) | (0 << 14) | ( 5 << 8) | 5,
    DMA_PERIPHERAL_SSP2TX   = (1 << 15) | (0 << 14) | ( 6 << 8) | 6,
    DMA_PERIPHERAL_SSP2RX   = (1 << 15) | (0 << 14) | ( 7 << 8) | 7,
    DMA_PERIPHERAL_ADC      = 8,
    DMA_PERIPHERAL_DAC      = 9,
    DMA_PERIPHERAL_UART0TX  = (1 << 15) | (0 << 14) | (10 << 8) | 10,
    DMA_PERIPHERAL_UART0RX  = (1 << 15) | (0 << 14) | (11 << 8) | 11,
    DMA_PERIPHERAL_UART1TX  = (1 << 15) | (0 << 14) | (12 << 8) | 12,
    DMA_PERIPHERAL_UART1RX  = (1 << 15) | (0 << 14) | (13 << 8) | 13,
    DMA_PERIPHERAL_UART2TX  = (1 << 15) | (0 << 14) | (14 << 8) | 14,
    DMA_PERIPHERAL_UART2RX  = (1 << 15) | (0 << 14) | (15 << 8) | 15,
    DMA_PERIPHERAL_T0MAT0   = (1 << 15) | (1 << 14) | ( 0 << 8) | 0,
    DMA_PERIPHERAL_T0MAT1   = (1 << 15) | (1 << 14) | ( 1 << 8) | 1,
    DMA_PERIPHERAL_T1MAT0   = (1 << 15) | (1 << 14) | ( 2 << 8) | 2,
    DMA_PERIPHERAL_T1MAT1   = (1 << 15) | (1 << 14) | ( 3 << 8) | 3,
    DMA_PERIPHERAL_T2MAT0   = (1 << 15) | (1 << 14) | ( 4 << 8) | 4,
    DMA_PERIPHERAL_T2MAT1   = (1 << 15) | (1 << 14) | ( 5 << 8) | 5,
    DMA_PERIPHERAL_I2S0CH0  = (1 << 15) | (1 << 14) | ( 6 << 8) | 6,
    DMA_PERIPHERAL_I2S0CH1  = (1 << 15) | (1 << 14) | ( 7 << 8) | 7,
    DMA_PERIPHERAL_UART3TX  = (1 << 15) | (1 << 14) | (10 << 8) | 10,
    DMA_PERIPHERAL_UART3RX  = (1 << 15) | (1 << 14) | (11 << 8) | 11,
    DMA_PERIPHERAL_UART4TX  = (1 << 15) | (1 << 14) | (12 << 8) | 12,
    DMA_PERIPHERAL_UART4RX  = (1 << 15) | (1 << 14) | (13 << 8) | 13,
    DMA_PERIPHERAL_T3MAT0   = (1 << 15) | (1 << 14) | (14 << 8) | 14,
    DMA_PERIPHERAL_T3MAT1   = (1 << 15) | (1 << 14) | (15 << 8) | 15,
#endif
} DMA_Peripheral;

typedef enum DMA_FlowControl {
    DMA_FLOW_MEM2MEM_DMA = 0,
    DMA_FLOW_MEM2PER_DMA = 1,
    DMA_FLOW_PER2MEM_DMA = 2,
    DMA_FLOW_PER2PER_DMA = 3,
    DMA_FLOW_PER2PER_DEST = 4,
    DMA_FLOW_MEM2PER_DEST = 5,
    DMA_FLOW_PER2MEM_SRC = 6,
    DMA_FLOW_PER2PER_SRC = 7,
} DMA_FlowControl;


/* Control word for a DMA channel (phase). */
typedef uint32_t DMA_JobPhaseParameter;


/* The struct DMA_JobPhase is used directly as a linked list item (LLI) for the DMA.
 * Hardware dictates the exact format of the first four uint32_t words of this LLI,
 * so we need to make sure that the compiler packs the structure correctly (which
 * it would do automatically anyway for the 32-bit architecture).
 */
typedef struct DMA_JobPhase {
    /*** DO NOT MODIFY THE FIRST FOUR MEMBERS! ***/
    const void *pSource;                    /**< Source (start) address */
    const void *pDest;                      /**< Destination (start) address */
    struct DMA_JobPhase *next;              /**< Pointer to next transaction phase (or NULL) */
    DMA_JobPhaseParameter parameter;        /**< Parameter of the job phase */

    /*** The following members can be modified ***/
    const void *callbackParameter;          /**< This value will be sent to the callback
                                             *   along with a DMA_EVENT_PHASE_COMPLETE event.
                                             */
} DMA_JobPhase;



/** DMA job descriptor. */
typedef struct DMA_Job {
    DMA_ChannelHandle channel;              /**< Channel handle */
    LPCLIB_Callback callback;               /**< Callback for terminal count or error events */
    DMA_JobPhase *firstPhase;               /**< Descriptor of first job phase */
    DMA_Peripheral source;                  /**< Source peripheral */
    DMA_Peripheral destination;             /**< Destination peripheral */
    DMA_FlowControl flowControl;            /**< Flow controller */
} DMA_Job;


/** Context for using the DMA memcpy convenience function. */
typedef struct DMA_MemcpyContext {
    DMA_Job job;
    DMA_JobPhase phase;
} DMA_MemcpyContext;



/** @} DMA Types, enums, macros */



/** \defgroup DMA_Public_Functions DMA API Functions
 *  @{
 */


/** Open the GPDMA block.
 *
 *  Get access to the DMA channel. Obtain a handle that must be used in calls to other
 *  DMA module functions.
 *
 *  \param[in] dmaNum Identifies the DMA block.
 *  \param[out] pHandle Receives the handle
 *  \retval DMA_SUCCESS Ok. Valid handle returned.
 *  \retval DMA_BUSY Error. No valid handle returned. (All DMA channels occupied)
 */
LPCLIB_Result DMA_open (DMA_Name dmaNum, DMA_Handle *pHandle);


/** Close the DMA block.
 *
 *  Closing the DMA block is only possible if no channel is in use.
 *
 *  \param pHandle Handle of the DMA block.
 *  \retval LPCLIB_SUCCESS Ok. Regular close.
 *  \retval LPCLIB_BUSY Error. At least one channel is active.
 */
LPCLIB_Result DMA_close (DMA_Handle *pHandle);


/** Set options of the DMA block.
 *
 *  \param[in] handle Handle of DMA block.
 *  \param[in] config Configuration descriptor
 */
void DMA_ioctl (DMA_Handle handle, const DMA_Config *pConfig);


/** Acquire access to a DMA channel.
 *
 *  \param[in] handle Handle of DMA block.
 *  \param[out] pChannelHandle Handle of DMA channel
 *  \retval LPCLIB_SUCCESS Ok! pChannelHandle holds a valid handle.
 *  \retval LPCLIB_BUSY Error! pChannelHandle is invalid.
 */
LPCLIB_Result DMA_acquireChannel (DMA_Handle handle, DMA_ChannelHandle *pChannelHandle);


/** Release access to a DMA channel.
 *
 *  \param[in] pChannelHandle Handle of DMA channel
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER not a valid handle
 */
LPCLIB_Result DMA_releaseChannel (DMA_ChannelHandle *pChannelHandle);


/** Memory-to-memory copy.
 *
 *  \param[in] pContext Pointer to a DMA_MemcpyContext
 *  \param[in] pDest Destination address
 *  \param[in] pSource Source address
 *  \param[in] size Size of memory block in bytes
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER not a valid handle
 */
LPCLIB_Result DMA_memcpy (DMA_MemcpyContext *pContext,
                          void *pDest,
                          void *pSource,
                          int size);


/** Ask for memory allocation for DMA phases.
 *
 *  This function is typically used by other drivers if they have to transfer an arbitrary
 *  number of phases. It is done centrally by the DMA driver.
 *  If no memory can be allocated (or if no system handler has been installed!), NULL is
 *  returned, and the other driver shall fall back to interrupt-driven mode, where possible.
 *
 *  \param[in] channel Channel handle
 *  \param[in] numPhases Number of requested phases
 *  \return NULL if no memory available. Otherwise pointer to a memory area that can
 *          hold a \ref DMA_Job plus numPhases elements of type \ref DMA_JobPhase.
 *          job->firstPhase points to phase array.
 */
DMA_Job * DMA_allocPhaseMemory (DMA_ChannelHandle channel, int numPhases);


/** Return memory previously allocated by \ref DMA_allocPhaseMemory.
 *
 *  \param[in] channel Channel handle
 *  \param[in] pJob Dynamically allocated memory
 *  \retval LPCLIB_SUCCESS
 */
LPCLIB_Result DMA_freePhaseMemory (DMA_ChannelHandle channel, DMA_Job *pJob);


/** Submit a job to the DMA driver.
 *
 *  \param[in] handle Handle for DMA channel
 *  \param[in] pJob Job descriptor
 *  \retval DMA_SUCCESS Ok.
 */
LPCLIB_Result DMA_submitJob (DMA_ChannelHandle handle, const DMA_Job *pJob);


/** Prematurely terminate an active job.
 *
 *  \param[in] handle Handle for DMA channel
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER not a valid handle
 */
LPCLIB_Result DMA_cancelJob (DMA_ChannelHandle handle);


/** Fill data into an DMA_JobPhase descriptor.
 *
 *  \param[in] phase Pointer to the phase descriptor.
 *  \param[in] from Source address of transfer.
 *  \param[in] to Destination address of transfer.
 *  \param[in] config Parameters of the transfer.
 *  \return Pointer to the phase descriptor (same as \a phase).
 */
__FORCEINLINE(DMA_JobPhase *DMA_makePhase (DMA_JobPhase *pPhase,
                             uint32_t from,
                             uint32_t to,
                             const void *callbackParameter,
                             DMA_JobPhaseParameter config))
{
    pPhase->next = NULL;
    pPhase->pSource = (void *)from;
    pPhase->pDest = (void *)to;
    pPhase->parameter = config;
    pPhase->callbackParameter = callbackParameter;

    return pPhase;
}


/** Combine all options of a DMA phase.
 *
 *  This is a helper function whose sole purpose is to combine all configurable
 *  options of a DMA phase into a 32-bit value that can be written into the DMA's
 *  control register.
 *  While this could be done with a macro, the advantage of this function is the ability
 *  to do type checking of the arguments. In order not to lose efficiency, this function
 *  *must* be inlined.
 *
 *  \param[in] numTransfers Number of transfers of size sourceBurstSize.
 *  \param[in] sourceBurstSize Number of bytes in a source burst.
 *  \param[in] destBurstSize Number of bytes in a destination burst.
 *  \param[in] sourceWidth Bus width (8/16/32 bits) of a source transfer.
 *  \param[in] destWidth Bus width (8/16/32 bits) of a destination transfer.
 *  \param[in] sourceIncrement Flag: Increment source address after each transfer.
 *  \param[in] destIncrement Flag: Increment destination address after each transfer.
 *  \param[in] interrupt Flag: Interrupt (=callback) at end of this phase. Forced for last phase.
 *  \return The control word for the DMA.
 */
__FORCEINLINE(DMA_JobPhaseParameter DMA_makePhaseParameter (
            int numTransfers,
            DMA_BurstSize sourceBurstSize,
            DMA_BurstSize destBurstSize,
            DMA_TransferWidth sourceWidth,
            DMA_TransferWidth destWidth,
            LPCLIB_Switch sourceIncrement,
            LPCLIB_Switch destIncrement,
            LPCLIB_Switch interrupt
            ))
{
    DMA_JobPhaseParameter x;

    x = ((numTransfers & 0xFFF) << 0) |
        (sourceBurstSize << 12) |
        (destBurstSize << 15) |
        (sourceWidth << 18) |
        (destWidth << 21) |
        ((sourceIncrement & 1) << 26) |
        ((destIncrement & 1) << 27) |
        ((interrupt & 1) << 31)
        ;

    return x;
}


/** Prepare a DMA_MemcpyContext for use with \ref DMA_memcpy(). */
__FORCEINLINE(DMA_MemcpyContext * DMA_prepareMemcpyContext (
            DMA_MemcpyContext *pContext,
            DMA_ChannelHandle channel,
            LPCLIB_Callback callback
            ))
{
    if (pContext) {
        pContext->job.channel = channel;
        pContext->job.callback = callback;
    }

    return pContext;
}


/** @} DMA API Functions */

#endif /* #if LPCLIB_DMA */

#endif /* #ifndef __LPC17XX_DMA_H__ */

/** @} */



