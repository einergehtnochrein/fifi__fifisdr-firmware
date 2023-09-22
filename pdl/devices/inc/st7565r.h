
#ifndef __ST7565R_H
#define __ST7565R_H

#include <stdint.h>

#include "lpclib.h"


#if LPCLIB_SSP

/** \defgroup ST7565R_Public_Types ST7565R Types, enums, macros
 *  @{
 */


/** Opcodes to specify the configuration command in a call to \ref ST7565R_ioctl. */
typedef enum ST7565R_Opcode {
    ST7565R_OPCODE_INVALID = 0,             /**< List terminator */
    ST7565R_OPCODE_INIT,                    /**< Initialize the display controller */
    ST7565R_OPCODE_CLEAR_SCREEN,            /**< Clear screen content */
} ST7565R_Opcode;



/** Descriptor to specify the configuration in a call to \ref ST7565R_ioctl. */
typedef struct ST7565R_Config {
    ST7565R_Opcode opcode;                  /**< Config action opcode */
} ST7565R_Config;

/** Config list terminator. */
#define ST7565R_CONFIG_END \
    {.opcode = ST7565R_OPCODE_INVALID}


/** Selector for command/data phase. */
typedef enum ST7565R_CommandDataSelect {
    ST7565R_COMMAND = 0,                    /**< A0 = 0: Command */
    ST7565R_DATA = 1,                       /**< A0 = 1: Data */
} ST7565R_CommandDataSelect;


/** Handle for an ST7565R device. */
typedef struct ST7565R_Context *ST7565R_Handle;



/** @} ST7565R Types, enums, macros */


/** \defgroup ST7565R_Public_Functions ST7565R API Functions
 *  @{
 */


/** Open the device.
 *
 *  Enable access to an ST7565R display controller.
 *
 *  \param[in] bus Handle of SSP bus to which the device is connected.
 *  \param[in] pDeviceSelect Method to handle SSEL demuxing
 *  \param[out] pHandle Device handle
 *  \retval LPCLIB_SUCCESS Success. \ref pHandle contains a valid handle.
 */
LPCLIB_Result ST7565R_open (SSP_Handle bus, const SSP_DeviceSelect *pDeviceSelect, ST7565R_Handle *pHandle);


/** Close device.
 *
 *  \param pHandle Device handle
 */
LPCLIB_Result ST7565R_close (ST7565R_Handle *pHandle);


/** Configure the device.
 *
 *  Pass a configuration command to the ST7565R.
 *
 *  \param[in] handle Device handle.
 *  \param[in] pConfig Pointer to a configuration descriptor
 */
LPCLIB_Result ST7565R_ioctl (ST7565R_Handle handle, const ST7565R_Config *pConfig);



/** Write a command or data string.
 *
 *  \param[in] handle Device handle.
 *  \param[in] numBytes Number of bytes to send.
 *  \param[in] pBuffer Pointer to data
 *  \param[in] select Select command/data
 */
LPCLIB_Result ST7565R_writeGeneric (ST7565R_Handle handle,
                                    int numBytes,
                                    const void *pBuffer,
                                    ST7565R_CommandDataSelect select);



/* Write a command string */
#define ST7565R_writeCommand(handle,numBytes,pBuffer) \
    ST7565R_writeGeneric(handle, numBytes, pBuffer, ST7565R_COMMAND)
#define ST7565R_writeData(handle,numBytes,pBuffer) \
    ST7565R_writeGeneric(handle, numBytes, pBuffer, ST7565R_DATA)


/** Write complete frame buffer.
 *
 *  \param[in] handle Device handle
 *  \param[in] frameBuffer Pointer to B/W frame buffer (organized properly...)  TODO
 *  \retval LPCLIB_SUCCESS Ok
 *  \retval LPCLIB_....
 */
LPCLIB_Result ST7565R_writeFullScreen (ST7565R_Handle handle, const uint8_t *frameBuffer);


/** @} ST7565R API Functions */

#endif  /* #if LPCLIB_SSP */

#endif
