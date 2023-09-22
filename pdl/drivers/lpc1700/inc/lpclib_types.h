
/** @defgroup LPC_Types
 *  @ingroup LPC1700CMSIS_FwLib_Drivers
 *  @{
 */


#ifndef LPCLIB_TYPES_H
#define LPCLIB_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#if defined ( __CC_ARM )        /*!< ARM Compiler */
  #define __FORCEINLINE(foo)    static __forceinline foo
  #define __WEAKFUNC(foo)       __weak foo
  #define __PACKED(type)        __packed type
  #define __ASMFUNC(foo)        __asm foo
  #define __SECTION(name)       __attribute__((section(name)))
  #define __ALIGN(n)            __attribute__((aligned(n)))
  #pragma anon_unions

#elif defined ( __ICCARM__ )    /*!< IAR Compiler. Only avaiable in High optimization mode! */
  #define __FORCEINLINE(foo)    static inline foo
  #define __WEAKFUNC(foo)       __weak foo

#elif defined ( __GNUC__ )      /*!< GNU Compiler */
  #define __FORCEINLINE(foo)    static inline foo __attribute__((always_inline)); foo
  #define __WEAKFUNC(foo)       foo __attribute__((weak)); foo
  #define __PACKED(type)        type __attribute__((__packed__))
  #define __SECTION(name)       __attribute__((section(name)))
  #define __ALIGN(n)            __attribute__((aligned(n)))

#elif defined ( __TASKING__ )   /*!< TASKING Compiler */
  #define __FORCEINLINE(foo)    static inline foo

#endif



#ifndef NULL
#define NULL ((void*)0)
#endif



/** @defgroup LPC_Types_Public_Types
 * @{
 */

/**
 * @brief Flag Status and Interrupt Flag Status type definition
 */
typedef enum {RESET = 0, SET = !RESET} FlagStatus, IntStatus, SetState;

/** Functional Switch.
 */
typedef enum {
    DISABLE = 0,                            /**< OFF state */
    ENABLE = 1,                             /**< ON state (*MUST* be 1, *NOT* !DISABLE) */

    LPCLIB_NO = 0,                          /**< OFF state */
    LPCLIB_YES = 1,                         /**< ON state (*MUST* be 1, *NOT* !DISABLE) */
} LPCLIB_Switch;


/**
 * @}
 */


/** Result type for many API functions.
 */
typedef enum LPCLIB_Result {
    LPCLIB_PENDING = 1,                     /**< Operation pending (async mode) */
    LPCLIB_SUCCESS = 0,                     /**< Operation completed successfully */
    LPCLIB_BUSY = -1,                       /**< Resource busy */
    LPCLIB_ERROR = -2,                      /**< (unspecified) serious error */
    LPCLIB_ILLEGAL_PARAMETER = -3,          /**< Illegal or out-of-range parameter */
    LPCLIB_NOT_IMPLEMENTED = -4,            /**< Requested feature not implemented */
    LPCLIB_OUT_OF_MEMORY = -5,              /**< Memory exhausted */
    LPCLIB_NO_RESPONSE = -6,                /**< No response from slave (I2C bus) */
    LPCLIB_UNDEFINED = -7,                  /**< unspecified */
    LPCLIB_NOT_PREPARED = -8,               /**< Device/block not prepared for this operation */
    LPCLIB_CRC_FAILURE = -9,                /**< CRC failure */
    LPCLIB_NOT_INITIALIZED = -10,           /**< Initialization missing */
} LPCLIB_Result;


/** Macro for bit-banding access to peripherals */
#define LPCLIB_BITBAND(reg,bit) \
    ((volatile uint32_t *)(32u * ((uint32_t)(reg) & 0x0FFFFFFF) + ((uint32_t)(reg) & 0xF0000000) + 0x02000000))[bit]


/** Macro to define register bits and mask in CMSIS style */
#define LPCLIB_DefineRegBit(name,pos,width) \
    enum { \
        name##_Pos = pos, \
        name##_Msk = (int)(((1ul << width) - 1) << pos), \
    }



/** Source ID for events */
enum LPCLIB_EventId {
    LPCLIB_EVENTID_I2C = 1,
    LPCLIB_EVENTID_I2S,
    LPCLIB_EVENTID_SSP,
    LPCLIB_EVENTID_TIMER,
    LPCLIB_EVENTID_GPIO,
    LPCLIB_EVENTID_ADC,
    LPCLIB_EVENTID_UART,
    LPCLIB_EVENTID_DAC,
    LPCLIB_EVENTID_DMA,
    LPCLIB_EVENTID_MCI,
    LPCLIB_EVENTID_LCD,
    LPCLIB_EVENTID_CRC,
    LPCLIB_EVENTID_RTC,
    LPCLIB_EVENTID_EEPROM,
    LPCLIB_EVENTID_USB,                     /**< USB (hardware driver) event */
    LPCLIB_EVENTID_USBDEV,                  /**< USB device event */
    LPCLIB_EVENTID_USBAUDIO,                /**< USB audio class event */
    LPCLIB_EVENTID_USBHID,                  /**< USB HID class event */
    LPCLIB_EVENTID_USBCDC,                  /**< USB CDC class event */
    LPCLIB_EVENTID_APPLICATION,
};


/** Events sent by drivers in callback functions */
typedef struct LPCLIB_Event {
    enum LPCLIB_EventId id;                 /**< Event category (e.g. TIMER) */
    uint8_t block;                          /**< Source block (e.g. 2 for 3rd timer) */
    uint8_t channel;                        /**< Sub-block identifier (channel) within source */
    int8_t opcode;                          /**< Source specific function id */
    void *parameter;                        /**< Function id specific parameter */
} LPCLIB_Event;

__FORCEINLINE(void LPCLIB_initEvent (LPCLIB_Event *pEvent, enum LPCLIB_EventId id))
{
    //TODO  This is hardcoded knowledge about the size of an event...
    ((uint32_t *)pEvent)[0] = 0;
    ((uint32_t *)pEvent)[1] = 0;
    pEvent->id = id;
}

/** Prototype for callback handlers */
typedef void (*LPCLIB_Callback)(LPCLIB_Event event);


/** File positioning */
typedef enum LPCLIB_FilePos {
    LPCLIB_SEEK_SET,
    LPCLIB_SEEK_CUR,
    LPCLIB_SEEK_END,
} LPCLIB_FilePos;


/** Invalid handle (NULL) */
#define LPCLIB_INVALID_HANDLE   (NULL)


/* Public Macros -------------------------------------------------------------- */
/** @defgroup LPC_Types_Public_Macros
 *  @{
 */

/* _BIT(n) sets the bit at position "n"
 * _BIT(n) is intended to be used in "OR" and "AND" expressions:
 * e.g., "(_BIT(3) | _BIT(7))".
 */
#undef _BIT
/* Set bit macro */
#define _BIT(n)	(1<<n)

/* _SBF(f,v) sets the bit field starting at position "f" to value "v".
 * _SBF(f,v) is intended to be used in "OR" and "AND" expressions:
 * e.g., "((_SBF(5,7) | _SBF(12,0xF)) & 0xFFFF)"
 */
#undef _SBF
/* Set bit field macro */
#define _SBF(f,v) (v<<f)

/* _BITMASK constructs a symbol with 'field_width' least significant
 * bits set.
 * e.g., _BITMASK(5) constructs '0x1F', _BITMASK(16) == 0xFFFF
 * The symbol is intended to be used to limit the bit field width
 * thusly:
 * <a_register> = (any_expression) & _BITMASK(x), where 0 < x <= 32.
 * If "any_expression" results in a value that is larger than can be
 * contained in 'x' bits, the bits above 'x - 1' are masked off.  When
 * used with the _SBF example above, the example would be written:
 * a_reg = ((_SBF(5,7) | _SBF(12,0xF)) & _BITMASK(16))
 * This ensures that the value written to a_reg is no wider than
 * 16 bits, and makes the code easier to read and understand.
 */
#undef _BITMASK
/* Bitmask creation macro */
#define _BITMASK(field_width) ( _BIT(field_width) - 1)


#if !defined(MAX)
    #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#if !defined(MIN)
    #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/** @} */


#endif /* LPCLIB_TYPES_H */

/** @} */

