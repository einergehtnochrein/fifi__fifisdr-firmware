/* Describe the available memory blocks */
MEMORY
{
  flash    : ORIGIN = 0x00002000, LENGTH = 504K
  ram      : ORIGIN = 0x10000000, LENGTH = 32K
  ram2     : ORIGIN = 0x2007C000, LENGTH = 16K
  ram3     : ORIGIN = 0x20080000, LENGTH = 16K
}

/* Stack grows downwards from the end of the first (main) RAM block.
 * Keep 32 bytes free for calls to the IAP flash programming functions.
 * Keep another 16 bytes free to work around a GDB issue where it reads
 * beyond the end of the stack.
 */
__stack_end__ = ORIGIN( ram ) + LENGTH( ram ) - 32 - 16;



/* --------------------------------------------------------
 * FreeRTOS -- Definitions for Memory Protection Unit (MPU)
 * --------------------------------------------------------
 */

/* The 'FLASH_segment' will be set to unprivileged read-only access.
 * Include the whole flash address range.
 */
__FLASH_segment_start__ = ORIGIN( flash );
__FLASH_segment_end__   = __FLASH_segment_start__ + LENGTH( flash );

/* The 'SRAM_segment' will be set to unprivileged read/write access.
 * Include the full range of the first RAM block. Access to the other
 * RAM blocks must be allowed on a per task basis!
 */
__SRAM_segment_start__ = ORIGIN( ram );
__SRAM_segment_end__   = __SRAM_segment_start__ + LENGTH( ram );

/* Define the size of the Flash (functions) and RAM (data) regions which are
 * protected for kernel use. The size must be sufficient for the kernel,
 * and it must be a valid MPU region size.
 * VERY IMPORTANT: Make sure that __FLASH_segment_start__ is aligned to
 *                 at least _sizeof_privileged_functions_!
 */
_sizeof_privileged_functions_ = 8K;
_sizeof_privileged_data_      = 512;

/* FreeRTOS assumes that the privileged functions always start at the
 * beginning of the FLASH_segment. We only have to define the end address.
 */
__privileged_functions_end__  = __FLASH_segment_start__ + _sizeof_privileged_functions_;

/* Protected RAM data for kernel use. Make sure to place it at the beginning
 * of the .data section below (We have to assume that this can be initialized
 * RAM, so it cannot be put in the .bss section).
 */
__privileged_data_start__ = ORIGIN( ram );
__privileged_data_end__   = __privileged_data_start__ + _sizeof_privileged_data_;





SECTIONS
{
    . = ORIGIN(flash);

    .text :
    {
        KEEP(*(.vectors))
/*    *(privileged_functions)/

    . = ALIGN(_sizeof_privileged_functions_);
*/
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.gnu.linkonce.r.*)
    } >flash

    /* Ordered library section goes separately. */
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >flash

    .data :
    {
        __data_start__ = .;
/*    *(privileged_data)

    . = ALIGN(_sizeof_privileged_data_);
*/
        *(.data .data.*)
    } >ram AT>flash

    .ramcode :
    {
        KEEP(*(LPCLIB_RAMCODE))
    } >ram AT>flash

    .noinit :
    {
        *(.noinit)
    } >ram

    .bss :
    {
        *(.bss COMMON)
    } >ram

    __newlib_heap_start__ = .;
    __heap_start = .;

    .data2 :
    {
        *(.data2)
    } >ram2 AT>flash

    .bss2 (NOLOAD) :
    {
        *(.bss2)
    } >ram2

    .data3 :
    {
        *(.data3)
    } >ram3 AT>flash

    .bss3 (NOLOAD) :
    {
        *(.bss3)
    } >ram3


    /* The scatter table defines sections to be initialized by the startup code.
     * An entry starts with one of these 32-bit words:
     * 0 = end of table
     * 1 = BSS section (initialize with zero)
     * 2 = DATA section. Initialze with non-zero values from flash.
     *
     * 1: Followed by two words for (1) start address and (b) length in bytes
     * 2. Followed by three words for (1) source address, (2) destination address,
     *    and (3) length in bytes
     */
    .scattertable :
    {
        __scattertable_start__ = .;

        LONG(2)
        LONG(LOADADDR(.data))
        LONG(ADDR(.data))
        LONG(SIZEOF(.data))

        LONG(2)
        LONG(LOADADDR(.data2))
        LONG(ADDR(.data2))
        LONG(SIZEOF(.data2))

        LONG(2)
        LONG(LOADADDR(.data3))
        LONG(ADDR(.data3))
        LONG(SIZEOF(.data3))

        LONG(1)
        LONG(ADDR(.bss))
        LONG(SIZEOF(.bss))

        LONG(1)
        LONG(ADDR(.bss2))
        LONG(SIZEOF(.bss2))

        LONG(1)
        LONG(ADDR(.bss3))
        LONG(SIZEOF(.bss3))

        LONG(0)
    } >flash

    /* Collect some statistical information */
    __ram_size__      = (SIZEOF(.data) + SIZEOF(.ramcode) + SIZEOF(.noinit) + SIZEOF(.bss));
    __ram2_size__     = (SIZEOF(.data2) + SIZEOF(.bss2));
    __ram3_size__     = (SIZEOF(.data3) + SIZEOF(.bss3));
    __flash_size__    = (SIZEOF(.text) + SIZEOF(.data) + SIZEOF(.ramcode) + SIZEOF(.data2) + SIZEOF(.data3));
}

