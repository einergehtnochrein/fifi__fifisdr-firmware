
#include <stdint.h>


/* External functions */
extern int main (void);
__attribute__((weak)) void software_init_hook (void);   /* NEWLIB! */
extern void SystemInit (void);

/* These section descriptors are defined in the linker script */
extern uint32_t __stack_end__;


/* Default interrupt service routines.
 * Defined as weak symbols. Can be redefined in the application.
 */
#define ALIAS(def) __attribute__((weak,alias(#def)))

void _NMI_Handler (void);
void _HardFault_Handler (void);
void _MPU_Handler (void);
void _BusFault_Handler (void);
void _SVC_Handler (void);
void _UsageFault_Handler (void);
void _PendSV_Handler (void);
void _SysTick_Handler (void);
void Default_IRQHandler (void);

void Reset_Handler (void);
void NMI_Handler (void) ALIAS(_NMI_Handler);
void HardFault_Handler (void) ALIAS(_HardFault_Handler);
void MPU_Handler (void) ALIAS(_MPU_Handler);
void BusFault_Handler (void) ALIAS(_BusFault_Handler);
void SVC_Handler (void) ALIAS(_SVC_Handler);
void UsageFault_Handler (void) ALIAS(_UsageFault_Handler);
void PendSV_Handler (void) ALIAS(_PendSV_Handler);
void SysTick_Handler (void) ALIAS(_SysTick_Handler);

void WDT_IRQHandler (void) ALIAS(Default_IRQHandler);
void Timer0_IRQHandler (void) ALIAS(Default_IRQHandler);
void Timer1_IRQHandler (void) ALIAS(Default_IRQHandler);
void Timer2_IRQHandler (void) ALIAS(Default_IRQHandler);
void Timer3_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART0_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART1_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART2_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART3_IRQHandler (void) ALIAS(Default_IRQHandler);
void PWM1_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C0_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C1_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C2_IRQHandler (void) ALIAS(Default_IRQHandler);
void SPI_IRQHandler (void) ALIAS(Default_IRQHandler);
void SSP0_IRQHandler (void) ALIAS(Default_IRQHandler);
void SSP1_IRQHandler (void) ALIAS(Default_IRQHandler);
void PLL0_IRQHandler (void) ALIAS(Default_IRQHandler);
void RTC_IRQHandler (void) ALIAS(Default_IRQHandler);
void EINT0_IRQHandler (void) ALIAS(Default_IRQHandler);
void EINT1_IRQHandler (void) ALIAS(Default_IRQHandler);
void EINT2_IRQHandler (void) ALIAS(Default_IRQHandler);
void EINT3_IRQHandler (void) ALIAS(Default_IRQHandler);
void ADC_IRQHandler (void) ALIAS(Default_IRQHandler);
void BOD_IRQHandler (void) ALIAS(Default_IRQHandler);
void USB_IRQHandler (void) ALIAS(Default_IRQHandler);
void CAN_IRQHandler (void) ALIAS(Default_IRQHandler);
void DMA_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2S_IRQHandler (void) ALIAS(Default_IRQHandler);
void Ethernet_IRQHandler (void) ALIAS(Default_IRQHandler);
void RIT_IRQHandler (void) ALIAS(Default_IRQHandler);
void MCPWM_IRQHandler (void) ALIAS(Default_IRQHandler);
void QEI_IRQHandler (void) ALIAS(Default_IRQHandler);
void PLL1_IRQHandler (void) ALIAS(Default_IRQHandler);
void USBActivity_IRQHandler (void) ALIAS(Default_IRQHandler);
void CANActivity_IRQHandler (void) ALIAS(Default_IRQHandler);



/** Vector table. */
void (*vector_table[])( void ) __attribute__ ((section(".vectors"))) =
{
    (void(*)(void))&__stack_end__,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MPU_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    0,
    0,
    PendSV_Handler,
    SysTick_Handler,

    WDT_IRQHandler,
    Timer0_IRQHandler,
    Timer1_IRQHandler,
    Timer2_IRQHandler,
    Timer3_IRQHandler,
    UART0_IRQHandler,
    UART1_IRQHandler,
    UART2_IRQHandler,
    UART3_IRQHandler,
    PWM1_IRQHandler,
    I2C0_IRQHandler,
    I2C1_IRQHandler,
    I2C2_IRQHandler,
    SPI_IRQHandler,
    SSP0_IRQHandler,
    SSP1_IRQHandler,

    PLL0_IRQHandler,
    RTC_IRQHandler,
    EINT0_IRQHandler,
    EINT1_IRQHandler,
    EINT2_IRQHandler,
    EINT3_IRQHandler,
    ADC_IRQHandler,
    BOD_IRQHandler,
    USB_IRQHandler,
    CAN_IRQHandler,
    DMA_IRQHandler,
    I2S_IRQHandler,
    Ethernet_IRQHandler,
    RIT_IRQHandler,
    MCPWM_IRQHandler,
    QEI_IRQHandler,
    PLL1_IRQHandler,
    USBActivity_IRQHandler,
    CANActivity_IRQHandler,
};




extern uint32_t __scattertable_start__;

void Reset_Handler (void)
{
    uint32_t *pScatter;
    uint32_t scatterSource;
    uint32_t scatterDest;
    uint32_t scatterSize;

    /* Init the clock tree */
    SystemInit();

    /* Scatter loading.
     * The scatter table contains three types of entries, marked by the first word of the entry:
     * 0 = end of scatter tabel
     * 1 = BSS section (initialize with zero)
     * 2 = DATA section. Initialze with non-zero values from flash.
     *
     * 1: Followed by two words for (1) start address and (b) length in bytes
     * 2. Followed by three words for (1) source address, (2) destination address,
     *    and (3) length in bytes
     */
//TODO Can we do word-wise copy?
    pScatter = &__scattertable_start__;
    while (*pScatter != 0) {
        switch (*pScatter++) {
        case 1:                             /* BSS section */
            scatterDest = *pScatter++;
            scatterSize = *pScatter++;
            while (scatterSize) {
                *((volatile uint8_t *)scatterDest) = 0;
                ++scatterDest;
                --scatterSize;
            }
            break;

        case 2:                             /* DATA section */
            scatterSource = *pScatter++;
            scatterDest = *pScatter++;
            scatterSize = *pScatter++;
            while (scatterSize) {
                *((volatile uint8_t *)scatterDest) = *((volatile uint8_t *)scatterSource);
                ++scatterSource;
                ++scatterDest;
                --scatterSize;
            }
            break;

        default:
            /* Illegal section type. Stop scatter loading. */
            break;
        }
    }

    /* Call a NEWLIB hook (used by RTX to start main() as a task) */
    software_init_hook();                   /* Linker inserts NOP if not defined! */

    /* Now we can jump to the C entry point main() */
    main();

    /* Loop here forever, in case main() ever returns. */
    while (1)
        ;
}



/*------ Core exception handlers --------------------------------------------*/

void _NMI_Handler (void)
{
    while (1)
        ;
}

void _HardFault_Handler (void)
{
    while (1)
        ;
}

void _MPU_Handler (void)
{
    while (1)
        ;
}

void _BusFault_Handler (void)
{
    while (1)
        ;
}

void _SVC_Handler (void)
{
    while (1)
        ;
}

void _UsageFault_Handler (void)
{
    while (1)
        ;
}

void _PendSV_Handler (void)
{
    while (1)
        ;
}

void _SysTick_Handler (void)
{
    while (1)
        ;
}

void Default_IRQHandler (void)
{
    while (1)
        ;
}

/*------ NEWLIB stubs ------------------------------------------------*/

extern void _exit (int returnCode);
__attribute__((noreturn)) void _exit (int returnCode)
{
    (void) returnCode;

    while (1)
        ;
}

extern void _init (void);
void _init (void)
{
}

extern void _fini (void);
void _fini (void)
{
}

