

/* External functions */
extern int __main (void);
extern void SystemInit (void);


/* Default interrupt service routines.
 * Defined as weak symbols. Can be redefined in the application.
 */
#define ALIAS(def) __attribute__((weak,alias(#def)))

void Default_IRQHandler (void);

void Reset_Handler (void);
void NMI_Handler (void) ALIAS(_NMI_Handler);
void HardFault_Handler (void) ALIAS(_HardFault_Handler);
void MemManage_Handler (void) ALIAS(_MemManage_Handler);
void BusFault_Handler (void) ALIAS(_BusFault_Handler);
void UsageFault_Handler (void) ALIAS(_UsageFault_Handler);
void SVC_Handler (void) ALIAS(_SVC_Handler);
void DebugMon_Handler (void) ALIAS(_DebugMon_Handler);
void PendSV_Handler (void) ALIAS(_PendSV_Handler);
void SysTick_Handler (void) ALIAS(_SysTick_Handler);

extern void Image$$ARM_LIB_STACK$$ZI$$Limit;

void WDT_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER0_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER1_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER2_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER3_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART0_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART1_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART2_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART3_IRQHandler (void) ALIAS(Default_IRQHandler);
void PWM1_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C0_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C1_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C2_IRQHandler (void) ALIAS(Default_IRQHandler);
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
void ENET_IRQHandler (void) ALIAS(Default_IRQHandler);
void MCI_IRQHandler (void) ALIAS(Default_IRQHandler);
void MCPWM_IRQHandler (void) ALIAS(Default_IRQHandler);
void QEI_IRQHandler (void) ALIAS(Default_IRQHandler);
void PLL1_IRQHandler (void) ALIAS(Default_IRQHandler);
void USBActivity_IRQHandler (void) ALIAS(Default_IRQHandler);
void CANActivity_IRQHandler (void) ALIAS(Default_IRQHandler);



/** Vector table.
 */
void (* const vector_table[])(void) __attribute__((section("RESET"))) =
{
    (void(*)(void))&Image$$ARM_LIB_STACK$$ZI$$Limit,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,                     /* -10: Usage Fault */
    0,
    0,
    0,
    0,
    SVC_Handler,                            /* -5: Supervisor Call (RTOS) */
    DebugMon_Handler,                       /* -4: Debug Monitor */
    0,
    PendSV_Handler,                         /* -2: PendSV (RTOS */
    SysTick_Handler,                        /* -1: SysTick (RTOS) */

    WDT_IRQHandler,                         /* 0: Watchdog Timer */
    TIMER0_IRQHandler,                      /* 1: Timer0 */
    TIMER1_IRQHandler,                      /* 2: Timer1 */
    TIMER2_IRQHandler,                      /* 3: Timer2 */
    TIMER3_IRQHandler,                      /* 4: Timer3 */
    UART0_IRQHandler,                       /* 5: UART0 */
    UART1_IRQHandler,                       /* 6: UART1 */
    UART2_IRQHandler,                       /* 7: UART2 */
    UART3_IRQHandler,                       /* 8: UART3 */
    PWM1_IRQHandler,                        /* 9: PWM1 */
    I2C0_IRQHandler,                        /* 10: I2C0 */
    I2C1_IRQHandler,                        /* 11: I2C1 */
    I2C2_IRQHandler,                        /* 12: I2C2 */
    0,
    SSP0_IRQHandler,                        /* 14: SSP0 */
    SSP1_IRQHandler,                        /* 15: SSP1 */
    PLL0_IRQHandler,                        /* 16: PLL0 Lock (Main PLL) */
    RTC_IRQHandler,                         /* 17: Real Time Clock */
    EINT0_IRQHandler,                       /* 18: External Interrupt 0 */
    EINT1_IRQHandler,                       /* 19: External Interrupt 1 */
    EINT2_IRQHandler,                       /* 20: External Interrupt 2 */
    EINT3_IRQHandler,                       /* 21: External Interrupt 3 */
    ADC_IRQHandler,                         /* 22: A/D Converter */
    BOD_IRQHandler,                         /* 23: Brown-Out Detect */
    USB_IRQHandler,                         /* 24: USB */
    CAN_IRQHandler,                         /* 25: CAN */
    DMA_IRQHandler,                         /* 26: General Purpose DMA */
    I2S_IRQHandler,                         /* 27: I2S */
    ENET_IRQHandler,                        /* 28: Ethernet */
    MCI_IRQHandler,                         /* 29: SD/MMC Card */
    MCPWM_IRQHandler,                       /* 30: Motor Control PWM */
    QEI_IRQHandler,                         /* 31: Quadrature Encoder */
    PLL1_IRQHandler,                        /* 32: PLL1 Lock (USB PLL) */
    USBActivity_IRQHandler,                 /* 33: USB Activity */
    CANActivity_IRQHandler,                 /* 34: CAN Activity */
};


__attribute__ ((section(".ARM.__at_0x02FC")))
const unsigned long CRP_Key = 0xFFFFFFFF;


void Reset_Handler (void)
{
    /* System initialization (Clocks, interrupts) */
    SystemInit();                           /* Note: Globals not yet initialized!! */

    /* Jump to the library entry point __main() */
    __main();

    /* Loop here forever, in case main() ever returns. */
    while (1) {
    }
}




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

void _MemManage_Handler (void)
{
    while (1)
        ;
}

void _BusFault_Handler (void)
{
    while (1)
        ;
}

void _UsageFault_Handler (void)
{
    while (1)
        ;
}

void _SVC_Handler (void)
{
    while (1)
        ;
}

void _DebugMon_Handler (void)
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


#if defined (__CC_ARM) && !defined(__MICROLIB)
__asm void __user_setup_stackheap (void)
{
    IMPORT  |Image$$ARM_LIB_HEAP$$ZI$$Base|
    IMPORT  |Image$$ARM_LIB_HEAP$$ZI$$Limit|
    IMPORT  |Image$$ARM_LIB_STACK$$ZI$$Base|

    ldr     r0, =|Image$$ARM_LIB_HEAP$$ZI$$Base|
    ldr     r2, =|Image$$ARM_LIB_HEAP$$ZI$$Limit|
    ldr     r3, =|Image$$ARM_LIB_STACK$$ZI$$Base|
    
    bx      lr
  ALIGN
}
#endif
