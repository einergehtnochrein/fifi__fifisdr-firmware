
#include <stdio.h>

#include "lpclib.h"
#include "bsp-fifisdr.h"

#include "task-sys.h"
#include "task-gui.h"
#include "usbuser.h"

#include "system_LPC17xx.h"
#include "params.h"


osThreadDef(GUI_task, osPriorityLow, 1, 0);
osThreadDef(SYS_task, osPriorityNormal, 1, 1024);
osThreadDef(USBUSER_task, osPriorityAboveNormal, 1, 1024);
osThreadDef(FIFIAUDIO_task, osPriorityRealtime, 1, 1920);


/** main() is started as an unprivileged task! */
#if defined(RTOS_FREERTOS)
static void systemInit_task (const void *pvParameters)
{
    (void) pvParameters;
#else
int main (void)
{
#endif

#if defined (__GNUC__)
    /* TODO
     * The 'manual' call to __sinit() is necessary with NEWLIB for the following reasons:
     * NEWLIB automatically compiles with option _REENT_SMALL if the compiler uses Thumb-2
     * (true for Cortex-M3). In this case the standard file (stdout etc) pointers in the global
     * impure_data point to shortened fake FILE* in ROM, until they get initialized by __sinit().
     * It is unclear who is responsible for calling __sinit(), but doing it here solves it for now.
     */
    void __sinit (void *);
    __sinit(_global_impure_ptr);
#endif

    paramsRead();                                       /* Global system settings */

    BSP_init();

    /* Create tasks */
    osThreadCreate(osThread(GUI_task), NULL);
    osThreadCreate(osThread(SYS_task), NULL);
    osThreadCreate(osThread(USBUSER_task), NULL);
    osThreadCreate(osThread(FIFIAUDIO_task), NULL);

    /* The systemInit task is no longer needed */
    osThreadTerminate(osThreadGetId());

    while (1)
        ;
}


#if defined(RTOS_FREERTOS)
osThreadDef(systemInit_task, osPriorityHigh, 1, 3060);

int main (void)
{
    /* Start the scheduler. */
    osKernelStart(osThread(systemInit_task), NULL);
    while (1)
        ;
}
#endif



/* Override system initialization (runs before main()!).
 * Set CPU clock to speed up the startup process.
 */
void SystemInit (void)
{
    /* Allow ITM printf debug output (channel 0) */
    ITM->TER |= (1u << 0);

    BSP_systemInit();

    /* Configure and enable interrupts used in this application */
    NVIC_SetPriority(I2C1_IRQn, 12);
    NVIC_SetPriority(USB_IRQn, 10);
    NVIC_SetPriority(USBActivity_IRQn, 0);
    NVIC_SetPriority(DMA_IRQn, 8);
    NVIC_SetPriority(I2S_IRQn, 8);
    NVIC_SetPriority(SSP1_IRQn, 8);
    NVIC_SetPriority(EINT3_IRQn, 8);
    NVIC_SetPriority(ADC_IRQn, 8);

    NVIC_EnableIRQ(I2C1_IRQn);
    NVIC_EnableIRQ(USB_IRQn);
    NVIC_EnableIRQ(USBActivity_IRQn);
    NVIC_EnableIRQ(DMA_IRQn);
    NVIC_EnableIRQ(I2S_IRQn);
    NVIC_EnableIRQ(SSP1_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);             /* GPIO interrupts */
    NVIC_EnableIRQ(ADC_IRQn);

    /* Erratum PCLKSELx.1: Must set dividers before enabling PLL0! */
    CLKPWR_setDivider(CLKPWR_DIVIDER_TIMER3, CLKPWR_RATIO_4);   /* used in fifiaudio.c */
    CLKPWR_setDivider(CLKPWR_DIVIDER_I2S, CLKPWR_RATIO_2);      /* NOTE: LPC17xx erratum:
                                                                 * PCLK(I2S) must be 72 MHz max!
                                                                 */

//TODO
#if defined(RTOS_FREERTOS)
    CLKPWR_setCpuClock(configCPU_CLOCK_HZ);             /* Take frequency from FreeRTOSConfig.h */
#else
    CLKPWR_setCpuClock(99840000ul);
#endif
}


#if defined(RTOS_FREERTOS)
/* FreeRTOS stubs */
extern void vApplicationIdleHook (void);
void vApplicationIdleHook (void)
{
}

void vApplicationStackOverflowHook (xTaskHandle *pxTask, signed portCHAR *pcTaskName);
void vApplicationStackOverflowHook (xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
    (void) pxTask;
    (void) pcTaskName;

}

#endif
