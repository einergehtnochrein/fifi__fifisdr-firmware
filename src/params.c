
#include <string.h>

#include "lpclib.h"

#include "params.h"



struct structParams g_params;



/**
 * Read parameters from flash
 */
void paramsRead (void)
{
    int i;


    /* Copy from parameter sector */
    memcpy (&g_params, (void *)0x78000, sizeof(g_params));

    /* Set defaults */
    g_params.magic = MAGIC;
    g_params.version = 1;

    if (g_params.freqxtal == 0xFFFFFFFF)
        g_params.freqxtal = _8_24(114.285);

    if (g_params.freqsubtract1121 == (int32_t)0xFFFFFFFF)
        g_params.freqsubtract1121 = _11_21(4*0.012);    /* 12 kHz for demodulator */
    if (g_params.freqmultiply == 0xFFFFFFFF)
        g_params.freqmultiply = _11_21(1.0);
    if (g_params.smoothtune == 0xFFFFFFFF)
        g_params.smoothtune = 3000;             /* ppm */
    if (g_params.freqstartup == 0xFFFFFFFF)
        g_params.freqstartup = _11_21(4*3.525);

    if (g_params.freq_3rd_harmonic == 0xFFFFFFFF)
        g_params.freq_3rd_harmonic = _11_21(4*30.0);
    if (g_params.freq_5th_harmonic == 0xFFFFFFFF)
        g_params.freq_5th_harmonic = _11_21(4*60.0);

    if (g_params.presel_mode == 0xFFFFFFFF)
        g_params.presel_mode = 1;
    for (i = 0; i < 16; i++) {
        if (g_params.presel_freq[i][0] == 0xFFFFFFFF) {
            switch (i) {
                case 0:     g_params.presel_freq[i][0] = _11_21(4*0.0);
                            g_params.presel_freq[i][1] = _11_21(4*0.123);
                            g_params.presel_pattern[i] = 0x01;
                break;
                case 1:     g_params.presel_freq[i][0] = _11_21(4*0.123);
                            g_params.presel_freq[i][1] = _11_21(4*0.307);
                            g_params.presel_pattern[i] = 0x07;
                break;
                case 2:     g_params.presel_freq[i][0] = _11_21(4*0.307);
                            g_params.presel_freq[i][1] = _11_21(4*0.768);
                            g_params.presel_pattern[i] = 0x00;
                break;
                case 3:     g_params.presel_freq[i][0] = _11_21(4*0.768);
                            g_params.presel_freq[i][1] = _11_21(4*1.92);
                            g_params.presel_pattern[i] = 0x02;
                break;
                case 4:     g_params.presel_freq[i][0] = _11_21(4*1.92);
                            g_params.presel_freq[i][1] = _11_21(4*4.8);
                            g_params.presel_pattern[i] = 0x06;
                break;
                case 5:     g_params.presel_freq[i][0] = _11_21(4*4.8);
                            g_params.presel_freq[i][1] = _11_21(4*12.0);
                            g_params.presel_pattern[i] = 0x05;
                break;
                case 6:     g_params.presel_freq[i][0] = _11_21(4*12.0);
                            g_params.presel_freq[i][1] = _11_21(4*30.0);
                            g_params.presel_pattern[i] = 0x03;
                break;
                case 7:     g_params.presel_freq[i][0] = _11_21(4*30.0);
                            g_params.presel_freq[i][1] = _11_21(4*75.0);
                            g_params.presel_pattern[i] = 0x04;
                break;
                case 8:     g_params.presel_freq[i][0] = _11_21(4*75.0);
                            g_params.presel_freq[i][1] = _11_21(4*150.0);
                            g_params.presel_pattern[i] = 0x0C;
                break;
                case 15:    g_params.presel_freq[i][0] = _11_21(4*0.0);
                            g_params.presel_freq[i][1] = _11_21(4*500.0);
                            g_params.presel_pattern[i] = 0x00;
                break;
                default:    g_params.presel_freq[i][0] = _11_21(4*0.0);
                            g_params.presel_freq[i][1] = _11_21(4*0.0);
                            g_params.presel_pattern[i] = 0x00;
                break;
            }
        }
        if (g_params.presel_freq[i][1] == 0xFFFFFFFF)
            g_params.presel_freq[i][1] = _11_21(0.0);
        if (g_params.presel_freq[i][1] < g_params.presel_freq[i][0])
            g_params.presel_freq[i][1] = g_params.presel_freq[i][0];
    }

    if (g_params.virtual_vco_factor == 0xFFFFFFFF)
        g_params.virtual_vco_factor = 4;

    /* Workaround for a bug that may have corrupted the "smoothtune" value. */
    g_params.smoothtune &= 0x0000FFFF;
}


/**
 * Write parameters to flash
 */
void paramsWrite (void)
{
    typedef void (*IAP)(uint32_t [], uint32_t []);
    IAP iap_entry =(IAP) 0x1FFF1FF1;
    uint32_t command[5];
    uint32_t result[5];


    /* Prepare the parameter sector */
    command[0] = 50;
    command[1] = 29;
    command[2] = 29;
    iap_entry (command, result);

    /* Enter critical section */
    __disable_irq();

    /* Erase the parameter sector */
    command[0] = 52;
    command[1] = 29;
    command[2] = 29;
    command[3] = CLKPWR_getBusClock(CLKPWR_CLOCK_CPU) / 1000;
    iap_entry (command, result);

    /* Prepare the parameter sector */
    command[0] = 50;
    command[1] = 29;
    command[2] = 29;
    iap_entry (command, result);

    /* Write parameter sector */
    command[0] = 51;
    command[1] = 0x78000;
    command[2] = (uint32_t)&g_params;
    command[3] = 1024;
    command[4] = CLKPWR_getBusClock(CLKPWR_CLOCK_CPU) / 1000;
    iap_entry (command, result);

    /* Leave critical section */
    __enable_irq();
}


