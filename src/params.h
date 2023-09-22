
#ifndef _PARAMS_H_
#define _PARAMS_H_

#include <inttypes.h>

#include "si570.h"



#define MAGIC     0xDEADBEEF


/*
 * Length must be a multiple of 16 bytes.
 */
struct structParams
{
    union {
        uint8_t __dummy[1024];
        struct
        {
            uint32_t magic;
            uint32_t version;

            uint32_t freqxtal;              /* Nominell 114.285 MHz. Format 8.24 */
            int32_t freqsubtract1121;
            uint32_t freqmultiply;
            uint32_t smoothtune;
            uint32_t freqstartup;

            uint16_t freqrxtrip[16];        /* NOTE: No longer used. */

            char serialnumber[20];

            /* Empfang auf Harmonischen der LO-Frequenz.
             * Wenn der LO auf eine Frequenz >= einer der Schaltschwellen eingestellt werden soll,
             * dann wird er statdessen auf ein Drittel bzw. ein Fünftel der geforderten
             * Frequenz gesetzt. Zur Applikation hin wird aber die höhere virtuelle Frequenz
             * gemeldet.
             */
            uint32_t freq_3rd_harmonic;     /* Ab dieser Frequenz (11.21) wird der LO für einen */
                                            /* Empfang auf der 3. Oberwelle abgestimmt. */
            uint32_t freq_5th_harmonic;     /* Ab dieser Frequenz (11.21) wird der LO für einen */
                                            /* Empfang auf der 5. Oberwelle abgestimmt. */

            /* Preselektor.
             * Es sind verschiedene Modi möglich:
             * 0 = ABPF, einstellbar über PE0FKO oder kompatible Software.
             *     Vier high-aktive digitale Ausgänge.
             * 1 = 16 separate Bänder mit Anfangs- und Endfrequenz. Zu jedem Band kann eine
             *     Kombination von acht Steuerpins festgelegt werden (FiFi-SDR 0.1/0.2 nur vier).
             *     Außerhalb der Bänder sind alle Ausgänge auf low.
             * 2 = Wie (1), aber nur drei digitale Ausgänge (noch immer 16 Bänder!), und zusätzlich
             *     eine serielle Ausgabe der Frequenz über UART.
             * 3 = Wie (1), es werden aber nur drei Ausgänge für die acht Bänder des
             *     FiFi-Preselektors benutzt.
             *     Der Pin X6.8 wird als PTT verwendet (PTT aktiv --> X6.8 = low).
             *     Pin X6.8 des FiFi-SDR entspricht Pin SV1.12 am FiFi-Preselektor.
             */
            uint32_t presel_freq[16][2];    /* Anfangs- und Endfrequenz. Format 11.21 */
            uint8_t presel_pattern[16];     /* Bitmuster für Ausgänge */
            uint32_t presel_mode;           /* Betriebsart (0...4) */

            /* Direkte Ansteuerung des Si570 über Register.
             * Software die den Si570 nur direkt ansteuern kann, bekommt die Register nicht direkt
             * zu Gesicht, sondern sieht einen virtuellen Registersatz. Hier folgen einstellbare
             * Parameter für diese Emulation.
             */
            uint32_t virtual_vco_factor;    /* Verhältnis VCO-Frequenz (die Ausgangsfrequenz des */
                                            /* Si570) zu Empfangsfrequenz */
        };
    };
};


extern struct structParams g_params;

void paramsRead( void );
void paramsWrite( void );

#endif
