
#include <stdint.h>
#include <stdio.h>

#include "bsp-fifisdr.h"

#include "lpclib.h"
#include "task-gui.h"
#include "task-sys.h"


#define GUI_QUEUE_LENGTH                        (4)



typedef struct {
    uint8_t opcode;
    uint32_t frequencyHz;
} GUI_Message;


/** Message opcodes for GUI task. */
enum {
    GUI_OPCODE_UNKNOWN = 0,
    GUI_OPCODE_LED_TICK,
    GUI_OPCODE_UPDATE_FREQUENCY,
};


/** Identifiers for OS timers. */
enum {
    GUI_TIMERMAGIC_LED,
};


static void GUI_eventHandler (LPCLIB_Event event);


/** Local task context. */
static struct {
    osMailQId queue;
    osTimerId ledTick;
    int led;
    LPCLIB_Callback oldSysCallback;
} gui;


/** Install a callback to become informed about system events. */
static const struct SYS_ConfigCallback sysCallback = {
    .callback = GUI_eventHandler,
    .pOldCallback = &gui.oldSysCallback,
};



/** GUI event handler */
static void GUI_eventHandler (LPCLIB_Event event)
{
    GUI_Message *pMessage;


    if (gui.oldSysCallback) {
        gui.oldSysCallback(event);
    }

    if (event.id == LPCLIB_EVENTID_APPLICATION) {
        pMessage = osMailAlloc(gui.queue, 0);
        if (pMessage == NULL) {
            return;
        }

        pMessage->opcode = GUI_OPCODE_UNKNOWN;

        switch (event.opcode) {
        case FIFISDR_EVENT_FREQUENCY_HZ:
            pMessage->opcode = GUI_OPCODE_UPDATE_FREQUENCY;
            pMessage->frequencyHz = (uint32_t)event.parameter;
            break;
        }

        osMailPut(gui.queue, pMessage);
    }
}



static void GUI_osalCallback (void const *pArgument)
{
    (void) pArgument;

    if (gui.queue == NULL) {
        return;
    }

    GUI_Message *pMessage = osMailAlloc(gui.queue, 0);

    if (pMessage == NULL) {
        return;
    }

    pMessage->opcode = GUI_OPCODE_LED_TICK;
    osMailPut(gui.queue, pMessage);
}


osMailQDef(guiQueue, GUI_QUEUE_LENGTH, GUI_Message);
osTimerDef(led, GUI_osalCallback);

void GUI_task (const void *pArgs)
{
    (void) pArgs;
    GUI_Message *pMessage;
    osEvent event;
int temp;
extern void BSP_show7seg (int x, int y, int c);


    gui.queue = osMailCreate(osMailQ(guiQueue), NULL);

    gui.ledTick = osTimerCreate(osTimer(led), osTimerPeriodic, (void *)GUI_TIMERMAGIC_LED);
    osTimerStart(gui.ledTick, 500);

    /* Stay informed about system events */
    SYS_installCallback(sysCallback);

    while (1) {
        /* Is there a new message? */
        event = osMailGet(gui.queue, osWaitForever);
        if (event.status == osEventMail) {
            pMessage = (GUI_Message *)event.value.p;
            switch (pMessage->opcode) {
            case GUI_OPCODE_LED_TICK:
                gui.led = gui.led ^ 1;
                BSP_setLed(gui.led);
                break;

            case GUI_OPCODE_UPDATE_FREQUENCY:

temp = pMessage->frequencyHz / 100000000;
BSP_show7seg( 0, 0, temp);
temp = (pMessage->frequencyHz % 100000000) / 10000000;
BSP_show7seg(15, 0, temp);
temp = (pMessage->frequencyHz % 10000000) / 1000000;
BSP_show7seg(30, 0, temp);

temp = (pMessage->frequencyHz % 1000000) / 100000;
BSP_show7seg(50, 0, temp);
temp = (pMessage->frequencyHz % 100000) / 10000;
BSP_show7seg(65, 0, temp);
temp = (pMessage->frequencyHz % 10000) / 1000;
BSP_show7seg(80, 0, temp);

temp = (pMessage->frequencyHz % 1000) / 100;
BSP_show7seg(100, 0, temp);
temp = (pMessage->frequencyHz % 100) / 10;
BSP_show7seg(115, 0, temp);
                break;
            }

            osMailFree(gui.queue, pMessage);
        }
    }
}

