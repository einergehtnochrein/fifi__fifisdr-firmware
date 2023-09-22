#ifndef __BSP_LCD_H
#define __BSP_LCD_H

#include "lpclib.h"


LPCLIB_Result BSP_openLcd (void);
LPCLIB_Result BSP_closeLcd (void);
void BSP_show7seg (int x, int y, int c);


#endif


