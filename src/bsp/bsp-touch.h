#ifndef __BSP_TOUCH_H
#define __BSP_TOUCH_H

#include "lpclib.h"


LPCLIB_Result BSP_TOUCH_open (void);
void BSP_TOUCH_worker (LPCLIB_Event event);


#endif


