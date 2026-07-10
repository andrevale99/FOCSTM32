#ifndef IHM_H
#define IHM_H

#include <stm32f411xe.h>

#include "lcd16x2.h"

int8_t ihm_init(lcd16x2_handle *const lcd,
                void (*init_periferico_lcd16x2)(void));

#endif