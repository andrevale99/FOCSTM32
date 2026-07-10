#include "ihm.h"

static lcd16x2_handle *lcd;

int8_t ihm_init(lcd16x2_handle *const lcd,
                void (*init_periferico_lcd16x2)(void))
{

    lcd16x2_init_4bits(&lcd, init_periferico_lcd16x2);

    return 0;
}