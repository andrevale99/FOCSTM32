#include "ihm.h"

static char buffer[16];
static int size = 0;

int8_t ihm_init(ihm_t *ihm,
                void (*init_periferico_lcd16x2)(void))
{

    lcd16x2_init_4bits(ihm->lcd, init_periferico_lcd16x2);

    lcd16x2_init_4bits(ihm->lcd, init_periferico_lcd16x2);
    lcd16x2_send_cmd(ihm->lcd, DISPLAY_ON | CURSOR_ON);

    return 0;
}

void ihm_menu_write(ihm_t *ihm)
{
    size = sprintf(buffer, "%ldkHz", inversor_get_frequency(ihm->inv));
    lcd16x2_write_string(ihm->lcd, buffer, size);
}