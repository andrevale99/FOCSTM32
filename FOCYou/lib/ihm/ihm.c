#include "ihm.h"

static char buffer[16];
static int size = 0;

int8_t ihm_init(ihm_t *ihm)
{
    size = sprintf(buffer, "%ldkHz", inversor_get_frequency(ihm->inv));
    lcd16x2_write_string(ihm->lcd, buffer, size);

    return 0;
}

void ihm_menu_write(ihm_t *ihm)
{
    size = sprintf(buffer, "               ");
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE);
    size = sprintf(buffer, "%i  %li %li", inversor_get_state(), ADC1->JDR1, ADC1->JDR2);
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE);
}