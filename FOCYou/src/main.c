#include <stdio.h>

#include <stm32f411xe.h>

#include <arm_math.h>

#include "init_rcc.h"
#include "init_systick.h"

#include "lcd16x2.h"
#include "init_lcd16x2.h"

#include "inversor.h"

const lcd16x2_handle lcd = {
    .d4.write = write_d4,
    .d5.write = write_d5,
    .d6.write = write_d6,
    .d7.write = write_d7,

    .en.write = write_en,
    .rs.write = write_rs,

    .delay = delay_ms,
};

inversor_t inv = {
    .Timer = {
        .advTimer = TIM1,
        .prescale = 1,
        .autoreload = 1248,
    },
};

int main(void)
{

    init_rcc();
    init_systick();

    lcd16x2_init_4bits(&lcd, init_periferico_lcd16x2);
    lcd16x2_send_cmd(&lcd, DISPLAY_ON | CURSOR_ON);

    inversor_init(&inv);
    inversor_set_duty(&inv, 1150, 40, 1);

    adc_injected_setup();

    char buffer[16];
    int size = 0;

    size = sprintf(buffer, "%ldkHz", inversor_get_frequency(&inv));
    lcd16x2_write_string(&lcd, buffer, size);
    lcd16x2_send_cmd(&lcd, SECOND_LINE);

    while (1)
    {
        size = sprintf(buffer, "               ");
        lcd16x2_write_string(&lcd, buffer, size);
        lcd16x2_send_cmd(&lcd, SECOND_LINE);
        size = sprintf(buffer, "%i  %li %li", inversor_get_state(), ia, ib);
        lcd16x2_write_string(&lcd, buffer, size);
        lcd16x2_send_cmd(&lcd, SECOND_LINE);
        delay_ms(500);
    }

    return 0;
}