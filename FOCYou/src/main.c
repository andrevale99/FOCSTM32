#include <stdio.h>

#include <stm32f411xe.h>

#include <arm_math.h>

#include "init_rcc.h"
#include "init_systick.h"

#include "lcd16x2.h"
#include "init_lcd16x2.h"

#include "inversor.h"
#include "init_inversor.h"

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
        .autoreload = 624,
    },
};

int map_value(uint32_t x,
                   uint32_t in_min,
                   uint32_t in_max,
                   uint32_t out_min,
                   uint32_t out_max)
{
    return (x - in_min) * (out_max - out_min) /
               (in_max - in_min) +
           out_min;
}

int main(void)
{

    init_rcc();
    init_systick();

    lcd16x2_init_4bits(&lcd, init_periferico_lcd16x2);
    lcd16x2_send_cmd(&lcd, DISPLAY_ON | CURSOR_ON);

    inversor_init(&inv);

    char buffer[16];
    int size = sprintf(buffer, "%ld", inversor_get_frequency(&inv));
    lcd16x2_write_string(&lcd, buffer, size);

    while (1)
    {
    }

    return 0;
}