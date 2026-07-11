#include <stm32f411xe.h>

#include <arm_math.h>

#include "init_lcd16x2.h"
#include "init_systick.h"

#include "rcc.h"
#include "ihm.h"
#include "lcd16x2.h"
#include "inversor.h"

lcd16x2_handle lcd = {
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

    .adc = {
        .GPIOx = GPIOA,
        .ADCx = ADC1,
        .channel_1 = 6,
        .channel_2 = 7,
        .channel_3 = -1,
    },
};

ihm_t ihm = {
    .inv = &inv,
    .lcd = &lcd,

    .on_off_bt = {
        .GPIOx = GPIOB,
        .gpio_pin = 4,
    },

    .setpoint_bt = {
        .GPIOx = GPIOB,
        .gpio_pin = 0,
    },

    .kp_bt = {
        .GPIOx = GPIOB,
        .gpio_pin = 1,
    },

    .ki_bt = {
        .GPIOx = GPIOB,
        .gpio_pin = 2,
    },
};

int main(void)
{

    rcc_clk_enable(RCC_CLK_HSE, true);
    rcc_switch_clk_system(RCC_CLK_HSE);
    rcc_AHB_set_prescale(AHB_DIV_1);

    init_systick();

    lcd16x2_init_4bits(&lcd, init_periferico_lcd16x2);

    inversor_init(&inv);
    inversor_set_duty(&inv, 1150, 40, 1);

    ihm_init(&ihm);
    delay_ms(2000);

    ihm_menu_write(&ihm);

    while (1)
    {
        // ihm_menu_write(&ihm);

        delay_ms(500);
    }

    return 0;
}