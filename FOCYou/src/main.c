#include <stm32f411xe.h>

#include <arm_math.h>

#include "env.h"
#include "init_lcd16x2.h"
#include "init_systick.h"

#include "rcc.h"
#include "ihm.h"
#include "lcd16x2.h"
#include "inversor.h"

typedef enum
{
    MENU_PRINCIPAL = 0,
    MENU_SETPOINT,
    MENU_KP,
    MENU_KI
} menu_t;

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
        .prescale = TIMER_PRESCALE,
        .autoreload = TIMER_AUTORELOAD,
    },

    .adc = {
        .GPIOx = GPIOA,
        .ADCx = ADC1,
        .channel_1 = INVERSOR_ADC_CHANNEL_1_GPIO,
        .channel_2 = INVERSOR_ADC_CHANNEL_2_GPIO,
        .channel_3 = INVERSOR_ADC_CHANNEL_3_GPIO,
    },
};

ihm_t ihm = {
    .inv = &inv,
    .lcd = &lcd,

    .button_on_off = {
        .GPIOx = IHM_GPIOx_ON_OFF,
        .gpio_pin = IHM_BUTTON_ON_OFF_GPIO,
    },

    .button_setpoint = {
        .GPIOx = IHM_GPIOx_SETPOINT,
        .gpio_pin = IHM_BUTTON_SETPOINT_GPIO,
    },

    .button_kp = {
        .GPIOx = IHM_GPIOx_KP,
        .gpio_pin = IHM_BUTTON_KP_GPIO,
    },

    .button_ki = {
        .GPIOx = IHM_GPIOx_KI,
        .gpio_pin = IHM_BUTTON_KI_GPIO,
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

    ihm_menu_principal_write(&ihm);
    delay_ms(2000);

    menu_t menu_state = MENU_PRINCIPAL;

    bool last_setpoint = true;
    bool last_kp = true;
    bool last_ki = true;

    while (1)
    {
        bool setpoint = ihm.button_setpoint.GPIOx->IDR &
                        (1 << ihm.button_setpoint.gpio_pin);

        bool kp = ihm.button_kp.GPIOx->IDR &
                  (1 << ihm.button_kp.gpio_pin);

        bool ki = ihm.button_ki.GPIOx->IDR &
                  (1 << ihm.button_ki.gpio_pin);

        /* Botão Setpoint */
        if (last_setpoint && !setpoint)
        {
            if (menu_state == MENU_SETPOINT)
                menu_state = MENU_PRINCIPAL;
            else
                menu_state = MENU_SETPOINT;
        }

        /* Botão KP */
        if (last_kp && !kp)
        {
            if (menu_state == MENU_KP)
                menu_state = MENU_PRINCIPAL;
            else
                menu_state = MENU_KP;
        }

        /* Botão KI */
        if (last_ki && !ki)
        {
            if (menu_state == MENU_KI)
                menu_state = MENU_PRINCIPAL;
            else
                menu_state = MENU_KI;
        }

        last_setpoint = setpoint;
        last_kp = kp;
        last_ki = ki;

        switch (menu_state)
        {
        case MENU_PRINCIPAL:

            ihm_menu_principal_write_data(&ihm,
                                          inversor_get_frequency(&inv),
                                          inversor_get_state(),
                                          45,
                                          87);
            break;

        case MENU_SETPOINT:

            ihm_menu_principal_write_data(&ihm,
                                          inversor_get_frequency(&inv),
                                          inversor_get_state(),
                                          455,
                                          87);
            break;

        case MENU_KP:

            ihm_menu_principal_write_data(&ihm,
                                          inversor_get_frequency(&inv),
                                          inversor_get_state(),
                                          0,
                                          0);
            break;

        case MENU_KI:

            ihm_menu_principal_write_data(&ihm,
                                          inversor_get_frequency(&inv),
                                          inversor_get_state(),
                                          1,
                                          1);
            break;
        }

        delay_ms(1000);
    }

    return 0;
}