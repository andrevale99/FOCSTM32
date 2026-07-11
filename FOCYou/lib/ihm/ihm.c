#include "ihm.h"

static char buffer[16];
static int size = 0;

static void ihm_welcome_write(ihm_t *ihm)
{
    size = sprintf(buffer, "FOCyou");
    lcd16x2_send_cmd(ihm->lcd, SET_DDRAM | 0x5);
    lcd16x2_write_string(ihm->lcd, buffer, size);

    size = sprintf(buffer, "v0.0.1");
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE | 0x5);
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, RETURN_HOME);
}

int8_t ihm_init(ihm_t *ihm)
{

    if (!(ihm->on_off_bt.GPIOx))
        return -1;
    if (!(ihm->setpoint_bt.GPIOx))
        return -2;
    if (!(ihm->kp_bt.GPIOx))
        return -3;
    if (!(ihm->ki_bt.GPIOx))
        return -4;

    ihm->on_off_bt.GPIOx->MODER &= ~(0x3 << (ihm->on_off_bt.gpio_pin * 2));
    ihm->on_off_bt.GPIOx->PUPDR &= ~(0x3 << (ihm->on_off_bt.gpio_pin * 2));
    ihm->on_off_bt.GPIOx->PUPDR |= (0x1 << (ihm->on_off_bt.gpio_pin * 2));

    ihm->setpoint_bt.GPIOx->MODER &= ~(0x3 << (ihm->setpoint_bt.gpio_pin * 2));
    ihm->setpoint_bt.GPIOx->PUPDR &= ~(0x3 << (ihm->setpoint_bt.gpio_pin * 2));
    ihm->setpoint_bt.GPIOx->PUPDR |= (0x1 << (ihm->setpoint_bt.gpio_pin * 2));

    ihm->kp_bt.GPIOx->MODER &= ~(0x3 << (ihm->kp_bt.gpio_pin * 2));
    ihm->kp_bt.GPIOx->PUPDR &= ~(0x3 << (ihm->kp_bt.gpio_pin * 2));
    ihm->kp_bt.GPIOx->PUPDR |= (0x1 << (ihm->kp_bt.gpio_pin * 2));

    ihm->ki_bt.GPIOx->MODER &= ~(0x3 << (ihm->ki_bt.gpio_pin * 2));
    ihm->ki_bt.GPIOx->PUPDR &= ~(0x3 << (ihm->ki_bt.gpio_pin * 2));
    ihm->ki_bt.GPIOx->PUPDR |= (0x1 << (ihm->ki_bt.gpio_pin * 2));

    ihm_welcome_write(ihm);
    return 0;
}

void ihm_menu_write(ihm_t *ihm)
{
    // size = sprintf(buffer, "%ldkHz", inversor_get_frequency(ihm->inv));
    // lcd16x2_write_string(ihm->lcd, buffer, size);
    size = sprintf(buffer, "               ");
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE);
    size = sprintf(buffer, "%i  %li %li", inversor_get_state(), ADC1->JDR1, ADC1->JDR2);
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE);
}