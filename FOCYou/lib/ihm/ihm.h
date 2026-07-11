#ifndef IHM_H
#define IHM_H

#include <stm32f411xe.h>

#include <stdio.h>


#include "lcd16x2.h"
#include "inversor.h"

typedef struct ihm_button
{
GPIO_TypeDef *const GPIOx;
int8_t gpio_pin;
}ihm_button_t;

typedef struct ihm
{
    ihm_button_t on_off_bt;
    ihm_button_t kp_bt;
    ihm_button_t ki_bt;
    ihm_button_t setpoint_bt;

    lcd16x2_handle *const lcd;

    inversor_t *const inv;
}ihm_t;


int8_t ihm_init(ihm_t *ihm);

void ihm_menu_write(ihm_t *ihm);

#endif