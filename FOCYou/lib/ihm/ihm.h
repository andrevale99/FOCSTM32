#ifndef IHM_H
#define IHM_H

#include <stm32f411xe.h>

#include <stdio.h>


#include "lcd16x2.h"
#include "inversor.h"

typedef struct ihm
{
    GPIO_TypeDef *const GPIOx;
    int8_t on_off_button_gpio;
    int8_t kp_button;
    int8_t ki_button;
    int8_t setpoint_button;

    lcd16x2_handle *const lcd;

    inversor_t *const inv;
}ihm_t;


int8_t ihm_init(ihm_t *ihm);

void ihm_menu_write(ihm_t *ihm);

#endif