#include "ihm.h"

#define F_PWM_OFFSET_LCD 0x5
#define ON_OFF_OFFSET_LCD 0xC
#define SETPOINT_OFFSET_LCD 0x3
#define RPM_OFFSET_LCD 0xB

static char buffer[16];
static int size = 0;

/**
 * @brief Exibe a tela de apresentação da interface.
 *
 * Escreve uma mensagem de boas-vindas no display LCD contendo o nome
 * da aplicação e sua versão, posicionando o cursor na origem ao final
 * da operação.
 *
 * @param[in] ihm Ponteiro para a estrutura da Interface Homem-Máquina.
 *
 * @note Esta função é destinada ao uso interno do driver.
 */
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

ihm_error_code ihm_init(ihm_t *ihm)
{

    if (!(ihm->button_on_off.GPIOx))
        return IHM_ERROR_NO_BUTTON_ON_OFF;
    if (!(ihm->button_setpoint.GPIOx))
        return IHM_ERROR_NO_BUTTON_SETPOINT;
    if (!(ihm->button_kp.GPIOx))
        return IHM_ERROR_NO_BUTTON_KP;
    if (!(ihm->button_ki.GPIOx))
        return IHM_ERROR_NO_BUTTON_KI;

    ihm->button_on_off.GPIOx->MODER &= ~(0x3 << (ihm->button_on_off.gpio_pin * 2));
    ihm->button_on_off.GPIOx->PUPDR &= ~(0x3 << (ihm->button_on_off.gpio_pin * 2));
    ihm->button_on_off.GPIOx->PUPDR |= (0x1 << (ihm->button_on_off.gpio_pin * 2));

    ihm->button_setpoint.GPIOx->MODER &= ~(0x3 << (ihm->button_setpoint.gpio_pin * 2));
    ihm->button_setpoint.GPIOx->PUPDR &= ~(0x3 << (ihm->button_setpoint.gpio_pin * 2));
    ihm->button_setpoint.GPIOx->PUPDR |= (0x1 << (ihm->button_setpoint.gpio_pin * 2));

    ihm->button_kp.GPIOx->MODER &= ~(0x3 << (ihm->button_kp.gpio_pin * 2));
    ihm->button_kp.GPIOx->PUPDR &= ~(0x3 << (ihm->button_kp.gpio_pin * 2));
    ihm->button_kp.GPIOx->PUPDR |= (0x1 << (ihm->button_kp.gpio_pin * 2));

    ihm->button_ki.GPIOx->MODER &= ~(0x3 << (ihm->button_ki.gpio_pin * 2));
    ihm->button_ki.GPIOx->PUPDR &= ~(0x3 << (ihm->button_ki.gpio_pin * 2));
    ihm->button_ki.GPIOx->PUPDR |= (0x1 << (ihm->button_ki.gpio_pin * 2));

    ihm_welcome_write(ihm);

    return IHM_OK;
}

void ihm_menu_principal_write(ihm_t *ihm)
{
    lcd16x2_send_cmd(ihm->lcd, CLEAR_DISPLAY);

    size = sprintf(buffer, "Fpwm:%ld kHz", inversor_get_frequency(ihm->inv));
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, SET_DDRAM | ON_OFF_OFFSET_LCD);
    size = sprintf(buffer, "%s",
                   (inversor_get_state() ? "ON" : "OFF"));
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE);

    size = sprintf(buffer, "SP:%dRPM ", 123);
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE | RPM_OFFSET_LCD);
    size = sprintf(buffer, "%d ", 124);
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, RETURN_HOME);
}

void ihm_menu_principal_write_data(ihm_t *ihm, int fpwm, bool on_off,
                         int setpoint, int rpm)
{

    lcd16x2_send_cmd(ihm->lcd, SET_DDRAM | F_PWM_OFFSET_LCD);
    size = sprintf(buffer, "  ");
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SET_DDRAM | F_PWM_OFFSET_LCD);
    size = sprintf(buffer, "%d", fpwm);
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, SET_DDRAM | ON_OFF_OFFSET_LCD);
    size = sprintf(buffer, "   ");
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SET_DDRAM | ON_OFF_OFFSET_LCD);
    size = sprintf(buffer, "%s",
                   (on_off ? "ON" : "OFF"));
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE | SETPOINT_OFFSET_LCD);
    size = sprintf(buffer, "   ");
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE | SETPOINT_OFFSET_LCD);
    size = sprintf(buffer, "%d", setpoint);
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE | RPM_OFFSET_LCD);
    size = sprintf(buffer, "   ");
    lcd16x2_write_string(ihm->lcd, buffer, size);
    lcd16x2_send_cmd(ihm->lcd, SECOND_LINE | RPM_OFFSET_LCD);
    size = sprintf(buffer, "%d", rpm);
    lcd16x2_write_string(ihm->lcd, buffer, size);

    lcd16x2_send_cmd(ihm->lcd, RETURN_HOME);
}