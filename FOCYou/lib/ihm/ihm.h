#ifndef IHM_H
#define IHM_H

#include <stm32f411xe.h>

#include <stdio.h>

#include "lcd16x2.h"
#include "inversor.h"

/**
 * @brief Códigos de retorno do driver da Interface Homem-Máquina.
 */
typedef enum
{
    /** Operação realizada com sucesso. */
    IHM_OK = 0,

    /** Botão ON/OFF não configurado. */
    IHM_ERROR_NO_BUTTON_ON_OFF = -1,

    /** Botão de ajuste do setpoint não configurado. */
    IHM_ERROR_NO_BUTTON_SETPOINT = -2,

    /** Botão de ajuste do ganho proporcional (Kp) não configurado. */
    IHM_ERROR_NO_BUTTON_KP = -3,

    /** Botão de ajuste do ganho integral (Ki) não configurado. */
    IHM_ERROR_NO_BUTTON_KI = -4,

} ihm_error_code;

/**
 * @brief Configuração de um botão da Interface Homem-Máquina (IHM).
 *
 * Armazena a porta GPIO e o número do pino associados a um botão
 * utilizado pela interface.
 */
typedef struct ihm_button
{
    /**
     * @brief Porta GPIO à qual o botão está conectado.
     */
    GPIO_TypeDef *const GPIOx;

    /**
     * @brief Número do pino do botão na porta GPIO.
     */
    int8_t gpio_pin;

} ihm_button_t;

/**
 * @brief Estrutura de configuração da Interface Homem-Máquina (IHM).
 *
 * Reúne os recursos de hardware utilizados pela interface, incluindo
 * os botões de controle, o display LCD e o driver do inversor.
 */
typedef struct ihm
{
    /**
     * @brief Botão para habilitar ou desabilitar o inversor.
     */
    ihm_button_t button_on_off;

    /**
     * @brief Botão de ajuste do ganho proporcional (Kp).
     */
    ihm_button_t button_kp;

    /**
     * @brief Botão de ajuste do ganho integral (Ki).
     */
    ihm_button_t button_ki;

    /**
     * @brief Botão para ajuste do setpoint de velocidade.
     */
    ihm_button_t button_setpoint;

    /**
     * @brief Ponteiro para o driver do display LCD 16x2.
     */
    lcd16x2_handle *const lcd;

    /**
     * @brief Ponteiro para a instância do driver do inversor.
     */
    inversor_t *const inv;

} ihm_t;


/**
 * @brief Inicializa a Interface Homem-Máquina (IHM).
 *
 * Configura os botões da interface como entradas digitais com resistor
 * de pull-up interno e exibe a tela de apresentação no display LCD.
 *
 * @param[in,out] ihm Ponteiro para a estrutura de configuração da IHM.
 *
 * @retval IHM_OK Inicialização realizada com sucesso.
 * @retval IHM_ERROR_NO_BUTTON_ON_OFF Botão ON/OFF não configurado.
 * @retval IHM_ERROR_NO_BUTTON_SETPOINT Botão de ajuste do setpoint não configurado.
 * @retval IHM_ERROR_NO_BUTTON_KP Botão de ajuste do ganho proporcional não configurado.
 * @retval IHM_ERROR_NO_BUTTON_KI Botão de ajuste do ganho integral não configurado.
 */
ihm_error_code ihm_init(ihm_t *ihm);

/**
 * @brief Escreve a tela principal da Interface Homem-Máquina.
 *
 * Limpa o display LCD e apresenta os campos da interface contendo:
 * - Frequência de chaveamento do inversor;
 * - Estado do inversor (ON/OFF);
 * - Setpoint de velocidade;
 * - Velocidade medida do motor.
 *
 * Após a escrita, o cursor retorna à posição inicial do display.
 *
 * @param[in] ihm Ponteiro para a estrutura da IHM.
 *
 * @note Esta função desenha a estrutura fixa da interface. Os valores
 *       podem ser atualizados posteriormente por meio de
 *       @ref ihm_menu_write_data().
 */
void ihm_menu_principal_write(ihm_t *ihm);

/**
 * @brief Atualiza os valores exibidos na tela principal da IHM.
 *
 * Atualiza apenas os campos numéricos e o estado do inversor,
 * preservando o restante da interface gráfica do display LCD.
 *
 * @param[in] ihm Ponteiro para a estrutura da IHM.
 * @param[in] fpwm Frequência de chaveamento do inversor, em kHz.
 * @param[in] on_off Estado do inversor:
 *                  - true: inversor habilitado;
 *                  - false: inversor desabilitado.
 * @param[in] setpoint Velocidade de referência do motor, em RPM.
 * @param[in] rpm Velocidade atual do motor, em RPM.
 *
 * @note A atualização é realizada sobrescrevendo apenas os campos
 *       variáveis do display, reduzindo o tempo de atualização e
 *       evitando cintilação perceptível.
 */
void ihm_menu_principal_write_data(ihm_t *ihm,
                         int fpwm,
                         bool on_off,
                         int setpoint,
                         int rpm);
#endif