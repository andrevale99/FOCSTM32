#ifndef INVERSOR_H
#define INVERSOR_H

#include <stm32f411xe.h>

#define INVERSOR_DEADTIME_VALUE 195
#define INVERSOR_MIN_DUTY 80
#define INVERSOR_MAX_DUTY 1170

#define INVERSOR_UH_GPIO 8
#define INVERSOR_VH_GPIO 9
#define INVERSOR_WH_GPIO 10

#define INVERSOR_UL_GPIO 13
#define INVERSOR_VL_GPIO 14
#define INVERSOR_WL_GPIO 15

#define INVERSOR_ON_OFF_GPIO 0

typedef enum
{
    phase_A = 0,
    phase_B = 1,
    phase_C = 2,
} phase;

typedef struct timer_inversor
{
    TIM_TypeDef *const advTimer;
    uint32_t prescale;
    uint32_t autoreload;
} timer_inversor_t;

typedef struct
{
    timer_inversor_t Timer;
} inversor_t;

/**
 * @brief Inicializa o inversor trifásico utilizando um temporizador avançado.
 *
 * Configura o temporizador para operação em PWM alinhado ao centro
 * (center-aligned up-down), habilita os canais principais e complementares,
 * ajusta o tempo morto (dead time), inicializa os registradores de comparação
 * com duty cycle nulo e habilita as saídas do temporizador.
 *
 * @param[in,out] inv Ponteiro para a estrutura de configuração do inversor.
 *                    Deve conter os parâmetros do temporizador previamente
 *                    inicializados.
 *
 * @return int8_t
 * @retval 0 Inicialização realizada com sucesso.
 * @retval -1 Ponteiro @p inv inválido.
 *
 * @note Os canais CH1, CH2 e CH3 são configurados no modo PWM 1 com
 *       preload habilitado.
 *
 * @note As saídas complementares (CH1N, CH2N e CH3N) também são habilitadas,
 *       permitindo o acionamento de um inversor trifásico com ponte completa.
 *
 * @note O tempo morto é definido pelo valor de @ref INVERSOR_DEADTIME_CNT.
 *
 * @warning Esta função não inicia a contagem do temporizador. Após a
 *          inicialização é necessário habilitar o contador para iniciar
 *          a geração dos sinais PWM.
 */
int8_t inversor_init(inversor_t *inv);

/**
 * @brief Atualiza o ciclo de trabalho dos três canais PWM do inversor.
 *
 * Configura os registradores de comparação (CCR1, CCR2 e CCR3) do timer
 * associado ao inversor, definindo o ciclo de trabalho das fases A, B e C.
 *
 * Caso algum valor de duty cycle seja maior ou igual ao valor de auto-reload
 * (ARR) configurado, ele será limitado automaticamente para
 * (ARR - 1), evitando que o valor ultrapasse a faixa válida do contador.
 *
 * @param[in,out] inv_t Ponteiro para a estrutura do inversor.
 * @param[in] duty_a Valor de comparação para a fase A (CCR1).
 * @param[in] duty_b Valor de comparação para a fase B (CCR2).
 * @param[in] duty_c Valor de comparação para a fase C (CCR3).
 *
 * @retval 0  Operação realizada com sucesso.
 * @retval -1 Ponteiro para a estrutura do inversor inválido (NULL).
 *
 * @note Quando os registradores de preload dos canais PWM estiverem
 * habilitados (OCxPE = 1), os valores escritos nos registradores CCRx
 * serão transferidos para os registradores ativos apenas no próximo
 * evento de atualização (Update Event).
 *
 * @warning Esta função assume que o membro invTimer da estrutura
 *          @p inv_t foi previamente inicializado e contém ponteiros válidos.
 */
int8_t inversor_set_duty(const inversor_t *inv_t,
                         uint32_t duty_a,
                         uint32_t duty_b,
                         uint32_t duty_c);

/**
 * @brief Obtém o valor atual do duty cycle de uma fase do inversor.
 *
 * Lê o valor do registrador de comparação (CCRx) associado à fase
 * especificada e retorna o duty cycle configurado para o canal PWM.
 *
 * @param[in] inv_t Ponteiro para a estrutura de configuração do inversor.
 * @param[in] phase Fase da qual o duty cycle será obtido
 *                  (phase_A, phase_B ou phase_C).
 *
 * @return Valor do registrador CCR correspondente à fase selecionada.
 * @return 0 Caso @p inv_t seja NULL ou a fase informada seja inválida.
 *
 * @note O valor retornado corresponde ao valor bruto do registrador CCR,
 *       não ao duty cycle em porcentagem. Para obter o duty cycle em
 *       porcentagem, é necessário relacioná-lo ao valor do ARR do timer.
 *
 * @warning O valor 0 pode indicar tanto erro quanto um duty cycle de 0%.
 */
uint32_t inversor_get_duty(inversor_t *, phase);

/**
 * @brief Obtém a frequência de saída do PWM do inversor.
 *
 * Calcula a frequência efetiva do PWM considerando o clock do sistema,
 * o prescaler (PSC) e o valor de auto-reload (ARR) do temporizador
 * configurado em modo center-aligned (contagem ascendente e descendente).
 *
 * A frequência retornada é expressa em kHz.
 *
 * Fórmula utilizada:
 * @f[
 * f_{PWM} = \frac{f_{CLK}}
 *                {2 \cdot (PSC + 1) \cdot (ARR + 1)}
 * @f]
 *
 * O valor retornado corresponde a:
 * @f[
 * f_{PWM(kHz)} = \frac{f_{PWM}}{1000}
 * @f]
 *
 * @param[in] inv Ponteiro para a estrutura de configuração do timer.
 *
 * @return Frequência do PWM em kHz.
 */
uint32_t inversor_get_frequency(const inversor_t *inv);

uint8_t inversor_get_state(void);

#endif