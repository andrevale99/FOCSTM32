#ifndef INVERSOR_H
#define INVERSOR_H

#include <stdbool.h>

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

/**
 * @brief Códigos de retorno do driver do inversor.
 */
typedef enum
{
    /** Operação realizada com sucesso. */
    INVERSOR_OK = 0,

    /** Ponteiro para a estrutura do inversor inválido. */
    INVERSOR_ERROR_INVERSOR_NULL = -1,

    /** Porta GPIO do ADC não configurada. */
    INVERSOR_ERROR_NO_GPIOx = -2,

    /** ADC não configurado. */
    INVERSOR_ERROR_NO_ADCx = -3,

} inversor_error_code;

typedef enum
{
    INVERSOR_PHASE_A,
    INVERSOR_PHASE_B,
    INVERSOR_PHASE_C
} inversor_phase_t;
/**
 * @brief Configuração do temporizador do inversor.
 */
typedef struct
{
    TIM_TypeDef *const advTimer; /**< Temporizador avançado utilizado pelo inversor. */
    uint32_t prescale;           /**< Valor do prescaler. */
    uint32_t autoreload;         /**< Valor do registrador ARR. */
} inversor_timer_t;

/**
 * @brief Configuração do ADC utilizado pelo inversor.
 */
typedef struct
{
    GPIO_TypeDef *const GPIOx; /**< Porta GPIO utilizada pelas entradas analógicas. */
    ADC_TypeDef *const ADCx;   /**< ADC utilizado para aquisição das correntes. */

    int8_t channel_1; /**< Canal da fase A. */
    int8_t channel_2; /**< Canal da fase B. */
    int8_t channel_3; /**< Canal da fase C (opcional). */

} inversor_adc_t;

/**
 * @brief Estrutura de configuração do inversor.
 */
typedef struct
{
    /** Configuração do temporizador PWM. */
    inversor_timer_t Timer;

    /** Configuração do ADC para aquisição das correntes. */
    inversor_adc_t adc;

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
 * @retval INVERSOR_OK Inicialização realizada com sucesso.
 * @retval INVERSOR_ERROR_INVERSOR_NULL Ponteiro @p inv inválido.
 * @retval INVERSOR_ERROR_NO_GPIOx Porta GPIO do ADC não configurada.
 * @retval INVERSOR_ERROR_NO_ADCx ADC não configurado.
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
inversor_error_code inversor_init(inversor_t *inv);

/**
 * @brief Atualiza o ciclo de trabalho dos três canais PWM do inversor.
 *
 * Configura os registradores de comparação (CCR1, CCR2 e CCR3) do timer
 * associado ao inversor, definindo o ciclo de trabalho das fases A, B e C.
 *
 * Caso o valor seja menor que INVERSOR_MIN_DUTY ou maior que INVERSOR_MAX_DUTY
 * eles serão limitados para essas mesmas constantes
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
inversor_error_code inversor_set_duty(const inversor_t *inv_t,
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
uint32_t inversor_get_duty(const inversor_t *inv_t,
                           inversor_phase_t phase);

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

/**
 * @brief Habilita ou desabilita as saídas PWM do inversor.
 *
 * Controla o estado operacional do inversor por meio do bit
 * Main Output Enable (MOE) do temporizador avançado. Quando
 * habilitado, um evento de atualização é gerado para garantir
 * que os registradores de preload sejam carregados antes da
 * ativação das saídas PWM.
 *
 * O estado atual do inversor também é armazenado na variável
 * interna utilizada pela função @ref inversor_get_state().
 *
 * @param[in] on_off Define o estado desejado do inversor:
 *                  - true: habilita as saídas PWM;
 *                  - false: desabilita as saídas PWM.
 *
 * @note Esta função não altera a configuração do temporizador nem
 *       os valores de duty cycle, apenas habilita ou desabilita as
 *       saídas do TIM1 por meio do bit MOE.
 *
 * @warning O inversor deve ter sido previamente inicializado por
 *          @ref inversor_init().
 */
void inversor_on_off(bool on_off);

/**
 * @brief Obtém o estado atual do inversor.
 *
 * Retorna o estado lógico utilizado pelo driver para indicar
 * se as saídas PWM estão habilitadas.
 *
 * @retval 0 Inversor desabilitado.
 * @retval 1 Inversor habilitado.
 */
uint8_t inversor_get_state(void);

#endif