#ifndef RCC_H
#define RCC_H

#include <stdbool.h>

#include <stm32f411xe.h>

/**
 * @brief Fontes de clock disponíveis para o sistema.
 */
typedef enum
{
    /** Oscilador interno de alta velocidade (HSI). */
    RCC_CLK_HSI = 0,

    /** Oscilador externo de alta velocidade (HSE). */
    RCC_CLK_HSE,

    /** Saída do Phase-Locked Loop (PLL). */
    RCC_CLK_PLL

} rcc_clk_source;

typedef enum
{
    AHB_DIV_1   = 0b0000,
    AHB_DIV_2   = 0b1000,
    AHB_DIV_4,
    AHB_DIV_8,
    AHB_DIV_16,
    AHB_DIV_64,
    AHB_DIV_128,
    AHB_DIV_256,
    AHB_DIV_512
} rcc_ahb_prescale;

typedef enum
{
    APB_DIV_1   = 0b000,
    APB_DIV_2   = 0b100,
    APB_DIV_4,
    APB_DIV_8,
    APB_DIV_16
} rcc_apb_prescale;

typedef enum
{
    APB1 = 0,
    APB2
} rcc_apb_bus;

/**
 * @brief Habilita ou desabilita uma fonte de clock do sistema.
 *
 * Liga ou desliga uma das fontes de clock disponíveis (HSI, HSE ou PLL),
 * aguardando sua estabilização ou desligamento antes de retornar.
 *
 * Ao desabilitar uma fonte de clock, a função verifica se ela está sendo
 * utilizada como clock do sistema, impedindo seu desligamento.
 *
 * @param[in] clk Fonte de clock a ser configurada.
 * @param[in] enable Define a operação:
 *                  - true: habilita a fonte de clock;
 *                  - false: desabilita a fonte de clock.
 *
 * @retval 0 Operação realizada com sucesso.
 * @retval -1 Fonte de clock inválida.
 * @retval -2 Tempo limite excedido durante a inicialização da fonte.
 * @retval -3 Tentativa de desabilitar a fonte atualmente utilizada pelo sistema.
 * @retval -4 Tempo limite excedido durante o desligamento da fonte.
 */
int8_t rcc_clk_enable(rcc_clk_source clk, bool enable);

/**
 * @brief Seleciona a fonte de clock do sistema.
 *
 * Altera a fonte utilizada como SYSCLK e aguarda a confirmação da
 * comutação pelo registrador de status do RCC.
 *
 * A fonte HSE ou PLL deve estar previamente habilitada e estabilizada.
 *
 * @param[in] clk Nova fonte de clock do sistema.
 *
 * @retval 0 Comutação realizada com sucesso.
 * @retval -1 Fonte de clock inválida.
 * @retval -2 Tempo limite excedido durante a comutação.
 * @retval -3 Oscilador HSE não está pronto.
 * @retval -4 PLL não está pronta.
 */
int8_t rcc_switch_clk_system(rcc_clk_source clk);

/**
 * @brief Configura o prescaler do barramento AHB.
 *
 * Define o fator de divisão aplicado ao clock AHB (HCLK).
 *
 * @param[in] div Fator de divisão do barramento AHB.
 *
 * @retval 0 Configuração realizada com sucesso.
 *
 * @note A alteração do prescaler modifica a frequência de operação do
 *       barramento AHB e dos periféricos conectados a ele.
 */
int8_t rcc_AHB_set_prescale(rcc_ahb_prescale div);

/**
 * @brief Configura o prescaler de um barramento APB.
 *
 * Define o fator de divisão do clock para o barramento APB1 ou APB2.
 *
 * @param[in] bus Barramento APB a ser configurado.
 * @param[in] div Fator de divisão do clock.
 *
 * @retval 0 Configuração realizada com sucesso.
 * @retval -1 Barramento inválido.
 *
 * @note A frequência dos temporizadores pode diferir da frequência do
 *       barramento APB quando o prescaler é diferente de 1, conforme a
 *       arquitetura dos microcontroladores STM32.
 */
int8_t rcc_APB_set_prescale(rcc_apb_bus bus, rcc_apb_prescale div);

#endif