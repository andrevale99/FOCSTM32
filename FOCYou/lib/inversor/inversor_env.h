#include <stm32f411xe.h>

#include "inversor.h"

// ===================================================
//  STATICS
// ===================================================

/**
 * @brief Configura um pino GPIO para a função alternativa AF1.
 *
 * Configura o pino selecionado para:
 * - Modo função alternativa (Alternate Function);
 * - Alta velocidade de comutação;
 * - Pull-down interno;
 * - Função alternativa AF1.
 *
 * @param[in,out] gpio Ponteiro para a porta GPIO.
 * @param[in] pin Número do pino a ser configurado.
 *
 * @note A função AF1 é utilizada pelos canais do TIM1 no STM32F411.
 */
static inline void gpio_af1(GPIO_TypeDef *gpio, uint32_t pin)
{
    gpio->MODER &= ~(0x3UL << (pin * 2));
    gpio->MODER |= (0x2UL << (pin * 2));

    gpio->OSPEEDR &= ~(0x3UL << (pin * 2));
    gpio->OSPEEDR |= (0x3UL << (pin * 2));

    gpio->PUPDR &= ~(0x3UL << (pin * 2));
    gpio->PUPDR |= (0x2UL << (pin * 2));

    gpio->AFR[pin >> 3] &= ~(0xFUL << ((pin & 0x7) * 4));
    gpio->AFR[pin >> 3] |= (0x1UL << ((pin & 0x7) * 4));
}

/**
 * @brief Inicializa os GPIOs e interrupções utilizados pelo inversor.
 *
 * Habilita os clocks dos periféricos TIM1, SYSCFG, GPIOA e GPIOB e
 * configura:
 *
 * - PA8, PA9 e PA10 como saídas TIM1_CH1, TIM1_CH2 e TIM1_CH3;
 * - PB13, PB14 e PB15 como saídas TIM1_CH1N, TIM1_CH2N e TIM1_CH3N;
 * - PB0 como entrada digital com resistor de pull-up interno;
 * - Interrupção EXTI0 acionada na borda de descida.
 *
 * Além disso, configura a prioridade e habilita a interrupção EXTI0
 * no controlador NVIC.
 *
 * @note Esta função é destinada ao uso interno do driver do inversor.
 *
 * Mapeamento dos sinais:
 *
 * | Sinal      | Pino |
 * |------------|------|
 * | TIM1_CH1   | PA8  |
 * | TIM1_CH2   | PA9  |
 * | TIM1_CH3   | PA10 |
 * | TIM1_CH1N  | PB13 |
 * | TIM1_CH2N  | PB14 |
 * | TIM1_CH3N  | PB15 |
 * | ON/OFF     | PB0  |
 *
 * O pino ON/OFF é configurado para gerar uma interrupção EXTI na
 * borda de descida.
 */
static void init_gpios_inversor(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN |
                    RCC_APB2ENR_SYSCFGEN;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN;

    /* CH1, CH2, CH3 */
    gpio_af1(GPIOA, INVERSOR_UH_GPIO); // PA8
    gpio_af1(GPIOA, INVERSOR_VH_GPIO); // PA9
    gpio_af1(GPIOA, INVERSOR_WH_GPIO); // PA10

    /* CH1N, CH2N, CH3N */
    gpio_af1(GPIOB, INVERSOR_UL_GPIO); // PB13
    gpio_af1(GPIOB, INVERSOR_VL_GPIO); // PB14
    gpio_af1(GPIOB, INVERSOR_WL_GPIO); // PB15

    // PB0, input, interrupt com falling edge
    GPIOB->MODER &= ~(0x3 << (INVERSOR_ON_OFF_GPIO * 2));
    GPIOB->OTYPER &= ~(0x1 << INVERSOR_ON_OFF_GPIO);

    GPIOB->OSPEEDR &= ~(0x3 << (INVERSOR_ON_OFF_GPIO * 2));
    GPIOB->OSPEEDR |= (0x1 << (INVERSOR_ON_OFF_GPIO * 2));

    GPIOB->PUPDR &= ~(0x3 << (INVERSOR_ON_OFF_GPIO * 2));
    GPIOB->PUPDR |= (0x1 << (INVERSOR_ON_OFF_GPIO * 2));

    SYSCFG->EXTICR[0] &= ~(0xF << INVERSOR_ON_OFF_GPIO);
    SYSCFG->EXTICR[0] |= (0x1 << INVERSOR_ON_OFF_GPIO);

    EXTI->IMR |= (0x1 << INVERSOR_ON_OFF_GPIO);
    EXTI->RTSR &= ~(0x1 << INVERSOR_ON_OFF_GPIO);
    EXTI->FTSR |= (0x1 << INVERSOR_ON_OFF_GPIO);

    NVIC_SetPriority(EXTI0_IRQn, 0);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
 * @brief Inicializa o TIM10 para o debounce da entrada ON/OFF.
 *
 * Configura o temporizador TIM10 para gerar interrupções periódicas
 * utilizadas na eliminação do efeito de bouncing do botão conectado
 * ao pino ON/OFF do inversor.
 *
 * A função habilita o clock do TIM10, configura seus registradores,
 * habilita a interrupção de atualização e registra a interrupção
 * correspondente no NVIC.
 *
 * @note Esta função é destinada ao uso interno do driver.
 */
static void init_timer10_inversor(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

    TIM10->PSC = 1023;
    TIM10->ARR = 1626;

    TIM10->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

    TIM10->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    TIM10->CR1 |= TIM_CR1_CEN;

    TIM10->EGR |= TIM_EGR_UG;
}

/**
 * @brief Configura o ADC1 para conversões injetadas sincronizadas pelo TIM1.
 *
 * Inicializa o ADC1 para realizar duas conversões no grupo *Injected*,
 * acionadas automaticamente pelo sinal TRGO do TIM1. Durante a
 * configuração são habilitados os clocks necessários, configurados os
 * GPIOs em modo analógico, definidos o tempo de amostragem, a sequência
 * de conversão, o disparo externo e a interrupção de fim de conversão.
 *
 * Configurações realizadas:
 * - Habilita os clocks do GPIOA e do ADC1;
 * - Configura PA6 e PA7 como entradas analógicas;
 * - Define o prescaler do ADC;
 * - Configura o tempo de amostragem dos canais;
 * - Configura o TIM1_TRGO como fonte de disparo das conversões injetadas;
 * - Define uma sequência de duas conversões no grupo *Injected*;
 * - Habilita o modo de varredura (Scan Mode);
 * - Habilita a interrupção de fim de conversão injetada (JEOC);
 * - Habilita a interrupção do ADC no NVIC;
 * - Liga o ADC1.
 *
 * @note As conversões são iniciadas automaticamente pelo evento TRGO
 *       gerado pelo TIM1, não sendo necessário iniciar as conversões
 *       por software.
 *
 * @note Esta função é destinada ao uso interno do driver de aquisição
 *       de corrente do motor.
 */
static inversor_error_code adc_injected_setup(inversor_adc_t *conf)
{

    if (!conf->GPIOx)
        return INVERSOR_ERROR_NO_GPIOx;

    if (!conf->ADCx)
        return INVERSOR_ERROR_NO_ADCx;

    /*--------------------------------------------------
     * Clocks
     *-------------------------------------------------*/
    if (conf->GPIOx == GPIOA)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if (conf->GPIOx == GPIOB)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if (conf->GPIOx == GPIOC)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    else if (conf->GPIOx == GPIOD)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    else if (conf->GPIOx == GPIOE)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    else if (conf->GPIOx == GPIOH)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Limpa registrador para configuracao
    conf->ADCx->JSQR = 0;

    // 2 conversoes
    conf->ADCx->JSQR |= (1 << ADC_JSQR_JL_Pos);

    /*--------------------------------------------------
     * modo analógico, Sample Time 3 ciclos
     *-------------------------------------------------*/
    if (conf->channel_1 != -1)
    {
        conf->GPIOx->MODER &= ~(3 << (2 * conf->channel_1));
        conf->GPIOx->PUPDR &= ~(3 << (2 * conf->channel_1));
        conf->GPIOx->MODER |= (3 << (2 * conf->channel_1));

        if (conf->channel_1 < 10)
        {
            conf->ADCx->SMPR2 &= ~(0x3 << (3 * conf->channel_1));
            conf->ADCx->SMPR2 |= (0 << (3 * conf->channel_1));
        }
        else
        {
            conf->ADCx->SMPR1 &= ~(0x3 << (3 * (conf->channel_1 - 10)));
            conf->ADCx->SMPR1 |= (0 << (3 * (conf->channel_1 - 10)));
        }

        conf->ADCx->JSQR &= ~ADC_JSQR_JSQ4_Msk;
        conf->ADCx->JSQR |= (conf->channel_1 << ADC_JSQR_JSQ4_Pos);
    }

    if (conf->channel_2 != -1)
    {
        conf->GPIOx->MODER &= ~(3 << (2 * conf->channel_2));
        conf->GPIOx->PUPDR &= ~(3 << (2 * conf->channel_2));
        conf->GPIOx->MODER |= (3 << (2 * conf->channel_2));

        if (conf->channel_2 < 10)
        {
            conf->ADCx->SMPR2 &= ~(0x3 << (3 * conf->channel_2));
            conf->ADCx->SMPR2 |= (0 << (3 * conf->channel_2));
        }
        else
        {
            conf->ADCx->SMPR1 &= ~(0x3 << (3 * (conf->channel_2 - 10)));
            conf->ADCx->SMPR1 |= (0 << (3 * (conf->channel_2 - 10)));
        }

        conf->ADCx->JSQR &= ~ADC_JSQR_JSQ3_Msk;
        conf->ADCx->JSQR |= (conf->channel_2 << ADC_JSQR_JSQ3_Pos);
    }

    if (conf->channel_3 != -1)
    {
        conf->GPIOx->MODER &= ~(3 << (2 * conf->channel_3));
        conf->GPIOx->PUPDR &= ~(3 << (2 * conf->channel_3));
        conf->GPIOx->MODER |= (3 << (2 * conf->channel_3));

        if (conf->channel_1 < 10)
        {
            conf->ADCx->SMPR2 &= ~(0x3 << (3 * conf->channel_3));
            conf->ADCx->SMPR2 |= (0 << (3 * conf->channel_3));
        }
        else
        {
            conf->ADCx->SMPR1 &= ~(0x3 << (3 * (conf->channel_3 - 10)));
            conf->ADCx->SMPR1 |= (0 << (3 * (conf->channel_3 - 10)));
        }

        conf->ADCx->JSQR &= ~ADC_JSQR_JSQ2_Msk;
        conf->ADCx->JSQR |= (conf->channel_3 << ADC_JSQR_JSQ2_Pos);
    }

    /*--------------------------------------------------
     * ADC desligado durante configuração
     *-------------------------------------------------*/
    conf->ADCx->CR2 &= ~ADC_CR2_ADON;

    /*--------------------------------------------------
     * Prescaler ADC
     * APB2 = 25 MHz
     * ADC Clock = 25 MHz
     *-------------------------------------------------*/
    ADC->CCR &= ~ADC_CCR_ADCPRE;

    /*--------------------------------------------------
     * Trigger externo injected
     *
     * JEXTSEL = 0001 = TIM1_TRGO
     * JEXTEN  = 01   = Rising Edge
     *-------------------------------------------------*/
    ADC1->CR2 &= ~(
        ADC_CR2_JEXTSEL |
        ADC_CR2_JEXTEN);

    ADC1->CR2 |=
        (0x1 << ADC_CR2_JEXTSEL_Pos) |
        (0x1 << ADC_CR2_JEXTEN_Pos);

    ADC1->CR1 |= ADC_CR1_JEOCIE | ADC_CR1_SCAN;

    NVIC_EnableIRQ(ADC_IRQn);

    /*--------------------------------------------------
     * Habilita ADC
     *-------------------------------------------------*/
    ADC1->CR2 |= ADC_CR2_ADON;

    return INVERSOR_OK;
}