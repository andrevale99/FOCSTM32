#include "inversor.h"

// Para realizar leitura dos valores
// sem alteracao dos dados
static const inversor_t *inv_local;

volatile uint8_t debounceFlag = 0;
volatile uint8_t start_stop = 0;

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
 * @brief Habilita as saídas PWM do inversor.
 *
 * Atualiza os registradores do temporizador e habilita a saída
 * principal (MOE), permitindo que os sinais PWM sejam aplicados
 * às fases do inversor.
 *
 * @note Função utilizada internamente pelo driver.
 */
static void inversor_start(void)
{
    inv_local->Timer.advTimer->EGR |= TIM_EGR_UG;
    inv_local->Timer.advTimer->BDTR |= TIM_BDTR_MOE;
}

/**
 * @brief Desabilita as saídas PWM do inversor.
 *
 * Limpa o bit MOE (Main Output Enable) do temporizador avançado,
 * interrompendo imediatamente os sinais PWM nas saídas do inversor.
 *
 * @note O temporizador continua em funcionamento; apenas as saídas
 * PWM são desabilitadas.
 */
static void inversor_stop(void)
{
    inv_local->Timer.advTimer->BDTR &= ~TIM_BDTR_MOE;
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
static void adc_injected_setup(void)
{
    /*--------------------------------------------------
     * Clocks
     *-------------------------------------------------*/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /*--------------------------------------------------
     * PA6 e PA7 em modo analógico
     *-------------------------------------------------*/
    GPIOA->MODER |= (3 << GPIO_MODER_MODER6_Pos);
    GPIOA->MODER |= (3 << GPIO_MODER_MODER7_Pos);

    GPIOA->PUPDR &= ~(3 << GPIO_PUPDR_PUPD6_Pos);
    GPIOA->PUPDR &= ~(3 << GPIO_PUPDR_PUPD7_Pos);

    /*--------------------------------------------------
     * ADC desligado durante configuração
     *-------------------------------------------------*/
    ADC1->CR2 &= ~ADC_CR2_ADON;

    /*--------------------------------------------------
     * Prescaler ADC
     * APB2 = 25 MHz
     * ADC Clock = 25 MHz
     *-------------------------------------------------*/
    ADC->CCR &= ~ADC_CCR_ADCPRE;

    /*--------------------------------------------------
     * Sample Time
     * 3 ciclos para reduzir ruído
     * Canal 6
     * Canal 7
     *-------------------------------------------------*/
    ADC1->SMPR2 &= ~(
        (7 << (3 * 6)) |
        (7 << (3 * 7)));

    ADC1->SMPR2 |=
        (0 << (3 * 6)) |
        (0 << (3 * 7));

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

    /*--------------------------------------------------
     * Sequência Injected
     *
     * JL = 1 => 2 conversões
     *-------------------------------------------------*/
    ADC1->JSQR = 0;

    ADC1->JSQR |= (1 << ADC_JSQR_JL_Pos);

    /*
     * Para JL=1:
     *
     * JSQ2 = primeira conversão
     * JSQ1 = segunda conversão
     */

    ADC1->JSQR |= (6 << ADC_JSQR_JSQ3_Pos) | (7 << ADC_JSQR_JSQ4_Pos);

    ADC1->CR1 |= ADC_CR1_JEOCIE | ADC_CR1_SCAN;

    NVIC_EnableIRQ(ADC_IRQn);

    /*--------------------------------------------------
     * Habilita ADC
     *-------------------------------------------------*/
    ADC1->CR2 |= ADC_CR2_ADON;
}

// ===================================================
//  INTERRUPCOES
// ===================================================

/**
 * @brief Rotina de atendimento da interrupção EXTI0.
 *
 * Executada quando ocorre uma borda de descida na entrada ON/OFF.
 * A interrupção inicia o processo de debounce habilitando o TIM10,
 * responsável por validar o acionamento do botão.
 *
 * @note Handler da interrupção EXTI0.
 */
void EXTI0_IRQHandler(void)
{
    EXTI->PR = EXTI_PR_PR0;

    debounceFlag = 1;

    TIM10->CNT = 0;
    TIM10->SR &= ~TIM_SR_UIF;
    TIM10->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Rotina de atendimento da interrupção de atualização do TIM10.
 *
 * Realiza o tratamento de debounce da entrada ON/OFF. Caso o botão
 * permaneça pressionado após o intervalo configurado, alterna o
 * estado do inversor entre habilitado e desabilitado.
 *
 * Após o processamento, o temporizador é interrompido até um novo
 * acionamento da interrupção EXTI0.
 *
 * @note Handler da interrupção TIM1_UP_TIM10.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    TIM10->SR &= ~TIM_SR_UIF;

    if (debounceFlag)
    {
        debounceFlag = 0;

        if (!(GPIOB->IDR & GPIO_IDR_ID0))
        {
            switch (start_stop)
            {
            case 0:
                inversor_start();
                start_stop = 1;
                break;
            case 1:
                inversor_stop();
                start_stop = 0;
                break;

            default:
                break;
            }
        }

        TIM10->CR1 &= ~TIM_CR1_CEN;
    }
}

/**
 * @brief Rotina de atendimento da interrupção do ADC1.
 *
 * Executada ao término da sequência de conversões do grupo *Injected*.
 * A rotina verifica a ocorrência da interrupção de fim de conversão
 * (JEOC), limpa a respectiva flag e disponibiliza os valores convertidos
 * armazenados nos registradores JDRx para processamento pelo algoritmo
 * de controle.
 *
 * @note As conversões são disparadas pelo evento TRGO do TIM1.
 *
 * @note Os resultados das conversões podem ser obtidos por meio dos
 *       registradores ADC1->JDR1, ADC1->JDR2, ADC1->JDR3 e ADC1->JDR4,
 *       conforme a sequência configurada.
 */
void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_JEOC)
    {
        ADC1->SR &= ~ADC_SR_JEOC;

        // ia = ADC1->JDR1;
        // ib = ADC1->JDR2;
    }
}

// ===================================================
//  INVERSOR
// ===================================================

int8_t inversor_init(inversor_t *inv)
{
    if (!inv)
        return -1;

    inv_local = inv;

    init_gpios_inversor();
    init_timer10_inversor();

    inv->Timer.advTimer->PSC = inv->Timer.prescale;
    inv->Timer.advTimer->ARR = inv->Timer.autoreload;

    // center-aligned pwm up-down
    inv->Timer.advTimer->CR1 &= ~TIM_CR1_CMS_Msk;
    inv->Timer.advTimer->CR1 |= (3 << TIM_CR1_CMS_Pos);

    inv->Timer.advTimer->CR1 |= TIM_CR1_ARPE;

    inv->Timer.advTimer->CR2 &= ~TIM_CR2_MMS_Msk;
    inv->Timer.advTimer->CR2 |= TIM_CR2_MMS_1;

    /* CH1 */
    inv->Timer.advTimer->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
    inv->Timer.advTimer->CCMR1 |= TIM_CCMR1_OC1PE;

    /* CH2 */
    inv->Timer.advTimer->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos);
    inv->Timer.advTimer->CCMR1 |= TIM_CCMR1_OC2PE;

    /* CH3 */
    inv->Timer.advTimer->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos);
    inv->Timer.advTimer->CCMR2 |= TIM_CCMR2_OC3PE;

    inv->Timer.advTimer->CCER |=
        TIM_CCER_CC1E | TIM_CCER_CC1NE |
        TIM_CCER_CC2E | TIM_CCER_CC2NE |
        TIM_CCER_CC3E | TIM_CCER_CC3NE;

    inv->Timer.advTimer->BDTR &= ~TIM_BDTR_DTG_Msk;
    inv->Timer.advTimer->BDTR |= (INVERSOR_DEADTIME_VALUE << TIM_BDTR_DTG_Pos);

    inv->Timer.advTimer->CCR1 = 0;
    inv->Timer.advTimer->CCR2 = 0;
    inv->Timer.advTimer->CCR3 = 0;

    /* Main output enable */
    inv->Timer.advTimer->BDTR |= TIM_BDTR_MOE | TIM_BDTR_OSSI;

    /* Update registers */
    inv->Timer.advTimer->EGR |= TIM_EGR_UG;

    inversor_stop();

    /* Start timer */
    inv->Timer.advTimer->CR1 |= TIM_CR1_CEN;

    // Inicializacao do ad
    adc_injected_setup();

    return 0;
}

int8_t inversor_set_duty(const inversor_t *inv_t,
                         uint32_t duty_a, uint32_t duty_b, uint32_t duty_c)
{
    if (!inv_t)
        return -1;

    if (duty_a >= INVERSOR_MAX_DUTY)
        duty_a = INVERSOR_MAX_DUTY;

    else if (duty_a < INVERSOR_MIN_DUTY)
        duty_a = INVERSOR_MIN_DUTY;

    if (duty_b >= INVERSOR_MAX_DUTY)
        duty_b = INVERSOR_MAX_DUTY;

    else if (duty_b < INVERSOR_MIN_DUTY)
        duty_b = INVERSOR_MIN_DUTY;

    if (duty_c >= INVERSOR_MAX_DUTY)
        duty_c = INVERSOR_MAX_DUTY;

    else if (duty_c < INVERSOR_MIN_DUTY)
        duty_c = INVERSOR_MIN_DUTY;

    inv_t->Timer.advTimer->CCR1 = duty_a;
    inv_t->Timer.advTimer->CCR2 = duty_b;
    inv_t->Timer.advTimer->CCR3 = duty_c;

    return 0;
}

uint32_t inversor_get_duty(inversor_t *inv_t, phase phase)
{
    if (!inv_t)
        return -1;

    uint32_t ret = 0;

    switch (phase)
    {
    case phase_A:
        ret = inv_t->Timer.advTimer->CCR1;
        break;

    case phase_B:
        ret = inv_t->Timer.advTimer->CCR2;
        break;

    case phase_C:
        ret = inv_t->Timer.advTimer->CCR3;
        break;

    default:
        ret = 0;
        break;
    }

    return ret;
}

uint32_t inversor_get_frequency(const inversor_t *inv)
{
    return (SystemCoreClock /
            (2 * (inv->Timer.advTimer->PSC + 1) *
             (inv->Timer.advTimer->ARR + 1)) /
            1000);
}

uint8_t inversor_get_state(void)
{
    return start_stop;
}