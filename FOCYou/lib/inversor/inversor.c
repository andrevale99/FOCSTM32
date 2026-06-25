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

static void inversor_start(void)
{
    inv_local->Timer.advTimer->EGR |= TIM_EGR_UG;
    inv_local->Timer.advTimer->BDTR |= TIM_BDTR_MOE;
}

static void inversor_stop(void)
{
    inv_local->Timer.advTimer->BDTR &= ~TIM_BDTR_MOE;
}

// ===================================================
//  INTERRUPCOES
// ===================================================

void EXTI0_IRQHandler(void)
{
    EXTI->PR = EXTI_PR_PR0;

    debounceFlag = 1;

    TIM10->CNT = 0;
    TIM10->SR &= ~TIM_SR_UIF;
    TIM10->CR1 |= TIM_CR1_CEN;
}

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