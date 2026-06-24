#include "inversor.h"

static volatile int8_t on_off_flag = 0;
static inversor_t *inv_local;

// ===================================================
//  STATICS
// ===================================================

static inline void inversor_APBx_AHBx_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    RCC->AHB1ENR |=
        RCC_AHB1ENR_GPIOAEN |
        RCC_AHB1ENR_GPIOBEN;
}

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

static void inversor_gpios_init(void)
{
    /* CH1, CH2, CH3 */
    gpio_af1(GPIOA, INVERSOR_UH_GPIO); // PA8
    gpio_af1(GPIOA, INVERSOR_VH_GPIO); // PA9
    gpio_af1(GPIOA, INVERSOR_WH_GPIO); // PA10

    /* CH1N, CH2N, CH3N */
    gpio_af1(GPIOB, INVERSOR_UL_GPIO); // PB13
    gpio_af1(GPIOB, INVERSOR_VL_GPIO); // PB14
    gpio_af1(GPIOB, INVERSOR_WL_GPIO); // PB15

    GPIOB->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
    GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR0;

    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_0;

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB;

    EXTI->IMR |= EXTI_IMR_IM0;
    EXTI->RTSR &= ~EXTI_RTSR_TR0_Msk;
    EXTI->FTSR |= EXTI_FTSR_TR0_Msk;

    NVIC_SetPriority(EXTI0_IRQn, 0);
    NVIC_EnableIRQ(EXTI0_IRQn);
}

// ===================================================
//  INTERRUPT
// ===================================================
void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)
    {
        if (!(GPIOB->IDR & GPIO_IDR_ID0))
        {
            if (on_off_flag)
            {
                inversor_stop(inv_local);
                on_off_flag = 0;
            }

            else
            {
                inversor_start(inv_local);
                on_off_flag = 1;
            }
        }

        EXTI->PR = EXTI_PR_PR0;
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

    inversor_APBx_AHBx_init();

    inversor_gpios_init();

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
    inv->Timer.advTimer->BDTR |= (INVERSOR_DEADTIME_CNT << TIM_BDTR_DTG_Pos);

    inv->Timer.advTimer->CCR1 = 0;
    inv->Timer.advTimer->CCR2 = 0;
    inv->Timer.advTimer->CCR3 = 0;

    /* Main output enable */
    inv->Timer.advTimer->BDTR |= TIM_BDTR_MOE;

    /* Update registers */
    inv->Timer.advTimer->EGR |= TIM_EGR_UG;

    return 0;
}

void inversor_start(inversor_t *inv)
{
    /* Start timer */
    inv->Timer.advTimer->CR1 |= TIM_CR1_CEN;
}

void inversor_stop(inversor_t *inv)
{
    /* Stop timer */
    inv->Timer.advTimer->CR1 &= ~TIM_CR1_CEN;
}

int8_t inversor_set_duty(const inversor_t *inv_t,
                         uint32_t duty_a, uint32_t duty_b, uint32_t duty_c)
{
    if (!inv_t)
        return -1;

    if (duty_a >= inv_t->Timer.autoreload)
        duty_a = inv_t->Timer.autoreload - 1;

    else if (duty_a < INVERSOR_MIN_DUTY)
        duty_a = INVERSOR_MIN_DUTY;

    if (duty_b >= inv_t->Timer.autoreload)
        duty_b = inv_t->Timer.autoreload - 1;

    else if (duty_b < INVERSOR_MIN_DUTY)
        duty_b = INVERSOR_MIN_DUTY;

    if (duty_c >= inv_t->Timer.autoreload)
        duty_c = inv_t->Timer.autoreload - 1;

    else if (duty_c < INVERSOR_MIN_DUTY)
        duty_c = INVERSOR_MIN_DUTY;

    inv_t->Timer.advTimer->CCR1 = duty_a;
    inv_t->Timer.advTimer->CCR2 = duty_b;
    inv_t->Timer.advTimer->CCR3 = duty_c;

    inv_t->Timer.advTimer->EGR |= TIM_EGR_UG;

    return 0;
}

uint32_t inversor_get_duty(inversor_t *inv_t, phase phase)
{
    if (!inv_t)
        return 0;

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

int8_t inversor_get_state(void)
{
    return (int8_t)on_off_flag;
}