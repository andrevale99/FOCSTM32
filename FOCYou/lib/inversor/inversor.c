#include "inversor.h"

#include "inversor_env.h"

// Para realizar leitura dos valores
// sem alteracao dos dados
static const inversor_t *inv_local;
volatile uint8_t debounceFlag = 0;
volatile uint8_t start_stop = 0;

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
                inversor_on_off(true);
                break;
            case 1:
                inversor_on_off(false);
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
        return INVERSOR_ERROR_INVERSOR_NULL;

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

    inversor_on_off(false);

    /* Start timer */
    inv->Timer.advTimer->CR1 |= TIM_CR1_CEN;

    return adc_injected_setup(&inv->adc);
}

int8_t inversor_set_duty(const inversor_t *inv_t,
                         uint32_t duty_a, uint32_t duty_b, uint32_t duty_c)
{
    if (!inv_t)
        return INVERSOR_ERROR_INVERSOR_NULL;

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

    return INVERSOR_OK;
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

void inversor_on_off(bool on_off)
{
    if (on_off)
    {
        inv_local->Timer.advTimer->EGR |= TIM_EGR_UG;
        inv_local->Timer.advTimer->BDTR |= TIM_BDTR_MOE;
        start_stop = 1;
    }
    else
    {
        inv_local->Timer.advTimer->BDTR &= ~TIM_BDTR_MOE;
        start_stop = 0;
    }
}

uint8_t inversor_get_state(void)
{
    return start_stop;
}