#include <stdio.h>

#include <stm32f411xe.h>

#include <arm_math.h>

#include "init_rcc.h"
#include "init_systick.h"

#include "lcd16x2.h"
#include "init_lcd16x2.h"

#include "inversor.h"

const lcd16x2_handle lcd = {
    .d4.write = write_d4,
    .d5.write = write_d5,
    .d6.write = write_d6,
    .d7.write = write_d7,

    .en.write = write_en,
    .rs.write = write_rs,

    .delay = delay_ms,
};

inversor_t inv = {
    .Timer = {
        .advTimer = TIM1,
        .prescale = 1,
        .autoreload = 1248,
    },
};

int map_value(uint32_t x,
              uint32_t in_min,
              uint32_t in_max,
              uint32_t out_min,
              uint32_t out_max)
{
    return (x - in_min) * (out_max - out_min) /
               (in_max - in_min) +
           out_min;
}

void adc_injected_setup(void)
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
     * APB2 = 16 MHz
     * ADC Clock = 8 MHz
     *-------------------------------------------------*/
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CCR_ADCPRE_0;

    /*--------------------------------------------------
     * Sample Time
     * 84 ciclos para reduzir ruído
     * Canal 6
     * Canal 7
     *-------------------------------------------------*/
    ADC1->SMPR2 &= ~(
        (7 << (3 * 6)) |
        (7 << (3 * 7)));

    ADC1->SMPR2 |=
        (4 << (3 * 6)) |
        (4 << (3 * 7));

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

    ADC1->JSQR |= (1 << 20);

    /*
     * Para JL=1:
     *
     * JSQ2 = primeira conversão
     * JSQ1 = segunda conversão
     */

    ADC1->JSQR |= (6 << 5); // PA6 -> primeira
    ADC1->JSQR |= (7 << 0); // PA7 -> segunda

    ADC1->CR1 |= ADC_CR1_JEOCIE;

    NVIC_EnableIRQ(ADC_IRQn);

    /*--------------------------------------------------
     * Habilita ADC
     *-------------------------------------------------*/
    ADC1->CR2 |= ADC_CR2_ADON;
}

volatile int32_t ia = 0;
volatile int32_t ib = 0;

void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_JEOC)
    {
        ADC1->SR &= ~ADC_SR_JEOC;
        ia = ADC1->JDR1;
        ib = ADC1->JDR2;
    }
}

int main(void)
{

    init_rcc();
    init_systick();

    lcd16x2_init_4bits(&lcd, init_periferico_lcd16x2);
    lcd16x2_send_cmd(&lcd, DISPLAY_ON | CURSOR_ON);

    inversor_init(&inv);
    inversor_set_duty(&inv, 1170, 40, 1);

    char buffer[16];
    int size = 0;

    size = sprintf(buffer, "%ldkHz", inversor_get_frequency(&inv));
    lcd16x2_write_string(&lcd, buffer, size);
    lcd16x2_send_cmd(&lcd, SECOND_LINE);

    while (1)
    {
        size = sprintf(buffer, "%ld ", ia);
        lcd16x2_write_string(&lcd, buffer, size);
        size = sprintf(buffer, "%ld ", ib);
        lcd16x2_write_string(&lcd, buffer, size);
        lcd16x2_send_cmd(&lcd, SECOND_LINE);
        delay_ms(500);
    }

    return 0;
}