#ifndef INIT_SYSTICK_H
#define INIT_SYSTICK_H

#include <stdint.h>
#include "libmcu.h"

static volatile uint32_t msTicks = 0;

/* Interrupção do SysTick */
void SysTick_Handler(void)
{
    msTicks++;
}

/* Inicializa SysTick para 1 ms */
void init_systick(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);
}

/* Habilita o contador de ciclos */
void init_delay_us(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

/* Delay em ms */
void delay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}

#endif