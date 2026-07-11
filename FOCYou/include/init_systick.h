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

/**
 * @brief Inicializacao do systick, timer
 * reponsavel por realizar o delay
 * 
 * @note NESTE projeto, sempre inicializa
 * o systick DEPOIS de configurar o clock
 * do sistem (HSI, HSE ou PLL).
 */
void init_systick(void)
{
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);
}

/**
 * @brief Função para realizar o delay
 * 
 * @param ms valor em milissegundos
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}

#endif