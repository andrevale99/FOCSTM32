#include "rcc.h"

int8_t rcc_clk_enable(rcc_clk_source clk, bool enable)
{
    uint32_t on_bit;
    uint32_t rdy_bit;
    uint32_t sws;
    uint32_t timeout = 1000000;

    switch (clk)
    {
        case RCC_CLK_HSI:
            on_bit = RCC_CR_HSION;
            rdy_bit = RCC_CR_HSIRDY;
            sws = RCC_CFGR_SWS_HSI;
            break;

        case RCC_CLK_HSE:
            on_bit = RCC_CR_HSEON;
            rdy_bit = RCC_CR_HSERDY;
            sws = RCC_CFGR_SWS_HSE;
            break;

        case RCC_CLK_PLL:
            on_bit = RCC_CR_PLLON;
            rdy_bit = RCC_CR_PLLRDY;
            sws = RCC_CFGR_SWS_PLL;
            break;

        default:
            return -1;
    }

    if (enable)
    {
        /* Liga a fonte de clock */
        RCC->CR |= on_bit;

        /* Aguarda estabilização */
        while (!(RCC->CR & rdy_bit))
        {
            if (--timeout == 0)
                return -2;
        }
    }
    else
    {
        /* Não permite desligar a fonte atualmente utilizada */
        if ((RCC->CFGR & RCC_CFGR_SWS) == sws)
            return -3;

        /* Desliga a fonte */
        RCC->CR &= ~on_bit;

        /* Aguarda desligamento */
        while (RCC->CR & rdy_bit)
        {
            if (--timeout == 0)
                return -4;
        }
    }

    return 0;
}

int8_t rcc_switch_clk_system(rcc_clk_source clk)
{
    uint32_t sw;
    uint32_t sws;
    uint32_t timeout = 1000000;

    switch (clk)
    {
    case RCC_CLK_HSI:
        sw = RCC_CFGR_SW_HSI;
        sws = RCC_CFGR_SWS_HSI;
        break;

    case RCC_CLK_HSE:
        if (!(RCC->CR & RCC_CR_HSERDY))
            return -3;
        sw = RCC_CFGR_SW_HSE;
        sws = RCC_CFGR_SWS_HSE;
        break;

    case RCC_CLK_PLL:
        if (!(RCC->CR & RCC_CR_PLLRDY))
            return -4;
        sw = RCC_CFGR_SW_PLL;
        sws = RCC_CFGR_SWS_PLL;
        break;

    default:
        return -1;
    }

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= sw;

    while ((RCC->CFGR & RCC_CFGR_SWS) != sws)
    {
        if (--timeout == 0)
            return -2;
    }

    return 0;
}

int8_t rcc_AHB_set_prescale(rcc_ahb_prescale div)
{
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    RCC->CFGR |= (div << RCC_CFGR_HPRE_Pos);

    return 0;
}

int8_t rcc_APB_set_prescale(rcc_apb_bus bus, rcc_apb_prescale div)
{
    switch (bus)
    {
    case APB1:

        RCC->CFGR &= ~RCC_CFGR_PPRE1;
        RCC->CFGR |= ((uint32_t)div << RCC_CFGR_PPRE1_Pos);

        break;

    case APB2:
        RCC->CFGR &= ~RCC_CFGR_PPRE2;
        RCC->CFGR |= ((uint32_t)div << RCC_CFGR_PPRE2_Pos);

        break;

    default:
        return -1;
    }

    return 0;
}