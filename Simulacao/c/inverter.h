#ifndef INVERTER_H
#define INVERTER_H

#include <stdio.h>
#include <stdbool.h>

/**

* @brief Estrutura responsável pela modelagem do inversor trifásico.
  */
typedef struct
{
    /**

    * @brief Tensão do barramento CC.
      */
    float Vdc;

} inverter_t;

/**

* @brief Limita um valor entre um limite inferior e superior.
*
* @param value Valor a ser limitado.
* @param min Valor mínimo.
* @param max Valor máximo.
*
* @return Valor limitado ao intervalo especificado.
  */
static inline float inverter_clamp(
    float value,
    float min,
    float max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}

/**

* @brief Converte o duty cycle na tensão média do polo da fase.
*
* A tensão média do polo é dada por:
*
* \f[
* V_{pole} = duty \cdot V_{dc}
* \f]
*
* Para:
*
* duty = 0 -> Vpole = 0 V
*
* duty = 1 -> Vpole = Vdc
*
* @param inverter Ponteiro para a estrutura do inversor.
* @param duty Duty cycle da fase.
*
* @return Tensão média do polo da fase.
  */
static inline float inverter_duty_to_pole_voltage(
    const inverter_t *inverter,
    float duty)
{
    return duty * inverter->Vdc;
}

/**

* @brief Calcula as tensões de fase aplicadas ao motor.
*
* Os duty cycles são inicialmente limitados ao intervalo:
*
* \f[
* 0 \leq duty \leq 1
* \f]
*
* As tensões dos polos em relação ao terminal negativo do barramento
* são calculadas por:
*
* \f[
* V_{a,pole} = duty_a V_{dc}
* \f]
*
* \f[
* V_{b,pole} = duty_b V_{dc}
* \f]
*
* \f[
* V_{c,pole} = duty_c V_{dc}
* \f]
*
* O ponto neutro virtual é calculado como:
*
* \f[
* V_n =
* \frac{
* V_{a,pole} +
* V_{b,pole} +
* V_{c,pole}
* }{3}
* \f]
*
* As tensões de fase aplicadas ao motor são:
*
* \f[
* V_a = V_{a,pole} - V_n
* \f]
*
* \f[
* V_b = V_{b,pole} - V_n
* \f]
*
* \f[
* V_c = V_{c,pole} - V_n
* \f]
*
* @param inverter Ponteiro para a estrutura do inversor.
* @param duty_a Duty cycle da fase A.
* @param duty_b Duty cycle da fase B.
* @param duty_c Duty cycle da fase C.
* @param Vabc Vetor de saída contendo as tensões das fases A, B e C.
  */
static inline void inverter_output_voltage(
    const inverter_t *inverter,
    float duty_a,
    float duty_b,
    float duty_c,
    float Vabc[3])
{
    /*

    * Limitação dos duty cycles
      */
    duty_a = inverter_clamp(
        duty_a,
        0.0f,
        1.0f);

    duty_b = inverter_clamp(
        duty_b,
        0.0f,
        1.0f);

    duty_c = inverter_clamp(
        duty_c,
        0.0f,
        1.0f);

    /*

    * Tensões dos polos em relação ao barramento negativo
      */
    float Va_pole =
        inverter_duty_to_pole_voltage(
            inverter,
            duty_a);

    float Vb_pole =
        inverter_duty_to_pole_voltage(
            inverter,
            duty_b);

    float Vc_pole =
        inverter_duty_to_pole_voltage(
            inverter,
            duty_c);

    /*

    * Tensão do ponto neutro virtual
      */
    float Vn =
        (Va_pole +
         Vb_pole +
         Vc_pole) /
        3.0f;

    /*

    * Tensões de fase
      */
    Vabc[0] = Va_pole - Vn;
    Vabc[1] = Vb_pole - Vn;
    Vabc[2] = Vc_pole - Vn;
}

#endif /* INVERTER_H */
