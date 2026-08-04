#ifndef INVERTER_H
#define INVERTER_H

#include <stdio.h>
#include <stdbool.h>

/**

* @brief Estrutura responsavel pela modelagem do inversor trifasico.
  */
typedef struct
{
    /**

    * @brief Tensao do barramento CC.
      */
    float Vdc;

} inverter_t;

/**

* @brief Limita um valor entre um limite inferior e superior.
*
* @param value Valor a ser limitado.
* @param min Valor minimo.
* @param max Valor maximo.
*
* @return Valor limitado ao intervalo especificado.
  */
float inverter_clamp(
    float value,
    float min,
    float max);

/**

* @brief Converte o duty cycle na tensao media do polo da fase.
*
* A tensao media do polo e dada por:
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
* @return Tensao media do polo da fase.
  */
float inverter_duty_to_pole_voltage(
    const inverter_t *inverter,
    float duty);
/**

* @brief Calcula as tensoes de fase aplicadas ao motor.
*
* Os duty cycles sao inicialmente limitados ao intervalo:
*
* \f[
* 0 \leq duty \leq 1
* \f]
*
* As tensoes dos polos em relacao ao terminal negativo do barramento
* sao calculadas por:
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
* O ponto neutro virtual e calculado como:
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
* As tensoes de fase aplicadas ao motor sao:
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
* @param Vabc Vetor de saida contendo as tensoes das fases A, B e C.
  */
void inverter_output_voltage(
    const inverter_t *inverter,
    float duty_a,
    float duty_b,
    float duty_c,
    float Vabc[3]);

#endif /* INVERTER_H */