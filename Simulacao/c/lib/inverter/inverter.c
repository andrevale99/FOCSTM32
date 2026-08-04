#include "inverter.h"

float inverter_clamp(
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

float inverter_duty_to_pole_voltage(
    const inverter_t *inverter,
    float duty)
{
    return duty * inverter->Vdc;
}

void inverter_output_voltage(
    const inverter_t *inverter,
    float duty_a,
    float duty_b,
    float duty_c,
    float Vabc[3])
{
    /*

    * Limitacao dos duty cycles
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

    * Tensoes dos polos em relacao ao barramento negativo
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

    * Tensao do ponto neutro virtual
      */
    float Vn =
        (Va_pole +
         Vb_pole +
         Vc_pole) /
        3.0f;

    /*

    * Tensoes de fase
      */
    Vabc[0] = Va_pole - Vn;
    Vabc[1] = Vb_pole - Vn;
    Vabc[2] = Vc_pole - Vn;
}