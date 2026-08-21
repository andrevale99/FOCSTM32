#include "bldc.h"

/**
 * @brief Calcula a forma de onda trapezoidal normalizada da FEM.
 *
 * Calcula uma funcao periodica de amplitude normalizada entre -1 e 1,
 * com formato trapezoidal, a partir da posicao angular eletrica.
 *
 * @param[in] theta Posicao angular eletrica, em radianos.
 *
 * @return Valor normalizado da forca contraeletromotriz no intervalo
 *         aproximado de -1 a 1.
 *
 * @note O angulo e normalizado para o intervalo [0, 2pi).
 */
float trapezoidal_back_emf(float theta)
{
    theta = fmodf(theta, TWO_PI);

    if (theta < 0.0f)
    {
        theta += TWO_PI;
    }

    if (theta < M_PI / 6.0f)
    {
        return 6.0f * theta / M_PI;
    }
    else if (theta < 5.0f * M_PI / 6.0f)
    {
        return 1.0f;
    }
    else if (theta < 7.0f * M_PI / 6.0f)
    {
        return -6.0f * theta / M_PI + 6.0f;
    }
    else if (theta < 11.0f * M_PI / 6.0f)
    {
        return -1.0f;
    }
    else
    {
        return 6.0f * theta / M_PI - 12.0f;
    }
}

float rads_to_rpm(float omega)
{
    return (omega * 60 / TWO_PI);
}

float rpm_to_rads(float rpm)
{
    return (rpm * TWO_PI / 60);
}

void bldc_step(float Vabc[JUST_THREE_PHASES],
               bldc_t *motor,
               time_simulation_t *time,
               float Tl,
               bool trapezoidal_back_emf_flag)
{
    float fabc[JUST_THREE_PHASES] = {0};
    float diabc[JUST_THREE_PHASES] = {0};
    float domega_r = 0;

    motor->theta_e = fmodf(motor->P * motor->theta_r, TWO_PI);

    if (motor->theta_e < 0.0f)
    {
        motor->theta_e += TWO_PI;
    }

    motor->omega_e = motor->P * motor->omega_r;

    if (trapezoidal_back_emf_flag)
    {
        fabc[0] = -trapezoidal_back_emf(motor->theta_e + PHI_A);
        fabc[1] = -trapezoidal_back_emf(motor->theta_e + PHI_B);
        fabc[2] = -trapezoidal_back_emf(motor->theta_e + PHI_C);
    }
    else
    {
        fabc[0] = -sinf(motor->theta_e + PHI_A);
        fabc[1] = -sinf(motor->theta_e + PHI_B);
        fabc[2] = -sinf(motor->theta_e + PHI_C);
    }

    motor->eabc[0] = motor->Ke * motor->omega_r * fabc[0];
    motor->eabc[1] = motor->Ke * motor->omega_r * fabc[1];
    motor->eabc[2] = motor->Ke * motor->omega_r * fabc[2];

    diabc[0] = (Vabc[0] - motor->R * motor->iabc[0] - motor->eabc[0]) / (motor->L + motor->M);
    diabc[1] = (Vabc[1] - motor->R * motor->iabc[1] - motor->eabc[1]) / (motor->L + motor->M);
    diabc[2] = (Vabc[2] - motor->R * motor->iabc[2] - motor->eabc[2]) / (motor->L + motor->M);

    motor->iabc[0] += diabc[0] * time->dt;
    motor->iabc[1] += diabc[1] * time->dt;
    motor->iabc[2] += diabc[2] * time->dt;

    motor->Te = motor->Kt * (motor->iabc[0] * fabc[0] +
                             motor->iabc[1] * fabc[1] +
                             motor->iabc[2] * fabc[2]);

    domega_r = (motor->Te - Tl - motor->omega_r * motor->B) / motor->J;

    motor->omega_r += domega_r * time->dt;
    motor->theta_r += motor->omega_r * time->dt;
}
