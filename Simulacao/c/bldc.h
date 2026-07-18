#ifndef BLDC_H
#define BLDC_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define JUST_THREE_PHASES 3
#define TWO_PI (2 * M_PI)

#define PHI_A 0
#define PHI_B (float)(-TWO_PI / 3.0f)
#define PHI_C (float)(TWO_PI / 3.0f)

typedef struct _bldc
{
    float iabc[JUST_THREE_PHASES];

    float R;
    float L;
    float M;
    float Ke;

    float J;
    float B;
    float Te;

    uint8_t P;
    float Kt;

    float theta_e;
    float theta_r;

    float omega_r;
    float omega_e;

    FILE *log;
} bldc_t;

typedef struct _time_simulation
{
    float t0;
    float tf;

    float dt;
} time_simulation_t;

static float trapezoidal_back_emf(float theta)
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

void bldc_log_header(const char *filename)
{
    FILE *file = fopen(filename, "w");

    if (file == NULL)
    {
        perror("Erro ao criar o arquivo de log");
        return;
    }

    fprintf(file,
            "time;"
            "ia;"
            "ib;"
            "ic;"
            "omega_r;"
            "theta_r;"
            "Te\n");

    fclose(file);
}

void bldc_log_data(bldc_t *motor, float time)
{
    fprintf(motor->log,
            "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n",
            time,
            motor->iabc[0],
            motor->iabc[1],
            motor->iabc[2],
            motor->omega_r,
            motor->theta_r,
            motor->Te);
}

float rads_to_rpm(float omega)
{
    return (omega * 60 / TWO_PI);
}

float rpm_to_rads(float rpm)
{
    return (rpm * TWO_PI / 60);
}

void bldc_step(float Vabc[JUST_THREE_PHASES], bldc_t *motor,
               time_simulation_t *time, float Tl,
               bool trapezoidal_back_emf_flag)
{
    float fabc[JUST_THREE_PHASES] = {0};
    float eabc[JUST_THREE_PHASES] = {0};
    float diabc[JUST_THREE_PHASES] = {0};
    float domega_r = 0;

    motor->theta_e = motor->P * motor->theta_r;
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

    eabc[0] = motor->Ke * motor->omega_e * fabc[0];
    eabc[1] = motor->Ke * motor->omega_e * fabc[1];
    eabc[2] = motor->Ke * motor->omega_e * fabc[2];

    diabc[0] = (Vabc[0] - motor->R * motor->iabc[0] - eabc[0]) / motor->L;
    diabc[1] = (Vabc[1] - motor->R * motor->iabc[1] - eabc[1]) / motor->L;
    diabc[2] = (Vabc[2] - motor->R * motor->iabc[2] - eabc[2]) / motor->L;

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

#endif