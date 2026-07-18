/*
 * main_closedloop.c
 *
 * Exemplo de simulacao em malha fechada de um motor BLDC/PMSM
 * controlado por corrente em coordenadas dq (controle vetorial),
 * equivalente ao script Python "simulation_closedloop.py".
 *
 * Estrutura da malha (a cada passo de simulacao):
 *
 *   1) Malha externa de velocidade (PI) -> gera iq_ref
 *   2) Medicao das correntes de fase -> Clarke -> Park (id, iq)
 *   3) Malhas internas de corrente (PI em d e em q) -> vd_ref, vq_ref
 *   4) Park inversa (dq -> alpha-beta)
 *   5) SVPWM (alpha-beta -> duty cycles)
 *   6) Inversor (duty cycles -> tensoes de fase Vabc)
 *   7) Planta (bldc_step) -> atualiza correntes, velocidade e posicao
 *   8) Log dos resultados em CSV
 *
 * Compilar:
 *   gcc -O2 -Wall main_closedloop.c -o closedloop -lm
 *
 * Executar:
 *   ./closedloop
 *
 * Gera o arquivo "closedloop_simulation.csv" com os resultados.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "PIcontroller.h"
#include "SVPWM.h"
#include "inverter.h"
#include "transforms.h"
#include "bldc.h"

/* ========================================================================
 *   PARAMETROS DA REDE / BARRAMENTO CC
 * ==================================================================== */
#define VDC   12.0f   /* V */

/* ========================================================================
 *   PARAMETROS DO MOTOR
 * ==================================================================== */
#define MOTOR_RS    1.5f            /* Ohm - resistencia de armadura   */
#define MOTOR_L     50e-3f          /* H   - indutancia de magnetizacao*/
#define MOTOR_KE    0.850f          /* constante eletrica (V/(rad/s))  */
#define MOTOR_KT    0.850f          /* constante de torque (N.m/A)     */
#define MOTOR_B     1e-3f           /* coeficiente de amortecimento    */
#define MOTOR_J     0.0036013854f   /* momento de inercia (kg.m^2)     */
#define MOTOR_POLOS 8
#define MOTOR_PARES_DE_POLOS (MOTOR_POLOS / 2)

#define TL 0.0f /* torque de carga (N.m) */

/* ========================================================================
 *   LIMITES DOS CONTROLADORES
 * ==================================================================== */
#define VDC_MAX  (VDC)
#define VDC_MIN  (-VDC)

#define PI_IQ_MAX 5.0
#define PI_IQ_MIN (-PI_IQ_MAX)

/* ========================================================================
 *   PARAMETROS DA SIMULACAO
 * ==================================================================== */
#define SIM_TI 0.0f     /* s */
#define SIM_TF 1.0f     /* s */
#define SIM_DT 1e-5f    /* s */

int main(void)
{
    /* --------------------------------------------------------------
     *   OBJETOS DA PLANTA
     * -------------------------------------------------------------- */
    bldc_t motor =
    {
        .iabc = {0.0f, 0.0f, 0.0f},

        .R = MOTOR_RS,
        .L = MOTOR_L,
        .M = 0.0f,
        .Ke = MOTOR_KE,

        .J = MOTOR_J,
        .B = MOTOR_B,
        .Te = 0.0f,

        .P = MOTOR_PARES_DE_POLOS,
        .Kt = MOTOR_KT,

        .theta_e = 0.0f,
        .theta_r = 0.0f,

        .omega_r = 0.0f,
        .omega_e = 0.0f,

        .log = NULL
    };

    time_simulation_t time_sim =
    {
        .t0 = SIM_TI,
        .tf = SIM_TF,
        .dt = SIM_DT
    };

    svpwm_t pwm;
    svpwm_init(&pwm, 10000.0f, 0.0f, VDC); /* Hz = 10 kHz, Ts calculado internamente */

    inverter_t inverter = { .Vdc = VDC };

    /* --------------------------------------------------------------
     *   CONTROLADORES PI
     * -------------------------------------------------------------- */
    double dtOmega = 5e-4, kpOmega = 2.0,  kiOmega = 0.5;
    double dtId    = 5e-4, kpId    = 10.0, kiId    = 5.0;
    double dtIq    = 5e-4, kpIq    = 10.0, kiIq    = 5.0;

    PIController pi_omega, pi_d, pi_q;

    pi_controller_init(&pi_omega, kpOmega, kiOmega, dtOmega,
                        true, PI_IQ_MIN, true, PI_IQ_MAX);

    pi_controller_init(&pi_d, kpId, kiId, dtId,
                        true, VDC_MIN, true, VDC_MAX);

    pi_controller_init(&pi_q, kpIq, kiIq, dtIq,
                        true, VDC_MIN, true, VDC_MAX);

    /* --------------------------------------------------------------
     *   REFERENCIAS
     * -------------------------------------------------------------- */
    double id_ref = 0.0; /* motor de imas permanentes: referencia de eixo d = 0 */

    float rpm_ref = 20.0f;
    float omega_ref = rpm_to_rads(rpm_ref);
    printf("omega_ref = %.6f rad/s\n", omega_ref);

    /* --------------------------------------------------------------
     *   ARQUIVO DE LOG
     * -------------------------------------------------------------- */
    const char *filename = "closedloop_simulation.csv";
    FILE *log_file = fopen(filename, "w");

    if (log_file == NULL)
    {
        perror("Erro ao criar o arquivo de log");
        return EXIT_FAILURE;
    }

    fprintf(log_file,
            "time;Va;Vb;Vc;ia;ib;ic;id;iq;Te;theta_r;omega_r;iq_ref;duty_a;duty_b;duty_c\n");

    /* --------------------------------------------------------------
     *   LACO DE SIMULACAO
     * -------------------------------------------------------------- */
    for (float t = time_sim.t0; t <= time_sim.tf; t += time_sim.dt)
    {
        /* A. Medicao (feedback do modelo) */
        float theta_e = motor.theta_r * (float)motor.P;

        /* B. Malha externa de velocidade -> referencia de iq */
        double iq_ref = pi_controller_update(&pi_omega, omega_ref, motor.omega_r);

        /* C. Transformada de Clarke (abc -> alpha-beta) */
        float i_alpha, i_beta;
        clarke_transform(motor.iabc[0], motor.iabc[1], motor.iabc[2],
                          &i_alpha, &i_beta);

        /* Transformada de Park (alpha-beta -> dq) */
        float i_d, i_q;
        park_transform(i_alpha, i_beta, theta_e, &i_d, &i_q);

        /* D. Malhas internas de corrente (PI em d e em q) */
        double vd_ref = pi_controller_update(&pi_d, id_ref, i_d);
        double vq_ref = pi_controller_update(&pi_q, iq_ref, i_q);

        /* E. Transformada inversa de Park (dq -> alpha-beta) */
        float v_alpha, v_beta;
        park_inverse_transform((float)vd_ref, (float)vq_ref, theta_e,
                                &v_alpha, &v_beta);

        /* F. SVPWM: duty cycles das 3 fases */
        float duty_a, duty_b, duty_c;
        svpwm_modulate(&pwm, v_alpha, v_beta, &duty_a, &duty_b, &duty_c);

        /* G. Inversor: tensoes de fase aplicadas ao motor */
        float Vabc[3];
        inverter_output_voltage(&inverter, duty_a, duty_b, duty_c, Vabc);

        /* H. Atualizacao da planta (motor BLDC) */
        bldc_step(Vabc, &motor, &time_sim, TL, false);

        /* I. Log dos dados */
        fprintf(log_file,
                "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n",
                t,
                Vabc[0], Vabc[1], Vabc[2],
                motor.iabc[0], motor.iabc[1], motor.iabc[2],
                i_d, i_q,
                motor.Te,
                motor.theta_r, motor.omega_r,
                iq_ref,
                duty_a, duty_b, duty_c);
    }

    fclose(log_file);

    printf("Simulacao concluida. Resultados em \"%s\".\n", filename);

    return EXIT_SUCCESS;
}