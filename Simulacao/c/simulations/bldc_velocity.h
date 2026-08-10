/**
 * @brief Simulacao com SOMENTE a malha de velocidade fechada por
 * realimentacao (PI). As malhas internas de corrente (id/iq) NAO
 * existem como controladores PI aqui -- em vez disso, a tensao de
 * referencia dq e sintetizada por FEEDFORWARD (malha aberta de
 * corrente):
 *
 *   id_ref = 0
 *   vd_ref = -omega_e * L * iq_ref
 *   vq_ref =  R * iq_ref + Ke * omega_r
 *
 * (desacoplamento classico dq em regime permanente, ignorando d(iq)/dt).
 *
 * O restante da cadeia (Park inversa -> SVPWM -> chaveamento real ->
 * inversor -> planta BLDC) permanece identico ao FOC completo, para
 * que as colunas do log tenham o MESMO significado/formato do
 * arquivo simulations_bldc.h anexado.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <getopt.h>

#include "structs.h"
#include "defs.h"

#include "PIcontroller.h"
#include "svpwm.h"
#include "inverter.h"
#include "transforms.h"
#include "bldc.h"

int simulation_bldc_malha_velocidade(sim_args_t *args)
{
    /* --------------------------------------------------------------
     *   OBJETOS DA PLANTA
     * -------------------------------------------------------------- */
    bldc_t motor =
        {
            .iabc = {0.0f, 0.0f, 0.0f},

            .R = (float)args->R,
            .L = (float)args->L,
            .M = (float)args->M,
            .Ke = (float)args->Ke,

            .J = (float)args->J,
            .B = (float)args->B,
            .Te = 0.0f,

            .P = args->P,
            .Kt = (float)args->Kt,

            .theta_e = 0.0f,
            .theta_r = 0.0f,

            .omega_r = 0.0f,
            .omega_e = 0.0f,

            .log = NULL};

    svpwm_t pwm;
    if (!svpwm_init(&pwm, (float)args->Fsw, 0.0f, (float)args->Vdc))
    {
        fprintf(stderr, "Erro: falha ao inicializar o SVPWM (verifique Fsw e Vdc).\n");
        return EXIT_FAILURE;
    }

    /* O passo de integracao (dt) e derivado do periodo de chaveamento
     * (Ts = 1/Fsw), para que a comparacao com a portadora triangular
     * -- e portanto o chaveamento real do inversor -- seja resolvida
     * no tempo. Isso faz com que mudar Fsw realmente altere o
     * resultado da simulacao (ripple de corrente, torque, etc).
     * Se o usuario informar Dt explicitamente (> 0), ele sobrescreve
     * esse calculo automatico. */
    float dt;
    if (args->Dt > 0.0)
    {
        dt = (float)args->Dt;

        if (dt > pwm.Ts / 2.0f)
        {
            fprintf(stderr,
                    "Aviso: Dt=%.6e s e maior que metade do periodo de "
                    "chaveamento (Ts=%.6e s). O chaveamento real do "
                    "inversor NAO sera resolvido corretamente; considere "
                    "um Dt menor ou omita -d/--dt para o calculo "
                    "automatico (Ts/PwmSamples).\n\n",
                    (double)dt, (double)pwm.Ts);
        }
    }
    else
    {
        dt = pwm.Ts / (float)args->PwmSamples;
    }

    time_simulation_t time_sim =
        {
            .t0 = (float)args->Ti,
            .tf = (float)args->Tf,
            .dt = dt,
        };

    long total_steps =
        (long)((double)(time_sim.tf - time_sim.t0) / (double)dt + 0.5) + 1;

    printf("Periodo de chaveamento (Ts) = %.9e s\n", (double)pwm.Ts);
    printf("Passo de integracao (dt)    = %.9e s\n", (double)dt);
    printf("Total de passos da simulacao ~ %ld\n\n", total_steps);

    if (total_steps > 5000000L)
    {
        fprintf(stderr,
                "Aviso: %ld passos -- a simulacao pode demorar bastante. "
                "Reduza --pwm-samples ou aumente --fsw se necessario.\n\n",
                total_steps);
    }

    inverter_t inverter = {.Vdc = (float)args->Vdc};

    /* --------------------------------------------------------------
     *   CONTROLADORES PI
     * -------------------------------------------------------------- */
    /* Os limites de saturacao das malhas de corrente (vd/vq) sao
     * +-Vdc, calculados a partir do parametro Vdc informado. */

    double dtOmega = 1e-3;

    /* --------------------------------------------------------------
     *   REFERENCIAS
     * -------------------------------------------------------------- */

    float rpm_ref = (float)args->rpm;
    float omega_ref = rpm_to_rads(rpm_ref);
    printf("omega_ref = %.6f rad/s\n", omega_ref);

    PIController pi_omega;

    pi_controller_init(&pi_omega, args->KpOmega, args->KiOmega, dtOmega,
                       true, PI_IQ_MIN, true, PI_IQ_MAX);

    /* Controle de amostragem de cada malha PI: cada controlador so
     * roda no seu proprio periodo (dtOmega/dtId/dtIq), independente
     * do passo fino de simulacao (dt). Entre ativacoes, o ultimo
     * valor calculado e mantido (zero-order hold). */
    double t_next_omega = (double)time_sim.t0;

    double iq_ref_hold = 0.0;
    /* --------------------------------------------------------------
     *   ARQUIVO DE LOG
     * -------------------------------------------------------------- */
    const char *filename = args->filename;
    FILE *log_file = fopen(filename, "w");

    if (log_file == NULL)
    {
        perror("Erro ao criar o arquivo de log");
        return EXIT_FAILURE;
    }

    fprintf(log_file,
            "time;Va;Vb;Vc;ia;ib;ic;id;iq;Te;theta_r;"
            "omega_r;iq_ref;vd_ref;vq_ref;"
            "duty_a;duty_b;duty_c;carrier;gate_a;gate_b;gate_c\n");

    /* mode: 0 = partida V/F em malha aberta | 1 = FOC em malha fechada */

    /* --------------------------------------------------------------
     *   LACO DE SIMULACAO
     * -------------------------------------------------------------- */

    for (long k = 0; k < total_steps; k++)
    {
        float t = time_sim.t0 + (float)((double)k * (double)dt);
        if (t > time_sim.tf)
            break;

        float Vabc[3];
        float v_alpha, v_beta, theta_e;
        float i_d = 0.0f, i_q = 0.0f;

        /* A. Medicao (feedback do modelo) */
        theta_e = motor.theta_r * (float)motor.P;

        /* B. Malha de velocidade (UNICA malha fechada por PI) */
        if ((double)t >= t_next_omega)
        {
            iq_ref_hold = pi_controller_update(&pi_omega, omega_ref, motor.omega_r);
            t_next_omega += dtOmega;
        }
        double iq_ref = iq_ref_hold;

        /* C. Clarke + Park (somente para log/observacao de id/iq reais;
         *    NAO realimentam nenhum controlador de corrente aqui) */
        float i_alpha, i_beta;
        clarke_transform(motor.iabc[0], motor.iabc[1], motor.iabc[2],
                         &i_alpha, &i_beta);
        park_transform(i_alpha, i_beta, theta_e, &i_d, &i_q);

        /* D. Sintese de tensao dq por FEEDFORWARD (sem PI de corrente) */
        float omega_e = (float)motor.P * motor.omega_r;
        double vd_ref = -(double)(omega_e * motor.L) * iq_ref;
        double vq_ref = (double)motor.R * iq_ref + (double)motor.Ke * (double)motor.omega_r;

        /* E. Park inversa (dq -> alpha-beta) */
        park_inverse_transform((float)vd_ref, (float)vq_ref, theta_e,
                               &v_alpha, &v_beta);

        /* F. SVPWM */
        float duty_a, duty_b, duty_c;
        svpwm_modulate(&pwm, v_alpha, v_beta, &duty_a, &duty_b, &duty_c);

        /* G. Chaveamento real */
        float carrier = svpwm_carrier(&pwm, t);
        int gate_a = svpwm_gate_state(duty_a, carrier);
        int gate_b = svpwm_gate_state(duty_b, carrier);
        int gate_c = svpwm_gate_state(duty_c, carrier);

        /* H. Inversor */
        inverter_output_voltage(&inverter, (float)gate_a, (float)gate_b,
                                (float)gate_c, Vabc);

        /* I. Planta */
        bldc_step(Vabc, &motor, &time_sim, args->Tl, false);

        /* J. Log */
        fprintf(log_file,
                "%.6f;%.3f;%.3f;%.3f;%.4f;%.4f;%.4f;%.4f"
                ";%.4f;%.4f;%.4f;%.3f;%.4f;%.4f;%.4f;"
                "%.4f;%.4f;%.4f;%.4f;%d;%d;%d\n",
                t,
                Vabc[0], Vabc[1], Vabc[2],
                motor.iabc[0], motor.iabc[1], motor.iabc[2],
                i_d, i_q,
                motor.Te,
                motor.theta_r, motor.omega_r,
                iq_ref, vd_ref, vq_ref,
                duty_a, duty_b, duty_c,
                carrier,
                gate_a, gate_b, gate_c);
    }

    fclose(log_file);
    printf("Simulacao concluida. Resultados em \"%s\".\n", filename);

    return EXIT_SUCCESS;
}