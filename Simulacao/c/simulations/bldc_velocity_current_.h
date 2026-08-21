/**
 * @brief Este arquivo contém a programacao/logica
 * de algumas simulacoes especificas dos motores
 */

#ifndef SIMULATIONS_BLDC_H
#define SIMULATIONS_BLDC_H

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
#include "progressbar.h"

int simulation_bldc_malha_corrente_velocidade(sim_args_t *args)
{
    /* --------------------------------------------------------------
     *   OBJETOS DA PLANTA
     * -------------------------------------------------------------- */
    bldc_t motor =
        {
            .iabc = {0.0f, 0.0f, 0.0f},
			.eabc = {0.0f, 0.0f, 0.0f},

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
        };

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
    double vdc_max = args->Vdc / sqrt(3);
    double vdc_min = -args->Vdc / sqrt(3);

    double dtOmega = 1e-3;
    double dtId = (double)pwm.Ts;
    double dtIq = (double)pwm.Ts;

    /* --------------------------------------------------------------
     *   REFERENCIAS
     * -------------------------------------------------------------- */
    const double id_ref = 0.0; /* motor de imas permanentes: referencia de eixo d = 0 */

    float rpm_ref = (float)args->rpm;
    float omega_ref = rpm_to_rads(rpm_ref);
    printf("omega_ref = %.6f rad/s\n", omega_ref);

    PIController pi_omega, pi_d, pi_q;

    pi_controller_init(&pi_omega, args->KpOmega, args->KiOmega, dtOmega,
                       true, PI_IQ_MIN, true, PI_IQ_MAX);

    pi_controller_init(&pi_d, args->KpId, args->KiId, dtId,
                       true, vdc_min, true, vdc_max);

    pi_controller_init(&pi_q, args->KpIq, args->KiIq, dtIq,
                       true, vdc_min, true, vdc_max);

    /* Controle de amostragem de cada malha PI: cada controlador so
     * roda no seu proprio periodo (dtOmega/dtId/dtIq), independente
     * do passo fino de simulacao (dt). Entre ativacoes, o ultimo
     * valor calculado e mantido (zero-order hold). */
    double t_next_omega = (double)time_sim.t0;
    double t_next_id = (double)time_sim.t0;
    double t_next_iq = (double)time_sim.t0;

    double iq_ref_hold = 0.0;
    double vd_ref_hold = 0.0;
    double vq_ref_hold = 0.0;
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
            "time;Va;Vb;Vc;ia;ib;ic;ea;eb;ec;id;iq;Te;theta_r;"
            "omega_r;iq_ref;vd_ref;vq_ref\n");
	/* --------------------------------------------------------------
     *   PROGRESS BAR 
     * -------------------------------------------------------------- */

	progress_bar_t pb;
	progress_bar_init(&pb, time_sim.t0, time_sim.tf, time_sim.dt);

    /* --------------------------------------------------------------
     *   LACO DE SIMULACAO
     * -------------------------------------------------------------- */
    for (long k = 0; k < total_steps; k++)
    {
        float t = time_sim.t0 + (float)((double)k * (double)dt);
        if (t > time_sim.tf)
            break;

        if (t >= args->Ttl)
            args->Tl = args->Tlnew;

        /* Variaveis compartilhadas pelas duas fases (preenchidas de
         * um jeito ou de outro abaixo, e usadas no log ao final) */
        float Vabc[3];
        float v_alpha, v_beta;
        float i_d = 0.0f, i_q = 0.0f;
        double iq_ref = 0.0;
        double vd_ref = 0.0, vq_ref = 0.0;

        /* ----------------------------------------------------------
         *   FASE 1: FOC EM MALHA FECHADA (controle vetorial)
         * ---------------------------------------------------------- */

        /* A. Medicao (feedback do modelo) */
        motor.theta_e = motor.theta_r * (float)motor.P;

        /* B. Malha externa de velocidade -> referencia de iq */
        if ((double)t >= t_next_omega)
        {
            iq_ref_hold = pi_controller_update(&pi_omega, omega_ref, motor.omega_r);
            t_next_omega += dtOmega;
        }
        iq_ref = iq_ref_hold;

        /* C. Transformada de Clarke (abc -> alpha-beta) */
        float i_alpha, i_beta;
        clarke_transform(motor.iabc[0], motor.iabc[1], motor.iabc[2],
                         &i_alpha, &i_beta);

        /* Transformada de Park (alpha-beta -> dq) */
        park_transform(i_alpha, i_beta, motor.theta_e, &i_d, &i_q);

        /* D. Malhas internas de corrente (PI em d e em q) */
        if ((double)t >= t_next_id)
        {
            vd_ref_hold = pi_controller_update(&pi_d, id_ref, i_d);
            t_next_id += dtId;
        }
        vd_ref = vd_ref_hold;

        if ((double)t >= t_next_iq)
        {
            vq_ref_hold = pi_controller_update(&pi_q, iq_ref, i_q);
            t_next_iq += dtIq;
        }
        vq_ref = vq_ref_hold;

        /* E. Transformada inversa de Park (dq -> alpha-beta) */
        park_inverse_transform((float)vd_ref, (float)vq_ref, motor.theta_e,
                               &v_alpha, &v_beta);

        /* F. SVPWM: duty cycles de referencia (sinal modulante) */
        float duty_a, duty_b, duty_c;
        svpwm_modulate(&pwm, v_alpha, v_beta, &duty_a, &duty_b, &duty_c);

        /* G. Chaveamento real: compara o duty de referencia com a
         *    portadora triangular instantanea -> estado 0/1 de cada
         *    braco do inversor (chave superior ligada/desligada) */
        float carrier = svpwm_carrier(&pwm, t);
        int gate_a = svpwm_gate_state(duty_a, carrier);
        int gate_b = svpwm_gate_state(duty_b, carrier);
        int gate_c = svpwm_gate_state(duty_c, carrier);

        /* H. Inversor: tensoes de fase aplicadas ao motor, calculadas
         *    a partir do estado REAL de chaveamento (0 ou Vdc por
         *    braco), nao mais do valor medio continuo */
        inverter_output_voltage(&inverter, (float)gate_a, (float)gate_b,
                                (float)gate_c, Vabc);

        /* I. Atualizacao da planta (motor BLDC) */
        bldc_step(Vabc, &motor, &time_sim, (float)args->Tl, false);

		progress_bar_update(&pb, t);

        /* J. Log dos dados */
        fprintf(log_file,
                "%.6f;%.3f;%.3f;%.3f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f"
                ";%.4f;%.4f;%.4f;%.3f;%.4f;%.4f;%.4f\n",
                t,
                Vabc[0], Vabc[1], Vabc[2],
                motor.iabc[0], motor.iabc[1], motor.iabc[2],
				motor.eabc[0], motor.eabc[1], motor.eabc[2],
                i_d, i_q,
                motor.Te,
                motor.theta_r, motor.omega_r,
                iq_ref, vd_ref, vq_ref);
                
    }

	progress_bar_finish(&pb);

    fclose(log_file);

    printf("\n\nSimulacao concluida. Resultados em \"%s\".\n\n", filename);

    return EXIT_SUCCESS;
}

#endif
