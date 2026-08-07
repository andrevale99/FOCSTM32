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

int simulation_bldc_malha_corrente_velocidade(sim_args_t *args)
{
    printf("\n\nParametros da simulacao:\n");
    printf("  arquivo de saida = %s\n", args->filename);
    printf("  R  = %.6f Ohm\n", args->R);
    printf("  L  = %.6f H\n", args->L);
    printf("  M  = %.6f H\n", args->M);
    printf("  Ke = %.6f V/(rad/s)\n", args->Ke);
    printf("  J  = %.9f kg.m^2\n", args->J);
    printf("  B  = %.6f\n", args->B);
    printf("  Tl = %.6f N.m\n", args->Tl);
    printf("  P  = %d\n", args->P);
    printf("  Kt = %.6f N.m/A\n", args->Kt);
    printf("\n");
    printf("  Fsw = %.1f Hz (chaveamento SVPWM real)\n", args->Fsw);
    printf("  PwmSamples = %d passos finos por periodo Ts\n", args->PwmSamples);
    printf("\n");
    printf("  Ti = %.6f s\n", args->Ti);
    printf("  Tf = %.6f s\n", args->Tf);
    printf("\n");
    printf("  rpm = %.6f RPM\n", args->rpm);
    printf("\n");
    printf("  Ttl = %.6f s\n", args->Ttl);
    printf("  Tlnew = %.6f N.m\n", args->Tlnew);
    printf("\n");
    if (args->Dt > 0.0)
        printf("  Dt = %.9e s (explicito, sobrescreve o calculo automatico)\n", args->Dt);
    else
        printf("  Dt = automatico (Ts / PwmSamples)\n");
    printf("\n");
    printf("  Vdc = %.6f V\n", args->Vdc);
    printf("\n");
    printf("  Controlador de velocidade: Kp = %.6f | Ki = %.6f\n",
           args->KpOmega, args->KiOmega);
    printf("  Controlador de corrente id: Kp = %.6f | Ki = %.6f\n",
           args->KpId, args->KiId);
    printf("  Controlador de corrente iq: Kp = %.6f | Ki = %.6f\n",
           args->KpIq, args->KiIq);
    printf("\n");
    printf("  Partida V/F em malha aberta: %s\n\n",
           args->UseVfStartup ? "SIM" : "NAO (FOC direto desde Ti)");
    printf("  Partida em malha aberta: %s\n\n",
           args->MalhaAberta ? "SIM" : "NAO");
    printf("\n\n");

    if (args->Fsw <= 0.0)
    {
        fprintf(stderr, "Erro: Fsw deve ser > 0.\n");
        return EXIT_FAILURE;
    }

    if (args->PwmSamples < 2)
    {
        fprintf(stderr, "Erro: PwmSamples deve ser >= 2.\n");
        return EXIT_FAILURE;
    }

    if (args->Tf <= args->Ti)
    {
        fprintf(stderr, "Erro: Tf deve ser maior que Ti.\n");
        return EXIT_FAILURE;
    }

    if (args->Dt < 0.0)
    {
        fprintf(stderr, "Erro: Dt nao pode ser negativo.\n");
        return EXIT_FAILURE;
    }

    if (args->Vdc <= 0.0)
    {
        fprintf(stderr, "Erro: Vdc deve ser > 0.\n");
        return EXIT_FAILURE;
    }

    if (args->MalhaAberta && args->UseVfStartup)
    {
        fprintf(stderr, "Erro: Somente um dos dois deve estar ativado\n");
        return EXIT_FAILURE;
    }

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
    double vdc_max = args->Vdc / sqrt(3);
    double vdc_min = -args->Vdc / sqrt(3);

    double dtOmega = 1e-3;
    double dtId = (double)pwm.Ts;
    double dtIq = (double)pwm.Ts;

    /* --------------------------------------------------------------
     *   REFERENCIAS
     * -------------------------------------------------------------- */
    double id_ref = 0.0; /* motor de imas permanentes: referencia de eixo d = 0 */

    float rpm_ref = (float)args->rpm;
    float omega_ref = rpm_to_rads(rpm_ref);
    printf("omega_ref = %.6f rad/s\n", omega_ref);

    /* --------------------------------------------------------------
     *   PARTIDA V/F EM MALHA ABERTA
     * -------------------------------------------------------------- */
    /* O alvo da rampa V/F (em rad/s eletrico) e derivado diretamente
     * da referencia de velocidade informada pelo usuario (omega_ref,
     * em rad/s mecanico), multiplicada pelo numero de pares de polos:
     * omega_e = P * omega_r. Assim a partida V/F sempre mira a MESMA
     * velocidade que o FOC vai manter depois, sem precisar editar
     * nenhuma constante quando o "rpm" do arquivo de parametros muda.
     *
     * V_max respeita a regiao linear do SVPWM com injecao de terceiro
     * harmonico (Vdc/sqrt(3)); aqui usa-se uma margem um pouco maior
     * (Vdc/1.8) para nao encostar no limite durante a rampa. */
    vf_startup_t vf;
    if (args->UseVfStartup)
    {
        float vf_omega_e_target = omega_ref * (float)motor.P;
        float vf_ramp_rate = vf_omega_e_target / VF_STARTUP_RAMP_TIME;

        printf("Partida V/F: alvo = %.6f rad/s eletrico (%.6f rad/s mecanico, "
               "= omega_ref), rampa em %.3f s\n",
               vf_omega_e_target, omega_ref, VF_STARTUP_RAMP_TIME);

        vf_startup_init(&vf,
                        VF_STARTUP_V_BOOST,
                        VF_STARTUP_V_PER_RAD_S,
                        (float)args->Vdc / 1.8f,
                        vf_ramp_rate,
                        vf_omega_e_target);
    }

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

        if (t >= args->Ttl)
            args->Tl = args->Tlnew;

        /* Variaveis compartilhadas pelas duas fases (preenchidas de
         * um jeito ou de outro abaixo, e usadas no log ao final) */
        float Vabc[3];
        float v_alpha, v_beta, theta_e;
        float i_d = 0.0f, i_q = 0.0f;
        double iq_ref = 0.0;
        double vd_ref = 0.0, vq_ref = 0.0;

        /* ----------------------------------------------------------
         *   FASE 1: FOC EM MALHA FECHADA (controle vetorial)
         * ---------------------------------------------------------- */

        /* A. Medicao (feedback do modelo) */
        theta_e = motor.theta_r * (float)motor.P;

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
        park_transform(i_alpha, i_beta, theta_e, &i_d, &i_q);

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
        park_inverse_transform((float)vd_ref, (float)vq_ref, theta_e,
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

        /* J. Log dos dados */
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

    printf("\n\nSimulacao concluida. Resultados em \"%s\".\n\n", filename);

    return EXIT_SUCCESS;
}

#endif
