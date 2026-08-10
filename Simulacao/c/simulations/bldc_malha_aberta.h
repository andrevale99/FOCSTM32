#ifndef BLDC_MALHA_ABERTA_H
#define BLDC_MALHA_ABERTA_H

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

int simulation_bldc_malha_aberta(sim_args_t *args)
{

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

    /* --------------------------------------------------------------
     *   LACO DE SIMULACAO
     * -------------------------------------------------------------- */
    for (long k = 0; k < total_steps; k++)
    {
		float t = time_sim.t0 + (float)((double)k * (double)dt);
        if (t > time_sim.tf)
            break;

        /* Variaveis compartilhadas pelas duas fases (preenchidas de
         * um jeito ou de outro abaixo, e usadas no log ao final) */
        float Vabc[3];

		double theta_e = motor.theta_r * (float)motor.P;

		Vabc[0] = args->Vdc * sinf(theta_e + PHI_A);
		Vabc[1] = args->Vdc * sinf(theta_e + PHI_B);
		Vabc[2] = args->Vdc * sinf(theta_e + PHI_C);
		/* I. Atualizacao da planta (motor BLDC) */
		bldc_step(Vabc, &motor, &time_sim, (float)args->Tl, false);

		/* J. Log dos dados */
		fprintf(log_file,
        	"%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;"
        	"%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;"
        	"%.6f;%.6f;%.6f;%.6f;%d;%d;%d\n",
        	t,
        	Vabc[0], Vabc[1], Vabc[2],
        	motor.iabc[0], motor.iabc[1], motor.iabc[2],
        	0.f, 0.f,
        	motor.Te,
        	motor.theta_r, motor.omega_r,
        	0.f, 0. /*vd_ref*/, 0. /*vq_ref*/,
        	0. /*duty_a*/, 0. /*duty_b*/, 0. /*duty_c*/,
        	0. /*carrier*/,
        	0 /*gate_a*/, 0 /*gate_b*/, 0 /*gate_c*/);
	}

	return EXIT_SUCCESS;
}

#endif
