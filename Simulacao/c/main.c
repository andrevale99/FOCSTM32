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
 */

#include "main.h"

#include "simulations/bldc_velocity_current_.h"
#include "simulations/bldc_malha_aberta.h"

int main(int argc, char **argv)
{
    sim_args_t args;
    parse_args(argc, argv, &args);

    print_data_simulation(&args);

    if(args.MalhaAberta == false)
    	simulation_bldc_malha_corrente_velocidade(&args);
    else
 		simulation_bldc_malha_aberta(&args);

    return EXIT_SUCCESS;
}
