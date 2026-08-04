#ifndef DEFS_H
#define DEFS_H

/* ========================================================================
 *   PARAMETROS DA REDE / BARRAMENTO CC
 * ==================================================================== */
#define DEFAULT_VDC 120.0f /* V */

/* ========================================================================
 *   PARAMETROS DO MOTOR
 * ==================================================================== */
#define DEFAULT_MOTOR_RS 1.5f          /* Ohm - resistencia de armadura   */
#define DEFAULT_MOTOR_L 50e-3f         /* H   - indutancia de magnetizacao*/
#define DEFAULT_MOTOR_M 0.0f           /* H   - indutancia mutua          */
#define DEFAULT_MOTOR_KE 0.850f        /* constante eletrica (V/(rad/s))  */
#define DEFAULT_MOTOR_KT 0.850f        /* constante de torque (N.m/A)     */
#define DEFAULT_MOTOR_B 1e-3f          /* coeficiente de amortecimento    */
#define DEFAULT_MOTOR_J 0.0036013854f  /* momento de inercia (kg.m^2)     */
#define DEFAULT_MOTOR_PARES_DE_POLOS 4 /* numero de pares de polos (P)    */
#define DEFAULT_RPM_REFERENCE 1        /* numero de pares de polos (P)    */

#define DEFAULT_TL 0.0f /* torque de carga (N.m) */

#define DEFAULT_TLNEW_TIME 0.0f /*Tempo onde vai ser inserido a novar carga*/
#define DEFAULT_TLNEW_NM 0.0f   /*Torque da nova carga*/

#define DEFAULT_OUTPUT_FILE "closedloop_simulation.csv"

/* ========================================================================
 *   PARAMETROS DO SVPWM (CHAVEAMENTO REAL)
 * ==================================================================== */
#define DEFAULT_SVPWM_HZ 10000.0 /* Hz - frequencia de chaveamento     */
#define DEFAULT_PWM_SAMPLES 20   /* passos finos de simulacao por Ts   */

/* ========================================================================
 *   LIMITES DOS CONTROLADORES
 * ==================================================================== */
/* Os limites de saturacao das malhas de corrente (vd/vq) sao +-Vdc,
 * calculados em tempo de execucao em main() a partir de args.Vdc,
 * ja que Vdc agora e configuravel via CLI/arquivo de config. */

#define PI_IQ_MAX 20
#define PI_IQ_MIN (-PI_IQ_MAX)

/* ========================================================================
 *   PARTIDA V/F EM MALHA ABERTA (vf_startup.h)
 * ==================================================================== */
/* Fase inicial em malha aberta, sem realimentacao de posicao: o angulo
 * eletrico usado para sintetizar o vetor de tensao comeca em 0 rad e
 * evolui apenas integrando uma rampa de frequencia comandada. Serve
 * para tirar o motor do repouso ate uma velocidade em que a FCEM seja
 * suficiente para um futuro observador de fluxo (ou o proprio sensor)
 * assumir o controle. Ajuste estas constantes conforme o motor. */
#define VF_STARTUP_V_BOOST 1.0f       /* V   - tensao de fase em omega_e=0   */
#define VF_STARTUP_V_PER_RAD_S 0.005f /* V/(rad/s eletrico) - ganho V/F (a "razao" V/F, \
                                       * independente do tempo de rampa) */

/* Tempo desejado para a rampa ir de 0 ate o alvo. O alvo (em rad/s
 * eletrico) NAO e mais uma constante fixa: e calculado em main(), em
 * tempo de execucao, a partir de "rpm" do arquivo de parametros
 * (omega_e_target = P * omega_ref) -- assim a partida V/F sempre
 * mira a mesma velocidade configurada pelo usuario. Para deixar a
 * partida mais lenta/suave, aumente este valor: V_BOOST e
 * V_PER_RAD_S (a razao V/F) nao mudam, pois dependem so de omega_e,
 * nao do tempo. */
#define VF_STARTUP_RAMP_TIME 0.2f /* s */

/* s - a fase V/F dura exatamente o tempo da rampa (nunca corta antes
 * nem se estende desnecessariamente depois de atingir o alvo) */
#define VF_STARTUP_DURATION VF_STARTUP_RAMP_TIME

/* Flag que liga/desliga a partida V/F em malha aberta. Se desativada,
 * a simulacao roda com o mesmo fluxo de antes da implementacao do
 * V/F: FOC em malha fechada desde t = Ti, com theta_e vindo direto
 * do modelo do motor. Configuravel via arquivo (VfStartup=0/1) ou
 * CLI (--vf-startup <0|1>). */
#define DEFAULT_USE_VF_STARTUP 0

/**
 * Flag de simualcao para gerar a amlha aberta. Caso esteja utilizada,
 * as ondas que alimentarao o motor serao geradas intermanente no laco
 * de forma idel (onda senoidais), com amplitude +-Vdc
 */
#define DEFAULT_USE_MALHA_ABERTA 1

/* ========================================================================
 *   GANHOS PADRAO DOS CONTROLADORES PI
 * ==================================================================== */
#define DEFAULT_KP_OMEGA 2.0
#define DEFAULT_KI_OMEGA 1.5
#define DEFAULT_KP_ID 6.0
#define DEFAULT_KI_ID 2.0
#define DEFAULT_KP_IQ 6.0
#define DEFAULT_KI_IQ 2.0

/* ========================================================================
 *   PARAMETROS DA SIMULACAO
 * ==================================================================== */
#define DEFAULT_SIM_TI 0.0f /* s */
#define DEFAULT_SIM_TF 0.5f /* s */
/* DEFAULT_SIM_DT = 0.0 significa "automatico": o passo de integracao e
 * derivado da frequencia de chaveamento do SVPWM (Ts / PwmSamples). Se
 * o usuario informar um Dt explicito (> 0), esse valor e usado
 * diretamente, sobrescrevendo o calculo automatico. Veja main(). */
#define DEFAULT_SIM_DT 0.0f

#endif