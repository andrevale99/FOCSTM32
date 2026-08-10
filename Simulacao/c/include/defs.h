#ifndef DEFS_H
#define DEFS_H

/* ========================================================================
 *   PARAMETROS DA REDE / BARRAMENTO CC
 * ==================================================================== */
#define DEFAULT_VDC 220.0f /* V */

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

#define PI_IQ_MAX 15
#define PI_IQ_MIN (-PI_IQ_MAX)

/* ========================================================================
 *   PARTIDA EM MALHA ABERTA (vf_startup.h)
 * ==================================================================== */
/**
 * Flag de simualcao para gerar a amlha aberta. Caso esteja utilizada,
 * as ondas que alimentarao o motor serao geradas intermanente no laco
 * de forma idel (onda senoidais), com amplitude +-Vdc
 */
#define DEFAULT_USE_MALHA_ABERTA 0

/* ========================================================================
 *   GANHOS PADRAO DOS CONTROLADORES PI
 * ==================================================================== */
#define DEFAULT_KP_OMEGA 0.01
#define DEFAULT_KI_OMEGA 0.01
#define DEFAULT_KP_ID 0.01
#define DEFAULT_KI_ID 0.01
#define DEFAULT_KP_IQ 0.01
#define DEFAULT_KI_IQ 0.01

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
