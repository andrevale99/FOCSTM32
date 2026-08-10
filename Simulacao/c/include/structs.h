#ifndef STRUCTS_H
#define STRUCTS_H

/* ========================================================================
 *   ARGUMENTOS DE LINHA DE COMANDO
 * ==================================================================== */
typedef struct
{
    char filename[256];
    double R;
    double L;
    double M;
    double Ke;
    double J;
    double B;
    double Tl;
    int P;
    double Kt;
    double Fsw;       /* frequencia de chaveamento do SVPWM [Hz]           */
    int PwmSamples;   /* passos finos de simulacao por periodo Ts do PWM  */
    double Ti;        /* tempo inicial da simulacao [s]                   */
    double Tf;        /* tempo final da simulacao [s]                     */
    double Dt;        /* passo de integracao explicito [s] (0 = automatico) */
    double Ttl;       /* Tempo onde vai ser inserido a nova carga*/
    double Tlnew;     /* Momento de inercia da carga inserida*/
    double Vdc;       /* tensao do barramento CC [V]                       */
    double KpOmega;   /* ganho proporcional - malha de velocidade          */
    double KiOmega;   /* ganho integral - malha de velocidade              */
    double KpId;      /* ganho proporcional - malha de corrente id         */
    double KiId;      /* ganho integral - malha de corrente id             */
    double KpIq;      /* ganho proporcional - malha de corrente iq         */
    double KiIq;      /* ganho integral - malha de corrente iq             */
    double rpm;       /*referencia de velocidade*/
    int MalhaAberta;  /*Gera as onda de alimentacao internamente e utiliza somente do atep do bldc*/
} sim_args_t;

/* Identificadores para opcoes de linha de comando que so existem na
 * forma longa (--vdc, --kp-omega, etc.), sem letra curta associada.
 * getopt_long aceita qualquer inteiro > 255 como "val" para essas. */
enum
{
    OPT_VDC = 1000,
    OPT_KP_OMEGA,
    OPT_KI_OMEGA,
    OPT_KP_ID,
    OPT_KI_ID,
    OPT_KP_IQ,
    OPT_KI_IQ,
    OPT_MALHAABERTA_STARTUP,
    OPT_TTL,
    OPT_TLNEW,
};

#endif