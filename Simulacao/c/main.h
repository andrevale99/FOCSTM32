#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include <getopt.h>

#include "PIcontroller.h"
#include "svpwm.h"
#include "inverter.h"
#include "transforms.h"
#include "bldc.h"
#include "VFstartup.h"

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
    int UseVfStartup; /* 1 = realiza partida V/F em malha aberta antes do FOC; 0 = FOC direto desde Ti */
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
    OPT_VF_STARTUP,
    OPT_MALHAABERTA_STARTUP,
    OPT_TTL,
    OPT_TLNEW,
};

/* Interpreta strings booleanas aceitas em CLI/arquivo de config para
 * a flag de partida V/F: "1"/"0", "true"/"false", "yes"/"no",
 * "on"/"off" (case-insensitive). Qualquer outra coisa cai no atoi(). */
int parse_bool(const char *v)
{
    if (strcasecmp(v, "true") == 0 || strcasecmp(v, "yes") == 0 ||
        strcasecmp(v, "on") == 0)
    {
        return 1;
    }

    if (strcasecmp(v, "false") == 0 || strcasecmp(v, "no") == 0 ||
        strcasecmp(v, "off") == 0)
    {
        return 0;
    }

    return atoi(v) != 0;
}

void usage(const char *prog)
{
    fprintf(stderr,
            "Uso: %s [opcoes]\n\n"
            "Os parametros podem ser passados diretamente na linha de comando\n"
            "OU atraves de um arquivo texto de configuracao (-c/--config).\n"
            "Se um parametro for informado nos dois lugares, o valor passado\n"
            "diretamente na linha de comando tem prioridade sobre o arquivo.\n\n"
            "Formato do arquivo de configuracao (uma linha por parametro):\n"
            "  R=1.5\n"
            "  L=0.05\n"
            "  M=0.0\n"
            "  Ke=0.85\n"
            "  J=0.0036013854\n"
            "  B=0.001\n"
            "  Tl=0.0\n"
            "  P=4\n"
            "  Kt=0.85\n"
            "  Fsw=10000\n"
            "  PwmSamples=20\n"
            "  Ti=0.0\n"
            "  Tf=1.0\n"
            "  Dt=0.0\n"
            "  Ttl=0.0\n"
            "  Tlnew=0.0\n"
            "  Vdc=120.0\n"
            "  KpOmega=2.0\n"
            "  KiOmega=1.5\n"
            "  KpId=6.0\n"
            "  KiId=2.0\n"
            "  KpIq=6.0\n"
            "  KiIq=2.0\n"
            "  VfStartup=1\n"
            "  file=saida.csv\n"
            "(linhas em branco ou iniciadas com '#' sao ignoradas)\n\n"
            "Opcoes:\n"
            "  -c, --config <arquivo> Arquivo texto com os parametros\n"
            "  -f, --file <nome>   Nome do arquivo CSV de saida (default: %s)\n"
            "  -R, --R <valor>     Resistencia de armadura [Ohm]      (default: %.6f)\n"
            "  -L, --L <valor>     Indutancia de magnetizacao [H]     (default: %.6f)\n"
            "  -M, --M <valor>     Indutancia mutua [H]                (default: %.6f)\n"
            "  -K, --Ke <valor>    Constante eletrica [V/(rad/s)]     (default: %.6f)\n"
            "  -J, --J <valor>     Momento de inercia [kg.m^2]        (default: %.6f)\n"
            "  -B, --B <valor>     Coeficiente de amortecimento       (default: %.6f)\n"
            "  -T, --Tl <valor>    Torque de carga [N.m]              (default: %.6f)\n"
            "  -P, --P <valor>     Numero de pares de polos           (default: %d)\n"
            "  -t, --Kt <valor>    Constante de torque [N.m/A]        (default: %.6f)\n"
            "  -s, --fsw <valor>   Frequencia de chaveamento do SVPWM [Hz] (default: %.1f)\n"
            "  -n, --pwm-samples <n> Passos finos de simulacao por periodo Ts do PWM\n"
            "                       (default: %d; maior = mais fiel, porem mais lento)\n"
            "  -i, --ti <valor>    Tempo inicial da simulacao [s]     (default: %.6f)\n"
            "  -e, --tf <valor>    Tempo final da simulacao [s]       (default: %.6f)\n"
            "  -d, --dt <valor>    Passo de integracao explicito [s]  (default: automatico,\n"
            "                       dt = 1/Fsw / PwmSamples; informe > 0 para sobrescrever)\n"
            "      --vdc <valor>   Tensao do barramento CC [V]        (default: %.6f)\n"
            "      --kp-omega <v>  Ganho proporcional - malha de velocidade (default: %.6f)\n"
            "      --ki-omega <v>  Ganho integral - malha de velocidade     (default: %.6f)\n"
            "      --kp-id <v>     Ganho proporcional - malha de corrente id (default: %.6f)\n"
            "      --ki-id <v>     Ganho integral - malha de corrente id     (default: %.6f)\n"
            "      --kp-iq <v>     Ganho proporcional - malha de corrente iq (default: %.6f)\n"
            "      --ki-iq <v>     Ganho integral - malha de corrente iq     (default: %.6f)\n"
            "      --rpm               Referencia de velocidade em RPM       (default: %.2f)\n"
            "      --vf-startup <0|1> Realiza partida V/F em malha aberta antes do FOC\n"
            "                       (aceita 0/1, true/false, yes/no)         (default: %d)\n"
            "      --Ttl    <v>    Tempo em segnudos onde vai ser inserido a nova carga (default: %.6f)\n"
            "      --Tlnew <v>     Torque da nova carga inserida (default: %.6f)\n"
            "  -malha-aberta       Simula o motor em malha aberta com ondas senoidais\n"
            "                      de alimentacao (default=%d)\n"
            "  -h, --help          Mostra esta mensagem de ajuda\n\n"
            "Observacao: a simulacao reproduz o chaveamento aproximado do inversor\n"
            "(comparacao do duty cycle com uma portadora triangular), nao um\n"
            "modelo de valor medio. Por isso, se -d/--dt nao for informado, o\n"
            "passo de integracao e derivado automaticamente de Fsw e PwmSamples\n"
            "(dt = 1/Fsw / PwmSamples), e mudar -s/--fsw tem efeito real no\n"
            "resultado (ripple de corrente, de torque, etc). Se -d/--dt for\n"
            "informado explicitamente, ele e usado no lugar do calculo\n"
            "automatico (util para comparar com um passo fixo), mas um aviso\n"
            "e emitido caso ele seja grande demais para resolver o chaveamento.\n"
            "\n"
            "Uma nova carga pode ser inserido passando como argumento\n"
            "sendo \"Ttl\" o tempo onde vai inserido a nova carga\n"
            "e \"Tlnew\" o torque da carga inserida.\n\n",
            prog,
            DEFAULT_OUTPUT_FILE,
            (double)DEFAULT_MOTOR_RS,
            (double)DEFAULT_MOTOR_L,
            (double)DEFAULT_MOTOR_M,
            (double)DEFAULT_MOTOR_KE,
            (double)DEFAULT_MOTOR_J,
            (double)DEFAULT_MOTOR_B,
            (double)DEFAULT_TL,
            DEFAULT_MOTOR_PARES_DE_POLOS,
            (double)DEFAULT_MOTOR_KT,
            (double)DEFAULT_SVPWM_HZ,
            DEFAULT_PWM_SAMPLES,
            (double)DEFAULT_SIM_TI,
            (double)DEFAULT_SIM_TF,
            (double)DEFAULT_VDC,
            (double)DEFAULT_KP_OMEGA,
            (double)DEFAULT_KI_OMEGA,
            (double)DEFAULT_KP_ID,
            (double)DEFAULT_KI_ID,
            (double)DEFAULT_KP_IQ,
            (double)DEFAULT_KI_IQ,
            (double)DEFAULT_RPM_REFERENCE,
            DEFAULT_USE_VF_STARTUP,
            DEFAULT_TLNEW_TIME,
            DEFAULT_TLNEW_NM,
            DEFAULT_USE_MALHA_ABERTA);
}

int parse_config_file(const char *path, sim_args_t *args)
{
    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        perror("Erro ao abrir arquivo de configuracao");
        return -1;
    }

    char line[512];
    int line_no = 0;

    while (fgets(line, sizeof(line), f) != NULL)
    {
        line_no++;

        /* remove comentarios e espacos iniciais */
        char *p = line;
        while (*p == ' ' || *p == '\t')
            p++;

        if (*p == '\0' || *p == '\n' || *p == '#' || *p == '\r')
            continue;

        char key[64] = {0};
        char value[256] = {0};

        /* aceita "CHAVE = VALOR" ou "CHAVE=VALOR" */
        if (sscanf(p, " %63[^=] = %255[^\r\n]", key, value) != 2)
        {
            /* aceita "CHAVE VALOR" separado por espaco/tab */
            if (sscanf(p, " %63s %255[^\r\n]", key, value) != 2)
            {
                fprintf(stderr,
                        "Aviso: linha %d do arquivo de configuracao ignorada "
                        "(formato invalido): %s",
                        line_no, line);
                continue;
            }
        }

        /* remove espacos no final da chave */
        for (int i = (int)strlen(key) - 1; i >= 0 && (key[i] == ' ' || key[i] == '\t'); i--)
            key[i] = '\0';

        /* remove espacos no inicio/fim do valor */
        char *v = value;
        while (*v == ' ' || *v == '\t')
            v++;
        for (int i = (int)strlen(v) - 1; i >= 0 && (v[i] == ' ' || v[i] == '\t'); i--)
            v[i] = '\0';

        if (strcasecmp(key, "R") == 0)
            args->R = atof(v);
        else if (strcasecmp(key, "L") == 0)
            args->L = atof(v);
        else if (strcasecmp(key, "M") == 0)
            args->M = atof(v);
        else if (strcasecmp(key, "Ke") == 0)
            args->Ke = atof(v);
        else if (strcasecmp(key, "J") == 0)
            args->J = atof(v);
        else if (strcasecmp(key, "B") == 0)
            args->B = atof(v);
        else if (strcasecmp(key, "Tl") == 0)
            args->Tl = atof(v);
        else if (strcasecmp(key, "P") == 0)
            args->P = atoi(v);
        else if (strcasecmp(key, "Kt") == 0)
            args->Kt = atof(v);
        else if (strcasecmp(key, "Fsw") == 0)
            args->Fsw = atof(v);
        else if (strcasecmp(key, "PwmSamples") == 0)
            args->PwmSamples = atoi(v);
        else if (strcasecmp(key, "Ti") == 0)
            args->Ti = atof(v);
        else if (strcasecmp(key, "Tf") == 0)
            args->Tf = atof(v);
        else if (strcasecmp(key, "Dt") == 0)
            args->Dt = atof(v);
        else if (strcasecmp(key, "Vdc") == 0)
            args->Vdc = atof(v);
        else if (strcasecmp(key, "KpOmega") == 0)
            args->KpOmega = atof(v);
        else if (strcasecmp(key, "KiOmega") == 0)
            args->KiOmega = atof(v);
        else if (strcasecmp(key, "KpId") == 0)
            args->KpId = atof(v);
        else if (strcasecmp(key, "KiId") == 0)
            args->KiId = atof(v);
        else if (strcasecmp(key, "KpIq") == 0)
            args->KpIq = atof(v);
        else if (strcasecmp(key, "KiIq") == 0)
            args->KiIq = atof(v);
        else if (strcasecmp(key, "rpm") == 0)
            args->rpm = atof(v);
        else if (strcasecmp(key, "VfStartup") == 0)
            args->UseVfStartup = parse_bool(v);
        else if (strcasecmp(key, "MalhaAberta") == 0)
            args->MalhaAberta = parse_bool(v);
        else if (strcasecmp(key, "Ttl") == 0)
            args->Ttl = atof(v);
        else if (strcasecmp(key, "Tlnew") == 0)
            args->Tlnew = atof(v);
        else if (strcasecmp(key, "file") == 0 || strcasecmp(key, "filename") == 0)
        {
            strncpy(args->filename, v, sizeof(args->filename) - 1);
            args->filename[sizeof(args->filename) - 1] = '\0';
        }
        else
        {
            fprintf(stderr,
                    "Aviso: linha %d do arquivo de configuracao ignorada "
                    "(chave desconhecida): '%s'\n",
                    line_no, key);
        }
    }

    fclose(f);
    return 0;
}

void parse_args(int argc, char **argv, sim_args_t *args)
{
    /* valores padrao */
    strncpy(args->filename, DEFAULT_OUTPUT_FILE, sizeof(args->filename) - 1);
    args->filename[sizeof(args->filename) - 1] = '\0';
    args->R = (double)DEFAULT_MOTOR_RS;
    args->L = (double)DEFAULT_MOTOR_L;
    args->M = (double)DEFAULT_MOTOR_M;
    args->Ke = (double)DEFAULT_MOTOR_KE;
    args->J = (double)DEFAULT_MOTOR_J;
    args->B = (double)DEFAULT_MOTOR_B;
    args->Tl = (double)DEFAULT_TL;
    args->P = DEFAULT_MOTOR_PARES_DE_POLOS;
    args->Kt = (double)DEFAULT_MOTOR_KT;
    args->Fsw = DEFAULT_SVPWM_HZ;
    args->PwmSamples = DEFAULT_PWM_SAMPLES;
    args->Ti = (double)DEFAULT_SIM_TI;
    args->Tf = (double)DEFAULT_SIM_TF;
    args->Dt = (double)DEFAULT_SIM_DT;
    args->Ttl = (double)DEFAULT_TLNEW_TIME;
    args->Tlnew = (double)DEFAULT_TLNEW_NM;
    args->Vdc = (double)DEFAULT_VDC;
    args->KpOmega = (double)DEFAULT_KP_OMEGA;
    args->KiOmega = (double)DEFAULT_KI_OMEGA;
    args->KpId = (double)DEFAULT_KP_ID;
    args->KiId = (double)DEFAULT_KI_ID;
    args->KpIq = (double)DEFAULT_KP_IQ;
    args->KiIq = (double)DEFAULT_KI_IQ;
    args->UseVfStartup = DEFAULT_USE_VF_STARTUP;
    args->MalhaAberta = DEFAULT_USE_MALHA_ABERTA;

    static struct option long_options[] =
        {
            {"config", required_argument, 0, 'c'},
            {"file", required_argument, 0, 'f'},
            {"R", required_argument, 0, 'R'},
            {"L", required_argument, 0, 'L'},
            {"M", required_argument, 0, 'M'},
            {"Ke", required_argument, 0, 'K'},
            {"J", required_argument, 0, 'J'},
            {"B", required_argument, 0, 'B'},
            {"Tl", required_argument, 0, 'T'},
            {"P", required_argument, 0, 'P'},
            {"Kt", required_argument, 0, 't'},
            {"fsw", required_argument, 0, 's'},
            {"pwm-samples", required_argument, 0, 'n'},
            {"ti", required_argument, 0, 'i'},
            {"tf", required_argument, 0, 'e'},
            {"dt", required_argument, 0, 'd'},
            {"Ttl", required_argument, 0, 's'},
            {"Tlnew", required_argument, 0, 'T'},
            {"vdc", required_argument, 0, OPT_VDC},
            {"kp-omega", required_argument, 0, OPT_KP_OMEGA},
            {"ki-omega", required_argument, 0, OPT_KI_OMEGA},
            {"kp-id", required_argument, 0, OPT_KP_ID},
            {"ki-id", required_argument, 0, OPT_KI_ID},
            {"kp-iq", required_argument, 0, OPT_KP_IQ},
            {"ki-iq", required_argument, 0, OPT_KI_IQ},
            {"vf-startup", required_argument, 0, OPT_VF_STARTUP},
            {"malha-aberta", required_argument, 0, OPT_MALHAABERTA_STARTUP},
            {"help", no_argument, 0, 'h'},
            {0, 0, 0, 0}};
    const char *optstring = "c:f:R:L:M:K:J:B:T:P:t:s:n:i:e:d:h";

    int opt;
    int option_index;

    /* ----------------------------------------------------------------
     * 1a passada: procura apenas por -c/--config, ignorando o restante,
     * para que o arquivo de configuracao seja carregado ANTES de
     * aplicarmos os argumentos explicitos da linha de comando.
     * ---------------------------------------------------------------- */
    char config_path[256] = "";

    opterr = 0; /* suprime mensagens de erro nesta passada */
    optind = 1;
    option_index = 0;
    while ((opt = getopt_long(argc, argv, optstring, long_options, &option_index)) != -1)
    {
        if (opt == 'c')
        {
            strncpy(config_path, optarg, sizeof(config_path) - 1);
            config_path[sizeof(config_path) - 1] = '\0';
        }
        /* demais opcoes/erros sao ignorados nesta passada */
    }

    if (config_path[0] != '\0')
    {
        printf("Carregando parametros do arquivo: %s\n", config_path);
        if (parse_config_file(config_path, args) != 0)
        {
            fprintf(stderr,
                    "Erro: nao foi possivel carregar o arquivo de configuracao '%s'\n",
                    config_path);
            exit(EXIT_FAILURE);
        }
    }

    /* ----------------------------------------------------------------
     * 2a passada: aplica normalmente os argumentos da linha de comando.
     * Qualquer valor passado aqui sobrescreve o default e o arquivo de
     * configuracao (prioridade: CLI > arquivo de config > default).
     * ---------------------------------------------------------------- */
    opterr = 1;
    optind = 1;
    option_index = 0;
    while ((opt = getopt_long(argc, argv, optstring, long_options, &option_index)) != -1)
    {
        switch (opt)
        {
        case 'c':
            /* ja tratado na 1a passada */
            break;
        case 'f':
            strncpy(args->filename, optarg, sizeof(args->filename) - 1);
            args->filename[sizeof(args->filename) - 1] = '\0';
            break;
        case 'R':
            args->R = atof(optarg);
            break;
        case 'L':
            args->L = atof(optarg);
            break;
        case 'M':
            args->M = atof(optarg);
            break;
        case 'K':
            args->Ke = atof(optarg);
            break;
        case 'J':
            args->J = atof(optarg);
            break;
        case 'B':
            args->B = atof(optarg);
            break;
        case 'T':
            args->Tl = atof(optarg);
            break;
        case 'P':
            args->P = atoi(optarg);
            break;
        case 't':
            args->Kt = atof(optarg);
            break;
        case 's':
            args->Fsw = atof(optarg);
            break;
        case 'n':
            args->PwmSamples = atoi(optarg);
            break;
        case 'i':
            args->Ti = atof(optarg);
            break;
        case 'e':
            args->Tf = atof(optarg);
            break;
        case 'd':
            args->Dt = atof(optarg);
            break;
        case OPT_VDC:
            args->Vdc = atof(optarg);
            break;
        case OPT_KP_OMEGA:
            args->KpOmega = atof(optarg);
            break;
        case OPT_KI_OMEGA:
            args->KiOmega = atof(optarg);
            break;
        case OPT_KP_ID:
            args->KpId = atof(optarg);
            break;
        case OPT_KI_ID:
            args->KiId = atof(optarg);
            break;
        case OPT_KP_IQ:
            args->KpIq = atof(optarg);
            break;
        case OPT_KI_IQ:
            args->KiIq = atof(optarg);
            break;
        case OPT_VF_STARTUP:
            args->UseVfStartup = parse_bool(optarg);
            break;
        case OPT_MALHAABERTA_STARTUP:
            args->MalhaAberta = parse_bool(optarg);
            break;
        case OPT_TTL:
            args->Ttl = atof(optarg);
            break;
        case OPT_TLNEW:
            args->Tlnew = atof(optarg);
            break;
        case 'h':
            usage(argv[0]);
            exit(EXIT_SUCCESS);
        default:
            usage(argv[0]);
            exit(EXIT_FAILURE);
        }
    }
}

#endif