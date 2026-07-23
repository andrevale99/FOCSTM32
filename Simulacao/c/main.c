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
#include <string.h>
#include <strings.h>
#include <getopt.h>

#include "PIcontroller.h"
#include "SVPWM.h"
#include "inverter.h"
#include "transforms.h"
#include "bldc.h"
#include "vf_startup.h"

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

#define PI_IQ_MAX 5.0
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
#define VF_STARTUP_V_BOOST 1.0f          /* V   - tensao de fase em omega_e=0   */
#define VF_STARTUP_V_PER_RAD_S 0.005f    /* V/(rad/s eletrico) - ganho V/F (a "razao" V/F,
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
#define DEFAULT_USE_VF_STARTUP 1

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
#define DEFAULT_SIM_TF 1.0f /* s */
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
    double Fsw;     /* frequencia de chaveamento do SVPWM [Hz]           */
    int PwmSamples; /* passos finos de simulacao por periodo Ts do PWM  */
    double Ti;      /* tempo inicial da simulacao [s]                   */
    double Tf;      /* tempo final da simulacao [s]                     */
    double Dt;      /* passo de integracao explicito [s] (0 = automatico) */
    double Vdc;     /* tensao do barramento CC [V]                       */
    double KpOmega; /* ganho proporcional - malha de velocidade          */
    double KiOmega; /* ganho integral - malha de velocidade              */
    double KpId;    /* ganho proporcional - malha de corrente id         */
    double KiId;    /* ganho integral - malha de corrente id             */
    double KpIq;    /* ganho proporcional - malha de corrente iq         */
    double KiIq;    /* ganho integral - malha de corrente iq             */
    double rpm;     /*referencia de velocidade*/
    int UseVfStartup; /* 1 = realiza partida V/F em malha aberta antes do FOC; 0 = FOC direto desde Ti */
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
    OPT_VF_STARTUP
};

/* Interpreta strings booleanas aceitas em CLI/arquivo de config para
 * a flag de partida V/F: "1"/"0", "true"/"false", "yes"/"no",
 * "on"/"off" (case-insensitive). Qualquer outra coisa cai no atoi(). */
static int parse_bool(const char *v)
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

static void usage(const char *prog)
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
            "  -h, --help          Mostra esta mensagem de ajuda\n\n"
            "Observacao: a simulacao reproduz o chaveamento REAL do inversor\n"
            "(comparacao do duty cycle com uma portadora triangular), nao um\n"
            "modelo de valor medio. Por isso, se -d/--dt nao for informado, o\n"
            "passo de integracao e derivado automaticamente de Fsw e PwmSamples\n"
            "(dt = 1/Fsw / PwmSamples), e mudar -s/--fsw tem efeito real no\n"
            "resultado (ripple de corrente, de torque, etc). Se -d/--dt for\n"
            "informado explicitamente, ele e usado no lugar do calculo\n"
            "automatico (util para comparar com um passo fixo), mas um aviso\n"
            "e emitido caso ele seja grande demais para resolver o chaveamento.\n",
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
            DEFAULT_USE_VF_STARTUP);
}

/* --------------------------------------------------------------------
 *   Le um arquivo texto de configuracao no formato "CHAVE=VALOR"
 *   (tambem aceita "CHAVE VALOR", separado por espaco/tab).
 *   Linhas em branco ou iniciadas com '#' sao ignoradas.
 *   Retorna 0 em caso de sucesso, -1 em caso de erro ao abrir o arquivo.
 * -------------------------------------------------------------------- */
static int parse_config_file(const char *path, sim_args_t *args)
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

static void parse_args(int argc, char **argv, sim_args_t *args)
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
    args->Vdc = (double)DEFAULT_VDC;
    args->KpOmega = (double)DEFAULT_KP_OMEGA;
    args->KiOmega = (double)DEFAULT_KI_OMEGA;
    args->KpId = (double)DEFAULT_KP_ID;
    args->KiId = (double)DEFAULT_KI_ID;
    args->KpIq = (double)DEFAULT_KP_IQ;
    args->KiIq = (double)DEFAULT_KI_IQ;
    args->UseVfStartup = DEFAULT_USE_VF_STARTUP;

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
            {"vdc", required_argument, 0, OPT_VDC},
            {"kp-omega", required_argument, 0, OPT_KP_OMEGA},
            {"ki-omega", required_argument, 0, OPT_KI_OMEGA},
            {"kp-id", required_argument, 0, OPT_KP_ID},
            {"ki-id", required_argument, 0, OPT_KI_ID},
            {"kp-iq", required_argument, 0, OPT_KP_IQ},
            {"ki-iq", required_argument, 0, OPT_KI_IQ},
            {"vf-startup", required_argument, 0, OPT_VF_STARTUP},
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
        case 'h':
            usage(argv[0]);
            exit(EXIT_SUCCESS);
        default:
            usage(argv[0]);
            exit(EXIT_FAILURE);
        }
    }
}

int main(int argc, char **argv)
{
    sim_args_t args;
    parse_args(argc, argv, &args);

    printf("Parametros da simulacao:\n");
    printf("  arquivo de saida = %s\n", args.filename);
    printf("  R  = %.6f Ohm\n", args.R);
    printf("  L  = %.6f H\n", args.L);
    printf("  M  = %.6f H\n", args.M);
    printf("  Ke = %.6f V/(rad/s)\n", args.Ke);
    printf("  J  = %.9f kg.m^2\n", args.J);
    printf("  B  = %.6f\n", args.B);
    printf("  Tl = %.6f N.m\n", args.Tl);
    printf("  P  = %d\n", args.P);
    printf("  Kt = %.6f N.m/A\n", args.Kt);
    printf("  Fsw = %.1f Hz (chaveamento SVPWM real)\n", args.Fsw);
    printf("  PwmSamples = %d passos finos por periodo Ts\n", args.PwmSamples);
    printf("  Ti = %.6f s\n", args.Ti);
    printf("  Tf = %.6f s\n", args.Tf);
    printf("  rpm = %.6f RPM\n", args.rpm);
    if (args.Dt > 0.0)
        printf("  Dt = %.9e s (explicito, sobrescreve o calculo automatico)\n", args.Dt);
    else
        printf("  Dt = automatico (Ts / PwmSamples)\n");
    printf("  Vdc = %.6f V\n", args.Vdc);
    printf("  Controlador de velocidade: Kp = %.6f | Ki = %.6f\n",
           args.KpOmega, args.KiOmega);
    printf("  Controlador de corrente id: Kp = %.6f | Ki = %.6f\n",
           args.KpId, args.KiId);
    printf("  Controlador de corrente iq: Kp = %.6f | Ki = %.6f\n",
           args.KpIq, args.KiIq);
    printf("  Partida V/F em malha aberta: %s\n\n",
           args.UseVfStartup ? "SIM" : "NAO (FOC direto desde Ti)");

    if (args.Fsw <= 0.0)
    {
        fprintf(stderr, "Erro: Fsw deve ser > 0.\n");
        return EXIT_FAILURE;
    }

    if (args.PwmSamples < 2)
    {
        fprintf(stderr, "Erro: PwmSamples deve ser >= 2.\n");
        return EXIT_FAILURE;
    }

    if (args.Tf <= args.Ti)
    {
        fprintf(stderr, "Erro: Tf deve ser maior que Ti.\n");
        return EXIT_FAILURE;
    }

    if (args.Dt < 0.0)
    {
        fprintf(stderr, "Erro: Dt nao pode ser negativo.\n");
        return EXIT_FAILURE;
    }

    if (args.Vdc <= 0.0)
    {
        fprintf(stderr, "Erro: Vdc deve ser > 0.\n");
        return EXIT_FAILURE;
    }

    /* --------------------------------------------------------------
     *   OBJETOS DA PLANTA
     * -------------------------------------------------------------- */
    bldc_t motor =
        {
            .iabc = {0.0f, 0.0f, 0.0f},

            .R = (float)args.R,
            .L = (float)args.L,
            .M = (float)args.M,
            .Ke = (float)args.Ke,

            .J = (float)args.J,
            .B = (float)args.B,
            .Te = 0.0f,

            .P = args.P,
            .Kt = (float)args.Kt,

            .theta_e = 0.0f,
            .theta_r = 0.0f,

            .omega_r = 0.0f,
            .omega_e = 0.0f,

            .log = NULL};

    svpwm_t pwm;
    if (!svpwm_init(&pwm, (float)args.Fsw, 0.0f, (float)args.Vdc))
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
    if (args.Dt > 0.0)
    {
        dt = (float)args.Dt;

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
        dt = pwm.Ts / (float)args.PwmSamples;
    }

    time_simulation_t time_sim =
        {
            .t0 = (float)args.Ti,
            .tf = (float)args.Tf,
            .dt = dt};

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

    inverter_t inverter = {.Vdc = (float)args.Vdc};

    /* --------------------------------------------------------------
     *   CONTROLADORES PI
     * -------------------------------------------------------------- */
    /* Os limites de saturacao das malhas de corrente (vd/vq) sao
     * +-Vdc, calculados a partir do parametro Vdc informado. */
    double vdc_max = args.Vdc;
    double vdc_min = -args.Vdc;

    double dtOmega = 5e-4;
    double dtId = 5e-4;
    double dtIq = 5e-4;

    /* --------------------------------------------------------------
     *   REFERENCIAS
     * -------------------------------------------------------------- */
    double id_ref = 0.0; /* motor de imas permanentes: referencia de eixo d = 0 */

    float rpm_ref = (float)args.rpm;
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
    float vf_omega_e_target = omega_ref * (float)motor.P;
    float vf_ramp_rate = vf_omega_e_target / VF_STARTUP_RAMP_TIME;

    printf("Partida V/F: alvo = %.6f rad/s eletrico (%.6f rad/s mecanico, "
           "= omega_ref), rampa em %.3f s\n",
           vf_omega_e_target, omega_ref, VF_STARTUP_RAMP_TIME);

    vf_startup_t vf;
    vf_startup_init(&vf,
                     VF_STARTUP_V_BOOST,
                     VF_STARTUP_V_PER_RAD_S,
                     (float)args.Vdc / 1.8f,
                     vf_ramp_rate,
                     vf_omega_e_target);

    PIController pi_omega, pi_d, pi_q;

    pi_controller_init(&pi_omega, args.KpOmega, args.KiOmega, dtOmega,
                       true, PI_IQ_MIN, true, PI_IQ_MAX);

    pi_controller_init(&pi_d, args.KpId, args.KiId, dtId,
                       true, vdc_min, true, vdc_max);

    pi_controller_init(&pi_q, args.KpIq, args.KiIq, dtIq,
                       true, vdc_min, true, vdc_max);

    /* --------------------------------------------------------------
     *   ARQUIVO DE LOG
     * -------------------------------------------------------------- */
    const char *filename = args.filename;
    FILE *log_file = fopen(filename, "w");

    if (log_file == NULL)
    {
        perror("Erro ao criar o arquivo de log");
        return EXIT_FAILURE;
    }

    fprintf(log_file,
            "time;mode;Va;Vb;Vc;ia;ib;ic;id;iq;Te;theta_r;omega_r;iq_ref;"
            "duty_a;duty_b;duty_c;carrier;gate_a;gate_b;gate_c\n");
    /* mode: 0 = partida V/F em malha aberta | 1 = FOC em malha fechada */

    /* --------------------------------------------------------------
     *   LACO DE SIMULACAO
     * -------------------------------------------------------------- */
    for (long k = 0; k < total_steps; k++)
    {
        float t = time_sim.t0 + (float)((double)k * (double)dt);
        if (t > time_sim.tf)
        {
            break;
        }

        /* Variaveis compartilhadas pelas duas fases (preenchidas de
         * um jeito ou de outro abaixo, e usadas no log ao final) */
        float v_alpha, v_beta, theta_e;
        float i_d = 0.0f, i_q = 0.0f;
        double iq_ref = 0.0;
        int mode;

        if (args.UseVfStartup && t < VF_STARTUP_DURATION)
        {
            /* ----------------------------------------------------------
             *   FASE 1: PARTIDA V/F EM MALHA ABERTA
             * ----------------------------------------------------------
             * theta_e eh sintetico (comeca em 0 rad / 0 graus) e nao vem
             * do modelo do motor: nao ha realimentacao de posicao nem
             * de corrente nesta fase. O vetor de tensao alpha-beta e
             * sintetizado diretamente por vf_startup_step(). */
            mode = 0;
            vf_startup_step(&vf, dt, &theta_e, &v_alpha, &v_beta);
        }
        else
        {
            /* ----------------------------------------------------------
             *   FASE 2: FOC EM MALHA FECHADA (controle vetorial)
             * ---------------------------------------------------------- */
            mode = 1;

            /* A. Medicao (feedback do modelo) */
            theta_e = motor.theta_r * (float)motor.P;

            /* B. Malha externa de velocidade -> referencia de iq */
            iq_ref = pi_controller_update(&pi_omega, omega_ref, motor.omega_r);

            /* C. Transformada de Clarke (abc -> alpha-beta) */
            float i_alpha, i_beta;
            clarke_transform(motor.iabc[0], motor.iabc[1], motor.iabc[2],
                             &i_alpha, &i_beta);

            /* Transformada de Park (alpha-beta -> dq) */
            park_transform(i_alpha, i_beta, theta_e, &i_d, &i_q);

            /* D. Malhas internas de corrente (PI em d e em q) */
            double vd_ref = pi_controller_update(&pi_d, id_ref, i_d);
            double vq_ref = pi_controller_update(&pi_q, iq_ref, i_q);

            /* E. Transformada inversa de Park (dq -> alpha-beta) */
            park_inverse_transform((float)vd_ref, (float)vq_ref, theta_e,
                                   &v_alpha, &v_beta);
        }

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
        float Vabc[3];
        inverter_output_voltage(&inverter, (float)gate_a, (float)gate_b,
                                (float)gate_c, Vabc);

        /* I. Atualizacao da planta (motor BLDC) */
        bldc_step(Vabc, &motor, &time_sim, (float)args.Tl, false);

        /* J. Log dos dados */
        fprintf(log_file,
                "%.6f;%d;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;"
                "%.6f;%.6f;%.6f;%.6f;%d;%d;%d\n",
                t, mode,
                Vabc[0], Vabc[1], Vabc[2],
                motor.iabc[0], motor.iabc[1], motor.iabc[2],
                i_d, i_q,
                motor.Te,
                motor.theta_r, motor.omega_r,
                iq_ref,
                duty_a, duty_b, duty_c,
                carrier,
                gate_a, gate_b, gate_c);
    }

    fclose(log_file);

    printf("Simulacao concluida. Resultados em \"%s\".\n", filename);

    return EXIT_SUCCESS;
}