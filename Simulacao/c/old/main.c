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
#include "_SVPWM.h"
// #include "SVPWM.h"
// #include "inverter.h"
#include "_inverter.h"
#include "transforms.h"
#include "bldc.h"

/* ========================================================================
 *   PARAMETROS DA REDE / BARRAMENTO CC
 * ==================================================================== */
#define VDC   120.0f   /* V */

/* ========================================================================
 *   PARAMETROS DO MOTOR
 * ==================================================================== */
#define DEFAULT_MOTOR_RS    1.5f            /* Ohm - resistencia de armadura   */
#define DEFAULT_MOTOR_L     50e-3f          /* H   - indutancia de magnetizacao*/
#define DEFAULT_MOTOR_M     0.0f            /* H   - indutancia mutua          */
#define DEFAULT_MOTOR_KE    0.850f          /* constante eletrica (V/(rad/s))  */
#define DEFAULT_MOTOR_KT    0.850f          /* constante de torque (N.m/A)     */
#define DEFAULT_MOTOR_B     1e-3f           /* coeficiente de amortecimento    */
#define DEFAULT_MOTOR_J     0.0036013854f   /* momento de inercia (kg.m^2)     */
#define DEFAULT_MOTOR_PARES_DE_POLOS 4      /* numero de pares de polos (P)    */

#define DEFAULT_TL 0.0f /* torque de carga (N.m) */

#define DEFAULT_OUTPUT_FILE "closedloop_simulation.csv"

/* ========================================================================
 *   LIMITES DOS CONTROLADORES
 * ==================================================================== */
#define VDC_MAX  (VDC)
#define VDC_MIN  (-VDC)

#define PI_IQ_MAX 50.0
#define PI_IQ_MIN (-PI_IQ_MAX)

/* ========================================================================
 *   PARAMETROS DA SIMULACAO
 * ==================================================================== */
#define SIM_TI 0.0f     /* s */
#define SIM_TF 1.0f     /* s */
#define SIM_DT 1e-5f    /* s */

/* ========================================================================
 *   ARGUMENTOS DE LINHA DE COMANDO
 * ==================================================================== */
typedef struct
{
    char   filename[256];
    double R;
    double L;
    double M;
    double Ke;
    double J;
    double B;
    double Tl;
    int    P;
    double Kt;
} sim_args_t;

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
        "  -h, --help          Mostra esta mensagem de ajuda\n",
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
        (double)DEFAULT_MOTOR_KT);
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

        char key[64]  = {0};
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
        else if (strcasecmp(key, "file") == 0 || strcasecmp(key, "filename") == 0)
        {
            strncpy(args->filename, v, sizeof(args->filename) - 1);
            args->filename[sizeof(args->filename) - 1] = '\0';
        }
        else
        {
            fprintf(stderr,
                    "Aviso: linha %d do arquivo de configuracao ignorada "
                    "(chave desconhecida): '%s'\n", line_no, key);
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
    args->R  = (double)DEFAULT_MOTOR_RS;
    args->L  = (double)DEFAULT_MOTOR_L;
    args->M  = (double)DEFAULT_MOTOR_M;
    args->Ke = (double)DEFAULT_MOTOR_KE;
    args->J  = (double)DEFAULT_MOTOR_J;
    args->B  = (double)DEFAULT_MOTOR_B;
    args->Tl = (double)DEFAULT_TL;
    args->P  = DEFAULT_MOTOR_PARES_DE_POLOS;
    args->Kt = (double)DEFAULT_MOTOR_KT;

    static struct option long_options[] =
    {
        {"config", required_argument, 0, 'c'},
        {"file",   required_argument, 0, 'f'},
        {"R",      required_argument, 0, 'R'},
        {"L",      required_argument, 0, 'L'},
        {"M",      required_argument, 0, 'M'},
        {"Ke",     required_argument, 0, 'K'},
        {"J",      required_argument, 0, 'J'},
        {"B",      required_argument, 0, 'B'},
        {"Tl",     required_argument, 0, 'T'},
        {"P",      required_argument, 0, 'P'},
        {"Kt",     required_argument, 0, 't'},
        {"help",   no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    const char *optstring = "c:f:R:L:M:K:J:B:T:P:t:h";

    int opt;
    int option_index;

    /* ----------------------------------------------------------------
     * 1a passada: procura apenas por -c/--config, ignorando o restante,
     * para que o arquivo de configuracao seja carregado ANTES de
     * aplicarmos os argumentos explicitos da linha de comando.
     * ---------------------------------------------------------------- */
    char config_path[256] = "";

    opterr = 0;   /* suprime mensagens de erro nesta passada */
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
            case 'R': args->R  = atof(optarg); break;
            case 'L': args->L  = atof(optarg); break;
            case 'M': args->M  = atof(optarg); break;
            case 'K': args->Ke = atof(optarg); break;
            case 'J': args->J  = atof(optarg); break;
            case 'B': args->B  = atof(optarg); break;
            case 'T': args->Tl = atof(optarg); break;
            case 'P': args->P  = atoi(optarg); break;
            case 't': args->Kt = atof(optarg); break;
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
    printf("  Kt = %.6f N.m/A\n\n", args.Kt);

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

        .log = NULL
    };

    time_simulation_t time_sim =
    {
        .t0 = SIM_TI,
        .tf = SIM_TF,
        .dt = SIM_DT
    };

    svpwm_t pwm;
    svpwm_init(&pwm, 1000.0f, 0.0f, VDC); /* Hz = 10 kHz, Ts calculado internamente */

    inverter_t inverter = { .Vdc = VDC };

    /* --------------------------------------------------------------
     *   CONTROLADORES PI
     * -------------------------------------------------------------- */
    double dtOmega = 5e-4, kpOmega = 2.0,  kiOmega = 1.5;
    double dtId    = 5e-4, kpId    = 6.0, kiId    = 2.0;
    double dtIq    = 5e-4, kpIq    = 6.0, kiIq    = 2.0;

    PIController pi_omega, pi_d, pi_q;

    pi_controller_init(&pi_omega, kpOmega, kiOmega, dtOmega,
                        true, PI_IQ_MIN, true, PI_IQ_MAX);

    pi_controller_init(&pi_d, kpId, kiId, dtId,
                        true, VDC_MIN, true, VDC_MAX);

    pi_controller_init(&pi_q, kpIq, kiIq, dtIq,
                        true, VDC_MIN, true, VDC_MAX);

    /* --------------------------------------------------------------
     *   REFERENCIAS
     * -------------------------------------------------------------- */
    double id_ref = 0.0; /* motor de imas permanentes: referencia de eixo d = 0 */

    float rpm_ref = 25.0f;
    float omega_ref = rpm_to_rads(rpm_ref);
    printf("omega_ref = %.6f rad/s\n", omega_ref);

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
            "time;Va;Vb;Vc;ia;ib;ic;id;iq;Te;theta_r;omega_r;iq_ref;duty_a;duty_b;duty_c\n");

    /* --------------------------------------------------------------
     *   LACO DE SIMULACAO
     * -------------------------------------------------------------- */
    for (float t = time_sim.t0; t <= time_sim.tf; t += time_sim.dt)
    {
        /* A. Medicao (feedback do modelo) */
        float theta_e = motor.theta_r * (float)motor.P;

        /* B. Malha externa de velocidade -> referencia de iq */
        double iq_ref = pi_controller_update(&pi_omega, omega_ref, motor.omega_r);

        /* C. Transformada de Clarke (abc -> alpha-beta) */
        float i_alpha, i_beta;
        clarke_transform(motor.iabc[0], motor.iabc[1], motor.iabc[2],
                          &i_alpha, &i_beta);

        /* Transformada de Park (alpha-beta -> dq) */
        float i_d, i_q;
        park_transform(i_alpha, i_beta, theta_e, &i_d, &i_q);

        /* D. Malhas internas de corrente (PI em d e em q) */
        double vd_ref = pi_controller_update(&pi_d, id_ref, i_d);
        double vq_ref = pi_controller_update(&pi_q, iq_ref, i_q);

        /* E. Transformada inversa de Park (dq -> alpha-beta) */
        float v_alpha, v_beta;
        park_inverse_transform((float)vd_ref, (float)vq_ref, theta_e,
                                &v_alpha, &v_beta);

        /* F. SVPWM: duty cycles das 3 fases */
        float duty_a, duty_b, duty_c;
        svpwm_modulate(&pwm, v_alpha, v_beta, &duty_a, &duty_b, &duty_c);

        /* G. Inversor: tensoes de fase aplicadas ao motor */
        float Vabc[3];
        inverter_output_voltage(&inverter, duty_a, duty_b, duty_c, Vabc);

        /* H. Atualizacao da planta (motor BLDC) */
        bldc_step(Vabc, &motor, &time_sim, (float)args.Tl, false);

        /* I. Log dos dados */
        fprintf(log_file,
                "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n",
                t,
                Vabc[0], Vabc[1], Vabc[2],
                motor.iabc[0], motor.iabc[1], motor.iabc[2],
                i_d, i_q,
                motor.Te,
                motor.theta_r, motor.omega_r,
                iq_ref,
                duty_a, duty_b, duty_c);
    }

    fclose(log_file);

    printf("Simulacao concluida. Resultados em \"%s\".\n", filename);

    return EXIT_SUCCESS;
}