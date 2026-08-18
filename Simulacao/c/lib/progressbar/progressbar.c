/**
 * @brief Implementacao da barra de progresso baseada em caracteres.
 *
 * A largura do terminal e obtida via ioctl(fd, TIOCGWINSZ, &w), que
 * preenche uma struct winsize com o numero de colunas/linhas atuais
 * do terminal associado ao descritor informado (aqui, stdout).
 */

/* necessario para expor clock_gettime()/CLOCK_MONOTONIC no <time.h>
 * quando se compila com -std=c11 (ISO C puro, sem extensoes POSIX) */
#define _POSIX_C_SOURCE 199309L

#include "progressbar.h"

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>

/* Espaco reservado na linha para o texto ao redor da barra:
 * "[..] 100% (000000/000000) decorrido 00:00:00 ETA 00:00:00" */
#define PROGRESS_BAR_RESERVED_COLS 55
#define PROGRESS_BAR_MIN_WIDTH 10
#define PROGRESS_BAR_MAX_WIDTH 100
#define PROGRESS_BAR_FALLBACK_TERM_WIDTH 80

/* No modo "nao-tty" (arquivo/pipe/nohup/CI), so imprime a cada N%
 * de progresso, em vez de a cada mudanca de percentual inteiro. */
#define PROGRESS_BAR_LOG_MILESTONE_STEP 5

static int progress_bar_get_terminal_width(void)
{
    struct winsize w;

    /* TIOCGWINSZ: pede ao driver do terminal o tamanho atual da
     * janela (colunas/linhas). So funciona se stdout for um TTY;
     * se a saida estiver redirecionada para arquivo/pipe, ioctl
     * falha e usamos um valor padrao. */
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0 && w.ws_col > 0)
        return (int)w.ws_col;

    return PROGRESS_BAR_FALLBACK_TERM_WIDTH;
}

static double progress_bar_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void progress_bar_format_time(double seconds, char *buf, size_t buflen)
{
    if (seconds < 0.0)
        seconds = 0.0;

    int h = (int)(seconds / 3600.0);
    int m = (int)((seconds - (double)h * 3600.0) / 60.0);
    int s = (int)(seconds - (double)h * 3600.0 - (double)m * 60.0);

    snprintf(buf, buflen, "%02d:%02d:%02d", h, m, s);
}

void progress_bar_init(progress_bar_t *pb, float t0, float tf, float dt)
{
    pb->t0 = t0;
    pb->tf = tf;
    pb->dt = dt;

    /* mesmo calculo de total_steps usado no laco de simulacao */
    pb->total_steps = (long)((double)(tf - t0) / (double)dt + 0.5) + 1;
    if (pb->total_steps < 1)
        pb->total_steps = 1;

    pb->current_step = 0;
    pb->last_percent = -1;
    pb->last_milestone = -1;

    /* isatty() diz se o descritor esta conectado a um terminal real.
     * Se a saida foi redirecionada (> arquivo, | tee, nohup, CI,
     * console de IDE etc.), isatty() retorna 0 e a barra passa para
     * o modo "log" (marcos com \n) em vez do modo "\r". */
    pb->is_tty = isatty(STDOUT_FILENO);

    int term_width = progress_bar_get_terminal_width();

    pb->bar_width = term_width - PROGRESS_BAR_RESERVED_COLS;
    if (pb->bar_width < PROGRESS_BAR_MIN_WIDTH)
        pb->bar_width = PROGRESS_BAR_MIN_WIDTH;
    if (pb->bar_width > PROGRESS_BAR_MAX_WIDTH)
        pb->bar_width = PROGRESS_BAR_MAX_WIDTH;

    pb->start_clock = progress_bar_now();

    if (!pb->is_tty)
    {
        printf("Progresso da simulacao (saida nao interativa detectada; "
               "reportando a cada %d%%):\n",
               PROGRESS_BAR_LOG_MILESTONE_STEP);
    }
}

void progress_bar_update(progress_bar_t *pb, float t_current)
{
    long step = (long)((double)(t_current - pb->t0) / (double)pb->dt + 0.5);

    if (step > pb->total_steps)
        step = pb->total_steps;
    if (step < 0)
        step = 0;

    pb->current_step = step;

    int percent = (int)((100.0 * (double)pb->current_step) / (double)pb->total_steps);

    /* Redesenha somente quando o percentual inteiro muda, evitando
     * I/O excessivo em lacos com milhoes de passos. */
    if (percent == pb->last_percent)
        return;

    pb->last_percent = percent;

    /* Modo "log": stdout NAO e um terminal (arquivo, pipe, nohup,
     * CI, console de IDE...). "\r" nao seria interpretado por quem
     * le a saida, entao so imprimimos marcos a cada N%, cada um em
     * sua propria linha (\n) -- assim o log fica com um numero
     * pequeno e previsivel de linhas, mesmo em simulacoes enormes. */
    if (!pb->is_tty)
    {
        if (percent < pb->last_milestone + PROGRESS_BAR_LOG_MILESTONE_STEP &&
            percent != 100)
            return;

        pb->last_milestone = percent;

        double elapsed = progress_bar_now() - pb->start_clock;
        double eta = 0.0;
        if (pb->current_step > 0)
        {
            eta = elapsed *
                  ((double)(pb->total_steps - pb->current_step) /
                   (double)pb->current_step);
        }

        char elapsed_str[16], eta_str[16];
        progress_bar_format_time(elapsed, elapsed_str, sizeof(elapsed_str));
        progress_bar_format_time(eta, eta_str, sizeof(eta_str));

        printf("  %3d%% (%ld/%ld) decorrido %s ETA %s\n",
               percent, pb->current_step, pb->total_steps, elapsed_str, eta_str);

        fflush(stdout);
        return;
    }

    /* Modo interativo: redesenha a mesma linha com "\r". */
    int filled = (int)(((double)percent / 100.0) * (double)pb->bar_width);
    if (filled > pb->bar_width)
        filled = pb->bar_width;

    double elapsed = progress_bar_now() - pb->start_clock;
    double eta = 0.0;
    if (pb->current_step > 0)
    {
        eta = elapsed *
              ((double)(pb->total_steps - pb->current_step) /
               (double)pb->current_step);
    }

    char elapsed_str[16], eta_str[16];
    progress_bar_format_time(elapsed, elapsed_str, sizeof(elapsed_str));
    progress_bar_format_time(eta, eta_str, sizeof(eta_str));

    printf("\r[");
    for (int i = 0; i < pb->bar_width; i++)
        putchar(i < filled ? '#' : '-');

    printf("] %3d%% (%ld/%ld) decorrido %s ETA %s",
           percent, pb->current_step, pb->total_steps, elapsed_str, eta_str);

    fflush(stdout);
}

void progress_bar_finish(progress_bar_t *pb)
{
    pb->current_step = pb->total_steps;
    pb->last_percent = 100;

    double elapsed = progress_bar_now() - pb->start_clock;
    char elapsed_str[16];
    progress_bar_format_time(elapsed, elapsed_str, sizeof(elapsed_str));

    if (!pb->is_tty)
    {
        printf("  100%% (%ld/%ld) concluido em %s\n",
               pb->total_steps, pb->total_steps, elapsed_str);
        fflush(stdout);
        return;
    }

    printf("\r[");
    for (int i = 0; i < pb->bar_width; i++)
        putchar('#');

    printf("] 100%% (%ld/%ld) concluido em %s\n",
           pb->total_steps, pb->total_steps, elapsed_str);

    fflush(stdout);
}
