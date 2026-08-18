/**
 * @brief Biblioteca simples de barra de progresso para simulacoes
 * (ex.: simulacoes de motores BLDC), baseada em caracteres e no
 * tamanho real do terminal (obtido via ioctl/TIOCGWINSZ).
 *
 * Uso tipico:
 *
 *      progress_bar_t pb;
 *      progress_bar_init(&pb, t0, tf, dt);
 *
 *      for (long k = 0; k < total_steps; k++) {
 *          float t = t0 + k * dt;
 *          ... passo da simulacao ...
 *          progress_bar_update(&pb, t);
 *      }
 *
 *      progress_bar_finish(&pb);
 */

#ifndef PROGRESSBAR_H
#define PROGRESSBAR_H

typedef struct
{
    float t0;           /* tempo inicial da simulacao            */
    float tf;           /* tempo final da simulacao               */
    float dt;           /* passo de integracao                    */

    long total_steps;   /* numero total de passos (derivado de dt)*/
    long current_step;  /* passo atual                            */

    int bar_width;       /* largura da barra em caracteres (ioctl) */
    int last_percent;    /* ultimo percentual calculado (throttle)  */

    int is_tty;          /* stdout e um terminal interativo?        */
    int last_milestone;  /* ultimo marco (%) impresso, modo nao-tty */

    double start_clock;  /* instante (monotonic) do inicio, p/ ETA */
} progress_bar_t;

/**
 * @brief Inicializa a barra de progresso a partir do tempo inicial,
 * tempo final e passo (dt) da simulacao. O numero total de passos e
 * calculado da mesma forma usada nas simulacoes:
 *
 *      total_steps = round((tf - t0) / dt) + 1
 *
 * A largura da barra e obtida dinamicamente com ioctl(TIOCGWINSZ)
 * sobre stdout; se nao houver terminal (saida redirecionada/pipe),
 * usa um valor padrao de 80 colunas.
 *
 * Tambem e feita a deteccao automatica de terminal via isatty():
 *   - Se stdout for um terminal interativo, a barra usa "\r" para
 *     ficar sempre na mesma linha (comportamento classico).
 *   - Se stdout NAO for um terminal (redirecionado para arquivo,
 *     pipe, nohup, screen/tmux desanexado, console de IDE, CI/CD),
 *     "\r" nao e interpretado por quem le a saida, entao a barra
 *     passa a imprimir apenas marcos (a cada 5%) em linhas normais
 *     terminadas com "\n" -- evitando encher o log/terminal de
 *     centenas de linhas em simulacoes grandes/longas.
 */
void progress_bar_init(progress_bar_t *pb, float t0, float tf, float dt);

/**
 * @brief Atualiza a barra de progresso com o tempo atual da
 * simulacao (t_current). Deve ser chamada a cada passo do laco de
 * simulacao; internamente a barra so e redesenhada quando o
 * percentual inteiro muda, para nao pesar no laco.
 */
void progress_bar_update(progress_bar_t *pb, float t_current);

/**
 * @brief Forca a barra a 100% e imprime uma linha final com o tempo
 * total decorrido. Deve ser chamada uma vez, apos o laco de
 * simulacao terminar.
 */
void progress_bar_finish(progress_bar_t *pb);

#endif /* PROGRESSBAR_H */
