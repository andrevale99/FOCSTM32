/*
 * vf_startup.h
 *
 * Partida em malha aberta por rampa V/F (tambem chamada I/F quando a
 * amplitude e regulada por corrente). Usada para "arrancar" o motor
 * PMSM/BLDC sem conhecimento da posicao real do rotor, ate que a
 * velocidade seja suficiente para um observador de fluxo (por
 * integracao de Valpha/Vbeta) conseguir estimar theta_e com SNR
 * razoavel.
 *
 * Premissa fundamental: o angulo eletrico usado para sintetizar o
 * vetor de tensao NAO vem do rotor real. Ele e um angulo sintetico,
 * que SEMPRE comeca em theta_e = 0 rad (0 graus) e evolui apenas
 * integrando uma frequencia eletrica comandada (rampa). O rotor e
 * "arrastado" por esse vetor girante -- exatamente como um motor de
 * inducao em partida direta, ou como a fase de "open-loop ramp" de
 * qualquer driver sensorless comercial.
 *
 * Por ser malha aberta, nao ha garantia de sincronismo: se a rampa
 * de aceleracao (ramp_rate) for agressiva demais para a inercia (J)
 * e o torque de carga (Tl) do sistema, o rotor perde o passo
 * (stall) e o angulo sintetico se descola do angulo real. Cabe ao
 * usuario escolher ramp_rate e V_boost compativeis com o motor.
 */

#ifndef VF_STARTUP_H
#define VF_STARTUP_H

#include <math.h>
#include <stdbool.h>

#ifndef VF_STARTUP_TWO_PI
#define VF_STARTUP_TWO_PI (2.0f * M_PI)
#endif

/**
 * @brief Estado do gerador de partida V/F em malha aberta.
 */
typedef struct
{
    /**
     * @brief Tensao minima de fase aplicada em omega_e = 0.
     *
     * Compensa a queda resistiva (R*i) e o torque de breakaway
     * necessarios para vencer o atrito estatico, ja que em omega=0
     * nao ha FCEM alguma para ajudar a produzir torque.
     */
    float V_boost;

    /**
     * @brief Ganho V/F: tensao adicional por rad/s eletrico.
     *
     * V_amp(omega_e) = V_boost + V_per_rad_s * |omega_e|
     */
    float V_per_rad_s;

    /**
     * @brief Amplitude maxima de tensao de fase permitida (V).
     *
     * Deve respeitar a regiao linear do SVPWM (tipicamente
     * Vdc/sqrt(3) para modulacao com injecao de terceiro harmonico).
     */
    float V_max;

    /**
     * @brief Taxa de aceleracao da rampa de frequencia eletrica,
     *        em rad/s^2 (elétrico).
     */
    float ramp_rate;

    /**
     * @brief Frequencia eletrica final da rampa, em rad/s.
     *
     * Ao ser atingida, a partida e considerada concluida e o
     * sistema pode comutar para malha fechada (sensor ou
     * observador de fluxo).
     */
    float omega_e_target;

    /* ---- Estado interno (nao inicializar manualmente) ---- */

    /**
     * @brief Frequencia eletrica comandada atual (rad/s), evolui
     *        em rampa ate omega_e_target.
     */
    float omega_e_cmd;

    /**
     * @brief Angulo eletrico sintetico acumulado (rad), sempre
     *        comeca em 0 e e mantido em [0, 2*pi).
     */
    float theta_e;

} vf_startup_t;

/**
 * @brief Inicializa o gerador de partida V/F.
 *
 * theta_e e omega_e_cmd sempre comecam zerados: o motor e
 * considerado na posicao eletrica 0 graus no instante em que a
 * partida se inicia.
 *
 * @param[out] vf Ponteiro para a estrutura a ser inicializada.
 * @param[in] V_boost Tensao minima de fase em omega_e = 0 (V).
 * @param[in] V_per_rad_s Ganho V/F (V por rad/s eletrico).
 * @param[in] V_max Amplitude maxima de tensao de fase (V).
 * @param[in] ramp_rate Taxa de aceleracao eletrica da rampa (rad/s^2).
 * @param[in] omega_e_target Frequencia eletrica final da rampa (rad/s).
 */
void vf_startup_init(
    vf_startup_t *vf,
    float V_boost,
    float V_per_rad_s,
    float V_max,
    float ramp_rate,
    float omega_e_target);

/**
 * @brief Executa um passo da rampa V/F em malha aberta.
 *
 * A cada chamada:
 *
 * 1. A frequencia eletrica comandada e incrementada (ou decrementada,
 *    se omega_e_target for negativo) em direcao a omega_e_target,
 *    respeitando ramp_rate;
 * 2. O angulo eletrico sintetico e integrado por Euler explicito e
 *    normalizado para [0, 2*pi);
 * 3. A amplitude de tensao e calculada pela reta V/F e saturada em
 *    V_max;
 * 4. O vetor de tensao alpha-beta e sintetizado diretamente a partir
 *    de theta_e (sem qualquer realimentacao de corrente ou posicao
 *    real do rotor).
 *
 * @param[in,out] vf Ponteiro para o estado da partida V/F.
 * @param[in] dt Passo de integracao, em segundos.
 * @param[out] theta_e_out Angulo eletrico sintetico resultante (rad).
 * @param[out] v_alpha_out Componente alfa do vetor de tensao (V).
 * @param[out] v_beta_out Componente beta do vetor de tensao (V).
 *
 * @return true se a rampa ja atingiu omega_e_target (partida
 *         concluida, pronto para comutar para malha fechada).
 */
bool vf_startup_step(
    vf_startup_t *vf,
    float dt,
    float *theta_e_out,
    float *v_alpha_out,
    float *v_beta_out);

#endif /* VF_STARTUP_H */