#ifndef BLDC_H
#define BLDC_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/**
 * @brief Numero de fases do motor BLDC.
 */
#define JUST_THREE_PHASES 3

/**
 * @brief Uma revolucao eletrica completa em radianos.
 */
#define TWO_PI (2 * M_PI)

/**
 * @brief Posicao angular da fase A.
 */
#define PHI_A 0

/**
 * @brief Posicao angular da fase B.
 */
#define PHI_B (float)(-TWO_PI / 3.0f)

/**
 * @brief Posicao angular da fase C.
 */
#define PHI_C (float)(TWO_PI / 3.0f)

/**
 * @brief Modelo de simulacao do motor BLDC.
 *
 * Armazena os parametros eletricos e mecanicos do motor, bem como
 * as variaveis de estado utilizadas durante a simulacao.
 */
typedef struct _bldc
{
    /**
     * @brief Resistencia dos enrolamentos, em ohms.
     */
    const float R;

    /**
     * @brief Indutancia propria dos enrolamentos, em henrys.
     */
    const float L;

    /**
     * @brief Indutancia mutua entre os enrolamentos, em henrys.
     *
     * @note Atualmente este parametro e armazenado na estrutura,
     *       mas nao e utilizado diretamente em @ref bldc_step().
     */
    const float M;

    /**
     * @brief Constante da forca contraeletromotriz.
     */
    const float Ke;

    /**
     * @brief Momento de inercia do rotor, em kg.m2.
     */
    const float J;

    /**
     * @brief Coeficiente de atrito viscoso.
     */
    const float B;

    /**
     * @brief Numero de pares de polos.
     */
    const uint8_t P;

    /**
     * @brief Constante de torque do motor.
     */
    const float Kt;
    
    /**
     * @brief Correntes das fases A, B e C, em amperes.
     */
    float iabc[JUST_THREE_PHASES];
    
    /**
     * @brief Tensoes induzidas ea, eb e ec, em Volts.
     */
    float eabc[JUST_THREE_PHASES];


    /**
     * @brief Torque eletromagnetico desenvolvido pelo motor, em N.m.
     */
    float Te;

    /**
     * @brief Posicao angular eletrica do rotor, em radianos.
     */
    float theta_e;

    /**
     * @brief Posicao angular mecanica do rotor, em radianos.
     */
    float theta_r;

    /**
     * @brief Velocidade angular mecanica, em rad/s.
     */
    float omega_r;

    /**
     * @brief Velocidade angular eletrica, em rad/s.
     */
    float omega_e;

} bldc_t;

/**
 * @brief Configuracao da simulacao temporal.
 */
typedef struct _time_simulation
{
    /**
     * @brief Instante inicial da simulacao, em segundos.
     */
    float t0;

    /**
     * @brief Instante final da simulacao, em segundos.
     */
    float tf;

    /**
     * @brief Passo de integracao numerica, em segundos.
     */
    float dt;

} time_simulation_t;

/**
 * @brief Converte velocidade angular de rad/s para rpm.
 *
 * @param[in] omega Velocidade angular em rad/s.
 *
 * @return Velocidade em rotacoes por minuto (rpm).
 */
float rads_to_rpm(float omega);

/**
 * @brief Converte velocidade de rpm para velocidade angular.
 *
 * @param[in] rpm Velocidade em rotacoes por minuto.
 *
 * @return Velocidade angular em rad/s.
 */
float rpm_to_rads(float rpm);

/**
 * @brief Executa um passo de integracao do modelo do motor BLDC.
 *
 * Atualiza o estado eletrico e mecanico do motor durante um intervalo
 * de tempo definido pelo passo de simulacao.
 *
 * A funcao realiza, nesta ordem:
 *
 * 1. Calculo da posicao angular eletrica;
 * 2. Calculo da velocidade angular eletrica;
 * 3. Calculo da forma de onda da FEM;
 * 4. Calculo das forcas contraeletromotrizes das tres fases;
 * 5. Calculo das derivadas das correntes;
 * 6. Integracao das correntes das fases;
 * 7. Calculo do torque eletromagnetico;
 * 8. Calculo da aceleracao angular;
 * 9. Atualizacao da velocidade mecanica;
 * 10. Atualizacao da posicao mecanica.
 *
 * A integracao numerica das variaveis de estado e realizada pelo
 * metodo de Euler explicito.
 *
 * @param[in] Vabc Vetor de tensoes aplicadas as fases A, B e C, em volts.
 * @param[in,out] motor Ponteiro para o modelo do motor BLDC.
 * @param[in] time Ponteiro para a configuracao da simulacao.
 * @param[in] Tl Torque de carga aplicado ao eixo, em N.m.
 * @param[in] trapezoidal_back_emf_flag
 *        Seleciona o formato da forca contraeletromotriz:
 *        - @c true: FEM trapezoidal;
 *        - @c false: FEM senoidal.
 *
 * @note A posicao eletrica e calculada por:
 *       @f[
 *       \theta_e = P\theta_r
 *       @f]
 *
 * @note A velocidade eletrica e calculada por:
 *       @f[
 *       \omega_e = P\omega_r
 *       @f]
 *
 * @note O modelo eletrico utiliza:
 *       @f[
 *       \frac{di}{dt} =
 *       \frac{V - Ri - e}{L}
 *       @f]
 *
 * @note A dinamica mecanica utiliza:
 *       @f[
 *       J\frac{d\omega_r}{dt}
 *       =
 *       T_e - T_L - B\omega_r
 *       @f]
 *
 * @warning Os ponteiros @p Vabc, @p motor e @p time devem ser validos.
 */
void bldc_step(float Vabc[JUST_THREE_PHASES],
               bldc_t *motor,
               time_simulation_t *time,
               float Tl,
               bool trapezoidal_back_emf_flag);

#endif
