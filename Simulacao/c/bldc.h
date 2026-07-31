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
     * @brief Correntes das fases A, B e C, em amperes.
     */
    float iabc[JUST_THREE_PHASES];

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
     * @brief Torque eletromagnetico desenvolvido pelo motor, em N.m.
     */
    float Te;

    /**
     * @brief Numero de pares de polos.
     */
    const uint8_t P;

    /**
     * @brief Constante de torque do motor.
     */
    const float Kt;

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

    /**
     * @brief Arquivo utilizado para o registro dos dados da simulacao.
     */
    FILE *log;

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
 * @brief Calcula a forma de onda trapezoidal normalizada da FEM.
 *
 * Calcula uma funcao periodica de amplitude normalizada entre -1 e 1,
 * com formato trapezoidal, a partir da posicao angular eletrica.
 *
 * @param[in] theta Posicao angular eletrica, em radianos.
 *
 * @return Valor normalizado da forca contraeletromotriz no intervalo
 *         aproximado de -1 a 1.
 *
 * @note O angulo e normalizado para o intervalo [0, 2pi).
 */
static float trapezoidal_back_emf(float theta)
{
    theta = fmodf(theta, TWO_PI);

    if (theta < 0.0f)
    {
        theta += TWO_PI;
    }

    if (theta < M_PI / 6.0f)
    {
        return 6.0f * theta / M_PI;
    }
    else if (theta < 5.0f * M_PI / 6.0f)
    {
        return 1.0f;
    }
    else if (theta < 7.0f * M_PI / 6.0f)
    {
        return -6.0f * theta / M_PI + 6.0f;
    }
    else if (theta < 11.0f * M_PI / 6.0f)
    {
        return -1.0f;
    }
    else
    {
        return 6.0f * theta / M_PI - 12.0f;
    }
}

/**
 * @brief Cria o arquivo de log e escreve o cabecalho dos dados.
 *
 * Cria ou sobrescreve o arquivo especificado e escreve os nomes das
 * variaveis que serao registradas durante a simulacao.
 *
 * @param[in] filename Nome do arquivo de log.
 *
 * @note Caso nao seja possivel criar o arquivo, uma mensagem de erro
 *       e exibida por meio de @c perror().
 */
void bldc_log_header(const char *filename)
{
    FILE *file = fopen(filename, "w");

    if (file == NULL)
    {
        perror("Erro ao criar o arquivo de log");
        return;
    }

    fprintf(file,
            "time;"
            "ia;"
            "ib;"
            "ic;"
            "omega_r;"
            "theta_r;"
            "Te\n");

    fclose(file);
}

/**
 * @brief Registra o estado atual do motor no arquivo de log.
 *
 * Escreve no arquivo associado ao motor os valores instantaneos
 * das correntes de fase, velocidade mecanica, posicao mecanica
 * e torque eletromagnetico.
 *
 * @param[in] motor Ponteiro para o modelo do motor BLDC.
 * @param[in] time Instante atual da simulacao, em segundos.
 *
 * @warning O membro @p motor->log deve apontar para um arquivo
 *          previamente aberto para escrita.
 */
void bldc_log_data(bldc_t *motor, float time)
{
    fprintf(motor->log,
            "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f\n",
            time,
            motor->iabc[0],
            motor->iabc[1],
            motor->iabc[2],
            motor->omega_r,
            motor->theta_r,
            motor->Te);
}

/**
 * @brief Converte velocidade angular de rad/s para rpm.
 *
 * @param[in] omega Velocidade angular em rad/s.
 *
 * @return Velocidade em rotacoes por minuto (rpm).
 */
float rads_to_rpm(float omega)
{
    return (omega * 60 / TWO_PI);
}

/**
 * @brief Converte velocidade de rpm para velocidade angular.
 *
 * @param[in] rpm Velocidade em rotacoes por minuto.
 *
 * @return Velocidade angular em rad/s.
 */
float rpm_to_rads(float rpm)
{
    return (rpm * TWO_PI / 60);
}

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
               bool trapezoidal_back_emf_flag)
{
    float fabc[JUST_THREE_PHASES] = {0};
    float eabc[JUST_THREE_PHASES] = {0};
    float diabc[JUST_THREE_PHASES] = {0};
    float domega_r = 0;

    motor->theta_e = motor->P * motor->theta_r;
    motor->theta_e = fmodf(motor->P * motor->theta_r, TWO_PI);

    if (motor->theta_e < 0.0f)
    {
        motor->theta_e += TWO_PI;
    }

    motor->omega_e = motor->P * motor->omega_r;

    if (trapezoidal_back_emf_flag)
    {
        fabc[0] = -trapezoidal_back_emf(motor->theta_e + PHI_A);
        fabc[1] = -trapezoidal_back_emf(motor->theta_e + PHI_B);
        fabc[2] = -trapezoidal_back_emf(motor->theta_e + PHI_C);
    }
    else
    {
        fabc[0] = -sinf(motor->theta_e + PHI_A);
        fabc[1] = -sinf(motor->theta_e + PHI_B);
        fabc[2] = -sinf(motor->theta_e + PHI_C);
    }

    eabc[0] = motor->Ke * motor->omega_r * fabc[0];
    eabc[1] = motor->Ke * motor->omega_r * fabc[1];
    eabc[2] = motor->Ke * motor->omega_r * fabc[2];

    diabc[0] = (Vabc[0] - motor->R * motor->iabc[0] - eabc[0]) / (motor->L + motor->M);
    diabc[1] = (Vabc[1] - motor->R * motor->iabc[1] - eabc[1]) / (motor->L + motor->M);
    diabc[2] = (Vabc[2] - motor->R * motor->iabc[2] - eabc[2]) / (motor->L + motor->M);

    motor->iabc[0] += diabc[0] * time->dt;
    motor->iabc[1] += diabc[1] * time->dt;
    motor->iabc[2] += diabc[2] * time->dt;

    motor->Te = motor->Kt * (motor->iabc[0] * fabc[0] +
                             motor->iabc[1] * fabc[1] +
                             motor->iabc[2] * fabc[2]);

    domega_r = (motor->Te - Tl - motor->omega_r * motor->B) / motor->J;

    motor->omega_r += domega_r * time->dt;
    motor->theta_r += motor->omega_r * time->dt;
}

#endif