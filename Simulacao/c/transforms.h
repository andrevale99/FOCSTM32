/*
 * transforms.h
 *
 * Transformadas de Clarke e Park (diretas e inversa), utilizadas para
 * converter as correntes/tensoes trifasicas (abc) para o referencial
 * estacionario (alpha-beta) e para o referencial sincrono (dq), e
 * vice-versa. Necessarias para fechar a malha de corrente do BLDC/PMSM
 * em coordenadas dq, como feito em simulation_closedloop.py.
 */

#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <math.h>

#ifndef TRANSFORMS_SQRT3_OVER_2
#define TRANSFORMS_SQRT3_OVER_2 (0.8660254037844386f)
#endif

/**
 * @brief Transformada de Clarke (abc -> alpha-beta), invariante em amplitude.
 *
 * ialpha = (2/3) * (ia - ib/2 - ic/2)
 * ibeta  = (2/3) * (sqrt(3)/2) * (ib - ic)
 */
static inline void clarke_transform(
    float ia, float ib, float ic,
    float *ialpha, float *ibeta)
{
    *ialpha = (2.0f / 3.0f) * (ia - 0.5f * ib - 0.5f * ic);
    *ibeta  = (2.0f / 3.0f) * TRANSFORMS_SQRT3_OVER_2 * (ib - ic);
}

/**
 * @brief Transformada inversa de Clarke (alpha-beta -> abc).
 *
 * Assume-se que o sistema trifasico e equilibrado:
 *
 * ia + ib + ic = 0
 *
 * As equacoes utilizadas sao:
 *
 * ia = ialpha
 *
 * ib = -0.5 * ialpha + (sqrt(3)/2) * ibeta
 *
 * ic = -0.5 * ialpha - (sqrt(3)/2) * ibeta
 */
static inline void clarke_inverse_transform(
    float ialpha, float ibeta,
    float *ia, float *ib, float *ic)
{
    *ia = ialpha;

    *ib = -0.5f * ialpha +
          TRANSFORMS_SQRT3_OVER_2 * ibeta;

    *ic = -0.5f * ialpha -
          TRANSFORMS_SQRT3_OVER_2 * ibeta;
}

/**
 * @brief Transformada de Park (alpha-beta -> dq).
 *
 * id =  ialpha*cos(theta) + ibeta*sin(theta)
 * iq = -ialpha*sin(theta) + ibeta*cos(theta)
 */
static inline void park_transform(
    float ialpha, float ibeta, float theta,
    float *id, float *iq)
{
    float c = cosf(theta);
    float s = sinf(theta);

    *id =  ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}

/**
 * @brief Transformada inversa de Park (dq -> alpha-beta).
 *
 * valpha = vd*cos(theta) - vq*sin(theta)
 * vbeta  = vd*sin(theta) + vq*cos(theta)
 */
static inline void park_inverse_transform(
    float vd, float vq, float theta,
    float *valpha, float *vbeta)
{
    float c = cosf(theta);
    float s = sinf(theta);

    *valpha = vd * c - vq * s;
    *vbeta  = vd * s + vq * c;
}

#endif /* TRANSFORMS_H */