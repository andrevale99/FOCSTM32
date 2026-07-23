#ifndef SVPWM_H
#define SVPWM_H

#include <math.h>
#include <stdbool.h>

#include "transforms.h"

#define SVPWM_TWO_PI (2.0f * M_PI)
#define SVPWM_SQRT3_OVER_2 (0.8660254037844386f)

/**

* @brief Estrutura do modulador SVPWM.
  */
typedef struct
{
  /**

  * @brief Tensão do barramento CC.
    */
  float Vdc;

  /**

  * @brief Período de chaveamento.
    */
  float Ts;

  /**

  * @brief Frequência de chaveamento.
    */
  float Hz;

} svpwm_t;

/**

* @brief Resultado da identificação do setor do vetor espacial.
  */
typedef struct
{
  /**

  * @brief Setor do vetor espacial, de 1 a 6.
    */
  int sector;

  /**

  * @brief Ângulo do vetor em radianos.
    */
  float angle;

  /**

  * @brief Magnitude do vetor.
    */
  float magnitude;

} svpwm_sector_t;

/**

* @brief Limita um valor a um intervalo.
  */
static inline float svpwm_clamp(
    float value,
    float min,
    float max)
{
  if (value < min)
  {
    return min;
  }

  if (value > max)
  {
    return max;
  }

  return value;
}

/**

* @brief Inicializa o modulador SVPWM.
*
* A frequência ou o período de chaveamento deve ser informado.
*
* Se Ts == 0:
*
* ```
  Ts = 1 / Hz
  ```
*
* Se Hz == 0:
*
* ```
  Hz = 1 / Ts
  ```
*
* @param svpwm Ponteiro para a estrutura do SVPWM.
* @param Hz Frequência de chaveamento.
* @param Ts Período de chaveamento.
* @param Vdc Tensão do barramento CC.
*
* @return true se a inicialização for válida.
* @return false caso contrário.
  */
static inline bool svpwm_init(
    svpwm_t *svpwm,
    float Hz,
    float Ts,
    float Vdc)
{
  if (svpwm == NULL)
  {
    return false;
  }

  if (Vdc <= 0.0f)
  {
    return false;
  }

  if (Ts <= 0.0f && Hz <= 0.0f)
  {
    return false;
  }

  if (Ts == 0.0f)
  {
    Ts = 1.0f / Hz;
  }

  if (Hz == 0.0f)
  {
    Hz = 1.0f / Ts;
  }

  svpwm->Vdc = Vdc;
  svpwm->Ts = Ts;
  svpwm->Hz = Hz;

  return true;
}

/**

* @brief Calcula os duty cycles utilizando SVPWM.
*
* A entrada é o vetor de tensão no referencial estacionário αβ:
*
* ```
  Valpha
  ```
* ```
  Vbeta
  ```
*
* Primeiro é realizada a transformação inversa de Clarke:
*
* ```
  Va_ref = Valpha
  ```
*
* ```
  Vb_ref =
  ```
* ```
      -0.5 Valpha
  ```
* ```
      + sqrt(3)/2 Vbeta
  ```
*
* ```
  Vc_ref =
  ```
* ```
      -0.5 Valpha
  ```
* ```
      - sqrt(3)/2 Vbeta
  ```
*
* Em seguida, é calculada a tensão de modo comum:
*
* ```
  Voffset =
  ```
* ```
      -0.5 (Vmax + Vmin)
  ```
*
* As tensões moduladas são:
*
* ```
  Va_mod = Va_ref + Voffset
  ```
*
* ```
  Vb_mod = Vb_ref + Voffset
  ```
*
* ```
  Vc_mod = Vc_ref + Voffset
  ```
*
* Finalmente:
*
* ```
  duty_a = Va_mod / Vdc + 0.5
  ```
*
* ```
  duty_b = Vb_mod / Vdc + 0.5
  ```
*
* ```
  duty_c = Vc_mod / Vdc + 0.5
  ```
*
* @param svpwm Ponteiro para a estrutura do SVPWM.
* @param Valpha Componente alfa da tensão de referência.
* @param Vbeta Componente beta da tensão de referência.
* @param duty_a Ponteiro para o duty cycle da fase A.
* @param duty_b Ponteiro para o duty cycle da fase B.
* @param duty_c Ponteiro para o duty cycle da fase C.
  */
static inline void svpwm_modulate(
    const svpwm_t *svpwm,
    float Valpha,
    float Vbeta,
    float *duty_a,
    float *duty_b,
    float *duty_c)
{
  float Va_ref;
  float Vb_ref;
  float Vc_ref;

  float Vmax;
  float Vmin;

  float Voffset;

  float Va_mod;
  float Vb_mod;
  float Vc_mod;

  /*

  * Transformação inversa de Clarke
    */
  clarke_inverse_transform(Valpha, Vbeta,
                           &Va_ref, &Vb_ref, &Vc_ref);

  /*

  * Maior e menor tensão
    */
  Vmax = fmaxf(
      Va_ref,
      fmaxf(Vb_ref, Vc_ref));

  Vmin = fminf(
      Va_ref,
      fminf(Vb_ref, Vc_ref));

  /*

  * Tensão de modo comum
    */
  Voffset =
      -0.5f * (Vmax + Vmin);

  /*

  * Tensões moduladas
    */
  Va_mod = Va_ref + Voffset;
  Vb_mod = Vb_ref + Voffset;
  Vc_mod = Vc_ref + Voffset;

  /*

  * Duty cycles
    */
  *duty_a =
      Va_mod / svpwm->Vdc + 0.5f;

  *duty_b =
      Vb_mod / svpwm->Vdc + 0.5f;

  *duty_c =
      Vc_mod / svpwm->Vdc + 0.5f;

  /*

  * Limitação dos duty cycles
    */
  *duty_a = svpwm_clamp(
      *duty_a,
      0.0f,
      1.0f);

  *duty_b = svpwm_clamp(
      *duty_b,
      0.0f,
      1.0f);

  *duty_c = svpwm_clamp(
      *duty_c,
      0.0f,
      1.0f);
}

/**

* @brief Obtém o setor do vetor espacial.
*
* Recebe as componentes αβ do vetor:
*
* ```
  alpha
  ```
* ```
  beta
  ```
*
* A magnitude é calculada por:
*
* ```
  magnitude =
  ```
* ```
      sqrt(alpha² + beta²)
  ```
*
* O ângulo é calculado por:
*
* ```
  angle =
  ```
* ```
      atan2(beta, alpha)
  ```
*
* e normalizado para o intervalo:
*
* ```
  0 <= angle < 2π
  ```
*
* Cada setor possui 60 graus:
*
* ```
  Setor 1: 0°   <= θ < 60°
  ```
* ```
  Setor 2: 60°  <= θ < 120°
  ```
* ```
  Setor 3: 120° <= θ < 180°
  ```
* ```
  Setor 4: 180° <= θ < 240°
  ```
* ```
  Setor 5: 240° <= θ < 300°
  ```
* ```
  Setor 6: 300° <= θ < 360°
  ```
*
* @param alpha Componente alfa do vetor.
* @param beta Componente beta do vetor.
*
* @return Estrutura contendo setor, ângulo e magnitude.
  */
static inline svpwm_sector_t svpwm_get_sector(
    float alpha,
    float beta)
{
  svpwm_sector_t result;

  /*

  * Magnitude do vetor
    */
  result.magnitude =
      hypotf(alpha, beta);

  /*

  * Ângulo do vetor
    */
  result.angle =
      atan2f(beta, alpha);

  /*

  * Normalização do ângulo
    */
  if (result.angle < 0.0f)
  {
    result.angle += SVPWM_TWO_PI;
  }

  /*

  * Cálculo do setor
    */
  result.sector =
      (int)(result.angle / (M_PI / 3.0f)) + 1;

  /*

  * Limitação do setor
    */
  if (result.sector > 6)
  {
    result.sector = 6;
  }

  return result;
}

/**

* @brief Gera a portadora triangular do PWM, normalizada entre 0 e 1.
*
* A portadora é simétrica (sobe e desce dentro de cada período Ts),
* que é a forma classicamente usada para comparação com o duty cycle
* em moduladores PWM de dois níveis (natural/regular sampling):
*
* ```
  fase = (t mod Ts) / Ts   (0 <= fase < 1)
  ```
*
* ```
  carrier = 2*fase          se fase < 0.5
  ```
* ```
  carrier = 2*(1 - fase)    se fase >= 0.5
  ```
*
* @param svpwm Ponteiro para a estrutura do SVPWM (usa svpwm->Ts).
* @param t Instante de tempo absoluto da simulação [s].
*
* @return Valor da portadora triangular, no intervalo [0, 1].
  */
static inline float svpwm_carrier(
    const svpwm_t *svpwm,
    float t)
{
  float phase = fmodf(t, svpwm->Ts) / svpwm->Ts;

  if (phase < 0.0f)
  {
    phase += 1.0f;
  }

  return (phase < 0.5f) ? (2.0f * phase) : (2.0f - 2.0f * phase);
}

/**

* @brief Compara o duty cycle de referência com a portadora triangular
* para gerar o estado de chaveamento (0 ou 1) de um braço do inversor.
*
* Esta é a comparação real feita pelo hardware de PWM: enquanto o
* duty de referência (saída de svpwm_modulate) estiver acima da
* portadora instantânea, a chave superior do braço fica ligada
* (estado 1 -> polo em Vdc); caso contrário, fica desligada
* (estado 0 -> polo em 0V).
*
* @param duty Duty cycle de referência do braço (0 a 1).
* @param carrier Valor instantâneo da portadora triangular (0 a 1).
*
* @return 1 se a chave superior estiver ligada, 0 caso contrário.
  */
static inline int svpwm_gate_state(
    float duty,
    float carrier)
{
  return (duty > carrier) ? 1 : 0;
}

#endif /* SVPWM_H */