#ifndef SVPWM_H
#define SVPWM_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "transforms.h"

#define SVPWM_TWO_PI (2.0f * M_PI)
#define SVPWM_SQRT3_OVER_2 (0.8660254037844386f)

/**

* @brief Estrutura do modulador SVPWM.
  */
typedef struct
{
  /**

  * @brief Tensao do barramento CC.
    */
  float Vdc;

  /**

  * @brief Periodo de chaveamento.
    */
  float Ts;

  /**

  * @brief Frequencia de chaveamento.
    */
  float Hz;

} svpwm_t;

/**

* @brief Resultado da identificacao do setor do vetor espacial.
  */
typedef struct
{
  /**

  * @brief Setor do vetor espacial, de 1 a 6.
    */
  int sector;

  /**

  * @brief Angulo do vetor em radianos.
    */
  float angle;

  /**

  * @brief Magnitude do vetor.
    */
  float magnitude;

} svpwm_sector_t;

/**

* @brief Inicializa o modulador SVPWM.
*
* A frequencia ou o periodo de chaveamento deve ser informado.
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
* @param Hz Frequencia de chaveamento.
* @param Ts Periodo de chaveamento.
* @param Vdc Tensao do barramento CC.
*
* @return true se a inicializacao for valida.
* @return false caso contrario.
  */
bool svpwm_init(
    svpwm_t *svpwm,
    float Hz,
    float Ts,
    float Vdc);

/**

* @brief Limita um valor a um intervalo.
*/
float svpwm_clamp(
    float value,
    float min,
    float max);

/**

* @brief Calcula os duty cycles utilizando SVPWM.
*
* A entrada e o vetor de tensao no referencial estacionario alphabeta:
*
* ```
  Valpha
  ```
* ```
  Vbeta
  ```
*
* Primeiro e realizada a transformacao inversa de Clarke:
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
* Em seguida, e calculada a tensao de modo comum:
*
* ```
  Voffset =
  ```
* ```
      -0.5 (Vmax + Vmin)
  ```
*
* As tensoes moduladas sao:
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
* @param Valpha Componente alfa da tensao de referencia.
* @param Vbeta Componente beta da tensao de referencia.
* @param duty_a Ponteiro para o duty cycle da fase A.
* @param duty_b Ponteiro para o duty cycle da fase B.
* @param duty_c Ponteiro para o duty cycle da fase C.
  */
void svpwm_modulate(
    const svpwm_t *svpwm,
    float Valpha,
    float Vbeta,
    float *duty_a,
    float *duty_b,
    float *duty_c);

/**

* @brief Obtem o setor do vetor espacial.
*
* Recebe as componentes alphabeta do vetor:
*
* ```
  alpha
  ```
* ```
  beta
  ```
*
* A magnitude e calculada por:
*
* ```
  magnitude =
  ```
* ```
      sqrt(alpha2 + beta2)
  ```
*
* O angulo e calculado por:
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
  0 <= angle < 2pi
  ```
*
* Cada setor possui 60 graus:
*
* ```
  Setor 1: 0 graus   <= theta < 60 graus
  ```
* ```
  Setor 2: 60 graus  <= theta < 120 graus
  ```
* ```
  Setor 3: 120 graus <= theta < 180 graus
  ```
* ```
  Setor 4: 180 graus <= theta < 240 graus
  ```
* ```
  Setor 5: 240 graus <= theta < 300 graus
  ```
* ```
  Setor 6: 300 graus <= theta < 360 graus
  ```
*
* @param alpha Componente alfa do vetor.
* @param beta Componente beta do vetor.
*
* @return Estrutura contendo setor, angulo e magnitude.
  */
svpwm_sector_t svpwm_get_sector(
    float alpha,
    float beta);

/**

* @brief Gera a portadora triangular do PWM, normalizada entre 0 e 1.
*
* A portadora e simetrica (sobe e desce dentro de cada periodo Ts),
* que e a forma classicamente usada para comparacao com o duty cycle
* em moduladores PWM de dois niveis (natural/regular sampling):
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
* @param t Instante de tempo absoluto da simulacao [s].
*
* @return Valor da portadora triangular, no intervalo [0, 1].
  */
float svpwm_carrier(
    const svpwm_t *svpwm,
    float t);

/**

* @brief Compara o duty cycle de referencia com a portadora triangular
* para gerar o estado de chaveamento (0 ou 1) de um braco do inversor.
*
* Esta e a comparacao real feita pelo hardware de PWM: enquanto o
* duty de referencia (saida de svpwm_modulate) estiver acima da
* portadora instantanea, a chave superior do braco fica ligada
* (estado 1 -> polo em Vdc); caso contrario, fica desligada
* (estado 0 -> polo em 0V).
*
* @param duty Duty cycle de referencia do braco (0 a 1).
* @param carrier Valor instantaneo da portadora triangular (0 a 1).
*
* @return 1 se a chave superior estiver ligada, 0 caso contrario.
  */
int svpwm_gate_state(
    float duty,
    float carrier);

#endif /* SVPWM_H */