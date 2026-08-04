#include "svpwm.h"

float svpwm_clamp(
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

bool svpwm_init(
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

void svpwm_modulate(
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

  * Transformacao inversa de Clarke
    */
  clarke_inverse_transform(Valpha, Vbeta,
                           &Va_ref, &Vb_ref, &Vc_ref);

  /*

  * Maior e menor tensao
    */
  Vmax = fmaxf(
      Va_ref,
      fmaxf(Vb_ref, Vc_ref));

  Vmin = fminf(
      Va_ref,
      fminf(Vb_ref, Vc_ref));

  /*

  * Tensao de modo comum
    */
  Voffset =
      -0.5f * (Vmax + Vmin);

  /*

  * Tensoes moduladas
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

  * Limitacao dos duty cycles
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

svpwm_sector_t svpwm_get_sector(
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

  * Angulo do vetor
    */
  result.angle =
      atan2f(beta, alpha);

  /*

  * Normalizacao do angulo
    */
  if (result.angle < 0.0f)
  {
    result.angle += SVPWM_TWO_PI;
  }

  /*

  * Calculo do setor
    */
  result.sector =
      (int)(result.angle / (M_PI / 3.0f)) + 1;

  /*

  * Limitacao do setor
    */
  if (result.sector > 6)
  {
    result.sector = 6;
  }

  return result;
}

float svpwm_carrier(
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

int svpwm_gate_state(
    float duty,
    float carrier)
{
  return (duty > carrier) ? 1 : 0;
}