#include "VFstartup.h"

void vf_startup_init(
    vf_startup_t *vf,
    float V_boost,
    float V_per_rad_s,
    float V_max,
    float ramp_rate,
    float omega_e_target)
{
    vf->V_boost = V_boost;
    vf->V_per_rad_s = V_per_rad_s;
    vf->V_max = V_max;
    vf->ramp_rate = ramp_rate;
    vf->omega_e_target = omega_e_target;

    vf->omega_e_cmd = 0.0f;
    vf->theta_e = 0.0f;
}

bool vf_startup_step(
    vf_startup_t *vf,
    float dt,
    float *theta_e_out,
    float *v_alpha_out,
    float *v_beta_out)
{
    /* 1. Rampa de frequencia eletrica comandada */
    float domega = vf->ramp_rate * dt;

    if (vf->omega_e_cmd < vf->omega_e_target)
    {
        vf->omega_e_cmd += domega;

        if (vf->omega_e_cmd > vf->omega_e_target)
        {
            vf->omega_e_cmd = vf->omega_e_target;
        }
    }
    else if (vf->omega_e_cmd > vf->omega_e_target)
    {
        vf->omega_e_cmd -= domega;

        if (vf->omega_e_cmd < vf->omega_e_target)
        {
            vf->omega_e_cmd = vf->omega_e_target;
        }
    }

    /* 2. Integracao do angulo eletrico sintetico (parte sempre de 0) */
    vf->theta_e = fmodf(vf->theta_e + vf->omega_e_cmd * dt, VF_STARTUP_TWO_PI);

    if (vf->theta_e < 0.0f)
    {
        vf->theta_e += VF_STARTUP_TWO_PI;
    }

    /* 3. Amplitude de tensao pela reta V/F, saturada em V_max */
    float V_amp = vf->V_boost + vf->V_per_rad_s * fabsf(vf->omega_e_cmd);

    if (V_amp > vf->V_max)
    {
        V_amp = vf->V_max;
    }

    /* 4. Sintese do vetor de tensao alpha-beta a partir do angulo
     *    sintetico -- equivalente a uma transformada inversa de Park
     *    com vd = 0, vq = V_amp, theta = theta_e (torque puro,
     *    sem enfraquecimento de campo nesta fase) */
    *theta_e_out = vf->theta_e;
    *v_alpha_out = -V_amp * sinf(vf->theta_e);
    *v_beta_out = V_amp * cosf(vf->theta_e);

    return (fabsf(vf->omega_e_cmd - vf->omega_e_target) < 1e-6f);
}
