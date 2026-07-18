/*
 * pi_controller.h
 *
 * Controlador PI (Proporcional-Integral) com anti-windup por
 * saturacao (clamping), convertido a partir da implementacao em Python
 * (PIcontroller.py).
 *
 * Uso:
 *   PIController pid;
 *   pi_controller_init(&pid, kp, ki, ts, out_min, out_max);
 *
 *   float u = pi_controller_update(&pid, referencia, realimentacao);
 *
 *   if (pid.saturated) {
 *       // diagnostico: saida saturada
 *   }
 *
 *   pi_controller_reset(&pid); // zera o termo integral
 */

#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdbool.h>

/* Use valores sentinela para "sem limite", equivalentes ao None do Python.
 * Por padrao, sem limite inferior/superior. Ajuste conforme necessario. */
#ifndef PI_NO_LIMIT
#define PI_NO_LIMIT_MIN (-1e30)
#define PI_NO_LIMIT_MAX ( 1e30)
#endif

typedef struct {
    double Kp;
    double Ki;
    double Ts;

    double output_min;   /* use PI_NO_LIMIT_MIN se nao houver limite inferior */
    double output_max;   /* use PI_NO_LIMIT_MAX se nao houver limite superior */
    bool   has_min;
    bool   has_max;

    double integral;
    bool   saturated;     /* instrumentacao para diagnostico */
} PIController;

/* Inicializa o controlador.
 * Passe has_min = false / has_max = false caso nao existam limites
 * (equivalente a output_min/output_max = None no Python). */
static inline void pi_controller_init(PIController *pid,
                                       double Kp, double Ki, double Ts,
                                       bool has_min, double output_min,
                                       bool has_max, double output_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Ts = Ts;

    pid->has_min = has_min;
    pid->has_max = has_max;
    pid->output_min = has_min ? output_min : PI_NO_LIMIT_MIN;
    pid->output_max = has_max ? output_max : PI_NO_LIMIT_MAX;

    pid->integral  = 0.0;
    pid->saturated = false;
}

/* Zera o termo integral. */
static inline void pi_controller_reset(PIController *pid)
{
    pid->integral = 0.0;
}

/* Executa um passo de controle e retorna a saida. */
static inline double pi_controller_update(PIController *pid,
                                           double reference, double feedback)
{
    double error = reference - feedback;
    double proportional = pid->Kp * error;

    pid->integral += pid->Ki * error * pid->Ts;
    double output = proportional + pid->integral;

    pid->saturated = false;

    if (pid->has_min && output < pid->output_min) {
        output = pid->output_min;
        pid->saturated = true;
        if (error < 0.0) {
            pid->integral -= pid->Ki * error * pid->Ts;
        }
    }

    if (pid->has_max && output > pid->output_max) {
        output = pid->output_max;
        pid->saturated = true;
        if (error > 0.0) {
            pid->integral -= pid->Ki * error * pid->Ts;
        }
    }

    return output;
}

#endif /* PI_CONTROLLER_H */