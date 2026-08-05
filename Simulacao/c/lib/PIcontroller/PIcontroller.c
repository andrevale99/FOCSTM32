#include "PIcontroller.h"

void pi_controller_init(PIController *pid,
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
}

void pi_controller_reset(PIController *pid)
{
    pid->integral = 0.0;
}

double pi_controller_update(PIController *pid,
                                           double reference, double feedback)
{
    pid->error = reference - feedback;
    double proportional = pid->Kp * pid->error;

    pid->integral += pid->Ki * pid->error * pid->Ts;
    double output = proportional + pid->integral;


    if (pid->has_min && output < pid->output_min) {
        output = pid->output_min;
        if (pid->error < 0.0) {
            pid->integral -= pid->Ki * pid->error * pid->Ts;
        }
    }

    if (pid->has_max && output > pid->output_max) {
        output = pid->output_max;
        if (pid->error > 0.0) {
            pid->integral -= pid->Ki * pid->error * pid->Ts;
        }
    }

    return output;
}
