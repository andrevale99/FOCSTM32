import csv
import numpy as np

from PIcontroller import PIController
from Inverter import Inverter
from bldc import BLDC, rpm_to_rads
from SVPWM import SVPWM


# =========================================
#   PARAMETROS REDE
# =========================================

# Amplitude da rede
Vdc = np.float32(24.0)  # V


# =========================================
#   PARAMETROS MOTOR
# =========================================

# Resistencia de armadura
Rs = np.float32(0.161)  # Ohm

# Indutancia de magnetizacao
L = np.float32(0.052e-3)  # H

# Constantes eletricas e mecanicas
Ke = np.float32(0.0255)
Kt = np.float32(Ke)

# Torque da carga
Tl = np.float32(0.0)

# Coeficiente de amortecimento
B = np.float32(1.2e-6)

# Momento de inercia
J = np.float32(43.3e-3)

# Quantidade de polos no motor
POLOS = np.int32(8)

# Quantidade de pares de polos
PARES_DE_POLOS = np.float32(POLOS / 2)


# =========================================
#   PARAMETROS DOS CONTROLADORES
# =========================================

VDC_MAX = np.float32(Vdc)
VDC_MIN = np.float32(-Vdc)

PI_IQ_MAX = np.float32(8.0)
PI_IQ_MIN = np.float32(-PI_IQ_MAX)


# =========================================
#   PARAMETROS SIMULACAO
# =========================================

ti = np.float32(0.0)  # s
tf = np.float32(0.2)  # s
dt = np.float32(1e-5)  # s

time = np.arange(ti, tf, dt, dtype=np.float32)
N = len(time)


# =========================================
#   OBJETOS
# =========================================

motor = BLDC(
    Rs,
    L,
    B,
    J,
    Ke,
    Kt,
    PARES_DE_POLOS,
    Vdc
)

pwm = SVPWM(
    Hz=np.float32(10000.0),
    Vdc=Vdc
)

inverter = Inverter(
    Vdc=Vdc
)


# =========================================
#   CONTROLADORES
# =========================================

dtOmega = np.float32(5e-4)  # s
kpOmega = np.float32(2.0)
kiOmega = np.float32(0.5)

dtId = np.float32(5e-4)  # s
kpId = np.float32(10.0)
kiId = np.float32(5.0)

dtIq = np.float32(5e-4)  # s
kpIq = np.float32(10.0)
kiIq = np.float32(5.0)


pi_omega = PIController(
    Kp=kpOmega,
    Ki=kiOmega,
    Ts=dtOmega,
    output_min=PI_IQ_MIN,
    output_max=PI_IQ_MAX
)

pi_d = PIController(
    Kp=kpId,
    Ki=kiId,
    Ts=dtId,
    output_min=VDC_MIN,
    output_max=VDC_MAX
)

pi_q = PIController(
    Kp=kpIq,
    Ki=kiIq,
    Ts=dtIq,
    output_min=VDC_MIN,
    output_max=VDC_MAX
)


# =========================================
#   SIMULACAO
# =========================================

motor.set_initial_conditions()

id_ref = np.float32(0.0)

rpm_ref = np.float32(20.0)
omega_ref = np.float32(rpm_to_rads(rpm_ref))

print(omega_ref)


# =========================================
#   ARQUIVO DE LOG (CSV)
# =========================================

arquivo_csv = "closedloop_simulation.csv"


with open(arquivo_csv, mode="w", newline="") as f:

    writer = csv.writer(f, delimiter=";")

    writer.writerow([
        "time",
        "Va", "Vb", "Vc",
        "ia", "ib", "ic",
        "id", "iq",
        "Te",
        "theta_r",
        "omega_r",
        "iq_ref",
        "duty_a", "duty_b", "duty_c"
    ])

    for k in range(N):

        # =========================================
        # A. MEDICAO
        # =========================================

        theta_e = np.float32(
            motor.theta_r * motor.P
        )

        omega_e = np.float32(
            motor.omega_r * motor.P
        )

        iq_ref = np.float32(
            pi_omega.update(
                omega_ref,
                motor.omega_r
            )
        )


        # =========================================
        # B. TRANSFORMADA DE CLARKE E PARK
        # =========================================

        i_alpha, i_beta = motor.Clarke(
            np.array(
                [
                    motor.ia,
                    motor.ib,
                    motor.ic
                ],
                dtype=np.float32
            )
        )

        i_alpha = np.float32(i_alpha)
        i_beta = np.float32(i_beta)

        i_d, i_q = motor.Park(
            np.array(
                [
                    i_alpha,
                    i_beta
                ],
                dtype=np.float32
            ),
            theta_e
        )

        i_d = np.float32(i_d)
        i_q = np.float32(i_q)


        # =========================================
        # C. CONTROLE PI - LOOP DE CORRENTE
        # =========================================

        vd_ref = np.float32(
            pi_d.update(
                id_ref,
                i_d
            )
        )

        vq_ref = np.float32(
            pi_q.update(
                iq_ref,
                i_q
            )
        )


        # =========================================
        # D. TRANSFORMADA INVERSA DE PARK
        # =========================================

        v_alpha, v_beta = motor.ParkInverse(
            np.array(
                [
                    vd_ref,
                    vq_ref
                ],
                dtype=np.float32
            ),
            theta_e
        )

        v_alpha = np.float32(v_alpha)
        v_beta = np.float32(v_beta)


        # =========================================
        # E. DUTY CYCLES DO PWM
        # =========================================

        duty_a, duty_b, duty_c = pwm.modulate(
            v_alpha,
            v_beta
        )

        duty_a = np.float32(duty_a)
        duty_b = np.float32(duty_b)
        duty_c = np.float32(duty_c)


        # =========================================
        # F. INVERSOR
        # =========================================

        va, vb, vc = inverter.output_voltage(
            duty_a,
            duty_b,
            duty_c
        )

        va = np.float32(va)
        vb = np.float32(vb)
        vc = np.float32(vc)


        # =========================================
        # G. ATUALIZACAO DA PLANTA
        # =========================================

        estados = motor.step(
            va,
            vb,
            vc,
            Tl=np.float32(Tl),
            dt=np.float32(dt),
            back_emf_trapezoidal_flag=False
        )


        # =========================================
        # H. LOG
        # =========================================

        writer.writerow([
            np.float32(time[k]),

            np.float32(va),
            np.float32(vb),
            np.float32(vc),

            np.float32(estados["ia"]),
            np.float32(estados["ib"]),
            np.float32(estados["ic"]),

            np.float32(i_d),
            np.float32(i_q),

            np.float32(estados["Te"]),

            np.float32(estados["theta_r"]),
            np.float32(estados["omega_r"]),

            np.float32(iq_ref),

            np.float32(duty_a),
            np.float32(duty_b),
            np.float32(duty_c)
        ])


print(
    f'Simulacao concluida. Resultados em "{arquivo_csv}".'
)
