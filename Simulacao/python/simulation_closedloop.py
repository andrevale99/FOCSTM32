from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from numpy import pi
import numpy as np
import os

from PIcontroller import PIController
from Inverter import Inverter
from bldc import BLDC
from SVPWM import SVPWM

PATH = os.getcwd()
print(f'PATH = {PATH + '/'}')
# print(os.listdir(PATH))

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.serif": ["cmr10", "DejaVu Serif", "serif"],
    "axes.formatter.use_mathtext": True,
    "font.size": 12
})

def cm_to_inches(cm):
    return cm/2.54

# ========================================= 
#   PARAMETROS REDE
# =========================================

# Amplitude da rede
Vdc = 24 #V

# Defasagens das fases
PHI_A = 0
PHI_B = -2*pi/3
PHI_C = 2*pi/3

# ========================================= 
#   PARAMETROS MOTOR
# =========================================

# Resistencia de armadura
Rs = 0.3 #Ohm

# Indutancia de magnetizacao
L = 5e-4 #H

# Constantes eletricas e mecanicas
Ke = 0.0676
Kt = 0.06

# Torque da carga
Tl = 0.1

# Coeficiente de amortecimento
B = 1.312e-7

# Momento de inercia
J = 4.6e-6

# Quantidade de Polos no motor
POLOS = 14
PARES_DE_POLOS = POLOS / 2

# ========================================= 
#   PARAMETROS DOS CONTROLADORES
# =========================================

VDC_MAX = 24
VDC_MIN = -24

RPM_MAX = 1500
RPM_MIN = -RPM_MAX

# ========================================= 
#   PARAMETROS SIMULACAO
# =========================================

ti = 0.0 #s
tf = 0.2 #s
dt = 1e-5 #s

time = np.arange(
    ti,
    tf,
    dt
)

N = len(time)

# ========================================= 
#   OBJETOS
# =========================================

motor = BLDC(Rs,L,B,J,Ke,Kt,PARES_DE_POLOS,Vdc)
pwm = SVPWM(Hz=10000, Vdc=Vdc)
inverter = Inverter(Vdc=Vdc)

pi_omega = PIController(Kp=1.5,Ki=0.1,Ts=dt, output_min=RPM_MAX, output_max=0)
pi_d = PIController(Kp=5,Ki=1.8,Ts=dt, output_min=VDC_MIN, output_max=VDC_MAX)
pi_q = PIController(Kp=5,Ki=.5,Ts=dt, output_min=VDC_MIN, output_max=VDC_MAX)

# ========================================= 
#   SIMULACAO
# =========================================

motor.set_initial_conditions()

# rpm_ref = 1000 
# omega_ref = motor.rpm_to_rads(rpm_ref)

iq_ref = 1
id_ref = 5

log_omega = np.zeros(N)
log_Vabc = np.zeros((N,3))
log_iabc = np.zeros((N,3))
log_idq = np.zeros((N,2))
log_Te = np.zeros(N)
log_theta = np.zeros(N)

for k in range(N):
    # A. Medição (feedback do modelo)
    # Suponha que 'motor' já foi instanciado e 'theta_r' existe no estado
    theta_e = motor.theta_r * motor.P 
    
    # B. Transformada de Clarke e Park
    i_alpha, i_beta = motor.Clarke([motor.ia, motor.ib, motor.ic])

    # C. Calcular o vetor fluxo
    # lambda_alpha += (r*i_alpha + v_alpha)*dt
    # lambda_beta += (r*i_beta + v_beta)*dt

    i_d, i_q = motor.Park(np.array([i_alpha, i_beta]), theta_e)
    
    # D. Controle PI (Loop de Corrente)
    vd_ref = pi_d.update(id_ref, i_d)
    vq_ref = pi_q.update(iq_ref, i_q)

    # E. Transformada Inversa de Park
    v_alpha, v_beta = motor.ParkInverse(np.array([vd_ref, vq_ref]), theta_e)

    # F.Dutys Cycles dos pwm's
    duties = pwm.modulate(v_alpha, v_beta)

    # G. Inversor
    v_abc = inverter.output_voltage(duties[0], duties[1], duties[2])

    # H. Atualização da Planta (Motor)
    estados = motor.step(v_abc[0], v_abc[1], v_abc[2], Tl=Tl, dt=dt)

    log_omega[k] = estados['omega_r']
    log_theta[k] = estados['theta_r']
    log_iabc[k,0] = estados["ia"]
    log_iabc[k,1] = estados["ib"]
    log_iabc[k,2] = estados["ic"]
    log_Te[k] = estados["Te"]
    log_Vabc[k] = v_abc
    log_idq[k,0] = i_d
    log_idq[k,1] = i_q

plt.figure(figsize=(cm_to_inches(25),cm_to_inches(20)))

plt.subplot(511)
plt.title(r"$V_{abc}$")
plt.plot(time, log_Vabc, label=[r"$V_a$", r"$V_b$", r"$V_c$"])
plt.legend()
plt.grid()
plt.ylabel(r"$V$")

plt.subplot(512)
plt.title(r"$i_{abc}$")
plt.plot(time, log_iabc, label=[r"$i_a$", r"$i_b$", r"$i_c$"])
plt.legend()
plt.grid()
plt.ylabel(r"$A$")

plt.subplot(513)
plt.title(r"$i_{dq}$")
plt.plot(time, log_idq, label=[r"$i_d$", r"$i_q$"])
plt.legend()
plt.grid()
plt.ylabel(r"$A$")

plt.subplot(514)
plt.title(r"$\omega_r$")
plt.plot(time, log_omega, label=r'$\omega_r$')
plt.legend()
plt.grid()
plt.ylabel(r"$\omega_r$")

plt.subplot(515)
plt.title(r"$T_e$")
plt.plot(time, log_Te, label=r'$T_e$')
plt.legend()
plt.grid()
plt.ylabel(r"$N \cdot m$")

plt.xlabel(r'$s$')

plt.tight_layout()

plt.show()