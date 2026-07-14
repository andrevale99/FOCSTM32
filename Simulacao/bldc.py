import numpy as np
import os
import matplotlib.pyplot as plt
from numpy import pi

PATH = os.getcwd()
print(PATH + '/')
print(os.listdir(PATH))

def back_emf_trapezoidal(theta):
    """
    Back-EMF trapezoidal normalizada (-1 a +1).
    theta em radianos.
    """

    theta = np.mod(theta, 2*np.pi)

    if theta < np.pi/6:
        return 6*theta/np.pi

    elif theta < 5*np.pi/6:
        return 1.0

    elif theta < 7*np.pi/6:
        return 1 - 6*(theta-5*np.pi/6)/np.pi

    elif theta < 11*np.pi/6:
        return -1.0

    else:
        return -1 + 6*(theta-11*np.pi/6)/np.pi

# =====================================================================
# =====================================================================
# =====================================================================

# Resistencia de armadura
Rs = 0.386 #Ohm

# Indutancia de magnetizacao
L = 6.53e-5 #H

# Constantes eletricas e mecanicas
Ke = 0.0276
Kt = Ke

# Torque da carga
Tl = 0.1

# Coeficiente de amortecimento
B = 6.75e-6

# Momento de inercia
J = 3.33e-6

# Quantidade de Polos no motor
POLOS = 8
PARES_DE_POLOS = POLOS / 2

# Amplitude da rede
Vm = 48*0.8 #V

# Defasagens das fases
PHI_A = 0
PHI_B = -2*pi/3
PHI_C = 2*pi/3

TEMPO_MAX = 0.1     # s
dt = 1e-5         # s

time = np.arange(0, TEMPO_MAX, dt)
N = len(time)

theta_r = np.zeros(N)
omega_r = np.zeros(N)

ia = np.zeros(N)
ib = np.zeros(N)
ic = np.zeros(N)

Te = np.zeros(N)

Va = np.zeros(N)
Vb = np.zeros(N)
Vc = np.zeros(N)

ea = np.zeros(N)
eb = np.zeros(N)
ec = np.zeros(N)

for k in range(N-1):

    # Ângulo e velocidade elétrica do rotor
    theta_e = PARES_DE_POLOS * theta_r[k]

    # Tensões trifásicas da alimentação
    Va[k] = Vm * np.sin(theta_e  + PHI_A)
    Vb[k] = Vm * np.sin(theta_e  + PHI_B)
    Vc[k] = Vm * np.sin(theta_e  + PHI_C)

    # Back-EMF
    fa = np.sin(theta_e + PHI_A)
    fb = np.sin(theta_e + PHI_B)
    fc = np.sin(theta_e + PHI_C)
    # fa = back_emf_trapezoidal(theta_e + PHI_A)
    # fb = back_emf_trapezoidal(theta_e + PHI_B)
    # fc = back_emf_trapezoidal(theta_e + PHI_C)


    ea[k] = Ke * omega_r[k] * fa
    eb[k] = Ke * omega_r[k] * fb
    ec[k] = Ke * omega_r[k] * fc

    # ea[k] = Ke*omega_r[k]*back_emf_trapezoidal(theta_e + PHI_A)
    # eb[k] = Ke*omega_r[k]*back_emf_trapezoidal(theta_e + PHI_B)
    # ec[k] = Ke*omega_r[k]*back_emf_trapezoidal(theta_e + PHI_C)
    
    # Derivadas das correntes
    dia = (Va[k] - Rs * ia[k] - ea[k]) / L
    dib = (Vb[k] - Rs * ib[k] - eb[k]) / L
    dic = (Vc[k] - Rs * ic[k] - ec[k]) / L

    # Integração (Euler)
    ia[k+1] = ia[k] + dia * dt
    ib[k+1] = ib[k] + dib * dt
    ic[k+1] = ic[k] + dic * dt

    # Torque eletromagnético
    Te[k] = Kt * (
        ia[k] * fa +
        ib[k] * fb +
        ic[k] * fc
    )

    # Dinâmica mecânica
    domega = (Te[k] - Tl - B * omega_r[k]) / J

    omega_r[k+1] = omega_r[k] + domega * dt
    theta_r[k+1] = theta_r[k] + omega_r[k+1] * dt


plt.figure(figsize=(12, 10))
plt.subplot(411)

plt.title("Vabc")
plt.plot(time, Va, label="Va")
plt.plot(time, Vb, label="Vb")
plt.plot(time, Vc, label="Vc")

plt.grid()
plt.legend()

plt.subplot(412)

plt.title("Iabc")
plt.plot(time, ia, label="ia")
plt.plot(time, ib, label="ib")
plt.plot(time, ic, label="ic")

plt.grid()
plt.legend()

plt.subplot(413)

plt.title("Omega_r")
plt.plot(time,omega_r, label='omega')

plt.grid()
plt.legend()

plt.subplot(414)

plt.title("Te")
plt.plot(time, Te, label="Te")

plt.grid()
plt.legend()

plt.tight_layout()

plt.show()