import numpy as np
import os
import matplotlib.pyplot as plt
from numpy import pi
from bldc import bldc

PATH = os.getcwd()
print(PATH + '/')
print(os.listdir(PATH))

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
POLOS = 4
PARES_DE_POLOS = POLOS / 2

# Amplitude da rede
Vm = 48*0.8 #V

# Defasagens das fases
PHI_A = 0
PHI_B = -2*pi/3
PHI_C = 2*pi/3

TEMPO_MAX = 0.1     # s
dt = 1e-5         # s

bldc1 = bldc(Rs,L,B,J,Ke,Kt,PARES_DE_POLOS,Vm)

time,Vabc,_,iabc,Te,omega_r,_ = bldc1.simulation_open_loop(t0=0,tf=0.1,dt=1e-4,
                                                           Tl=0.1, 
                                                           back_emf_trapezoidal_flag=False)

plt.figure(figsize=(12, 10))
plt.subplot(411)

plt.title("Vabc")
plt.plot(time, Vabc[0], label="Va")
plt.plot(time, Vabc[1], label="Vb")
plt.plot(time, Vabc[2], label="Vc")

plt.grid()
plt.legend()

plt.subplot(412)

plt.title("Iabc")
plt.plot(time, iabc[0], label="ia")
plt.plot(time, iabc[1], label="ib")
plt.plot(time, iabc[2], label="ic")

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