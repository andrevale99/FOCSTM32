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
Rs = 0.6 #Ohm

# Indutancia de magnetizacao
L = 5e-3 #H

# Constantes eletricas e mecanicas
Ke = 0.0276
Kt = Ke

# Torque da carga
Tl = 0.0

# Coeficiente de amortecimento
B = 7.312e-7

# Momento de inercia
J = 7.312e-6

# Quantidade de Polos no motor
POLOS = 14
PARES_DE_POLOS = POLOS / 2

# Amplitude da rede
Vm = 12 #V

# Defasagens das fases
PHI_A = 0
PHI_B = -2*pi/3
PHI_C = 2*pi/3

ti = 0.0
tf = 0.2     # s
dt = 1e-4         # s

bldc1 = bldc(Rs,L,B,J,Ke,Kt,PARES_DE_POLOS,Vm)

time,Vabc,eabc,iabc,Te,omega_r,_ = bldc1.simulation_open_loop(t0=ti,tf=tf,dt=dt,
                                                           Tl=Tl, 
                                                           back_emf_trapezoidal_flag=True)

plt.figure(figsize=(12, 10))
plt.subplot(411)

plt.title("Vabc")
plt.plot(time, eabc.T,label=["ea","eb","ec"])

plt.grid()
plt.legend()

plt.subplot(412)

plt.title("Iabc")
plt.plot(time, iabc.T, label=["ia","ib","ic"])

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