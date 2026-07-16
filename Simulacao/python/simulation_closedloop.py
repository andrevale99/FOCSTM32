from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from bldc import bldc, svpwm, PIController
from numpy import pi
import numpy as np
import os


PATH = os.getcwd()
print(PATH + '/')
print(os.listdir(PATH))

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.serif": ["cmr10", "DejaVu Serif", "serif"],
    "axes.formatter.use_mathtext": True,
    "font.size": 12
})

# Amplitude da rede
Vdc = 24 #V

# Resistencia de armadura
Rs = 0.36 #Ohm

# Indutancia de magnetizacao
L = 6e-4 #H

# Constantes eletricas e mecanicas
Ke = 0.0276
Kt = Ke

# Torque da carga
Tl = 0.125

# Coeficiente de amortecimento
B = 7.312e-7

# Momento de inercia
J = 4.6e-6

# Quantidade de Polos no motor
POLOS = 8
PARES_DE_POLOS = POLOS / 2

# Defasagens das fases
PHI_A = 0
PHI_B = -2*pi/3
PHI_C = 2*pi/3

ti = 0.0 #s
tf = 0.1 #s
dt = 1e-4 #s

Ts = dt

I_MAX = 2
V_MAX = Vdc

bldc1 = bldc(Rs,L,B,J,Ke,Kt,PARES_DE_POLOS,Vdc)
svpwm1 = svpwm(Hz=10000, Vdc=Vdc)

speed_controller = PIController(
    Kp=...,
    Ki=...,
    Ts=Ts,
    output_min=-I_MAX,
    output_max=I_MAX
)

id_controller = PIController(
    Kp=...,
    Ki=...,
    Ts=Ts,
    output_min=-V_MAX,
    output_max=V_MAX
)

iq_controller = PIController(
    Kp=...,
    Ki=...,
    Ts=Ts,
    output_min=-V_MAX,
    output_max=V_MAX
)