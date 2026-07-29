import numpy as np

R  = 0.5              # Ohm       - resistencia de armadura
L  = 0.3e-3           # H         - indutancia de magnetizacao
M  = 0.0              # H         - indutancia mutua (nao usada em bldc_step)
Ke = 0.00909483       # V/(rad/s) - constante eletrica (FCEM)
J  = 1.0e-4           # kg.m^2    - momento de inercia do rotor
B  = 5e-5             # N.m/(rad/s) - coeficiente de atrito viscoso
Tl = 0.0              # N.m       - torque de carga aplicado ao eixo
P  = 14               # -         - numero de pares de polos
Kt = 0.00909483       # N.m/A     - constante de torque


omegacc = 3141

Kpq = L * omegacc
Kiq = R * omegacc

print(f'Kpq = {Kpq}')
print(f'Kiq = {Kiq}')

omegacs = 314
Kpomega = J * omegacs / Kt
Kiomega = J * omegacs**2 / (5*Kt)

print(f'Kpomega = {Kpomega}')
print(f'Kiomega = {Kiomega}')