import numpy as np

R  = 0.161           # Ohm  - resistencia de armadura
L  = 0.052e-3         # H    - indutancia de magnetizacao
M  = 0.0            # H    - indutancia mutua
Ke = 0.0255     # V/(rad/s) - constante eletrica (2.67 V/krpm)
J  = 43.3e-3         # kg.m^2 - momento de inercia
B  = 1.2e-6         # coeficiente de amortecimento
Tl = 0.0            # N.m  - torque de carga
P  = 8             # numero de pares de polos
Kt = 0.0255     # N.m/A - constante de torque

omegacc = 6200

Kpq = L * omegacc
Kiq = R * omegacc

print(f'Kpq = {Kpq}')
print(f'Kiq = {Kiq}')

omegacs = 314
Kpomega = J * omegacs / Kt
Kiomega = J * omegacs**2 / (5*Kt)

print(f'Kpomega = {Kpomega}')
print(f'Kiomega = {Kiomega}')