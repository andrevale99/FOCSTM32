import numpy as np

# --- Parametros do motor ---
R  = 0.161           # Ohm  - resistencia de armadura
L  = 0.052e-3         # H    - indutancia de magnetizacao
M  = 0.0            # H    - indutancia mutua
Ke = 0.0255     # V/(rad/s) - constante eletrica (2.67 V/krpm)
J  = 43.3e-3         # kg.m^2 - momento de inercia
B  = 1.2e-6         # coeficiente de amortecimento
Tl = 0.0            # N.m  - torque de carga
P  = 8             # numero de pares de polos
Kt = 0.0255     # N.m/A - constante de torque

# --- Parametros do SVPWM (chaveamento real do inversor) ---
Fsw        = 10000   # Hz - frequencia de chaveamento

omegacc = Fsw / 20 * 2 * np.pi
Kpq = L * omegacc
Kiq = R * omegacc

Kpq = round(Kpq,3)
Kiq = round(Kiq,3)

omegacs = omegacc / 5 
Kpomega = J * omegacs / Kt
Kiomega = J * omegacs**2 / (5*Kt)

Kpomega = round(Kpomega,3)
Kiomega = round(Kiomega,3)

print(f'KpOmega = {Kpomega}     # ganho proporcional - malha de velocidade')
print(f'KiOmega = {Kiomega}     # ganho integral - malha de velocidade')
print(f'KpId    = {Kpq}         # ganho proporcional - malha de corrente id')
print(f'KiId    = {Kiq}         # ganho integral - malha de corrente id')
print(f'KpIq    = {Kpq}         # ganho proporcional - malha de corrente iq')
print(f'KiIq    = {Kiq}         # ganho integral - malha de corrente iq')
