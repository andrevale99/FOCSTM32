import numpy as np

# --- Parametros do motor ---
R  = 2.9              # Ohm       - resistencia de armadura
L  = 0.0202           # H         - indutancia de magnetizacao
M  = 0.0              # H         - indutancia mutua (nao usada em bldc_step)
Ke = 1.1       # V/(rad/s) - constante eletrica (FCEM)
J  = 1.0e-3           # kg.m^2    - momento de inercia do rotor
B  = 1e-2             # N.m/(rad/s) - coeficiente de atrito viscoso
Tl = 0.0              # N.m       - torque de carga aplicado ao eixo
P  = 4               # -         - numero de pares de polos
Kt = 1.1       # N.m/A     - constante de torque

# --- Parametros do SVPWM (chaveamento real do inversor) ---
Fsw        = 10000   # Hz - frequencia de chaveamento

omegacc = Fsw / 10 * 2 * np.pi
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
