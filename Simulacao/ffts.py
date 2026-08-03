import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ============================================================
# Configurações
# ============================================================

arquivo = "bldc_1_simulation.csv"   # Nome do arquivo CSV
f_max = 20000                       # Frequência máxima do gráfico (Hz)

# ============================================================
# Leitura dos dados
# ============================================================

df = pd.read_csv(arquivo, sep=';')

# Nome da coluna de tempo (altere se necessário)
tempo = df["time"].values

# Correntes trifásicas
ia = df["ia"].values
ib = df["ib"].values
ic = df["ic"].values

# ============================================================
# Frequência de amostragem
# ============================================================

Ts = np.mean(np.diff(tempo))
Fs = 1 / Ts

print(f"Frequência de amostragem: {Fs:.2f} Hz")

# ============================================================
# Função FFT
# ============================================================

def calcular_fft(sinal, Fs):

    N = len(sinal)

    # Remove componente DC
    sinal = sinal - np.mean(sinal)

    # Janela de Hann
    janela = np.hanning(N)

    sinal = sinal * janela

    # FFT
    fft = np.fft.rfft(sinal)

    # Correção da amplitude
    amplitude = 2 * np.abs(fft) / np.sum(janela)

    frequencia = np.fft.rfftfreq(N, d=1/Fs)

    return frequencia, amplitude

# ============================================================
# FFT das três correntes
# ============================================================

f_ia, A_ia = calcular_fft(ia, Fs)
f_ib, A_ib = calcular_fft(ib, Fs)
f_ic, A_ic = calcular_fft(ic, Fs)

# ============================================================
# Limita até 20 kHz
# ============================================================

idx = f_ia <= f_max

# ============================================================
# Plota
# ============================================================

plt.figure(figsize=(12,7))

plt.plot(f_ia[idx], A_ia[idx], label='Ia')
plt.plot(f_ib[idx], A_ib[idx], label='Ib')
plt.plot(f_ic[idx], A_ic[idx], label='Ic')

plt.title("FFT das Correntes Trifásicas")
plt.xlabel("Frequência (Hz)")
plt.ylabel("Amplitude (A)")
plt.xlim(0, f_max)
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()