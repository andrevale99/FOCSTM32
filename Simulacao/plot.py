import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# Parâmetros
# ============================================================

arquivo = "closedloop_simulation.csv"

# Referência de velocidade utilizada na simulação (rpm).
# Ajuste aqui caso a referência utilizada em main_closedloop.c seja outra.
OMEGA_REF_RPM = 25.0


# ============================================================
# Leitura do arquivo CSV
# ============================================================

dados = pd.read_csv(
    arquivo,
    sep=";"
)


# ============================================================
# Conversões
# ============================================================

# Velocidade angular:
# rad/s -> rpm
dados["omega_r_rpm"] = (
    dados["omega_r"] * 60.0 / (2.0 * 3.141592653589793)
)


# ============================================================
# Criação da janela com subplots
# ============================================================

fig, axes = plt.subplots(
    nrows=4,
    ncols=1,
    figsize=(12, 10),
    sharex=True
)


# ============================================================
# 1. Correntes trifásicas
# ============================================================

axes[0].plot(
    dados["time"],
    dados["ia"],
    label="ia"
)

axes[0].plot(
    dados["time"],
    dados["ib"],
    label="ib"
)

axes[0].plot(
    dados["time"],
    dados["ic"],
    label="ic"
)

axes[0].set_title("Correntes trifásicas")
axes[0].set_ylabel("Corrente (A)")
axes[0].grid(True)
axes[0].legend()


# ============================================================
# 2. Correntes dq
# ============================================================

axes[1].plot(
    dados["time"],
    dados["id"],
    label="id"
)

axes[1].plot(
    dados["time"],
    dados["iq"],
    label="iq"
)

axes[1].plot(
    dados["time"],
    dados["iq_ref"],
    label="iq_ref",
    linestyle="--"
)

axes[1].set_title("Correntes dq")
axes[1].set_ylabel("Corrente (A)")
axes[1].grid(True)
axes[1].legend()


# ============================================================
# 3. Velocidade mecânica com linha de referência
# ============================================================

axes[2].plot(
    dados["time"],
    dados["omega_r_rpm"],
    label="ωr"
)

axes[2].axhline(
    y=OMEGA_REF_RPM,
    color="black",
    linestyle="--",
    label="ωr_ref"
)

axes[2].set_title("Velocidade mecânica do rotor")
axes[2].set_ylabel("Velocidade (rpm)")
axes[2].grid(True)
axes[2].legend()


# ============================================================
# 4. Torque eletromagnético
# ============================================================

axes[3].plot(
    dados["time"],
    dados["Te"],
    label="Te"
)

axes[3].set_title("Torque eletromagnético")
axes[3].set_xlabel("Tempo (s)")
axes[3].set_ylabel("Torque (N·m)")
axes[3].grid(True)
axes[3].legend()


# ============================================================
# Ajuste do layout
# ============================================================

plt.tight_layout()

plt.show()