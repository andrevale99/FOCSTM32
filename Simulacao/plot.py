"""
plot_all.py

Plota TODAS as variaveis do CSV gerado pela simulacao closedloop,
organizadas em janelas (figuras) separadas por tema, e salva cada
uma delas em um arquivo PDF individual.

Uso:
    python3 plot_all.py [arquivo.csv] [pasta_saida]

    arquivo.csv   -- caminho do CSV de saida da simulacao
                     (default: "closedloop_simulation.csv")
    pasta_saida   -- pasta onde os PDFs serao salvos
                     (default: "graficos")

O script e tolerante a diferentes versoes do CSV: colunas que nao
existirem no arquivo (ex.: carrier/gate_a/b/c em simulacoes antigas
sem chaveamento real) sao simplesmente puladas, com um aviso no
terminal. Qualquer coluna numerica que nao se encaixe em nenhum dos
grupos pre-definidos e plotada em uma janela extra ("outras
variaveis"), para garantir que absolutamente tudo seja mostrado.
"""

import os
import sys

import pandas as pd
import matplotlib.pyplot as plt


plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "font.serif": ["cmr10", "DejaVu Serif", "serif"],
    "axes.formatter.use_mathtext": True,

    # Fontes
    "font.size": 20,
    "axes.titlesize": 22,
    "axes.labelsize": 24,
    "xtick.labelsize": 18,
    "ytick.labelsize": 18,
    "legend.fontsize": 18,
    "figure.titlesize": 24,

    # Espessura dos eixos
    "axes.linewidth": 1.2,

    # Tamanho dos ticks
    "xtick.major.size": 6,
    "ytick.major.size": 6,
    "xtick.major.width": 1.2,
    "ytick.major.width": 1.2,
})

# ============================================================
# Parametros
# ============================================================

arquivo = sys.argv[1] if len(sys.argv) > 1 else "closedloop_simulation.csv"
pasta_saida = sys.argv[2] if len(sys.argv) > 2 else "graficos"

# Referencia de velocidade utilizada na simulacao (rpm).
# Ajuste aqui caso a referencia utilizada em main.c seja outra.
# OMEGA_REF_RPM = 100.0


# ============================================================
# Leitura do arquivo CSV
# ============================================================

dados = pd.read_csv(arquivo, sep=";")

colunas_disponiveis = set(dados.columns)
colunas_usadas = {"time"}

os.makedirs(pasta_saida, exist_ok=True)


# ============================================================
# Conversoes
# ============================================================

if "omega_r" in colunas_disponiveis:
    dados["omega_r_rpm"] = dados["omega_r"] * 60.0 / (2.0 * 3.141592653589793)
    colunas_disponiveis.add("omega_r_rpm")


# ============================================================
# Funcoes auxiliares
# ============================================================

def colunas_faltando(cols):
    """Retorna as colunas do grupo que nao existem no CSV."""
    return [c for c in cols if c not in colunas_disponiveis]


def nova_figura(titulo_janela):
    fig = plt.figure(titulo_janela, figsize=(12, 5))
    ax = fig.add_subplot(111)
    return fig, ax


def salvar_figura(fig, nome_arquivo):
    caminho = os.path.join(pasta_saida, nome_arquivo)
    fig.tight_layout()
    fig.savefig(caminho, format="pdf")
    print(f"Salvo: {caminho}")


def plotar_grupo(nome_janela, nome_pdf, titulo, ylabel, series,
                  linhas_ref=None, ylim=None):
    """
    series: lista de tuplas (coluna, rotulo, estilo_de_linha)
    linhas_ref: lista opcional de tuplas (valor, rotulo) para linhas
                horizontais de referencia (ex.: velocidade de referencia)
    """
    faltando = colunas_faltando([c for c, _, _ in series])
    if len(faltando) == len(series):
        print(f"Aviso: nenhuma coluna do grupo '{titulo}' encontrada "
              f"no CSV ({faltando}); janela pulada.")
        return

    fig, ax = nova_figura(nome_janela)

    for coluna, rotulo, estilo in series:
        if coluna not in colunas_disponiveis:
            print(f"Aviso: coluna '{coluna}' nao encontrada no CSV; "
                  f"omitida do grafico '{titulo}'.")
            continue
        ax.plot(dados["time"], dados[coluna], label=rotulo, linestyle=estilo)
        colunas_usadas.add(coluna)

    if linhas_ref:
        for valor, rotulo in linhas_ref:
            ax.axhline(y=valor, color="black", linestyle="--", label=rotulo)

    ax.set_title(titulo)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel(ylabel)
    if ylim is not None:
        ax.set_ylim(*ylim)
    ax.grid(True)
    ax.legend()

    salvar_figura(fig, nome_pdf)


# ============================================================
# 1. Tensoes de fase aplicadas ao motor
# ============================================================

plotar_grupo(
    "Tensoes de fase",
    "01_tensoes_fase.pdf",
    "Tensões de fase aplicadas ao motor",
    "Tensão (V)",
    [("Va", "Va", "-"), ("Vb", "Vb", "-"), ("Vc", "Vc", "-")],
)


# ============================================================
# 2. Correntes trifasicas
# ============================================================

plotar_grupo(
    "Correntes trifasicas",
    "02_correntes_trifasicas.pdf",
    "Correntes trifásicas",
    "Corrente (A)",
    [("ia", "ia", "-"), ("ib", "ib", "-"), ("ic", "ic", "-")],
)


# ============================================================
# 3. Correntes dq
# ============================================================

plotar_grupo(
    "Correntes dq",
    "03_correntes_dq.pdf",
    "Correntes no referencial dq",
    "Corrente (A)",
    [("id", "id", "-"), ("iq", "iq", "-"), ("iq_ref", "iq_ref", "--")],
)


# ============================================================
# 4. Velocidade mecanica do rotor
# ============================================================

plotar_grupo(
    "Velocidade mecanica",
    "04_velocidade.pdf",
    "Velocidade mecânica do rotor",
    "Velocidade (rpm)",
    [("rpm", r"$rpm_{ref}$", "-")],
)


# ============================================================
# 5. Torque eletromagnetico
# ============================================================

plotar_grupo(
    "Torque eletromagnetico",
    "05_torque.pdf",
    "Torque eletromagnético",
    "Torque (N·m)",
    [(r"$T_e$", r"$T_e$", "-")],
)

# plt.show()


# # ============================================================
# # 6. Posicao angular do rotor
# # ============================================================

# plotar_grupo(
#     "Posicao angular",
#     "06_posicao_angular.pdf",
#     "Posição angular do rotor",
#     "Ângulo (rad)",
#     [("theta_r", "θr", "-")],
# )


# # ============================================================
# # 7. Duty cycles do SVPWM
# # ============================================================

# plotar_grupo(
#     "Duty cycles SVPWM",
#     "07_duty_cycles.pdf",
#     "Duty cycles de referência (SVPWM)",
#     "Duty cycle",
#     [("duty_a", "duty_a", "-"), ("duty_b", "duty_b", "-"),
#      ("duty_c", "duty_c", "-")],
#     ylim=(-0.05, 1.05),
# )


# # ============================================================
# # 8. Chaveamento real: portadora e estados das chaves (gate_a/b/c)
# # ============================================================

# if colunas_faltando(["carrier", "gate_a", "gate_b", "gate_c"]) != \
#         ["carrier", "gate_a", "gate_b", "gate_c"]:

#     fig8, (ax8a, ax8b) = plt.subplots(
#         nrows=2, ncols=1, figsize=(12, 7), sharex=True,
#         num="Chaveamento real (portadora e gates)"
#     )

#     if "carrier" in colunas_disponiveis and "duty_a" in colunas_disponiveis:
#         ax8a.plot(dados["time"], dados["carrier"], label="portadora",
#                   color="gray")
#         ax8a.plot(dados["time"], dados["duty_a"], label="duty_a",
#                   linestyle="--")
#         colunas_usadas.update({"carrier"})
#     ax8a.set_title("Portadora triangular vs. duty cycle de referência (fase A)")
#     ax8a.set_ylabel("Amplitude")
#     ax8a.grid(True)
#     ax8a.legend()

#     for coluna, rotulo in [("gate_a", "gate_a"), ("gate_b", "gate_b"),
#                             ("gate_c", "gate_c")]:
#         if coluna in colunas_disponiveis:
#             ax8b.step(dados["time"], dados[coluna], label=rotulo,
#                       where="post")
#             colunas_usadas.add(coluna)
#     ax8b.set_title("Estado de chaveamento (0/1) por braço do inversor")
#     ax8b.set_xlabel("Tempo (s)")
#     ax8b.set_ylabel("Estado")
#     ax8b.set_ylim(-0.2, 1.2)
#     ax8b.grid(True)
#     ax8b.legend()

#     salvar_figura(fig8, "08_chaveamento_real.pdf")
# else:
#     print("Aviso: colunas de chaveamento (carrier/gate_a/b/c) nao "
#           "encontradas no CSV; janela pulada (CSV de uma simulacao "
#           "sem chaveamento real / versao antiga do main.c).")


# # ============================================================
# # 9. Qualquer outra coluna numerica nao coberta acima
# # ============================================================

# colunas_restantes = [
#     c for c in dados.columns
#     if c not in colunas_usadas and pd.api.types.is_numeric_dtype(dados[c])
# ]

# if colunas_restantes:
#     print(f"Colunas adicionais encontradas no CSV: {colunas_restantes} "
#           f"-- plotando em janelas extras.")
#     for i, coluna in enumerate(colunas_restantes, start=1):
#         plotar_grupo(
#             f"Outras variaveis - {coluna}",
#             f"09_{i:02d}_{coluna}.pdf",
#             f"Variável: {coluna}",
#             coluna,
#             [(coluna, coluna, "-")],
#         )


# # ============================================================
# # Exibe todas as janelas na tela (alem de ja terem sido salvas em PDF)
# # ============================================================

# print(f"\nTodos os graficos foram salvos em PDF na pasta '{pasta_saida}/'.")
# # plt.show()