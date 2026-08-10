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
    "font.size": 18,
    "axes.titlesize": 18,
    "axes.labelsize": 18,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16,
    "legend.fontsize": 18,
    "figure.titlesize": 22,

    # Espessura dos eixos
    "axes.linewidth": 1.2,

    # Tamanho dos ticks
    "xtick.major.size": 6,
    "ytick.major.size": 6,
    "xtick.major.width": 1.2,
    "ytick.major.width": 1.2,
})

DEFAULT_CSV = "closedloop_simulation.csv"
DEFAULT_PASTA = "img"

arq = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
pasta_saida = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PASTA

data = pd.read_csv(arq, sep=';')
print(data.head())
