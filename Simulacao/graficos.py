import os
import sys

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

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
DEFAULT_PASTA = "img/"
DEFAULT_FIGSIZE = (15,10)

no_xlabel = plt.tick_params(axis='x', labelbottom=False)

str_time = 'time'
str_va = 'Va'
str_vb = 'Vb'
str_vc = 'Vc'
str_ia = 'ia'
str_ib = 'ib'
str_ic = 'ic'
str_ea = 'ea'
str_eb = 'eb'
str_ec = 'ec'
str_id = 'id'
str_iq = 'iq'
str_te = 'Te'
str_thetar = 'theta_r'
str_omegar = 'omega_r'
str_iqref = 'iq_ref'
str_vdref = 'vd_ref'
str_vqref = 'vq_ref'

#===========================================================
#===========================================================
#===========================================================

arq = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
pasta_saida = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PASTA

data = pd.read_csv(arq, sep=';')

time = data[str_time]
vabc = np.array([data[str_va],data[str_vb],data[str_vc]])
iabc = np.array([data[str_ia],data[str_ib],data[str_ic]])
eabc = np.array([data[str_ea],data[str_eb],data[str_ec]])
iq = data[str_iq]
_id = data[str_id]
te = data[str_te]
omegar = data[str_omegar]
rpm = omegar * 60./(2*np.pi)
iqref = data[str_iqref]
vdref = data[str_vdref]
vqref = data[str_vqref]

#===========================================================
# tempo x iabc,rpm,Te,iq,id
#===========================================================

plt.figure(figsize=DEFAULT_FIGSIZE)
plt.subplot(211)
plt.plot(time, iabc.T)
plt.ylabel('A')
plt.grid()
no_xlabel

plt.subplot(212)
plt.plot(time,rpm)
plt.grid()
plt.ylabel('RPM')
plt.xlabel('s')

plt.tight_layout()

plt.savefig(pasta_saida+"01_corrente-rpm-Te.pdf")

#===========================================================
# tempo x iq,id,iqref,te
#===========================================================

plt.figure(figsize=DEFAULT_FIGSIZE)
plt.subplot(211)
plt.plot(time, _id, label=r'$i_{d}$')
plt.plot(time, iq, label=r'$i_{q}$')
plt.plot(time, iqref, label=r'$i_{qref}$', ls='--')
plt.ylabel('A')
plt.grid()
no_xlabel

plt.subplot(212)
plt.plot(time,te)
plt.grid()
plt.ylabel('Nm')
plt.xlabel('s')

plt.tight_layout()

plt.savefig(pasta_saida+"02_iq-id-Te.pdf")
