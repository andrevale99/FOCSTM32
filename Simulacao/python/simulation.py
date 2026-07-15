from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from bldc import bldc, svpwm
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

def plot_simulation_data_1(time, eabc, iabc, omega_r, Te):
    plt.figure(figsize=(12, 10))
    plt.subplot(411)

    plt.title("Vabc")
    plt.plot(time, eabc,label=["ea","eb","ec"])

    plt.xlabel('s')
    plt.ylabel('V')
    plt.grid()
    plt.legend()

    plt.subplot(412)

    plt.title("Iabc")
    plt.plot(time, iabc, label=["ia","ib","ic"])

    plt.xlabel('s')
    plt.ylabel('A')
    plt.grid()
    plt.legend()

    plt.subplot(413)

    plt.title("RPM")
    plt.plot(time,omega_r, label='RPM')

    plt.xlabel('s')
    plt.ylabel('RPM')
    plt.grid()
    plt.legend()

    plt.subplot(414)

    plt.title("Te")
    plt.plot(time, Te, label="Te")

    plt.xlabel('s')
    plt.ylabel('Nm')
    plt.grid()
    plt.legend()

    plt.tight_layout()

def plot_simulation_data_2(time, Xalphabeta, XalphabetaLabels, 
                           Xdq, XdqLabels):
    
    plt.figure(figsize=(12, 10))
    
    plt.subplot(211)
    plt.plot(time, Xalphabeta, label=XalphabetaLabels[1])

    plt.title(XalphabetaLabels[0])
    plt.xlabel(XalphabetaLabels[2])
    plt.ylabel(XalphabetaLabels[3])
    plt.legend()
    plt.grid()

    plt.subplot(212)
    plt.plot(time, Xdq, label=XdqLabels[1])

    plt.title(XdqLabels[0])
    plt.xlabel(XdqLabels[2])
    plt.ylabel(XdqLabels[3])
    plt.legend()
    plt.grid()

    plt.tight_layout()

def plot_space_vector(Xalphabeta):
    """
    Plota a trajetória do vetor espacial no plano alpha-beta.
    
    Parameters
    ----------
    Xalphabeta : ndarray
        Array contendo as colunas [Valpha, Vbeta]
    """
    valpha = Xalphabeta[:, 0]
    vbeta = Xalphabeta[:, 1]
    
    plt.figure(figsize=(8, 8))
    plt.plot(valpha, vbeta, label="Trajetória do Vetor")
    
    # Adiciona a origem para referência
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    
    plt.title("Espaço Vetorial (Plano $\\alpha\\beta$)")
    plt.xlabel("$\\alpha$")
    plt.ylabel("$\\beta$")
    plt.grid(True, linestyle='--')
    plt.axis('equal')  # Importante para manter a escala correta e não deformar o círculo
    plt.legend()


def animate_space_vector(time, Xalphabeta, interval_ms=20):
    """
    Cria uma animação do vetor espacial no plano alpha-beta.
    
    Parameters
    ----------
    time : array
        Vetor de tempo da simulação
    Xalphabeta : ndarray
        Array [N, 2] com [Valpha, Vbeta]
    interval_ms : int
        Tempo entre quadros em milissegundos
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Configurações do gráfico
    max_val = np.max(np.abs(Xalphabeta)) * 1.1
    ax.set_xlim(-max_val, max_val)
    ax.set_ylim(-max_val, max_val)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    ax.grid(True, linestyle='--')
    ax.set_aspect('equal')
    ax.set_title("Animação do Vetor Espacial")
    ax.set_xlabel("$\\alpha$")
    ax.set_ylabel("$\\beta$")
    
    # Elementos da animação
    line, = ax.plot([], [], lw=2, label="Trajetória")
    point, = ax.plot([], [], 'ro', label="Posição Atual")
    vector, = ax.plot([], [], 'r-', lw=1)
    ax.legend()

    def init():
        line.set_data([], [])
        point.set_data([], [])
        vector.set_data([], [])
        return line, point, vector

    def update(frame):
        # Mostra a trajetória até o frame atual
        line.set_data(Xalphabeta[:frame, 0], Xalphabeta[:frame, 1])
        # Ponto atual
        point.set_data([Xalphabeta[frame, 0]], [Xalphabeta[frame, 1]])
        # Vetor saindo da origem
        vector.set_data([0, Xalphabeta[frame, 0]], [0, Xalphabeta[frame, 1]])
        return line, point, vector

    # Cria a animação
    ani = FuncAnimation(fig, update, frames=len(time),
                        init_func=init, blit=True, interval=interval_ms)
    
    plt.show()
    return ani

# =================================================

def plot_simulation_all_data(time,
                             eabc,
                             iabc,
                             omega_r,
                             Te,
                             Xalphabeta,
                             XalphabetaLabels,
                             Xdq,
                             XdqLabels):

    plot_simulation_data_1(
        time=time,
        eabc=eabc,
        iabc=iabc,
        omega_r=omega_r,
        Te=Te
    )

    plot_simulation_data_2(
        time=time,
        Xalphabeta=Xalphabeta,
        XalphabetaLabels=XalphabetaLabels,
        Xdq=Xdq,
        XdqLabels=XdqLabels
    )

    plot_space_vector(Xalphabeta)

    plt.tight_layout()

    plt.show()



# =====================================================================
# =====================================================================
# =====================================================================

# Resistencia de armadura
Rs = 0.6 #Ohm

# Indutancia de magnetizacao
L = 5e-3 #H

# Constantes eletricas e mecanicas
Ke = 0.0276
Kt = Ke

# Torque da carga
Tl = 0.0

# Coeficiente de amortecimento
B = 7.312e-7

# Momento de inercia
J = 7.312e-6

# Quantidade de Polos no motor
POLOS = 7
PARES_DE_POLOS = POLOS / 2

# Amplitude da rede
Vdc = 12 #V

# Defasagens das fases
PHI_A = 0
PHI_B = -2*pi/3
PHI_C = 2*pi/3

ti = 0.0
tf = 0.1     # s
dt = 1e-4         # s

bldc1 = bldc(Rs,L,B,J,Ke,Kt,PARES_DE_POLOS,Vdc)
svpwm1 = svpwm(Hz=10000)

sector, angle, mag = svpwm1.get_sector(np.array([1.0, 1.0]))


time,Vabc,eabc,iabc,Te,omega_r,theta_r = bldc1.simulation_open_loop(t0=ti,tf=tf,dt=dt,
                                                           Tl=Tl, 
                                                           back_emf_trapezoidal_flag=True)

# plot_simulation_data_1(time, eabc.T, iabc.T, bldc1.rads_to_rpm(omega_r), Te)

theta_e = theta_r * bldc1.P
Valphabeta = bldc1.Clarke(Vabc)
Vdq = bldc1.Park(Valphabeta, theta_e)

# plot_simulation_data_2(time, Valphabeta.T, ["Valpha e Vbeta",["Valpha","Vbeta"], "s", "V"],
#                        Vdq.T, ["Vd e Vq",["Vd","Vq"], "s", "V"])

# plot_simulation_all_data(
#     time=time,
#     eabc=eabc.T,
#     iabc=iabc.T,
#     omega_r=bldc1.rads_to_rpm(omega_r),
#     Te=Te,
#     Xalphabeta=Valphabeta.T,
#     XalphabetaLabels=[
#         "Valpha e Vbeta",
#         ["Valpha", "Vbeta"],
#         "s",
#         "V"
#     ],
#     Xdq=Vdq.T,
#     XdqLabels=[
#         "Vd e Vq",
#         ["Vd", "Vq"],
#         "s",
#         "V"
#     ]
# )

animate_space_vector(time, Valphabeta.T, interval_ms=10)