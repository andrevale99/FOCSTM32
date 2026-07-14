import numpy as np
from numpy import pi

class bldc:
    def __init__(self, R,L,B,J,Ke,Kt,P,Vdc):
        
        self.R = R
        self.L = L
        self.J = J
        self.B = B
        self.P = P
        self.Ke = Ke
        self.Kt = Kt
        self.Vdc = Vdc

        self.PHI_A = 0
        self.PHI_B = -2*pi/3
        self.PHI_C = 2*pi/3

    def simulation_open_loop(self, t0, tf, dt=1e-5, Tl=0, back_emf_trapezoidal_flag=True):
        time = np.arange(t0, tf, dt)
        N = len(time)

        theta_r = np.zeros(N)
        omega_r = np.zeros(N)

        ia = np.zeros(N)
        ib = np.zeros(N)
        ic = np.zeros(N)

        Te = np.zeros(N)

        Va = np.zeros(N)
        Vb = np.zeros(N)
        Vc = np.zeros(N)

        ea = np.zeros(N)
        eb = np.zeros(N)
        ec = np.zeros(N)

        for k in range(N-1):

            # Ângulo e velocidade elétrica do rotor
            theta_e = self.P * theta_r[k]

            # Tensões trifásicas da alimentação
            Va[k] = self.Vdc * np.sin(theta_e  + self.PHI_A)
            Vb[k] = self.Vdc * np.sin(theta_e  + self.PHI_B)
            Vc[k] = self.Vdc * np.sin(theta_e  + self.PHI_C)

            # Back-EMF
            if not back_emf_trapezoidal_flag: 
                fa = np.sin(theta_e + self.PHI_A)
                fb = np.sin(theta_e + self.PHI_B)
                fc = np.sin(theta_e + self.PHI_C)
            else:
                fa = self.back_emf_trapezoidal(theta_e + self.PHI_A)
                fb = self.back_emf_trapezoidal(theta_e + self.PHI_B)
                fc = self.back_emf_trapezoidal(theta_e + self.PHI_C)


            ea[k] = self.Ke * omega_r[k] * fa
            eb[k] = self.Ke * omega_r[k] * fb
            ec[k] = self.Ke * omega_r[k] * fc
    
            # Derivadas das correntes
            dia = (Va[k] - self.R * ia[k] - ea[k]) / self.L
            dib = (Vb[k] - self.R * ib[k] - eb[k]) / self.L
            dic = (Vc[k] - self.R * ic[k] - ec[k]) / self.L

            # Integração (Euler)
            ia[k+1] = ia[k] + dia * dt
            ib[k+1] = ib[k] + dib * dt
            ic[k+1] = ic[k] + dic * dt

            # Torque eletromagnético
            Te[k] = self.Kt * (
                ia[k] * fa +
                ib[k] * fb +
                ic[k] * fc
            )

            # Dinâmica mecânica
            domega = (Te[k] - Tl - self.B * omega_r[k]) / self.J

            omega_r[k+1] = omega_r[k] + domega * dt
            theta_r[k+1] = theta_r[k] + omega_r[k+1] * dt

        return [time, np.array([Va,Vb,Vc]), np.array([ea,eb,ec]),
                np.array([ia,ib,ic]), Te, omega_r, theta_r]

    def back_emf_trapezoidal(self,theta):
        '''
        Back-EMF trapezoidal normalizada (-1 a +1).
        theta em radianos.
        '''
        theta = np.mod(theta, 2*np.pi)

        if theta < np.pi/6:
            return 6*theta/np.pi

        elif theta < 5*np.pi/6:
            return 1.0

        elif theta < 7*np.pi/6:
            return 1 - 6*(theta-5*np.pi/6)/np.pi

        elif theta < 11*np.pi/6:
            return -1.0

        else:
            return -1 + 6*(theta-11*np.pi/6)/np.pi
        
        
# ======================================
# ======================================
# ======================================

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Resistencia de armadura
    Rs = 0.386 #Ohm

    # Indutancia de magnetizacao
    L = 6.53e-5 #H

    # Constantes eletricas e mecanicas
    Ke = 0.0276
    Kt = Ke

    # Torque da carga
    Tl = 0.1

    # Coeficiente de amortecimento
    B = 6.75e-6

    # Momento de inercia
    J = 3.33e-6

    # Quantidade de Polos no motor
    POLOS = 8
    PARES_DE_POLOS = POLOS / 2

    # Amplitude da rede
    Vm = 48*0.8 #V

    # Defasagens das fases
    PHI_A = 0
    PHI_B = -2*pi/3
    PHI_C = 2*pi/3

    TEMPO_MAX = 0.1     # s
    dt = 1e-5         # s

    teste = bldc(Rs,L,B,J,Ke,Kt,PARES_DE_POLOS,Vm)

    time,Vabc,_,iabc,Te,omega_r,_ = teste.simulation_open_loop(0,0.10,Tl=0.1)

    plt.figure(figsize=(12, 10))
    plt.subplot(411)

    plt.title("Vabc")
    plt.plot(time, Vabc[0], label="Va")
    plt.plot(time, Vabc[1], label="Vb")
    plt.plot(time, Vabc[2], label="Vc")

    plt.grid()
    plt.legend()

    plt.subplot(412)

    plt.title("Iabc")
    plt.plot(time, iabc[0], label="ia")
    plt.plot(time, iabc[1], label="ib")
    plt.plot(time, iabc[2], label="ic")

    plt.grid()
    plt.legend()

    plt.subplot(413)

    plt.title("Omega_r")
    plt.plot(time,omega_r, label='omega')

    plt.grid()
    plt.legend()

    plt.subplot(414)

    plt.title("Te")
    plt.plot(time, Te, label="Te")

    plt.grid()
    plt.legend()

    plt.tight_layout()

    plt.show()