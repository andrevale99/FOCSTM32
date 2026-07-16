import numpy as np
from numpy import pi

def Clarke_Transform(xa, xb, xc):
    xalpha = 2/3 * (xa - 1/2*xb - 1/2*xc)
    xbeta = 2/3 * (np.sqrt(3)/2*xb - np.sqrt(3)/2*xc)
    return np.array([xalpha, xbeta])

def Park_Transform(xalpha, xbeta, theta):
    xd = xalpha*np.cos(theta) + xbeta*np.sin(theta)
    xq = -xalpha*np.sin(theta) + xbeta*np.cos(theta)
    return np.array([xd, xq])
   
def Park_Inverse_Transform(xd, xq, theta):
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    xalpha = xd*cos_t - xq*sin_t
    xbeta  = xd*sin_t + xq*cos_t
    return np.array([xalpha, xbeta])

def Clarke_Inverse_Transform(xalpha, xbeta):
    xa = xalpha
    xb = (
        -0.5*xalpha +
        np.sqrt(3)/2*xbeta
    )
    xc = (
        -0.5*xalpha -
        np.sqrt(3)/2*xbeta
    )
    return np.array([xa, xb, xc])

class BLDC():

    def __init__(self, R, L, B, J, Ke, Kt, P, Vdc):

        self.R = R
        self.L = L
        self.B = B
        self.J = J
        self.P = P
        self.Ke = Ke
        self.Kt = Kt
        self.Vdc = Vdc

        self.PHI_A = 0
        self.PHI_B = -2 * pi / 3
        self.PHI_C = 2 * pi / 3

        # ============================================================
        # ESTADOS DO MOTOR
        # ============================================================

        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0

        self.omega_r = 0.0001
        self.theta_r = 0.0001

        self.Te = 0.0

    def rads_to_rpm(self, omega):
        return omega * 60 / (2*pi)
    
    def rpm_to_rads(self,rpm):
        return rpm * 2*pi/60

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

        return [time, np.array([Va,Vb,Vc], dtype=np.float32), np.array([ea,eb,ec], dtype=np.float32),
                np.array([ia,ib,ic], dtype=np.float32), Te, omega_r, theta_r]

    def step(self, Va, Vb, Vc, Tl=0.0, dt=1e-5):
        """
        Executa um passo da simulação do motor BLDC.

        Parameters
        ----------
        Va, Vb, Vc : float
            Tensões instantâneas aplicadas às fases A, B e C [V].

        Tl : float
            Torque de carga [N.m].

        dt : float
            Passo de integração [s].

        Returns
        -------
        dict
            Estados atualizados do motor.
        """

        # ============================================================
        # 1. ÂNGULO ELÉTRICO
        # ============================================================

        theta_e = self.P * self.theta_r

        theta_e = np.mod(
            theta_e,
            2 * np.pi
        )


        # ============================================================
        # 2. FUNÇÕES DA FORÇA CONTRAELETROMOTRIZ
        # ============================================================

        fa = self.back_emf_trapezoidal(
            theta_e + self.PHI_A
        )

        fb = self.back_emf_trapezoidal(
            theta_e + self.PHI_B
        )

        fc = self.back_emf_trapezoidal(
            theta_e + self.PHI_C
        )


        # ============================================================
        # 3. FORÇAS CONTRAELETROMOTRIZES
        # ============================================================

        ea = self.Ke * self.omega_r * fa

        eb = self.Ke * self.omega_r * fb

        ec = self.Ke * self.omega_r * fc


        # ============================================================
        # 4. DERIVADAS DAS CORRENTES
        # ============================================================

        dia = (
            Va
            - self.R * self.ia
            - ea
        ) / self.L

        dib = (
            Vb
            - self.R * self.ib
            - eb
        ) / self.L

        dic = (
            Vc
            - self.R * self.ic
            - ec
        ) / self.L


        # ============================================================
        # 5. INTEGRAÇÃO DAS CORRENTES
        # ============================================================

        self.ia += dia * dt

        self.ib += dib * dt

        self.ic += dic * dt


        # ============================================================
        # 6. TORQUE ELETROMAGNÉTICO
        # ============================================================

        self.Te = self.Kt * (

            self.ia * fa
            + self.ib * fb
            + self.ic * fc

        )


        # ============================================================
        # 7. DINÂMICA MECÂNICA
        # ============================================================

        domega = (

            self.Te
            - Tl
            - self.B * self.omega_r

        ) / self.J


        # ============================================================
        # 8. INTEGRAÇÃO DA VELOCIDADE
        # ============================================================

        self.omega_r += domega * dt


        # ============================================================
        # 9. INTEGRAÇÃO DA POSIÇÃO
        # ============================================================

        self.theta_r += self.omega_r * dt


        # ============================================================
        # 10. RETORNO DOS ESTADOS
        # ============================================================

        return {

            "ia": self.ia,
            "ib": self.ib,
            "ic": self.ic,

            "ea": ea,
            "eb": eb,
            "ec": ec,

            "Te": self.Te,

            "omega_r": self.omega_r,

            "theta_r": self.theta_r,

            "theta_e": theta_e

        }

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

    def Clarke(self, xabc):
        return Clarke_Transform(xabc[0],xabc[1],xabc[2])
    
    def ClarkeInverse(self, xalphabeta):
        return Clarke_Inverse_Transform(xalphabeta[0], xalphabeta[1])
    
    def Park(self, xalphabeta, theta):
        return Park_Transform(xalphabeta[0], xalphabeta[1], theta)

    def ParkInverse(self, xdq, theta):
        return Park_Inverse_Transform(xdq[0],xdq[1], theta)
    
    def set_initial_conditions(self):
        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0

        self.omega_r = 0.0001
        self.theta_r = 0.0001

        self.Te = 0.0