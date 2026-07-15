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

class bldc():

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
    
class svpwm:
    def __init__(self, Hz=0, Ts=0):
        if Ts <= 0 and Hz <= 0:
            raise ValueError("Ts ou Hz devem ser maiores que zero.")

        if Ts == 0:
            Ts = 1/Hz

        if Hz == 0:
            Hz = 1/Ts

        self.Ts = Ts
        self.Hz = Hz

    def get_sector(self, xalphabeta):
        """
        Retorna o setor (1 a 6), o ângulo do vetor espacial e seu módulo.

        Parameters
        ----------
        xalphabeta : array_like
            [Valpha, Vbeta]

        Returns
        -------
        sector : int
            Setor do SVPWM (1 a 6)
        angle : float
            Ângulo do vetor espacial em radianos [0, 2π)
        magnitude : float
            Módulo do vetor espacial
        """

        alpha = xalphabeta[0]
        beta = xalphabeta[1]

        # Módulo
        magnitude = np.hypot(alpha, beta)

        # Ângulo entre 0 e 2π
        angle = np.arctan2(beta, alpha)

        if angle < 0:
            angle += 2*np.pi

        # Setor (1 a 6)
        sector = int(angle // (np.pi/3)) + 1

        # Evita setor 7 por erro numérico
        if sector > 6:
            sector = 6

        return sector, angle, magnitude