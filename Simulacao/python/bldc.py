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

def rads_to_rpm(omega):
    return omega * 60 / (2*pi)
    
def rpm_to_rads(rpm):
    return rpm * 2*pi/60

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

        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0

        self.omega_r = 0.0001
        self.theta_r = 0.0001

        self.Te = 0.0

    def step(self, Va, Vb, Vc, Tl=0.0, dt=1e-5, back_emf_trapezoidal_flag=True):

        theta_e = self.P * self.theta_r
        theta_e = np.mod(theta_e, 2 * np.pi)

        if back_emf_trapezoidal_flag:
            fa = -self.back_emf_trapezoidal(theta_e + self.PHI_A)
            fb = -self.back_emf_trapezoidal(theta_e + self.PHI_B)
            fc = -self.back_emf_trapezoidal(theta_e + self.PHI_C)
        else:
            fa = -np.sin(theta_e + self.PHI_A)
            fb = -np.sin(theta_e + self.PHI_B)
            fc = -np.sin(theta_e + self.PHI_C)

        omega_e = self.P * self.omega_r

        ea = self.Ke * omega_e * fa
        eb = self.Ke * omega_e * fb
        ec = self.Ke * omega_e * fc

        dia = (Va - self.R * self.ia - ea) / self.L
        dib = (Vb - self.R * self.ib - eb) / self.L
        dic = (Vc - self.R * self.ic - ec) / self.L

        self.ia += dia * dt
        self.ib += dib * dt
        self.ic += dic * dt

        self.Te = self.Kt * (self.ia * fa + self.ib * fb + self.ic * fc)

        domega = (self.Te - Tl - self.B * self.omega_r) / self.J

        self.omega_r += domega * dt
        self.theta_r += self.omega_r * dt

        return {
            "ia": self.ia, "ib": self.ib, "ic": self.ic,
            "ea": ea, "eb": eb, "ec": ec,
            "Te": self.Te,
            "omega_r": self.omega_r,
            "theta_r": self.theta_r,
            "theta_e": theta_e
        }

    def back_emf_trapezoidal(self, theta):
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
        return Clarke_Transform(xabc[0], xabc[1], xabc[2])

    def ClarkeInverse(self, xalphabeta):
        return Clarke_Inverse_Transform(xalphabeta[0], xalphabeta[1])

    def Park(self, xalphabeta, theta):
        return Park_Transform(xalphabeta[0], xalphabeta[1], theta)

    def ParkInverse(self, xdq, theta):
        return Park_Inverse_Transform(xdq[0], xdq[1], theta)

    def set_initial_conditions(self):
        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0
        self.omega_r = 0.0001
        self.theta_r = 0.0001
        self.Te = 0.0