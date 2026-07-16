import numpy as np
from numpy import pi

class SVPWM:

    def __init__(self, Hz=0, Ts=0, Vdc=0):

        if Vdc <= 0:
            raise ValueError(
                "Vdc deve ser maior que zero."
            )

        if Ts <= 0 and Hz <= 0:
            raise ValueError(
                "Ts ou Hz devem ser maiores que zero."
            )

        if Ts == 0:
            Ts = 1 / Hz

        if Hz == 0:
            Hz = 1 / Ts

        self.Vdc = Vdc
        self.Ts = Ts
        self.Hz = Hz


    def modulate(self, Valpha, Vbeta):

        # Transformação inversa de Clarke
        Va_ref = Valpha

        Vb_ref = (
            -0.5 * Valpha +
            np.sqrt(3) / 2 * Vbeta
        )

        Vc_ref = (
            -0.5 * Valpha -
            np.sqrt(3) / 2 * Vbeta
        )

        # Maior e menor tensão
        Vmax = max(
            Va_ref,
            Vb_ref,
            Vc_ref
        )

        Vmin = min(
            Va_ref,
            Vb_ref,
            Vc_ref
        )

        # Tensão de modo comum
        Voffset = -0.5 * (
            Vmax + Vmin
        )

        # Tensões moduladas
        Va_mod = Va_ref + Voffset
        Vb_mod = Vb_ref + Voffset
        Vc_mod = Vc_ref + Voffset

        # Duty cycles
        duty_a = (
            Va_mod / self.Vdc
            + 0.5
        )

        duty_b = (
            Vb_mod / self.Vdc
            + 0.5
        )

        duty_c = (
            Vc_mod / self.Vdc
            + 0.5
        )

        return np.clip(
            np.array([
                duty_a,
                duty_b,
                duty_c
            ]),
            0.0,
            1.0
        )


    def get_sector(self, xalphabeta):

        alpha = xalphabeta[0]
        beta = xalphabeta[1]

        magnitude = np.hypot(
            alpha,
            beta
        )

        angle = np.mod(
            np.arctan2(beta, alpha),
            2 * np.pi
        )

        sector = int(
            angle // (np.pi / 3)
        ) + 1

        if sector > 6:
            sector = 6

        return (
            sector,
            angle,
            magnitude
        )