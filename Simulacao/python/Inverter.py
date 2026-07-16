import numpy as np
from numpy import pi


class Inverter:

    def __init__(self, Vdc):

        self.Vdc = Vdc

    def duty_to_pole_voltage(self, duty):

        """
        Converte o duty cycle na tensão média do polo da fase.

        duty = 0  -> 0 V
        duty = 1  -> Vdc
        """

        return duty * self.Vdc

    def output_voltage(self, duty_a, duty_b, duty_c):

        """
        Calcula as tensões de fase aplicadas ao motor.

        Entrada:
            duty_a
            duty_b
            duty_c

        Saída:
            Va
            Vb
            Vc
        """

        # Limitação dos duty cycles
        duty_a = np.clip(duty_a, 0.0, 1.0)
        duty_b = np.clip(duty_b, 0.0, 1.0)
        duty_c = np.clip(duty_c, 0.0, 1.0)

        # Tensões dos polos em relação ao barramento negativo
        Va_pole = duty_a * self.Vdc
        Vb_pole = duty_b * self.Vdc
        Vc_pole = duty_c * self.Vdc

        # Tensão do ponto neutro virtual
        Vn = (
            Va_pole +
            Vb_pole +
            Vc_pole
        ) / 3.0

        # Tensões de fase
        Va = Va_pole - Vn
        Vb = Vb_pole - Vn
        Vc = Vc_pole - Vn

        return np.array([
            Va,
            Vb,
            Vc
        ])