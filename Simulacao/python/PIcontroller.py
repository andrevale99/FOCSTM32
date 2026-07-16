import numpy as np
from numpy import pi

class PIController:

    def __init__(self, Kp, Ki, Ts, output_min=None, output_max=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Ts = Ts
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.saturated = False  # instrumentacao para diagnostico

    def reset(self):
        self.integral = 0.0

    def update(self, reference, feedback):
        error = reference - feedback
        proportional = self.Kp * error
        self.integral += self.Ki * error * self.Ts
        output = proportional + self.integral

        self.saturated = False

        if self.output_min is not None:
            if output < self.output_min:
                output = self.output_min
                self.saturated = True
                if error < 0:
                    self.integral -= self.Ki * error * self.Ts

        if self.output_max is not None:
            if output > self.output_max:
                output = self.output_max
                self.saturated = True
                if error > 0:
                    self.integral -= self.Ki * error * self.Ts

        return output