import numpy as np
import parameters as P


class Controller():
    pass


class VTOLcontroller(Controller):
    def __init__(self) -> None:
        super().__init__()
        self.kp_h = P.kp_h
        self.kd_h = P.kd_h
        self.kp_z = P.kp_z
        self.kd_z = P.kd_z
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th
        self.F_max = 20
        self.Tau_max = 10*P.d

    def PDupdate(self, z_r, h_r, q, qdot):
        z, h, th = q
        zdot, hdot, thdot = qdot
        e_h = (h_r - h)
        e_z = (z_r - z)

        # Outer Loop lateral for required theta for DC gain 1
        th_r = self.kp_z * e_z - self.kd_z * zdot
        # Inner loop lateral
        Tau = self.kp_th * (th_r - th) - self.kd_th * thdot
        # Longitudinal
        F = self.kp_h * e_h - self.kd_h * hdot
        return self.saturate(F, self.F_max), self.saturate(Tau, self.Tau_max)

    def PIDupdate(self, z_r, h_r, q, qdot):
        pass

    def saturate(self, u, limit):
        if (abs(u) > limit):
            u = self.u_max*np.sign(u)
        return u
