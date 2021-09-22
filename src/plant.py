import numpy as np
import parameters as P


class Plant:
    pass


class VTOL(Plant):

    def __init__(self, h0, z_v0, theta0, z_t0) -> None:
        super().__init__()
        self.q = np.array([h0, z_v0, theta0, z_t0])
        self.history = {
            "h": [h0],
            "z_v": [z_v0],
            "theta": [theta0],
            "z_t": [z_t0]
        }

    def KE(self):
        th = self.q[2]
        hdot, zvdot, thdot = self.qdot[0:2]
        K = 0.5 * (P.Mc) * (zvdot**2 + hdot**2) + 0.5 * (P.Jc) * thdot**2
        K += 0.5 * (P.Ml + P.Mr) * (zvdot**2 + hdot**2)
        K += 0.5 * (P.Ml + P.Mr) * (P.d)**2 * thdot**2
        K += (P.Ml-P.Mr) * (zvdot*np.sin(th) - hdot*np.cos(th)) * (P.d) * thdot
        return K

    def PE(self):
        h = self.q[1]
        return (P.Ml + P.Mr + P.Mc) * P.g * h
