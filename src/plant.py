import numpy as np
import parameters as P


class Plant:
    pass


class VTOL(Plant):
    def __init__(self, h0, z0, theta0, tstep) -> None:
        super().__init__()
        self.delT = tstep
        self.q = np.array([1.0*z0, 1.0*h0, 1.0*theta0])
        self.qdot = np.array([0.0, 0.0, 0.0])
        self.history = {
            "z": [z0],
            "h": [h0],
            "tht": [theta0],
            "zdot": [0.0],
            "hdot": [0.0],
            "thtdot": [0.0]
        }
        self.M = (P.Mc + P.Ml + P.Mr)
        self.J = (P.Ml * P.d**2 + P.Mr * P.d**2 + P.Jc)

    def eom(self, q, qdot, F, tau):
        tht = q[2]
        zdot = qdot[0]
        qddot = np.array([
            - (P.mu / self.M) * zdot - (F / self.M) * np.sin(tht),
            - P.g + (F / self.M) * np.cos(tht),
            tau / self.J
        ])
        return qddot

    def state(self):
        return np.array([
            self.q[0], self.q[1], self.q[2],
            self.qdot[0], self.qdot[1], self.qdot[2]
            ])

    def statedot(self, state, F, tau):
        q = state[:3]
        qdot = state[3:]
        qddot = self.eom(q, qdot, F, tau)
        return np.array([
            qdot[0], qdot[1], qdot[2],
            qddot[0], qddot[1], qddot[2]
        ])

    def update(self, state):
        self.history["z"].append(self.q[0])
        self.history["h"].append(self.q[1])
        self.history["tht"].append(self.q[2])
        self.history["zdot"].append(self.qdot[0])
        self.history["hdot"].append(self.qdot[1])
        self.history["thtdot"].append(self.qdot[2])
        self.q = state[:3]
        self.qdot = state[3:]

    def rk4_step(self, F, tau):
        state = self.state()
        k1 = self.statedot(state, F, tau)
        k2 = self.statedot(state + 0.5 * self.delT * k1, F, tau)
        k3 = self.statedot(state + 0.5 * self.delT * k2, F, tau)
        k4 = self.statedot(state + self.delT * k3, F, tau)
        state += (self.delT / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return state


class Target(Plant):
    def __init__(self, z_0) -> None:
        super().__init__()
        self.q = [z_0]
        self.history = {
            "z": [z_0]
        }

    def move(self, z):
        self.q[0] = z
