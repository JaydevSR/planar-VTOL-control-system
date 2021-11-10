import numpy as np
import parameters as P


class Controller():
    pass


class PIDcontroller(Controller):
    def __init__(self, sample_rate) -> None:
        super().__init__()
        self.kp_h = P.kp_h
        self.kd_h = P.kd_h
        self.kp_z = P.kp_z
        self.kd_z = P.kd_z
        self.ki_z = P.ki_z
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th
        self.F_max = 20
        self.Tau_max = 10*P.d
        self.Ts = sample_rate
        self.integrator_z = 0
        self.error_prev_z = 0

    def PIDupdate(self, z_r, h_r, q, qdot):
        z, h, th = q
        zdot, hdot, thdot = qdot
        e_h = (h_r - h)
        e_z = (z_r - z)

        # Integrate the errors
        self.integrator_z = self.integrate_error(e_z,
                                                 self.error_prev_z,
                                                 self.integrator_z)
        self.error_prev_z = e_z

        # Outer Loop lateral for required theta for DC gain 1 PID controlled
        th_r = self.kp_z*e_z - self.kd_z*zdot + self.ki_z*self.integrator_z

        # Inner loop lateral PD controlled
        Tau = self.kp_th * (th_r - th) - self.kd_th * thdot
        if self.ki_z > 0:
            # To prevent integrator wind-up
            Tau_sat = self.saturate(Tau, self.Tau_max)
            self.integrator_z = self.anti_wind_up(self.integrator_z,
                                                  Tau,
                                                  Tau_sat,
                                                  self.ki_z)

        # Longitudinal PD controlled
        F = self.kp_h * e_h - self.kd_h * hdot

        return self.saturate(F, self.F_max), self.saturate(Tau, self.Tau_max)

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

    def integrate_error(self, error, error_prev, integrator):
        # Using Trapazoidal rule
        integrator = integrator + (error + error_prev) * (self.Ts / 2)
        return integrator

    def anti_wind_up(self, u_I, u, u_sat, k_I):
        return u_I + (u_sat - u) / k_I

    def saturate(self, u, u_max):
        if (abs(u) > u_max):
            u = u_max*np.sign(u)
        return u


class FeedController(Controller):
    def __init__(self, sample_rate):
        super().__init__()
        self.K_lon = P.K_lon  # longitudinal state feedback gain
        self.K_lat = P.K_lat  # lateral state feedback gain
        self.kr_lon = P.kr_lon  # long. input gain
        # self.kr_lat = P.kr_lat  # lat. input gain
        self.ki_lat = P.ki_lat
        self.integrator = 0.0
        self.error_prev = 0.0
        self.F_max = 20  # maximum force long.
        self.Tau_max = 10*P.d  # maximum torque lat.
        self.Ts = sample_rate  # controller sample rate

    def update(self, z_r, h_r, x_lat, x_lon):
        z = x_lat.item(0)
        error = z_r - z
        self.integrate_e(error)
        F = -self.K_lon @ x_lon + self.kr_lon * h_r
        Tau = -self.K_lat @ x_lat - self.ki_lat * self.integrator
        if self.ki_lat > 0:
            # To prevent integrator wind-up
            Tau_sat = self.saturate(Tau, self.Tau_max)
            self.integrator = self.anti_wind_up(self.integrator,
                                                Tau,
                                                Tau_sat,
                                                self.ki_lat)

        return self.saturate(F, self.F_max), self.saturate(Tau, self.Tau_max)

    def integrate_e(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_prev)
        self.error_d1 = error

    def saturate(self, u, u_max):
        if (abs(u) > u_max):
            u = u_max*np.sign(u)
        return u

    def anti_wind_up(self, u_I, u, u_sat, k_I):
        return u_I + (u_sat - u) / k_I
