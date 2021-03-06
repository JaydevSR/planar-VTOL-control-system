import matplotlib.patches as patch
import matplotlib.pyplot as plt
import numpy as np

import parameters as par


class Animation:
    def __init__(self, timedelta=0.05) -> None:
        self.fig, self.ax = plt.subplots()
        self.ax.set_ymargin(0)
        self.scatter = self.ax.plot([], [])
        self.init = True
        self.timedelta = timedelta
        self.history = dict()

    def draw_figure(self, z, h, theta) -> None:
        self.draw_VTOL(z, h, theta)
        self.scatter[0].set_xdata(self.history['z'])
        self.scatter[0].set_ydata(self.history['h'])
        # x_range = (min(z, z_t)-1, max(z_t, z)+1)
        # y_range = (0, h+1)
        # window_side = max(abs(x_range[0] - x_range[1]),
        #                   abs(y_range[0] - y_range[1]))
        # x_padding = 0.5*(window_side - abs(x_range[0] - x_range[1]))
        # self.ax.set_xlim(x_range[0] - x_padding, x_range[1] + x_padding)
        # self.ax.set_ylim(0, window_side)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.show()
        plt.pause(self.timedelta)

    def draw_VTOL(self, z, h, theta):
        # centers of the rotors
        rot1 = (z - par.d * np.cos(theta), h - par.d * np.sin(theta))
        rot2 = (z + par.d * np.cos(theta), h + par.d * np.sin(theta))

        # bottom left corner of the body (square)
        bod = (z - par.side_bod * np.cos(theta + np.pi/4) / np.sqrt(2),
               h - par.side_bod * np.sin(theta + np.pi/4) / np.sqrt(2))

        # # bottom left corner of the target (square)
        # tar = (z_t - par.side_tar / 2, 0)

        if self.init:
            self.history["z"] = [z]
            self.history["h"] = [h]
            self.body = patch.Rectangle(bod, par.side_bod, par.side_bod,
                                        facecolor='black')
            self.rotor1 = patch.Ellipse(rot1, par.rad_rot*2.5, par.rad_rot,
                                        angle=theta, facecolor='black')
            self.rotor2 = patch.Ellipse(rot2, par.rad_rot*2.5, par.rad_rot,
                                        angle=theta, facecolor='black')
            self.rod_joint = plt.Line2D((rot1[0], rot2[0]),
                                        (rot1[1], rot2[1]),
                                        lw=1, color='black')
            # self.target = patch.Rectangle(tar, par.side_tar, par.side_tar)

            self.ax.add_line(self.rod_joint)
            self.ax.add_patch(self.body)
            self.ax.add_patch(self.rotor1)
            self.ax.add_patch(self.rotor2)
            # self.ax.add_patch(self.target)
            self.init = False
        else:
            self.history["z"].append(z)
            self.history["h"].append(h)
            # self.target.set_xy(tar)
            self.body.set_xy(bod)
            self.body.angle = np.rad2deg(theta)
            self.rotor1.set_center(rot1)
            self.rotor1.set_angle(np.rad2deg(theta))
            self.rotor2.set_center(rot2)
            self.rotor2.set_angle(np.rad2deg(theta))
            self.rod_joint.set_xdata((rot1[0], rot2[0]))
            self.rod_joint.set_ydata((rot1[1], rot2[1]))


# Test Animation
if __name__ == "__main__":
    plt.ion()
    anim = Animation()
    delta = 0.1
    t = z = h = theta = 0
    h = 4
    while (t < 20):
        anim.draw_figure(h * (np.sin(t) ** 2) + 1,
                         z + 0.5 * np.sin(t),
                         np.sin(theta),
                         z + 0.5 * np.cos(t))
        t += delta
        z += delta
        theta += delta
    plt.ioff()
