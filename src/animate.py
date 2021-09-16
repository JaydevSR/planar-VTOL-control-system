import matplotlib.pyplot as plt
import matplotlib.patches as patch
import numpy as np
import parameters as par


class Animation:
    def __init__(self, timedelta=0.05) -> None:
        self.fig, self.ax = plt.subplots()
        self.init = True
        self.timedelta = timedelta

    def draw_figure(self, h, z, theta) -> None:
        self.draw_VTOL(h, z, theta)
        plt.axis([z-2, z+2, h-2, h+2])
        plt.show()
        plt.pause(self.timedelta)

    def draw_VTOL(self, h, z, theta):
        # centers of the rotors
        rot1 = (z - par.d * np.cos(theta), h - par.d * np.sin(theta))
        rot2 = (z + par.d * np.cos(theta), h + par.d * np.sin(theta))

        # bottom left corner of the body (square)
        bod = (z - par.side_bod * np.cos(theta + np.pi/4) / np.sqrt(2),
               h - par.side_bod * np.sin(theta + np.pi/4) / np.sqrt(2))

        if self.init:
            self.body = patch.Rectangle(bod, par.side_bod, par.side_bod,
                                        facecolor='black')
            self.rotor1 = patch.Ellipse(rot1, par.rad_rot*1.5, par.rad_rot,
                                        angle=theta, facecolor='black')
            self.rotor2 = patch.Ellipse(rot2, par.rad_rot*1.5, par.rad_rot,
                                        angle=theta, facecolor='black')
            self.rod_joint = plt.Line2D((rot1[0], rot2[0]),
                                        (rot1[1], rot2[1]),
                                        lw=2, color='black')
            self.ax.add_line(self.rod_joint)
            self.ax.add_patch(self.body)
            self.ax.add_patch(self.rotor1)
            self.ax.add_patch(self.rotor2)
            self.init = False
        else:
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
    while (t < 10):
        anim.draw_figure(h, z, np.sin(theta))
        t += delta
        h += delta
        z += delta
        theta += delta
    plt.ioff()
