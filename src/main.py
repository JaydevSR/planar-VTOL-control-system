from matplotlib import pyplot as plt
import numpy as np
from plant import VTOL, Target
from animate import Animation
from signals import SignalBox
import parameters as P

tstart = 0.0
tend = 20.0
tstep = 0.01
tplot = 0.05

vtol = VTOL(0, 0, 0, tstep)
tar = Target(0)
anim = Animation(tplot)
inputf = SignalBox(freq=0.5, y_offset=1)
ref = SignalBox()

plt.ion()
t = tstart
while (t < tend):
    f_l = inputf.square(t)
    f_r = inputf.sin(t)

    F = f_l + f_r
    Tau = P.d * (f_r - f_l)
    new_state = vtol.rk4_step(F, Tau)
    vtol.update(new_state)
    tar.move(1 + ref.sin(t))
    anim.draw_figure(vtol.q[0], vtol.q[1], vtol.q[2], tar.q[0])
    t += tstep

plt.plot(np.arange(tstart, tend, tstep), vtol.history["h"][1:])
plt.pause(5)
plt.ioff()
