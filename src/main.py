from matplotlib import pyplot as plt
import numpy as np
from plant import VTOL
from animate import Animation
from signals import SignalBox
# import parameters as P
from controller import PIDcontroller, FeedController

tstart = 0.0
tend = 40.0
tstep = 0.01
tplot = 0.0001

vtol = VTOL(0, 0, 0, tstep)
# tar = Target(0)
control = FeedController(tstep)
anim = Animation(tplot)
ref = SignalBox()
ref2 = SignalBox(2.5, 0.08, 3)

plt.ion()
t = tstart
while (t < tend):
    h_r = ref.step(t)
    z_r = ref2.sin(t)

    xlon = np.array([vtol.q[1], vtol.qdot[1]])
    xlat = np.array([vtol.q[0], vtol.qdot[0], vtol.q[2], vtol.qdot[2]])

    F, Tau = control.update(z_r, h_r, xlat, xlon)
    new_state = vtol.rk4_step(F, Tau)
    vtol.update(new_state)
    anim.draw_figure(vtol.q[0], vtol.q[1], vtol.q[2])
    t += tstep

plt.cla()
plt.plot(np.arange(tstart, tend, tstep),
         vtol.history["h"][:-1], label="response")
plt.plot(np.arange(tstart, tend, tstep),
         [ref.step(t) for t in np.arange(tstart, tend, tstep)],
         label="reference")
plt.xlabel("time (s)")
plt.ylabel("height (m)")
plt.legend()
plt.waitforbuttonpress()
# plt.savefig("reports/plots/step_response_long_F8.png")

plt.cla()

# steady_state_error = np.abs(np.array([ref2.step(t) for t in np.arange(tstart, tend, tstep)]) - np.array(vtol.history["z"][:-1]))
# plt.plot(np.arange(tstart, tend, tstep),
#          vtol.history["z"][:-1], label="response")
# plt.plot(np.arange(tstart, tend, tstep),
#          [ref2.sin(t) for t in np.arange(tstart, tend, tstep)],
#          label="reference")

# plt.plot(np.arange(tstart, tend, tstep), steady_state_error, label="steady state error")
# plt.xlabel("time (s)")
# plt.ylabel("displacement (m)")
# plt.legend()
# plt.waitforbuttonpress()
# plt.savefig("reports/plots/step_steady_state_err_F10.png")
plt.ioff()
