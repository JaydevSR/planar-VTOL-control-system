import numpy as np


class SignalBox:

    def __init__(self, amp=1.0, freq=0.001, y_offset=0):
        self.amp = amp  # signal amplitude
        self.freq = freq  # signal frequency
        self.y_offset = y_offset  # signal y_offset

    def square(self, t):
        if t % (1.0/self.freq) <= 0.5/self.freq:
            out = self.amp + self.y_offset
        else:
            out = - self.amp + self.y_offset
        return out

    def sawtooth(self, t):
        tmp = t % (0.5/self.freq)
        out = 4 * self.amp * self.freq*tmp - self.amp + self.y_offset
        return out

    def step(self, t):
        if t >= 0.0:
            out = self.amp + self.y_offset
        else:
            out = self.y_offset
        return out

    def random(self, t):
        out = np.random.normal(self.y_offset, self.amp)
        return out

    def sin(self, t):
        out = self.amp * np.sin(2*np.pi*self.freq*t) + self.y_offset
        return out
