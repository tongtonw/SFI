# -*- coding: utf-8 -*-
"""
Created on Tue Oct 24 10:22:17 2017

@author: laht
"""

import math

TWO_PHI = math.pi * 2


def clamp(x, min, max):
    if x > max:
        x = max
    elif x < min:
        x = min
    return x


def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def sine(A, period, phi, t):
    omega = 2 * math.pi * (1.0 / period)
    return A * math.sin(omega * t + phi)


def dot(v1, v2):
    result = v1.x() * v2.x() + v1.y() * v2.y()
    return result


def angle(v1, v2):
    def angle_trunc(a):
        while a < 0.0:
            a += math.pi * 2
        return a
    return angle_trunc(math.atan2(v2.y()-v1.y(), v2.x() - v1.x()))


class SineWaveGenerator:

    def __init__(self, A=1, period=10, heading=0):
        self.A = A
        self.period = period
        self.heading = math.radians(heading)

    def get_height(self, size, x, y, t):
        if self.heading != 0:
            c = math.cos(self.heading)
            s = math.sin(self.heading)

            x = x * c - y * s
            y = x * s + y * c

        phi = map(x, -size / 8, size / 8, -TWO_PHI, TWO_PHI)
        return sine(self.A, self.period, phi, t)


