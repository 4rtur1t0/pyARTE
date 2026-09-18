#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/chair_euler_angles.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2026
"""
import numpy as np

from artelib.euler import Euler
from robots.simulation import Simulation
from robots.objects import ReferenceFrame

if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start(name='/ReferenceFrameB')
    alpha = np.pi/4
    beta = np.pi/2
    gamma = np.pi/4
    n = 200
    a = np.linspace(0, alpha, n)
    b = np.linspace(0, beta, n)
    g = np.linspace(0, gamma, n)
    for i in range(n):
        frame.set_orientation(Euler([a[i], b[0], g[0]]))
    for i in range(n):
        frame.set_orientation(Euler([a[-1], b[i], g[0]]))
    for i in range(n):
        frame.set_orientation(Euler([a[-1], b[-1], g[i]]))
    simulation.stop()

