#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/chair_euler_angles.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2026
"""
import numpy as np
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.objects import ReferenceFrame
from robots.simulation import Simulation

if __name__ == "__main__":
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start(name='/ReferenceFrameB')
    position = Vector([.5, .5, .5])
    alfa = np.linspace(0, np.pi/3, 1000)
    beta = np.linspace(0, np.pi/2, 1000)
    gamma = np.linspace(0, np.pi, 1000)
    for i in range(len(alfa)):
        orientation = Euler([alfa[i], 0, 0])
        frame.show_target_point(target_position=position, target_orientation=orientation)

    for i in range(len(beta)):
        orientation = Euler([alfa[-1], beta[i], 0])
        frame.show_target_point(target_position=position, target_orientation=orientation)

    for i in range(len(beta)):
        orientation = Euler([alfa[-1], beta[-1], gamma[i]])
        frame.show_target_point(target_position=position, target_orientation=orientation)

    # with beta = pi/2,

    for i in range(len(beta)):
        orientation = Euler([gamma[i], np.pi/2, gamma[i]])
        frame.show_target_point(target_position=position, target_orientation=orientation)
    simulation.stop()


