#!/usr/bin/env python
# encoding: utf-8
"""
Given three Euler angles, compute a rotation matrix for a given convention, e. g. XYZ.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.rotationmatrix import RotationMatrix


def rot(alpha, axis):
    if axis == 'x':
        R = np.array([[1, 0, 0],
                         [0, np.cos(alpha), -np.sin(alpha)],
                         [0, np.sin(alpha), np.cos(alpha)]])
    elif axis == 'y':
        R = np.array([[np.cos(alpha), 0, np.sin(alpha)],
                         [0, 1, 0],
                         [-np.sin(alpha), 0, np.cos(alpha)]])
    else:
        R = np.array([[np.cos(alpha), -np.sin(alpha), 0],
                         [np.sin(alpha), np.cos(alpha), 0],
                         [0, 0, 1]])
    return RotationMatrix(R)


if __name__ == '__main__':
    e = [np.pi/2, 0, np.pi/2]
    alpha = e[0]
    beta = e[1]
    gamma = e[2]
    Rx = rot(alpha, 'x')
    Ry = rot(beta, 'y')
    Rz = rot(gamma, 'z')

    R = Rx*Ry*Rz
    print(R)
    R.plot()


