#!/usr/bin/env python
# encoding: utf-8
"""
This script introduces you to the use of the classes:
    HomogeneousMatrix, RotationMatrix, Euler, Quaternion

    Some useful conversions are carried out.

@Time: July 2027
"""
import numpy as np
from artelib.rotationmatrix import Rx, R2
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.euler import Euler

if __name__ == "__main__":
    T = HomogeneousMatrix([[1, 0, 0, 0.5], [0, 1, 0, 0.7], [0, 0, 1, 0.8], [0, 0, 0, 1]])
    e = Euler([np.pi/4, np.pi/4, np.pi/4])
    T = HomogeneousMatrix(Vector([0.5, 0.7, 0.8]), e)
    T.print()
    T.plot('Homogeneous transformation T1')
    # Matriu de rotació
    print('Rotació:')
    R = T.R()
    R.print()
    # Vector de posició
    print('Posició: ')
    v = T.vector()
    v.print()
    # Euler
    print('Euler: ')
    e = T.euler()
    e[0].print()
    e[1].print()
    # Quaternió
    print('Quaternió: ')
    Q = T.Q()
    Q.print()
    # Altres conversions
    R2 = Q.R()
    R2.print()
    T2 = e[0].T()
    T2.print()
    e2 = Q.euler()
    e2[0].print()
    e2[1].print()