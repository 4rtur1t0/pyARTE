#!/usr/bin/env python
# encoding: utf-8
"""
Conversions using pyARTE between Euler angles and rotation matrices and viceceversa.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from artelib.rotationmatrix import RotationMatrix


def EulertoR():
    print('Given Euler, compute Rotation matrix:')
    e = Euler([-np.pi/2,  0, 0])
    print('Euler angles XYZ:')
    print(e.abg)

    print('Conversion from Euler to a rotation matrix:')
    R = e.R()
    R.print()


def RtoEuler():
    print('Given a Rotation Matrix, compute Euler angles:')
    # Convert any R to Euler angles
    R = RotationMatrix(np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]))
    print('The initial matrix R:')
    R.print()

    print('Euler angles (XYZ) that yield R:')
    [e1, e2] = R.euler()
    print(e1.abg, e2.abg)

    print('Please check that R1, R2 and R are equal')
    R1 = e1.R()
    R2 = e2.R()
    R1.print()
    R2.print()


if __name__ == '__main__':
    EulertoR()
    RtoEuler()





