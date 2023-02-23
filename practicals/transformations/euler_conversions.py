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

if __name__ == '__main__':
    e = Euler([-np.pi/2,  0, 0])
    print('Euler angles XYZ:')
    print(e.abg)

    print('Conversion from Euler to a rotation matrix:')
    R = e.R()
    print('R\n', R)
    R.plot()

    # convert R to Euler angles
    print('Euler angles (XYZ) that yield R (e1, e2):')
    [e1, e2] = R.euler()
    print(e1.abg, e2.abg)
    print('Please check that R1, R2 and R are equal')
    R1 = e1.R()
    R2 = e2.R()
    print('R1:\n', R1)
    print('R2:\n', R2)

    # Convert any R to Euler angles
    R = RotationMatrix(np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]))
    print('R\n', R)
    R.plot()
    print('Euler angles (XYZ) that yield R:')
    [e3, e4] = R.euler()
    print(e3.abg, e4.abg)
    print('Please check that R3, R4 and R are equal')
    R3 = e3.R()
    R4 = e4.R()
    print('R3:\n', R3)
    print('R4:\n', R4)
