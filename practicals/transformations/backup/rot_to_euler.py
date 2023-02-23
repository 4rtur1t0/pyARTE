#!/usr/bin/env python
# encoding: utf-8
"""
Conversions using pyARTE between a Rotation Matrix and XYZ Euler angles.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.rotationmatrix import RotationMatrix

if __name__ == '__main__':
    orientation = RotationMatrix(np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]]))
    [e1, e2] = orientation.euler()
    print('The two corresponding sets of Euler angles are:')
    print(e1.abg, e2.abg)

    print('Which, of course, yield matrices R1 and R2:')
    R1 = e1.R()
    R2 = e2.R()
    print(R1)
    print(R2)
