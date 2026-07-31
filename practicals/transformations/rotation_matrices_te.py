#!/usr/bin/env python
# encoding: utf-8
"""
The script introduces you to the use of the classes:
    RotationMatrix

    ... and its representation on a plot

@Time: July 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.rotationmatrix import RotationMatrix, Rx, Ry, Rz
from artelib.vector import Vector

if __name__ == "__main__":
    e = Euler([np.pi/2, np.pi/3, np.pi/2])
    R = e.R()
    R.print()
    R.plot(title='e=(pi/3, pi/3, pi/3)')
    [e1, e2] = R.euler()
    print(e1, e2)

    R1 = e1.R()
    R2 = e2.R()
    R1.plot(title='e=(pi/3, pi/3, pi/3)')
    R1.plot(title='e=(-2*pi/3, 2*pi/3, -2*pi/3)')



