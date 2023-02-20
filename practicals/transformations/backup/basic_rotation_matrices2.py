#!/usr/bin/env python
# encoding: utf-8
"""
The script introduces you to the use of the classes:
    RotationMatrix

    ... and its representation on a plot

@Time: July 2022
"""
import numpy as np
from artelib.rotationmatrix import RotationMatrix, Rx, Ry, Rz, R2

if __name__ == "__main__":
    R = RotationMatrix(2)
    R.plot('A 2D identity rotation matrix (2x2)', block=True)

    R = R2(theta=np.pi/2)
    R.plot('A 2D rotation (pi/2)', block=True)

    R3 = RotationMatrix(3)
    R3.plot('Identity 3x3', block=True)

    Ra = Rx(np.pi/4)
    Rb = Ry(np.pi/4)
    Rc = Rz(np.pi/4)

    Ra.plot(title='Rx pi/4', block=True)
    Rb.plot(title='Ry pi/4', block=True)
    Rc.plot(title='Rz pi/4', block=True)

    R = Ra*Rb*Rc
    R.plot(title='Three consecutive rotations of pi/4', block=True)

    print('cool!')


