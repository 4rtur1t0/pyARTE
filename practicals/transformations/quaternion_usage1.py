#!/usr/bin/env python
# encoding: utf-8
"""
Example usage of the Quaternion class
@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np

from artelib.quaternion import Quaternion
from artelib.rotationmatrix import Rx, Ry, Rz, RotationMatrix

if __name__ == '__main__':
    R1 = RotationMatrix([[1, 0, 0],
                        [0, 0, -1],
                        [0, 1, 0]])
    R2 = RotationMatrix([[0, -1, 0],
                         [1, 0, 0],
                         [0, 0, 1]])
    print('R1: ', R1)
    print('R2: ', R2)

    print('Determinant R1', R1.det())
    print('Trasposta R1: ', R1.inv())
    eul = R1.euler()
    print('Angles dEuler:', eul[0], eul[1])
    print('Producte:', R1*R2)

    # quat1 = Quaternion([0, 0, 0, 1])
    quat1 = R1.Q()
    quat2 = R2.Q()


    print('quat1: ', quat1)
    print('quat2: ', quat2)

    print('R1 de quat1:', quat1.R())
    print('R2 de quat2:', quat2.R())

