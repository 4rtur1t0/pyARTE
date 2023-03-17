#!/usr/bin/env python
# encoding: utf-8
"""
Example usage of the Quaternion class
@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from artelib.quaternion import Quaternion
from artelib.rotationmatrix import Rx, Ry, Rz, RotationMatrix

if __name__ == '__main__':
    eul = Euler([np.pi/2, np.pi/4, np.pi/2])
    R = eul.R()
    sols_euler = R.euler()

    print('R: ', R)

    print('Otra vez Euler', sols_euler[0])
    print('Otra vez R.euler()', sols_euler[1])

    quat1 = eul.Q()
    quat2 = R.Q()
    print('De Euler a un quaternion: ', quat1)
    print('De R a un quaternion: ', quat2)

    print(quat1.R())
