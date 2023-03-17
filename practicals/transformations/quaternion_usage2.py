#!/usr/bin/env python
# encoding: utf-8
"""
Example usage of the Quaternion class
@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler


if __name__ == '__main__':
    eul = Euler([np.pi/2, np.pi/4, np.pi/2])
    R = eul.R()
    sols_euler = R.euler()

    print('Original Euler XYZ:', eul)
    print('Corresponding Rotation matrix: \n', R)
    print('Equivalent Euler angles (solution 0):', sols_euler[0])
    print('Equivalent Euler angles (solution 1):', sols_euler[1])

    quat1 = eul.Q()
    quat2 = R.Q()
    print('From Euler to a Quaternion: ', quat1)
    print('From a rotation matrix to a Quaternion: ', quat2)

    print('And back to a Rotation matrix: \n', quat1.R())


