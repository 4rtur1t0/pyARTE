#!/usr/bin/env python
# encoding: utf-8
"""
Given three Euler angles, compute a rotation matrix for a given convention, e. g. XYZ.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np

from artelib.euler import Euler
from artelib.rotationmatrix import Rx, Ry, Rz, RotationMatrix


def euler2rot(abg, convention):
    """
    Compute the rotation matrix for a given convention (e. g. XYZ) always working on mobile axes.
    """
    if convention == 'xyx':
        Ra = Rx(abg[0])
        Rb = Ry(abg[1])
        Rc = Rx(abg[2])
    elif convention == 'xyz':
        Ra = Rx(abg[0])
        Rb = Ry(abg[1])
        Rc = Rz(abg[2])
    elif convention == 'zxz':
        Ra = Rz(abg[0])
        Rb = Rx(abg[1])
        Rc = Rz(abg[2])
    elif convention == 'xzx':
        Ra = Rx(abg[0])
        Rb = Rz(abg[1])
        Rc = Rx(abg[2])
    elif convention == 'zyz':
        Ra = Rz(abg[0])
        Rb = Ry(abg[1])
        Rc = Rz(abg[2])
    else:
        print('UNDEFINED CONVENTION')
        raise Exception
    R = Ra*Rb*Rc
    return R


if __name__ == '__main__':
    Rxyz = euler2rot([np.pi/2, 0, np.pi/2], 'xyz')
    Rzxz = euler2rot([np.pi/2, 0, np.pi/2], 'zxz')
    Rxzx = euler2rot([np.pi/2, 0, np.pi/2], 'xzx')

    print('Resulting matrices: ')
    print('Rxyz:\n', Rxyz)
    print('Rzxz:\n', Rzxz)
    print('Rxzx:\n', Rxzx)
    Rxyz.plot('Rxyz')
    Rzxz.plot('Rzxz')
    Rxzx.plot('Rxzx')

    print('Convert every R to XYZ Euler angles')
    print('Rxyz to XYZ (obvious):')
    print(Rxyz.euler()[0], Rxyz.euler()[1])
    print('Rzxz to XYZ:')
    print(Rxyz.euler()[0], Rxyz.euler()[1])
    print('Rxzx to XYZ:')
    print(Rxyz.euler()[0], Rxyz.euler()[1])

    # Rzyz = euler2rot([-np.pi / 2, -np.pi / 2, np.pi / 2], 'zyz')
    # Rzyz.print_nice()
    #
    # Rxyx = euler2rot([np.pi / 2, -np.pi / 2, np.pi / 2], 'xyx')
    # Rxyx.print_nice()