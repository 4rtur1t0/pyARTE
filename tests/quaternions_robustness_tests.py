#!/usr/bin/env python
# encoding: utf-8
"""
This script does not need a Coppelia Scene.

The script does the following:
    a) produces a random q in the joint range of an ABBIRB140 robot.
    b) Next, the directkinematics of the robot is computed.
    c) The orientation matrix R is isolated and tested.

The idea is to detect that some Rs are, due to floating point errors, ill conditioned. If a quaternion Q is computed
from R it may produce an erroneous result. As a result, the rot2quaternion method in tools tests this condition and
normalizes R if needed.

@Authors: Arturo Gil
@Time: April 2023
"""
import numpy as np

from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.tools import slerp
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from artelib.path_planning import random_q


if __name__ == "__main__":
    M = 50000
    robot = RobotABBIRB140(clientID=None)
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))
    # for i in range(M):
    # q = random_q(robot)

    # create perfect R
    R1 = RotationMatrix(np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]))
    # compute from perfect matrix
    Q1 = R1.Q()
    print('THE PERFECT Q1: ', Q1)


    # create noisy R
    q = np.array([4.18688200e-01,  7.54374149e-01, -2.92517718e-01, -9.88778849e-17, 1.10893990e+00,  4.18688200e-01])
    T = robot.directkinematics(q)
    R = T.R()
    Q = R.Q()
    print('THE NOISY Q: ', Q)

    print('COMPARE BOTH')
    print('Q-Q1', np.linalg.norm((Q - Q1).array))

    Qi = slerp(Q1, Q, 0.5)
    print(Qi)


    # print('Det R: ', R.det())
    # print('Det R1: ', R1.det())
    #     if R.det() > 1.0:
    #         print('CAUTION')
    #         print(R.det())
    #
    #     # compute from noisy matrix
    #
    #
    #     if np.isnan(np.sum(Q.array)):
    #         print('DEBUG')
    #
    #     print('R noisy: Quaternion is: ', Q)
    #     print('R1 perfect: Quaternion is: ', Q1)
    #     print('Reconstructing matrices: ')
    #     print('R from Q: ', Q.R())
    #     print('R from Q1: ', Q1.R())
    #
    #     print('Q-Q1', np.linalg.norm((Q-Q1).array))
    #
    #
    # print('FINISHED: NO ERRORS FOUND!')


