#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from robots.ur5 import RobotUR5


def check_jacobians():
    """
    Checks whether the symbolic Jacobian and geometric Jacobian Match
    """
    # simulation = Simulation()
     # clientID = simulation.start()
    robot = RobotUR5(clientID=None)
    # robot.start()
    np.set_printoptions(suppress=True)

    q = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    Js, Jv, Jw = robot.get_jacobian(q)
    J = robot.manipulator_jacobian(q)

    print('Symbolic Jacobian assessed at q: ')
    print(Js)

    print('Geometric Jacobian: ')
    print(J)

    print('Equals zero?')
    print(J-Js)


if __name__ == "__main__":
    check_jacobians()

