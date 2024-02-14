#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

This script uses the built-in inverse kinematics of the robot with closed equations.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.ur5 import RobotUR5


def inverse_kinematics_test():
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0]), RotationMatrix(np.eye(3))))
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # Find an initial T
    q = np.array([np.pi / 2, np.pi / 2 , -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
    T = robot.directkinematics(q)
    print('Current T total: ')
    T.print_nice()
    print(30 * '*')
    frame.show_target_point(T.pos(), T.R())
    robot.moveAbsJ(q)
    qinv = robot.inversekinematics(T.pos(), T.R())
    print('FOUND SOLUTIONS')
    print(qinv)

    for i in range(qinv.shape[1]):
        # robot.moveAbsJ(q_target=q0, precision=True, endpoint=True)
        print('Current solution')
        print(np.array_str(qinv[:, i], precision=3, suppress_small=True))
        T2 = robot.directkinematics(qinv[:, i])
        T.print_nice()
        T2.print_nice()
        robot.moveAbsJ(q_target=qinv[:, i], precision=True, endpoint=True)
        robot.wait(5)
        print(T2 - T)
        E = np.square((T2 - T).array)
        E = np.sum(np.sum(E))
        print('Error E: ', E)
        if E > 0.01:
           raise Exception('ERROR in kinematics found')
    print('FOUND SOLUTIONS dimensions: ', qinv.shape)
    print(qinv)
    simulation.stop()


if __name__ == "__main__":
    inverse_kinematics_test()