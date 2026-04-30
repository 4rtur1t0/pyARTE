#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

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


def compute_direct_kinematics():
    """
    Check direct and inverse kinematics
    Using Gradient descent/Jacobian based kinematics
    """
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0]), RotationMatrix(np.eye(3))))
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # Find an initial T
    q = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q)
    q = robot.get_joint_positions()
    T0 = robot.directkinematics(q)
    print('Current T total: ')
    T0.print_nice()
    print(30*'*')

    T = HomogeneousMatrix()
    for i in range(6):
        print(i)
        A = robot.serialrobot.dh(q, i)
        T = T*A
        T.print_nice()
        frame.show_target_point(T.pos(), T.R())
    robot.wait(300)

    simulation.stop()


def inverse_kinematics_test():
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0]), RotationMatrix(np.eye(3))))
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # Find an initial T
    # q = np.array([np.pi/4, -np.pi/3, -np.pi/3,  3*np.pi/4, np.pi / 2, np.pi / 2])
    # q = np.array([np.pi / 4, -np.pi / 3, -np.pi / 3, 3 * np.pi / 4, np.pi / 8, np.pi / 8])
    # q = np.array([np.pi / 4, -np.pi / 3, -np.pi / 3,  np.pi / 2, np.pi / 8, np.pi / 8])
    # q = np.array([np.pi / 4, np.pi / 3, -np.pi / 3, 3 * np.pi / 8, np.pi/ 8, np.pi / 8])
    # q = np.array([0, -np.pi / 2, np.pi / 2, 0, np.pi/16, np.pi / 8])
    # q = np.array([0, -np.pi / 2, np.pi / 2, -np.pi / 2, 0, np.pi / 8])
    # q = np.array([np.pi / 16, np.pi / 16, np.pi / 16, np.pi / 2, 0, np.pi / 8])
    # q = np.array([0, 0, 0, +np.pi / 2, np.pi/2, np.pi / 8])
    # q = np.array([0, np.pi/16, np.pi/16, 0, 0, 0])
    # q = np.array([0, 0, 0, 0, 0, 0]) TEST OK
    # q = np.array([-np.pi / 2, 0, 0, np.pi / 2, 0, 0]) TEST OK
    # q = np.array([np.pi, 0, 0, -3*np.pi / 2, 0, np.pi/2])
    # q = np.array([0, 0, 0, np.pi/2, np.pi/2, 0])
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
    # compute_direct_kinematics()
    inverse_kinematics_test()