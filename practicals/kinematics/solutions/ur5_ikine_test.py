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
from robots.simulation import Simulation
from robots.ur5 import RobotUR5


def check_kinematics():
    """
    Check direct and inverse kinematics
    Using Gradient descent/Jacobian based kinematics
    """
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))
    robot.start()
    # Find an initial T
    q = np.array([-np.pi/2, np.pi/4, -np.pi/4, 0.3, 0.3, 0.3])
    T = robot.directkinematics(q)
    print('Current T: ')
    T.print_nice()
    # try to find a solution
    q0 = np.array([-0.1, -0.1, -0.1, -0.1, -0.1, -0.1])
    qinv = robot.inversekinematics(target_position=T.pos(),
                                   target_orientation=T.R(), q0=q0)

    print('CHECKING CONSISTENCY')
    print('FOUND solution qinv: ', qinv)
    T_reached = robot.directkinematics(qinv)
    print('T reached : ')
    T_reached.print_nice()
    Tdiff = T - T_reached
    print('Difference in T')
    Tdiff.print_nice()

    print('Difference in the solutions')
    print(q-qinv)

    robot.moveAbsJ(q)
    robot.moveAbsJ(qinv)

    simulation.stop()


if __name__ == "__main__":
    check_kinematics()

