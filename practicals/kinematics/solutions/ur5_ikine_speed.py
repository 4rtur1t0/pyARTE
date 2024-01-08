#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: December 2023
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import compute_kinematic_errors
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from robots.simulation import Simulation
from robots.ur5 import RobotUR5


def follow_line():
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
    q = np.array([-np.pi/2, 0.3, 0.3, 0.3, 0.3, 0.3])
    robot.moveAbsJ(q)
    T = robot.directkinematics(q)
    T.print_nice()
    print('Current T: ')
    # relative point
    v = np.array([0.0, 0.005, -0.3])
    total_time = 2
    pb = T.pos() + total_time*v
    vw = np.array([v, [0, 0, 0]]).flatten()

    while True:
        q = robot.get_joint_positions()
        J, _, _ = robot.manipulator_jacobian(q)
        T = robot.directkinematics(q)
        e = np.linalg.norm(pb - T.pos())
        print(e)
        if e < 0.02:
            robot.set_joint_target_velocities([0, 0, 0, 0, 0, 0])
            break
        qd = np.dot(np.linalg.inv(J), vw.T)
        robot.set_joint_target_velocities(qd)
        robot.wait()

    print('POSITION REACHED', T.pos())
    print('ERROR: ', np.linalg.norm(T.pos()-pb))
    simulation.stop()


if __name__ == "__main__":
    follow_line()

