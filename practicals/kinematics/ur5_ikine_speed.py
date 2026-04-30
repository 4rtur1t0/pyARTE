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
    Follow a line in space with speed v.
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
    # linear speed
    v = np.array([0.0, 0.005, -0.3])
    # total time seconds
    total_time = 1.5
    # final point reached
    pb = T.pos() + total_time*v
    # linear and angular speed
    vw = np.array([v, [0, 0, 0]]).flatten()

    # COMPLETE EL CÓDIGO PARA SEGUIR LA VELOCIDAD vw EN EL EXTREMO DEL ROBOT


    print('POSITION REACHED', T.pos())
    print('ERROR: ', np.linalg.norm(T.pos()-pb))
    simulation.stop()


if __name__ == "__main__":
    follow_line()

