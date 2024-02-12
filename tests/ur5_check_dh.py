#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np

from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.ur5 import RobotUR5



if __name__ == "__main__":
    """
    Checks whether the symbolic Jacobian and geometric Jacobian Match
    """
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    q = robot.get_joint_positions()
    # T = robot.directkinematics(q)
    # frame.show_target_point(T.pos(), T.R())

    T = HomogeneousMatrix()
    for i in range(3):
        A = robot.serialrobot.dh(q, i)
        A.print_nice()
        T = T*A
        frame.show_target_point(T.pos(), T.R())
        robot.wait(300)

