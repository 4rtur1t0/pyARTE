#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_paint_application.ttt scene before running this script.

@Authors: Arturo Gil
@Time: November 2022
"""
import numpy as np

from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.simulation import Simulation


def paint():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.2]), RotationMatrix(np.eye(3))))

    q0 = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q0, precision=True)

    target_positions = [[0.5, -0.5, 0.6],
                        [0.5, 0.5, 0.6],
                        [0.5, 0.5, 0.3],
                        [0.5, 0.5, 0.1]]
    target_orientations = [[0, np.pi/2, 0],
                           [0, np.pi/2, 0],
                           [0, np.pi/2, 0],
                           [0, np.pi/2, 0]]

    for i in range(len(target_positions)):
        robot.moveJ(target_position=target_positions[i],
                    target_orientation=target_orientations[i])

    simulation.stop()


if __name__ == "__main__":
    paint()

