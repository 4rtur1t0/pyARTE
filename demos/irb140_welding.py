#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/irb140_welding.ttt scene before running this script.

TTD: As an exercise, the student should solve the inverse kinematic problem on a line,
so as to perform a valid welding operation.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.simulation import Simulation


def welding():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.12]), RotationMatrix(np.eye(3))))
    q0 = np.array([0, 0, 0, 0, 0, 0])
    target_positions = [[-0.3, 0.1, 0.75],
                        [-0.3, 0.6, 0.75],
                        [-0.2, 0.6, 0.75],
                        [-0.1, 0.6, 0.75],
                        [0.0, 0.6, 0.75],
                        [0.1, 0.6, 0.75],
                        [0.15, 0.6, 0.75],
                        [0.2, 0.6, 0.75],
                        [0.25, 0.6, 0.75],
                        [0.3, 0.6, 0.75],
                        [0.3, 0.1, 0.75]]  # pick
    target_orientation = Euler([-np.pi/2, 0, np.pi/2])
    robot.moveAbsJ(q_target=q0, precision=False)
    robot.moveJ(target_position=target_positions[0], target_orientation=target_orientation)

    for target_position in target_positions:
        robot.moveL(target_position=target_position, target_orientation=target_orientation)

    simulation.stop()


if __name__ == "__main__":
    welding()

