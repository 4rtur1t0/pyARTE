#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import path_planning_line
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def show_target_points(clientID, target_positions, target_orientations, wait_time=10):
        frame = ReferenceFrame(clientID=clientID)
        frame.start()
        for i in range(len(target_positions)):
            T = HomogeneousMatrix(target_positions[i], target_orientations[i])
            frame.set_position_and_orientation(T)
            frame.wait(wait_time)


def path_in_workspace():
    # Start simulation
    simulation = Simulation()
    clientID = simulation.start()
    # constant orientation
    # targetA = HomogeneousMatrix(Vector([0.5, -0.5, 0.9]), Euler([0, 0, 0]))
    # targetB = HomogeneousMatrix(Vector([0.5, 0.5, 0.4]), Euler([0, 0, 0]))

    # constant position
    # targetA = HomogeneousMatrix(Vector([0.5, 0, 0.9]), Euler([0, 0, 0]))
    # targetB = HomogeneousMatrix(Vector([0.5, 0, 0.9]), Euler([np.pi / 2, np.pi / 2, 0]))

    # changes in position and orientation
    targetA = HomogeneousMatrix(Vector([0.5, -0.5, 0.9]), Euler([0, 0, 0]))
    targetB = HomogeneousMatrix(Vector([0.5, 0.5, 0.4]), Euler([0, np.pi / 2, 0]))

    tps, tos = path_planning_line(targetA.pos(), targetA.R(), targetB.pos(), targetB.R())
    show_target_points(clientID=clientID, target_positions=tps, target_orientations=tos, wait_time=3)

    simulation.wait(100)

    # Stop arm and simulation
    simulation.stop()


if __name__ == "__main__":
    path_in_workspace()

