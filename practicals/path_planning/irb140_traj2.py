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
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140 import RobotABBIRB140
from robots.simulation import Simulation


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    clientID = simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(clientID=clientID)
    robot.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.19]), RotationMatrix(np.eye(3))))

    q0 = np.array([0, 0, 0, 0, -np.pi / 2, 0])
    target_positions = [Vector([0.6, -0.5, 0.8]),
                        Vector([0.6, -0.5, 0.3]),
                        Vector([0.6, 0.5, 0.3]),
                        Vector([0.6, 0.5, 0.8]),
                        Vector([0.6, 0, 0.8])]
    target_orientations = [Euler([0, np.pi / 2, 0]),
                           Euler([0, np.pi / 2, 0]),
                           Euler([0, np.pi / 2, 0]),
                           Euler([0, np.pi / 2, 0]),
                           Euler([0, np.pi / 2, 0])]

    # mostrar en Coppelia los target points anteriores
    robot.show_target_points(target_positions, target_orientations, wait_time=1)

    robot.moveAbsJ(q0, precision=True)
    for i in range(len(target_positions)):
        robot.moveL(target_position=target_positions[i], target_orientation=target_orientations[i], precision=False)

    # Stop arm and simulation
    simulation.stop()
    robot.plot_trajectories()

