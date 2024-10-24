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
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # set the TCP of the RG2 gripper
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))
    # robot.set_TCP(HomogeneousMatrix(Vector([0, 0.065, 0.105]), Euler([-np.pi / 2, 0, 0])))

    q0 = np.array([-np.pi/4, -np.pi/4, np.pi/8, -np.pi/4, np.pi / 4, -np.pi/4])
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
    frame.show_target_points(target_positions, target_orientations, wait_time=0.5)
    robot.moveAbsJ(q0, endpoint=True)
    for i in range(len(target_positions)):
        frame.show_target_point(target_positions[i], target_orientations[i], wait_time=0.5)
        robot.moveL(target_position=target_positions[i], target_orientation=target_orientations[i],
                    endpoint=True,
                    vmax=0.4)

    # Stop arm and simulation
    simulation.stop()
    robot.plot_trajectories()

