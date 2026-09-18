#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

The script calls directly the internal function inversekinematics to retrieve solutions to different positions
and orientations

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from robots.grippers import GripperRG2
from robots.simulation import Simulation
from robots.ur5 import RobotUR5


def ikine():
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start(name='/UR5/RG2_openCloseJoint')

    target_positions = [[0.2, -0.45, 0.4],
                        [0.6, -0.2, 0.25]]
    target_orientations = [[-np.pi, 0, 0],
                           [-np.pi/2, 0, -np.pi/2]]

    q = np.array([0, 0, 0, 0, 0, 0])
    robot.moveAbsJ(q_target=q)
    for i in range(len(target_positions)):
        q = robot.inversekinematics(target_position=target_positions[i],
                                    target_orientation=Euler(target_orientations[i]), q0=q)
        robot.moveAbsJ(q, precision=True)

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikine()

