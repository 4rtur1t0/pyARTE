#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_lbr_iiwa_R800.ttt scene before running this script.

This demo presents a simple inverse kinematic scheme with the KUKA IIWA LBR arm

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def ikine():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotKUKALBR(clientID=clientID)
    robot.start()
    gripper = GripperRG2(clientID=clientID)
    gripper.start()
    target_positions = [[0.6, -0.2, 0.5],
                        [0.6, 0.1, 0.5],
                        [0.6, -0.1, 0.5],
                        [0.2, -0.55, 0.8],
                        [0.3, 0.3, 0.5],
                        [-0.3, -0.3, 0.5]]
    target_orientations = [[-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi, 0, 0],
                           [0, 0, 0],
                           [np.pi/2, 0, 0]]

    q = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1])

    T = robot.directkinematics(q)

    for i in range(6):
        # robot.set_target_position_orientation(target_positions[i], target_orientations[i])
        q = robot.inversekinematics(target_position=target_positions[i],
                                    target_orientation=Euler(target_orientations[i]), q0=q)
        robot.set_joint_target_positions(q, precision=True)

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikine()

