#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np

from artelib.plottools import plot
from artelib.tools import buildT, compute_w_between_R, compute_e_between_R, euler2q
from sceneconfig.scene_configs import init_simulation_UR5


def ikine():
    robot = init_simulation_UR5()
    # OK
    # target_positions = [[0.6, -0.2, 0.25],
    #                     [0.6, 0.1, 0.25],
    #                     [0.6, -0.1, 0.35],
    #                     [0.2, -0.55, 0.4],
    #                     [0.3, 0.3, 0.58],
    #                     [-0.3, -0.3, 0.5]]
    # target_orientations = [[-np.pi / 2, 0, -np.pi / 2],
    #                        [-np.pi / 2, 0, -np.pi / 2],
    #                        [-np.pi / 2, 0, -np.pi / 2],
    #                        [-np.pi, 0, 0],
    #                        [0, 0, 0],
    #                        [np.pi/2, 0, 0]]

    target_positions = [[0.2, -0.45, 0.4],
                        [0.6, -0.2, 0.25]]
    target_orientations = [[-np.pi, 0, 0],
                           [-np.pi/2, 0, -np.pi/2]]

    q = np.array([0, 0, 0, 0, 0, 0])

    for i in range(len(target_positions)):
        robot.set_target_position_orientation(target_positions[i], target_orientations[i])
        q = robot.inversekinematics(target_position=target_positions[i],
                                    target_orientation=euler2q(target_orientations[i]), q0=q)
        robot.set_joint_target_positions(q, precision=True)

    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikine()

