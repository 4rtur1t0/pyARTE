#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_lbr_iiwa_R800.ttt scene before running this script.

This demo presents a simple inverse kinematic scheme with the KUKA IIWA LBR arm

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from sceneconfig.scene_configs import  init_simulation_KUKALBR


def ikine():
    robot, _ = init_simulation_KUKALBR()
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

    robot.directkinematics(q)

    for i in range(6):
        robot.set_target_position_orientation(target_positions[i], target_orientations[i])
        q = robot.inversekinematics(target_position=target_positions[i],
                                    target_orientation=target_orientations[i], q0=q)
        robot.set_joint_target_positions(q, precision=True)

    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikine()

