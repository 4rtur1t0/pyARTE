#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5_velodyne.ttt scene before running this script.

The script provides an example to move a UR5 robot and get laser data.
 Laser data may be used to rectonstruct the environment, recognise objects... etc.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np

from artelib.euler import Euler
from sceneconfig.scene_configs import init_simulation_UR5, init_simulation_UR5BarrettHand


if __name__ == "__main__":
    robot = init_simulation_UR5BarrettHand()
    target_positions = [[0.4, -0.3, 0.2]]  # drop the piece
    target_orientations = [[np.pi / 2, 0, 0]]

    q0 = np.pi / 8 * np.array([-1, 1, 1, 1, 1, 1])
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)

    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    q_path = robot.inversekinematics_line(target_position=target_positions[0],
                                          target_orientation=Euler(target_orientations[0]),
                                          q0=q0, vmax=0.5)
    robot.open_gripper(precision=True)
    robot.set_joint_target_trajectory(q_path, precision='last')
    robot.close_gripper(precision=True)
    robot.set_joint_target_trajectory(q_path[::-1], precision='last')
    robot.open_gripper(precision=True)


    robot.stop_arm()

