#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil arturo.gil@umh.es
@Time: April 2021
"""
import numpy as np
from artelib.orientation import Euler
from sceneconfig.scene_configs import init_simulation_UR5


def pick_and_place_rep():
    """
    A repeated pick and place application.
    """
    robot = init_simulation_UR5()
    target_positions = [[0.6, -0.2, 0.25],
                        [0.6, 0.1, 0.25],
                        [0.6, -0.1, 0.35],
                        [0.2, -0.45, 0.4]]
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi, 0, 0]]

    # set the target we are willing to reach
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    vmax = 0.5

    for i in range(0, 6):
        q0 = np.array([-np.pi, 0.1, np.pi / 2, 0.1, 0.1, 0.1])
        # set initial position of robot
        robot.set_joint_target_positions(q0, precision=True)
        # plan trajectories
        q1_path = robot.inversekinematics_line(target_position=target_positions[0],
                                               target_orientation=Euler(target_orientations[0]), q0=q0, vmax=vmax)
        q2_path = robot.inversekinematics_line(target_position=target_positions[1],
                                               target_orientation=Euler(target_orientations[1]), q0=q1_path[-1], vmax=vmax)
        q3_path = robot.inversekinematics_line(target_position=target_positions[2],
                                               target_orientation=Euler(target_orientations[2]), q0=q2_path[-1], vmax=vmax)
        target_pos = target_positions[3] + i * np.array([0, 0.07, 0])
        target_orient = target_orientations[3]
        q4_path = robot.inversekinematics_line(target_position=target_pos,
                                               target_orientation=Euler(target_orient), q0=q3_path[-1])
        target_pos = target_pos + np.array([0, 0, -0.1])
        q5_path = robot.inversekinematics_line(target_position=target_pos,
                                               target_orientation=Euler(target_orient), q0=q4_path[-1])
        target_pos = target_pos + np.array([0, 0, 0.1])
        q6_path = robot.inversekinematics_line(target_position=target_pos,
                                               target_orientation=Euler(target_orient), q0=q5_path[-1])
        # execute trajectories
        robot.open_gripper()
        robot.set_joint_target_trajectory(q1_path, precision='last')
        robot.set_joint_target_trajectory(q2_path, precision='last')
        robot.close_gripper(precision=True)
        robot.set_joint_target_trajectory(q3_path, precision='last')
        robot.set_joint_target_trajectory(q4_path, precision='last')
        robot.set_joint_target_trajectory(q5_path, precision='last')
        robot.open_gripper(precision=True)
        robot.set_joint_target_trajectory(q6_path, precision='last')

    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place_rep()
