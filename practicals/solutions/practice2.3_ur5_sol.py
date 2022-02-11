#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

Using robot.inversekinematics_line to plan the next target point and follow a line in task space.
Orientation is interpolated.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from sceneconfig.scene_configs import init_simulation_UR5


def pick_and_place():
    robot = init_simulation_UR5()
    target_positions = [[0.6, -0.2, 0.25], # initial in front of conveyor
                        [0.6, 0.1, 0.25], # pick the piece
                        [0.6, -0.1, 0.35], # bring the piece up
                        [0.4, -0.1, 0.35], # middle point
                        [0.2, -0.55, 0.4], # over the table
                        [0.2, -0.55, 0.3],
                        [0.2, -0.55, 0.4]] # drop the piece
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi / 2, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    open_gripper = [True,
                    True,
                    False,
                    False,
                    False,
                    False,
                    True]
    q = np.array([-np.pi/2, -np.pi / 8, np.pi / 2, np.pi / 2, 0.1, 0.1])
    robot.set_joint_target_positions(q)
    robot.wait(20)
    for i in range(len(target_positions)):
        q_path = robot.inversekinematics_line(target_position=target_positions[i],
                                              target_orientation=Euler(target_orientations[i]),
                                              q0=q, vmax=0.5)
        if open_gripper[i]:
            robot.open_gripper(precision=True)
        else:
            robot.close_gripper(precision=True)
        robot.set_joint_target_trajectory(q_path, precision='last')
        q = q_path[-1]

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place()

