#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/irb140_abb_conveyor_barret_hand.ttt scene before running this script.

@Authors: Arturo Gil arturo.gil@umh.es
@Time: April 2021
"""
import numpy as np
from sceneconfig.scene_configs_irb140 import init_simulation_ABBIRB140


def pick_and_place_rep():
    """
    A repeated pick and place application.
    """
    robot = init_simulation_ABBIRB140()
    q0 = np.array([0.0, 0.2, np.pi / 4, 0.1, 0.1, np.pi/2])
    robot.set_joint_target_positions(q0, precision=True)
    robot.open_gripper(precision=True)

    q0 = np.array([0.0, 0.5, 0.25, 0.1, 0.8, np.pi / 2])
    robot.set_joint_target_positions(q0, precision=True)
    robot.close_gripper(precision=True)
    q0 = np.array([0.0, 0.2, 0, 0, 0, np.pi / 2])
    robot.set_joint_target_positions(q0, precision=True)
    robot.open_gripper(precision=True)

    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place_rep()
