#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from sceneconfig.scene_configs import init_simulation_ABBIRB140


def inverse_kinematics(robot, target_position, target_orientation):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion.

    caution: closest tooooo
    """
    q = robot.inversekinematics(target_position, target_orientation)
    return q


def get_closest_to(qa, qb):
    """
    From a column wise list of solutions in qa, find the closest to qb.
    """
    distances = []
    for i in range(8):
        d = np.linalg.norm(qa[:, i]-qb)
        distances.append(d)
    distances = np.array(distances)
    distances = np.nan_to_num(distances, nan=np.inf)
    idx = np.argmin(distances)
    return qa[:, idx]




def try_to_reach():
    robot = init_simulation_ABBIRB140()
    q0 = np.array([0, 0, 0, 0, np.pi / 2, 0])
    target_position = [0.5, -0.1, 0.6]
    target_orientation = [0, np.pi/2, 0]

    q = inverse_kinematics(robot=robot, target_position=target_position,
                           target_orientation=Euler(target_orientation))

    robot.set_joint_target_positions(q0, precision=True)

    for i in range(8):
        qi = q[:, i]
        robot.set_joint_target_positions(qi, precision=True)
        robot.wait(2)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    try_to_reach()

