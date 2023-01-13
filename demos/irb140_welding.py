#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140_welding.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from sceneconfig.scene_configs_irb140 import init_simulation_ABBIRB140


def filter_joint_limits(robot, q):
    n_valid_solutions = q.shape[1]
    q_in_range = []
    for i in range(n_valid_solutions):
        qi = q[:, i]
        total, partial = robot.check_joints(qi)
        if total:
            q_in_range.append(qi)
    q_in_range = np.array(q_in_range).T
    return q_in_range


def get_closest_to(qa, qb):
    """
    From a column wise list of solutions in qa, find the closest to qb.
    """
    n_solutions = qa.shape[1]
    distances = []
    for i in range(n_solutions):
        d = np.linalg.norm(qa[:, i]-qb)
        distances.append(d)
    distances = np.array(distances)
    distances = np.nan_to_num(distances, nan=np.inf)
    idx = np.argmin(distances)
    return qa[:, idx]


def inverse_kinematics(robot, target_position, target_orientation, q0):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion in the robot.inversekinematics function

    caution: closest tooooo
    """
    q = robot.inversekinematics(target_position, target_orientation)
    # filter in joint ranges
    q = filter_joint_limits(robot, q)
    # for i in range(q.shape[1]):
    #     robot.set_joint_target_positions(q[:, i])
    # get closest solution to q0
    qc = get_closest_to(q, q0)
    return qc


def welding():
    robot, conveyor_sensor = init_simulation_ABBIRB140()
    q0 = np.array([np.pi, 0, 0, 0, 0, 0])
    target_positions = [[-0.3, 0.1, 0.75],
                        [-0.3, 0.494, 0.75],
                        [-0.2, 0.494, 0.75],
                        [-0.1, 0.494, 0.75],
                        [0.0, 0.494, 0.75],
                        [0.1, 0.494, 0.75],
                        [0.15, 0.494, 0.75],
                        [0.2, 0.494, 0.75],
                        [0.25, 0.494, 0.75],
                        [0.3, 0.494, 0.75],
                        [0.3, 0.1, 0.75]]  # pick
    target_orientation = [-np.pi/2, 0, np.pi/2]

    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientation), q0=q0)
    robot.set_joint_target_positions(q1, precision=True)
    qi = q1
    for target_position in target_positions:
        qi = inverse_kinematics(robot=robot, target_position=target_position,
                            target_orientation=Euler(target_orientation), q0=qi)
        robot.set_joint_target_positions(qi, precision=True)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()


if __name__ == "__main__":
    welding()

