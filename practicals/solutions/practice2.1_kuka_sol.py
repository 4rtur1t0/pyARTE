#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from sceneconfig.scene_configs import init_simulation_KUKALBR

def pick_and_place():
    robot, scene = init_simulation_KUKALBR()
    target_positions = [[0.6, -0.2, 0.25],  # initial in front of conveyor
                    [0.6, 0.1, 0.25],  # pick the piece
                    [0.6, 0.0, 0.4],  # bring the piece up (and backwards)
                    [0.2, -0.6, 0.4],  # over the table
                    [0.2, -0.6, 0.3]]  # drop the piece
    target_orientations = [[-np.pi / 2, 0, -np.pi / 2],
                       [-np.pi / 2, 0, -np.pi / 2],
                       [-np.pi / 2, 0, -np.pi / 2],
                       [-np.pi, 0, 0],
                       [-np.pi, 0, 0]]
    # q0 = np.array([-np.pi, 0, np.pi / 2, 0, 0.1, 0.1, 0.1])
    q0 = np.array([-np.pi/8, 0, 0, -np.pi/2, 0, 0, 0])

    # plan trajectories
    q1 = robot.inversekinematics(target_position=target_positions[0],
                                 target_orientation=target_orientations[0], q0=q0)
    q2 = robot.inversekinematics(target_position=target_positions[1],
                                 target_orientation=target_orientations[1], q0=q1)
    q3 = robot.inversekinematics(target_position=target_positions[2],
                                 target_orientation=target_orientations[2], q0=q2)
    q4 = robot.inversekinematics(target_position=target_positions[3],
                                 target_orientation=target_orientations[3], q0=q3)
    q5 = robot.inversekinematics(target_position=target_positions[4],
                                 target_orientation=target_orientations[4], q0=q4)

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)
    robot.open_gripper(wait=True)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1, wait=True)
    robot.set_joint_target_positions(q2, wait=True)
    robot.close_gripper(wait=True)
    robot.set_joint_target_positions(q3, wait=True)
    robot.set_joint_target_positions(q4, wait=True)
    robot.set_joint_target_positions(q5, wait=True)
    robot.open_gripper(wait=True)
    robot.set_joint_target_positions(q4)

    [image, resolution] = robot.get_image()
    robot.save_image(image=image, resolution=resolution, filename='test.png')

    # Stop arm and simulation
    robot.stop_arm()
    scene.stop_simulation()
    robot.plot_trajectories()


def pick_and_place_rep():
    """
    A repeated pick and place application.
    """
    robot, scene = init_simulation_KUKALBR()
    target_positions = [[0.6, -0.2, 0.25],
                        [0.6, 0.1, 0.25],
                        [0.6, -0.1, 0.35],
                        [0.2, -0.6, 0.4]]
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi, 0, 0]]

    q0 = np.array([-np.pi/8, 0, 0, -np.pi/2, 0, 0, 0])

    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)

    # set the target we are willing to reach
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    vmax = 1.5

    for i in range(0, 6):
        # plan trajectories
        [q1_path, _] = robot.inversekinematics_line(target_position=target_positions[0],
                                                    target_orientation=target_orientations[0], q0=q0, vmax=vmax)
        [q2_path, _] = robot.inversekinematics_line(target_position=target_positions[1],
                                                    target_orientation=target_orientations[1], q0=q1_path[-1], vmax=vmax)
        [q3_path, _] = robot.inversekinematics_line(target_position=target_positions[2],
                                                    target_orientation=target_orientations[2], q0=q2_path[-1], vmax=vmax)
        target_pos = target_positions[3] + i * np.array([0, 0.07, 0])
        target_orient = target_orientations[3]
        [q4_path, _] = robot.inversekinematics_line(target_position=target_pos,
                                                    target_orientation=target_orient, q0=q3_path[-1])
        target_pos = target_pos + np.array([0, 0, -0.1])
        [q5_path, _] = robot.inversekinematics_line(target_position=target_pos,
                                                    target_orientation=target_orient, q0=q4_path[-1])
        target_pos = target_pos + np.array([0, 0, 0.1])
        [q6_path, _] = robot.inversekinematics_line(target_position=target_pos,
                                                    target_orientation=target_orient, q0=q5_path[-1])
        # q0 = q6_path[-1]

        # execute trajectories
        robot.open_gripper()
        robot.set_joint_target_trajectory(q1_path, wait=False)
        robot.set_joint_target_trajectory(q2_path, wait=False)
        robot.close_gripper(wait=True)
        robot.set_joint_target_trajectory(q3_path, wait=False)
        robot.set_joint_target_trajectory(q4_path, wait=False)
        robot.set_joint_target_trajectory(q5_path, wait=False)
        robot.open_gripper(wait=True)
        robot.set_joint_target_trajectory(q6_path, wait=False)
        robot.set_joint_target_positions(q0)

    robot.stop_arm()
    scene.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    # pick_and_place()
    pick_and_place_rep()
