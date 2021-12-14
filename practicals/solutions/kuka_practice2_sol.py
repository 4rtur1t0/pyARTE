#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.tools import buildT
from sceneconfig.scene_configs import init_simulation_KUKALBR

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


def pick_and_place():
    robot, scene = init_simulation_KUKALBR()
    # target_positions = [[0.6, -0.2, 0.25], # initial in front of conveyor
    #                     [0.6, 0.1, 0.25], # pick the piece
    #                     [0.6, 0.1, 0.35], # bring the piece up
    #                     [0.2, -0.6, 0.4], # over the table
    #                     [0.2, -0.6, 0.3]] # drop the piece
    # target_orientations = [[-np.pi/2, 0, -np.pi/2],
    #                        [-np.pi/2, 0, -np.pi/2],
    #                        [-np.pi/2, 0, -np.pi/2],
    #                        [-np.pi, 0, 0],
    #                        [-np.pi, 0, 0]]

    target_positions = [[0.4, 0.4, 0.25]]
    target_orientations = [[-3*np.pi/2, np.pi/2, np.pi]]

    q0 = np.array([-np.pi/8, 0, 0, -np.pi/2, 0, 0, 0])
    # set initial position of robot
    robot.set_arm_joint_target_positions(q0, wait=True)

    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    # plan trajectories
    [q1_path, _] = robot.inversekinematics_line(target_position=target_positions[0],
                                               target_orientation=target_orientations[0], q0=q0)
    robot.follow_q_trajectory(q1_path, wait=False)

    # [q2_path, _] = robot.inversekinematics_line(target_position=target_positions[1],
    #                                        target_orientation=target_orientations[1], q0=q1_path[-1])
    # [q3_path, _] = robot.inversekinematics_line(target_position=target_positions[2],
    #                                        target_orientation=target_orientations[2], q0=q2_path[-1])
    # [q4_path, _] = robot.inversekinematics_line(target_position=target_positions[3],
    #                                        target_orientation=target_orientations[3], q0=q3_path[-1])
    # [q5_path, _] = robot.inversekinematics_line(target_position=target_positions[4],
    #                                        target_orientation=target_orientations[4], q0=q4_path[-1])

    # execute trajectories
    robot.open_gripper()
    robot.follow_q_trajectory(q1_path, wait=False)
    # robot.follow_q_trajectory(q2_path)
    robot.close_gripper(wait=True)
    # robot.follow_q_trajectory(q3_path)
    # robot.follow_q_trajectory(q4_path)
    # robot.follow_q_trajectory(q5_path)
    robot.open_gripper(wait=True)
    # robot.follow_q_trajectory([q4])

    # get an image and save it as an example
    [image, resolution] = robot.get_image()
    robot.save_image(image=image, resolution=resolution, filename='test.png')
    robot.stop_arm()
    scene.stop_simulation()
    robot.plot_trajectories()

if __name__ == "__main__":
    pick_and_place()

