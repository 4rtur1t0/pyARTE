#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820_2.ttt scene before running this script.
The demo represents a KUKA LBR IIWA robot trying to avoid collisions with a sphere.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.euler import Euler
from sceneconfig.scene_configs_kukalbr import init_simulation_KUKALBR


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.5, 0.5, 0.5],  # initial in front of conveyor
                        [0.5, -0.5, 0.5]]  # drop the piece on the table
    target_orientations = [[0, np.pi / 8, 0],
                           [0, np.pi / 8, 0]]
    sphere_position = [0.6, 0.0, 0.5]
    q0 = np.array([np.pi/2,  0.2,   0, -np.pi / 2,   0., -np.pi / 2,   0.])
    vmax = 0.2
    sphere.set_object_position(sphere_position)

    # plan trajectories
    q1_path = robot.inversekinematics_line(target_position=target_positions[0],
                                           target_orientation=Euler(target_orientations[0]), q0=q0,
                                           sphere_position=sphere_position, vmax=vmax)
    q2_path = robot.inversekinematics_line(target_position=target_positions[1],
                                           target_orientation=Euler(target_orientations[1]), q0=q1_path[-1],
                                           sphere_position=sphere_position, vmax=vmax)

    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, precision='last')
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_trajectory(q2_path, precision='last')
    robot.wait(15)


def application():
    robot, sphere = init_simulation_KUKALBR()
    follow_line_obstacle(robot, sphere)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    application()
