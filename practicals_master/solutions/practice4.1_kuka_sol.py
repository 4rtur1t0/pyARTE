#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.
The demo represents a KUKA LBR IIWA robot trying to avoid collisions with a sphere.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.euler import Euler
from artelib.path_planning import move_target_positions_obstacles, generate_target_positions, generate_target_orientations_Q, n_movements
from artelib.plottools import plot3d
from sceneconfig.scene_configs_kukalbr import init_simulation_KUKALBR


def planif_path_task_space(target_positions, target_orientations, sphere_position):
    n = n_movements(target_positions[0], target_positions[1], vmax=0.1, delta_time=0.05)
    target_positions = generate_target_positions(target_positions[0],
                                                 target_positions[1], n)
    target_orientations = generate_target_orientations_Q(target_orientations[0].Q(),
                                                         target_orientations[1].Q(), n)
    target_positions = move_target_positions_obstacles(target_positions, sphere_position)
    p_positions = np.array(target_positions)
    plot3d(p_positions[:, 0], p_positions[:, 1], p_positions[:, 2])
    return target_positions, target_orientations


def planif_path(target_positions, target_orientations, sphere_position):
    orient1 = Euler(target_orientations[0])
    orient2 = Euler(target_orientations[1])
    target_ps = [target_positions[0], target_positions[1]]
    target_ors = [orient1, orient2]
    path_task_space = planif_path_task_space(target_ps, target_ors, sphere_position)
    return path_task_space


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.5, 0.5, 0.5],  # initial in front of conveyor
                        [0.5, -0.5, 0.5]]  # drop the piece on the table
    target_orientations = [[0, np.pi / 8, 0],
                           [0, np.pi / 8, 0]]
    sphere_position = [0.6, 0.0, 0.5]
    # necesita cambiar la posición central
    sphere.set_object_position(sphere_position)

    # initial arm position
    q0 = np.array([0.65,  0.2,   0, -1.9,   0., -1.1,   0.])
    robot.set_joint_target_positions(q0)
    path = planif_path(target_positions, target_orientations, sphere_position)
    q1_path = robot.inversekinematics_path(target_positions=path[0],
                                           target_orientations=path[1], q0=q0)

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1_path[0], precision=True)
    # set the target we are willing to reach on Coppelia
    robot.set_joint_target_trajectory(q1_path, precision='last')
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_trajectory(q1_path[::-1], precision='last')
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
