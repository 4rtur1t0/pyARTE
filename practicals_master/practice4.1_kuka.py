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
from sceneconfig.scene_configs import init_simulation_KUKALBR

DELTA_TIME = 50.0/1000.0

def slerp(Qa, Qb, c):

    return Q


def generate_target_positions(pa, pb, n):

    return target_positions


def generate_target_orientations(Qa, Qb, n):

    return target_orientations


def move_target_positions_obstacles(target_positions, sphere_position):
    """
    Moves a series of points on a path considering a repulsion potential field.
    """
    sphere_position = np.array(sphere_position)
    final_positions = target_positions
    while True:
        total_potential = 0
        for i in range(len(final_positions)):
            r = np.linalg.norm(sphere_position-final_positions[i])
            u = final_positions[i]-sphere_position
            if r > 0:
                u = u/r
            pot = potential(r)
            # modify current position in the direction of u considering potential > 0
            final_positions[i] = final_positions[i] + 0.01*pot*u
            total_potential += pot
        if total_potential < 0.01:
            break
    return final_positions


def planif_path_task_space(target_positions, target_orientations, sphere_position):
    n = n_movements(target_positions[0], target_positions[1], vmax=0.1, delta_time=0.05)
    target_positions = generate_target_positions(target_positions[0],
                                                 target_positions[1], n)
    target_orientations = generate_target_orientations_Q(target_orientations[0].Q(),
                                                         target_orientations[1].Q(), n)
    # target_positions = move_target_positions_obstacles(target_positions, sphere_position)
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
    sphere_position = [0.55, 0.0, 0.5]
    # necesita cambiar la posición central
    sphere.set_object_position(sphere_position)

    # initial arm position
    q0 = np.array([-np.pi / 8, 0, 0, -np.pi / 2, 0, 0, 0])
    path = planif_path(target_positions, target_orientations, sphere_position)
    q1_path = robot.inversekinematics_path(target_positions=path[0],
                                           target_orientations=path[1], q0=q0)

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1_path[0], precision=True)
    robot.wait(15)
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
