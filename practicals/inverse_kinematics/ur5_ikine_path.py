#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from artelib.quaternion import Quaternion
from artelib.tools import buildT, compute_kinematic_errors, slerp
from sceneconfig.scene_configs_ur5 import init_simulation_UR5

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


def n_movements(p_current, p_target, vmax):
    # CALCULE EL NÚMERO DE MOVIMIENTOS

    return int(n)


def path_planning_p(p_current, p_target, n):
    """
    Generates n points over the line defined by p_current and p_target.
    """
    # DEVUELVA N puntos entre p_current y p_target

    return target_positions


def path_planning_o(o_current, o_target, n):
    """
    Generate a set of n quaternions between Q1 and Q2. Use SLERP to find an interpolation between them.
    """
    Q1 = o_current.Q()
    Q2 = o_target.Q()
    tt = np.linspace(0, 1, int(n))
    target_orientations = []
    for t in tt:
        Q = slerp(Q1, Q2, t)
        target_orientations.append(Q)
    return target_orientations


def inverse_kinematics_path(robot, path_p, path_o, q):

    return q_path



def pick_and_place():
    robot = init_simulation_UR5()
    target_positions = [[0.6, -0.3, 0.4],
                        [0.6, -0.2, 0.25],  # initial in front of conveyor
                        [0.6, 0.1, 0.25],  # pick the piece
                        [0.6, -0.1, 0.35],  # bring the piece up
                        [0.4, -0.1, 0.35],  # middle point
                        [0.2, -0.55, 0.4],  # over the table
                        [0.2, -0.55, 0.3],
                        [0.2, -0.55, 0.4]]  # drop the piece
    target_orientations = [[-np.pi / 2, 0, 0],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    # a simple way to open or close the gripper at each target
    open_gripper = [False,
                    True,  # initial in front of conveyor
                    False,  # pick the piece
                    False,  # bring the piece up
                    False,  # middle point
                    False,  # over the table
                    True,
                    True]  # drop the piece
    q = np.array([-np.pi / 2, -np.pi / 8, np.pi / 2, -0.1, -0.1, -0.1])

    for i in range(len(target_positions)-1):
        robot.set_target_position_orientation(target_positions[i+1], target_orientations[i+1])
        n = n_movements(target_positions[i], target_positions[i+1], vmax=0.5)
        path_p = path_planning_p(target_positions[i], target_positions[i+1], n)
        path_o = path_planning_o(Euler(target_orientations[i]), Euler(target_orientations[i+1]), n)
        q_path = inversekinematics_path(robot, path_p, path_o, q)
        robot.set_joint_target_trajectory(q_path, precision='last')

        if open_gripper[i]:
            robot.open_gripper(precision=True)
        else:
            robot.close_gripper(precision=True)
        q = q_path[-1]

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place()

