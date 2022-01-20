#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np

from artelib.plottools import plot_vars
from artelib.tools import buildT
from sceneconfig.scene_configs import init_simulation_KUKALBR

DELTA_TIME = 50/1000


def null_space(J):
    """
    Obtain a unit vector in the direction
    """
    u, s, vh = np.linalg.svd(J, full_matrices=True)
    qd = vh.T[:, 6]
    return qd


def move_null_space(robot):
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    robot.set_joint_target_positions(q0, precision=True)
    # ok perform n movements in null space
    n_movements_in_null_space = 300
    q = q0
    q_path = []
    qd_path = []
    for i in range(0, n_movements_in_null_space):
        print('Movement number: ', i)
        J, Jv, Jw = robot.get_jacobian(q)
        qd = null_space(J)
        # integrate movement. Please check that Delta_time matches coppelia simulation time step
        if qd[2] < 0:
            qd = -qd
        qd_path.append(qd)
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
        q_path.append(q)
    robot.set_joint_target_trajectory(q_path, precision='none')
    plot_vars(qd_path, title='JOINT SPEEDS')


if __name__ == "__main__":
    robot, _ = init_simulation_KUKALBR()
    move_null_space(robot)
    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()
