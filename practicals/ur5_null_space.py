#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from sceneconfig.scene_configs import init_simulation_UR5

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0

# move on the null space (with vh, column 6)
def null_space_along(robot, m, col, n_steps=20):
    """
    Define the task (m)
    Define the direction of movement (column of V)
    Define the number of movements (n_steps)
    """
    for i in range(0, n_steps):
        q = robot.get_joint_positions()
        J, Jv, Jw = robot.get_jacobian(q)
        Jr = J[0:m, :]
        u, s, vh = np.linalg.svd(Jr, full_matrices=True)
        # caution, obtaining v transposed --> must transpose
        qd = vh.T[:, col]
        q = q + DELTA_TIME*qd
        robot.set_joint_target_positions(q, wait=True)


if __name__ == "__main__":
    robot, scene = init_simulation_UR5()
    q0 = np.pi / 8 * np.array([-6, 1, 3, 1, 2, 1])
    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)
    null_space_along(robot, m=3, col=3)
    null_space_along(robot, m=3, col=4)
    null_space_along(robot, m=3, col=5)

    # robot.plot_trajectories()
    robot.stop_arm()
    scene.stop_simulation()

