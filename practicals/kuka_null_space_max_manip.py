#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from sceneconfig.scene_configs import init_simulation_KUKALBR
from artelib.plottools import plot
# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0

# move on the null space (with vh, column 6)
def null_space_max_manip(robot, m, col):
    """
    Define the task (m)
    Define the direction of movement (column of V)
    Define the number of movements (n_steps)
    """
    q = robot.get_joint_positions()
    J, Jv, Jw = robot.get_jacobian(q)
    Jr = J[0:m, :]
    u, s, vh = np.linalg.svd(Jr, full_matrices=True)
    manip1 = np.linalg.det(np.dot(J, J.T))
    # caution, obtaining v transposed --> must transpose
    qd = vh.T[:, col]
    q = q + qd
    J, Jv, Jw = robot.get_jacobian(q)
    manip2 = np.linalg.det(np.dot(J, J.T))
    if manip2 >= manip1:
        return qd
    else:
        return -qd


def path_planner1(robot):
    """
    vanilla: no manip maximization.
    """
    q = np.pi / 8 * np.array([-6, 3, 3, 3, 2, 1, 0])
    v = np.array([0.2, 0, 0, 0, 0, 0])
    # set initial position of robot
    robot.set_joint_target_positions(q, wait=True)
    q_path = []
    manips = []
    for i in range(0, 120):
        J, Jv, Jw = robot.get_jacobian(q)
        qda = np.dot(np.linalg.pinv(J), v)
        [qda, _, _] = robot.check_speed(qda)
        q = q + 0.05*qda
        q_path.append(q)
        manips.append(robot.compute_manipulability(q))
    plot(manips, 'Vanilla')
    return q_path

def path_planner2(robot):
    q = np.pi / 8 * np.array([-6, 3, 3, 3, 2, 1, 0])
    v = np.array([0.2, 0, 0, 0, 0, 0])
    # set initial position of robot
    robot.set_joint_target_positions(q, wait=True)

    q_path = []
    manips = []
    for i in range(0, 120):
        # q = robot.get_joint_positions()
        J, Jv, Jw = robot.get_jacobian(q)
        qda = np.dot(np.linalg.pinv(J), v)
        [qda, _, _] = robot.check_speed(qda)
        qdb = null_space_max_manip(robot, m=6, col=5)
        # la velocidad en el espacio nulo es el 3% de la velocidad en el espacio activo
        qdb = np.dot(0.5 * np.linalg.norm(qda), qdb)
        # ojo, 0.05 es el tiempo de integración de Coppelia
        q = q + 0.05 * (qda - qdb)
        q_path.append(q)
        manips.append(robot.compute_manipulability(q))
    plot(manips, 'Manip increase')

    return q_path

if __name__ == "__main__":
    robot, scene = init_simulation_KUKALBR()

    q_path1 = path_planner1(robot)
    q_path2 = path_planner2(robot)
        # robot.wait(1)
    robot.set_joint_target_trajectory(q_path=q_path1, wait=False)
    robot.set_joint_target_trajectory(q_path=q_path2, wait=False)
    # stop arm, stop simulation... then do some plotting
    robot.stop_arm()
    scene.stop_simulation()
    robot.plot_trajectories()

