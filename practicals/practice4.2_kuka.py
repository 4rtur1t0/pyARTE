#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820_2.ttt scene before running this script.

The demo represents a KUKA LBR IIWA robot trying to maximize the distance to obstacles in the null
 space.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import moore_penrose_damped, delta_q_transpose
from artelib.euler import Euler
from artelib.path_planning import n_movements, generate_target_positions, generate_target_orientations_Q, \
    generate_target_orientations
from artelib.plottools import plot_vars, plot_xy, plot
from artelib.tools import null_space, compute_kinematic_errors
from sceneconfig.scene_configs import init_simulation_KUKALBR

DELTA_TIME = 50.0/1000.0


def move_null_space(robot, q0, dir, nsteps):
    robot.set_joint_target_positions(q0, precision=True)
    # ok perform n movements in null space
    n_movements_in_null_space = nsteps
    q = q0
    q_path = []
    ds = []
    for i in range(0, n_movements_in_null_space):
        print('Movement number: ', i)
        J, Jv, Jw = robot.get_jacobian(q)
        qd = null_space(J, 6)
        if dir == '+' and qd[2] < 0:
            qd = -qd
        elif dir == '-' and qd[2] > 0:
            qd = -qd
        # qd = np.dot(DELTA_TIME, qd)
        q = q + 0.05*qd
        [q, out_of_range] = robot.apply_joint_limits(q)
        if out_of_range:
            break
        q_path.append(q)
    samples = range(0, len(q_path))
    for i in samples:
        robot.set_joint_target_positions(q_path[i], precision=False)
        d = robot.get_min_distance_to_objects()
        ds.append(d)
    return ds, q_path


def maximize_distance_to_obstacles(robot, q):
    ds, qs = move_null_space(robot, q, '-', 200)
    index = np.argmax(ds)
    return qs[index]


def follow_line_obstacle(robot, sphere):
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    sphere.set_object_position([0.1, -0.4, 0.75])

    # EJERCICIO: MUEVA AL ROBOT EN EL ESPACIO NULO Y
    # HALLE q0 que lo aleje lo más posible de los obstáculos
    q0 = maximize_distance_to_obstacles(robot, q0)
    robot.set_joint_target_positions(q0, precision=True)
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
