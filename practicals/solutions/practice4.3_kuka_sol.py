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
    generate_target_orientations, move_target_positions_obstacles, potential
from artelib.plottools import plot_vars, plot_xy, plot
from artelib.tools import null_space, compute_kinematic_errors
from sceneconfig.scene_configs import init_simulation_KUKALBR


# DELTA_TIME = 50.0/1000.0


def increase_distance_to_obstacles(robot, q):
    robot.set_joint_target_positions(q, precision=True)
    d1 = robot.get_min_distance_to_objects()
    f1 = potential(d1)
    J, Jv, Jw = robot.get_jacobian(q)
    qd = null_space(J, 6)
    q = q + 0.1 * qd
    robot.set_joint_target_positions(q, precision=True)
    robot.wait()
    d2 = robot.get_min_distance_to_objects()
    f2 = potential(d2)

    # just returning zero if both potential functions are zero
    if f2 == 0 and f1 == 0:
        return 0 * qd

    if f2 < f1:
        return qd
    else:
        return -qd

    # if d2 > d1:
    #     return qd
    # else:
    #     return -qd


def inversekinematics_obstacles(robot, target_position, target_orientation, q0):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    q = q0
    max_iterations = 1500
    for i in range(max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < robot.max_error_dist_inversekinematics and error_orient < robot.max_error_orient_inversekinematics:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        qda = moore_penrose_damped(J, e)
        qdb = increase_distance_to_obstacles(robot, q)
        qdb = 0.1 * np.linalg.norm(qda) * qdb
        q = q + qda + qdb
    return q


def inversekinematics_path(robot, target_positions, target_orientations, q0):
    """
        Solve iteratively q for each of the target positions and orientation specified
    """
    q_path = []
    q = q0
    # now try to reach each target position on the line
    for i in range(len(target_positions)):
        q = inversekinematics_obstacles(robot=robot, target_position=target_positions[i],
                                        target_orientation=target_orientations[i], q0=q)
        q_path.append(q)
    return q_path


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.6, 0.5, 0.5],  # initial in front of conveyor
                        [0.6, -0.5, 0.5]]  # drop the piece on the table
    target_orientations = [[0, np.pi / 8, 0],
                           [0, np.pi / 8, 0]]
    sphere_position = [0.7, 0.0, 0.5]
    # necesita cambiar la posición central
    sphere.set_object_position(sphere_position)
    # initial arm position
    q0 = np.array([0.65, 0.2, 0, -1.9, 0., -1.1, 0.])

    n = n_movements(target_positions[0], target_positions[1], vmax=0.2)
    path_p = generate_target_positions(target_positions[0], target_positions[1], n)
    path_o = generate_target_orientations_Q(Euler(target_orientations[0]), Euler(target_orientations[1]), n)
    path_p = move_target_positions_obstacles(path_p, sphere_position)

    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    # set the target we are willing to reach on Coppelia
    robot.set_joint_target_positions(q0, precision=True)

    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    q1_path = inversekinematics_path(robot=robot, target_positions=path_p,
                                     target_orientations=path_o, q0=q0)

    robot.set_joint_target_trajectory(q1_path, precision='last')

    # robot.set_joint_target_trajectory(q2_path, precision='last')
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
