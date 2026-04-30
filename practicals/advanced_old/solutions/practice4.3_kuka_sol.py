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
from artelib.inverse_kinematics import moore_penrose_damped
from artelib.euler import Euler
from artelib.path_planning import n_movements, generate_target_positions, generate_target_orientations_Q, move_target_positions_obstacles, potential
from artelib.plottools import plot_xy
from artelib.tools import null_space, compute_kinematic_errors
from sceneconfig.scene_configs_kukalbr import init_simulation_KUKALBR

Dds_global = []


def potentialo(d):
    K = 0.03
    rs = 0.1  # radius of the sphere
    rmax = 0.6
    if d < rs:
        d = rs
    p = K * (1 / d - 1 / rmax)
    if p < 0.0:
        p = 0.0
    return p


def increase_distance_to_obstacles(robot, q):
    robot.set_joint_target_trajectory([q], precision='low')
    d1 = robot.get_min_distance_to_objects()
    f1 = potentialo(d1)
    J, Jv, Jw = robot.manipulator_jacobian(q)
    qd = null_space(J, 6)
    q = q + f1*qd
    robot.set_joint_target_trajectory([q], precision='low')
    d2 = robot.get_min_distance_to_objects()
    f2 = potentialo(d2)
    if f2 < f1:
        return f2*qd, d2-d1
    else:
        return -f1*qd, d1-d2


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
        J, Jv, Jw = robot.manipulator_jacobian(q)
        # compute joint speed to achieve the reference
        qda = moore_penrose_damped(J, e)
        [qdb, ds] = increase_distance_to_obstacles(robot, q)
        Dds_global.append(ds)
        # qdb = 0.8*np.linalg.norm(qda) * qdb
        q = q + qda + qdb
    return q


def inversekinematics_path(robot, target_positions, target_orientations, q0):
    """
        Solve iteratively q for each of the target positions and orientation specified
    """
    q_path = []
    q = q0
    # increase initially the distance to objects
    Dds = []
    for i in range(30):
        print('Increasing distance at line start. Iteration: ', i)
        [qd, Dd] = increase_distance_to_obstacles(robot, q)
        q = q + qd
        Dds.append(Dd)
    # plot_xy(np.arange(len(Dds)), np.array(Dds))
    print("Distance Increased: ", np.sum(np.array(Dds)))

    # now try to reach each target position on the line
    for i in range(len(target_positions)):
        q = inversekinematics_obstacles(robot=robot, target_position=target_positions[i],
                                        target_orientation=target_orientations[i], q0=q)
        q_path.append(q)
    return q_path


def eval_potentials():
    rhos = np.linspace(0, 1, 50)
    p = []
    po = []
    for rho in rhos:
        p.append(potential(rho))
        po.append(potentialo(rho))
    # plot_xy(np.array(rhos), np.array(p), title='potencial trayectoria esfera')
    # plot_xy(np.array(rhos), np.array(po), title='potencial obstáculos')


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.6, 0.5, 0.5],  # initial in front of conveyor
                        [0.6, -0.5, 0.5]]  # drop the piece on the table
    target_orientations = [[0, np.pi / 8, 0],
                           [0, np.pi / 8, 0]]
    sphere_position = [0.7, 0.0, 0.5]
    # eval_potentials()
    sphere.set_object_position(sphere_position)
    q0 = np.array([0.9, 1.4, 1.35, 0.85, -0.75, -0.4,  0.])
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    q0 = robot.inversekinematics(target_positions[0], Euler(target_orientations[0]), q0=q0)
    robot.set_joint_target_positions(q0, precision=True)

    n = n_movements(target_positions[0], target_positions[1], vmax=0.5)
    path_p = generate_target_positions(target_positions[0], target_positions[1], n)
    path_o = generate_target_orientations_Q(Euler(target_orientations[0]), Euler(target_orientations[1]), n)
    path_p = move_target_positions_obstacles(path_p, sphere_position)

    q1_path = inversekinematics_path(robot=robot, target_positions=path_p,
                                     target_orientations=path_o, q0=q0)

    # plot_xy(np.arange(len(Dds_global)), np.array(Dds_global))

    robot.set_joint_target_positions([0, 0, 0, 0, 0, 0, 0], precision=True)
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1_path[0], precision=True)
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_trajectory(q1_path, precision='last')


def application():
    robot, sphere = init_simulation_KUKALBR()
    follow_line_obstacle(robot, sphere)
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    application()
