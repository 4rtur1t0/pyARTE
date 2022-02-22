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


def potential(r):
    K = 0.4
    rs = 0.1 # radius of the sphere
    rmax = 0.15
    if r < rs:
        r = rs
    p = K * (1 / r - 1 / rmax)
    if p < 0:
        p = 0
    return p


def compute_repulsion(pe, ps):
    u = pe - ps
    r = np.linalg.norm(u)
    if r > 0.0:
        u = u / r
    p = potential(r)
    vrep = np.dot(p, u)
    vrep = np.hstack((vrep, np.array([0, 0, 0])))
    return vrep


def inversekinematics_obstacles(robot, sphere, target_position, target_orientation, q0):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    q = q0
    max_iterations = 1500
    ps = sphere.get_position()
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < robot.max_error_dist_inversekinematics and error_orient < robot.max_error_orient_inversekinematics:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        # qd = moore_penrose_damped(J, e)
        qd = delta_q_transpose(J, e)
        q = q + qd
        # [q, _] = robot.apply_joint_limits(q)
    return q


def inversekinematics_path(robot, sphere, target_positions, target_orientations, q0):
    """
        Solve iteratively q for each of the target positions and orientation specified
    """
    q_path = []
    q = q0
    # now try to reach each target position on the line
    for i in range(len(target_positions)):
        q = inversekinematics_obstacles(robot=robot, sphere=sphere, target_position=target_positions[i],
                                        target_orientation=target_orientations[i], q0=q)
        q_path.append(q)
    manips = []
    for q in q_path:
        J, Jv, Jw = robot.get_jacobian(q)
        manip = np.linalg.det(np.dot(J, J.T))
        manips.append(manip)
    plot(manips)
    return q_path


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.5, 0.4, 0.7],  # initial in front of conveyor
                        [0.5, -0.4, 0.7]]  # drop the piece on the table
    target_orientations = [[0, np.pi/8, 0],
                           [0, np.pi/8, 0]]
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    sphere.set_object_position([0.1, -0.4, 0.75])

    # EJERCICIO: MUEVA AL ROBOT EN EL ESPACIO NULO Y
    # HALLE q0 que lo aleje lo más posible de los obstáculos
    q0 = maximize_distance_to_obstacles(robot, q0)
    n = n_movements(target_positions[0], target_positions[1], vmax=0.2)
    path_p = generate_target_positions(target_positions[0], target_positions[1], n)
    path_o = generate_target_orientations_Q(Euler(target_orientations[0]), Euler(target_orientations[1]), n)

    robot.set_joint_target_positions(q0, precision=True)
    q1_path = inversekinematics_path(robot=robot, sphere=sphere, target_positions=path_p,
                                     target_orientations=path_o, q0=q0)

    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, precision='last')
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
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
