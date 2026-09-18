#!/usr/bin/env python
# encoding: utf-8
"""
Simple path planning functions.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
#from artelib.tools import slerp


def interpolate_targets_trapezoidal_line(robot, target_position, target_orientation, speed_factor,
                                         plot, precision):
    v_max = robot.joint_parameters.max_linear_velocity * speed_factor
    a_max = robot.joint_parameters.max_linear_acceleration * speed_factor
    w_max = robot.joint_parameters.max_angular_velocity * speed_factor
    alpha_max = robot.joint_parameters.max_angular_acceleration * speed_factor
    # current joint position
    q0 = robot.get_joint_positions()
    T0 = robot.directkinematics(q0)
    T1 = HomogeneousMatrix(target_position, target_orientation)
    # compute Euclidean distance
    d_euclidea = np.linalg.norm(T0.pos() - T1.pos())
    # compute the angle between both orientations
    R0 = T0.R()
    R1 = T1.R()
    d_angle = R0.angle_between(R1)
    # perform planning as a trapezoidal profile on euclidean space or angle space
    # selecting max lineal speed as indicated in the function and also max angular speed
    st, sdt, sddt, time = robot.planner.trapezoidal_coordinated(s0=[0.0, 0.0],
                                                                sf=[d_euclidea, d_angle],
                                                                v_max=[v_max, w_max],
                                                                a_max=[a_max, alpha_max],
                                                                precision=precision,
                                                                plot=plot)
    # normalize factors to 1 max. Caution: the distance should not be zero
    if np.abs(d_euclidea) > 0.01:
        st_factors = st[0] / d_euclidea
    elif np.abs(d_angle) > 0.01:
        st_factors = st[1] / d_angle
    else:
        # a case in which the st is zero or mostly zero
        st_factors = st[0]
    target_positions, target_orientations = path_planning_line_factors(T0.pos(), T0.R(),
                                                                       T1.pos(), T1.R(),
                                                                       factors=st_factors)
    return target_positions, target_orientations, time


def compute_qpath_along_line(robot, target_positions, target_orientations, extended):
    """
    Given a number of target positions and orientations along the line.
        At each target_position, compute qinv
        Select the closest qinv considering the previous solution.
        Filter for joint ranges
    """
    # the last joint position is saved to help find a solution in singularities
    q0 = robot.get_joint_positions()
    q_path = []
    # now try to reach each target position on the line
    for i in range(len(target_positions)):
        # q0 is specified here to solve kinematic singularities
        qis = robot.inverse_kinematics(target_position=target_positions[i],
                                       target_orientation=target_orientations[i],
                                       extended=extended, q0=q0)
        # continue in case no solutions exist
        if len(qis) == 0:
            continue
        # find the solutions that is closest to the previous solution
        qi = get_closest_to(q0, qis)
        #d = np.linalg.norm(q0 - qi)
        # print('Distance to closest last solution: ', d)
        # filter solutions out of limits
        qi = robot.filter_joint_limits(qi)
        qi = qi.squeeze()
        if len(qi):
            q_path.append(qi.T)
            # caution, store next value to get closest joint on the next iteration
            q0 = qi
        else:
            robot.error_print.print('moveL KINEMATIC ERROR: one of the joints went out of range', 'red')
    # build the path with the correct dimensions
    q_path = np.vstack(q_path).T
    return q_path

def random_q(robot):
    """
    Generate a random q uniformly distributed in the joint ranges
    """
    q = []
    for i in range(robot.DOF):
         qi = np.random.uniform(robot.joint_ranges[0, i], robot.joint_ranges[1, i], 1)
         q.append(qi[0])
    return np.array(q)


def n_movements(p_current, p_target, vmax=1.0, delta_time=0.05):
    """
    Compute the number of points on the line, considering a very simple planning:
        - constant speed vmax.
        - simulation delta_time in Coppelia.
    """
    total_time = np.linalg.norm(np.array(p_target) - np.array(p_current)) / vmax
    n = total_time / delta_time
    # at least, two movements, that correspond to the begining and end target point
    n = np.ceil(n) + 1
    return int(n)


def n_movements_slerp(Q_current, Q_target, wmax=3.0, delta_time=0.05):
    """
    Compute the number of points in orientation based on cosinus distance between quaternions
        - constant speed wmax.
        - simulation delta_time in Coppelia.
    """
    cth = np.abs(Q_current.dot(Q_target))
    # caution: saturate to +-1
    cth = np.clip(cth, -1.0, 1.0)
    th = np.arccos(cth)
    total_time = th / wmax
    n = total_time / delta_time
    n = np.ceil(n)
    return int(n)


def interpolate_target_positions(p_current, p_target, n):
    """
    Generate n points between the current and target positions p_current and p_target
    """
    if isinstance(n, int):
        tt = np.linspace(0, 1, int(n))
    else:
        tt = n
    target_positions = []
    p_current = np.array(p_current)
    p_target = np.array(p_target)
    for t in tt:
        target_pos = t*p_target + (1-t)*p_current
        target_positions.append(target_pos)
    return target_positions


def interpolate_target_orientations(abc_current, abc_target, n):
    """
    Generate a set of interpolated orientations. The initial Euler angles are converted to Quaternions
    """
    if isinstance(n, int):
        tt = np.linspace(0, 1, int(n))
    else:
        tt = n
    Q1 = abc_current.Q()
    Q2 = abc_target.Q()
    target_orientations = []
    for t in tt:
        Q = Q1.slerp(Q2, t)
        target_orientations.append(Q.Euler())
    return target_orientations


def interpolate_target_orientations_Q(Q1, Q2, n):
    """
    Generate a set of n quaternions between Q1 and Q2. Use SLERP to find an interpolation between them.
    Caution: if n is an int, then, n equally spaced orientations are placed between Q1 and Q2.
    if n is a list, it is assumed that it is a list of factors for slerp in [0, 1]
    """
    if isinstance(n, int):
        tt = np.linspace(0, 1, int(n))
    else:
        tt = n
    Q1 = Q1.Q()
    Q2 = Q2.Q()
    target_orientations = []
    for t in tt:
        Q = Q1.slerp(Q2, t)
        target_orientations.append(Q)
    return target_orientations


def get_closest_to(q0, qb):
    """
    Given a solution q0, find the closest solution in qb
    """
    n_solutions = qb.shape[1]
    distances = []
    # weigths = np.array([10, 10, 10, 2, 2, 2])
    weigths = np.array([1, 1, 1, 1, 1, 1])
    for i in range(n_solutions):
        diff = qb[:, i]-q0
        diff = weigths*diff
        d = np.linalg.norm(diff)
        distances.append(d)
    distances = np.array(distances)
    # distances = np.nan_to_num(distances, nan=np.inf)
    idx = np.argmin(distances)
    # dd = np.min(distances)
    # if dd > 5:
    #     print('debug')
    # dd = distances[idx]
    # if dd > 0.5:
    #     print('NOT SMOOTHHHH')
    return qb[:, idx]


def filter_joint_limits_in_path(robot,qs):
    """
    Removes solutions with joint values out of joint range.
    """
    # remove joints out of range
    for i in range(len(qs)):
        if len(qs[i]) == 0:
            print('WARNING: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
            continue
        qs[i] = robot.filter_joint_limits(qs[i])
    return qs


def build_path_by_closest(q0, qs):
    """
    Then selects the closest joints.
    Computes a path starting at q0 by finding the closest neighbours at each time step.
    """
    q_traj = []
    # remove joints out of range
    # for i in range(len(qs)):
    #     if len(qs[i]) == 0:
    #         print('WARNING: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
    #         continue
    #     qs[i] = robot.filter_joint_limits(qs[i])
    # find the closest solution in a continuous path
    for i in range(len(qs)):
        # print('Movement i: ', i)
        if len(qs[i]) == 0:
            print('WARNING: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
            continue
        qi = get_closest_to(q0, qs[i])
        q0 = qi
        q_traj.append(qi)
    q_traj = np.array(q_traj).T
    return q_traj


def path_planning_line_factors(current_position, current_orientation, target_position, target_orientation, factors):
    """
    Plan a path along a line with linear interpolation between positions and orientations.
    """
    Ti = HomogeneousMatrix(current_position, current_orientation)
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    p_current = Ti.pos()
    Qcurrent = Ti.Q()
    p_target = Ttarget.pos()
    Qtarget = target_orientation.Q()

    # generate n target positions
    target_positions = interpolate_target_positions(p_current, p_target, factors)
    # generating quaternions on the line. Use SLERP to interpolate between quaternions
    target_orientations = interpolate_target_orientations_Q(Qcurrent, Qtarget, factors)
    return target_positions, target_orientations



def compute_3D_coordinates(index, n_x, n_y, n_z, piece_length, piece_gap):
    """
    Compute 3D coordinates for cubic pieces in a 3D array.
    Used to get 3D positions for palletizing.
    """
    dxy = piece_length + piece_gap
    dz = piece_length
    # get the indices of a n_i xn_j x n_k array
    i, j, k = np.indices((n_z, n_x, n_y))
    i = i.flatten()
    j = j.flatten()
    k = k.flatten()
    pxyz = []
    for n in range(n_z * n_x * n_y):
        pxyz.append([j[n]*dxy, k[n]*dxy, i[n]*dz])
    pxyz = np.array(pxyz)
    if index < n_z * n_x * n_y:
        return pxyz[index, :]
    else:
        print('WARNING: N PIECES IS LARGER THAN NX*NY*NZ')
        index = index - n_z * n_x * n_y
        return pxyz[index, :]
