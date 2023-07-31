#!/usr/bin/env python
# encoding: utf-8
"""
Simple path planning functions.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import euler2rot, rot2quaternion, slerp, rot2euler, quaternion2rot, slerp


def potential(r):
    K = 0.3
    rs = 0.1  # radius of the sphere
    rmax = 0.3
    if r < rs:
        r = rs
    p = K * (1 / r - 1 / rmax)
    if p < 0.0:
        p = 0.0
    return p


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
        Q = slerp(Q1, Q2, t)
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
        Q = slerp(Q1, Q2, t)
        target_orientations.append(Q)
    return target_orientations


def get_closest_to(q0, qb):
    """
    Given a solution q0, find the closest solution in qb
    """
    n_solutions = qb.shape[1]
    distances = []
    for i in range(n_solutions):
        d = np.linalg.norm(qb[:, i]-q0)
        distances.append(d)
    distances = np.array(distances)
    distances = np.nan_to_num(distances, nan=np.inf)
    idx = np.argmin(distances)
    # dd = distances[idx]
    # if dd > 0.5:
    #     print('NOT SMOOTHHHH')
    return qb[:, idx]


def filter_path(robot, q0, qs):
    """
    Computes a path starting at q0 by finding the closest neighbours at each time step.
    """
    q_traj = []
    # remove joints out of range
    for i in range(len(qs)):
        if len(qs[i]) == 0:
            print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
            continue
        qs[i] = robot.filter_joint_limits(qs[i])
    # find the closest solution in a continuous path
    for i in range(len(qs)):
        # print('Movement i: ', i)
        if len(qs[i]) == 0:
            print('ERROR: NO MATHEMATICAL SOLUTIONS TO THE INVERSE KINEMATICS EXIST')
            continue
        qi = get_closest_to(q0, qs[i])
        q0 = qi
        q_traj.append(qi)
    q_traj = np.array(q_traj).T
    return q_traj


def path_planning_line_constant_speed(current_position, current_orientation, target_position, target_orientation,
                       linear_speed=1.0, angular_speed=0.5):
    """
    Plan a path along a line with linear interpolation between positions and orientations.
    """
    Ti = HomogeneousMatrix(current_position, current_orientation)
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    p_current = Ti.pos()
    Qcurrent = Ti.Q()
    p_target = Ttarget.pos()
    Qtarget = target_orientation.Q()

    # Two options:
    # a) if p_current==p_target --> compute number of movements based on slerp distance
    # b) if p_current != p_target--> compute number of movements based on euclidean distance
    n1 = n_movements(p_current, p_target, linear_speed)
    n2 = n_movements_slerp(Qcurrent, Qtarget, angular_speed)

    n = max(n1, n2)
    # generate n target positions
    target_positions = interpolate_target_positions(p_current, p_target, n)
    # generating quaternions on the line. Use SLERP to interpolate between quaternions
    target_orientations = interpolate_target_orientations_Q(Qcurrent, Qtarget, n)
    return target_positions, target_orientations


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


def path_trapezoidal_i(qA, qB, qdA, ttotal, endpoint=False):
    delta_time = 0.05
    if endpoint:
        Ta = 0.2
        Td = 0.2
        qdB = 0
        # Waypoint CaseA
        Tcte = ttotal - Ta - Td
        if Tcte <= 0:
            Tcte = 0
        qdcte = (qB - qA - 0.5 * (qdA * Ta + qdB * Td)) / (Tcte + 0.5 * (Ta + Td))
    else:
        Ta = 0.2
        Td = 0.2
        # Waypoint CaseA
        Tcte = ttotal - Ta - Td
        if Tcte <= 0:
            Tcte = 0
        qdcte = (qB - qA - 0.5*qdA*Ta)/(Tcte + Td + 0.5*Ta)
        qdB = qdcte

    # the two waypoints in the trapezoidal profile
    q1 = qA + qdA * Ta + 0.5 * (qdcte - qdA) * Ta
    q2 = q1 + qdcte * Tcte
    # matrix of restrictions to match qa, qdA, qb, qdB and qdcte
    A = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [1, Ta, Ta**2, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, Ta, Ta**2, 0, 0, 0],
                  [0, 0, 0, 0, 1, 2*Ta, 0, 0, 0],
                  [0, 0, 0, 1, (Ta+Tcte), (Ta+Tcte)**2, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, (Ta+Tcte), (Ta+Tcte)**2],
                  [0, 0, 0, 0, 0, 0, 0, 1, 2*(Ta+Tcte)],
                  [0, 0, 0, 0, 0, 0, 1, ttotal, ttotal**2]])
    # print(A)
    Q = np.array([qA, qdA, q1,
                  q1, qdcte, q2,
                  q2, qdcte, qB])
    k = np.dot(np.linalg.inv(A), Q)
    # in the previous function time_path, ttotal is a factor of delta_time
    n_samples = int(np.round(ttotal/delta_time))
    t = np.linspace(0, ttotal, n_samples)
    qt = []
    qdt = []
    for ti in t:
        if ti <= Ta:
            qt.append(k[0] + k[1]*ti + k[2]*ti**2)
            qdt.append(k[1] + 2*k[2]*ti)
        elif Ta < ti <= Ta+Tcte:
            qt.append(k[3] + k[4] * ti + k[5] * ti ** 2)
            qdt.append(k[4] + 2 * k[5] * ti)
        else:
            qt.append(k[6] + k[7] * ti + k[8] * ti ** 2)
            qdt.append(k[7] + 2 * k[8] * ti)
    return t, np.array(qt), np.array(qdt)


def time_trapezoidal_path_i(qA, qB, qdA, qdmax, endpoint=False):
    """
    Computes the time needed for a trapezoidal speed profile for joint i.
    The joint must move form joint position qA to joint position qB
    The starting speed at position qA is qdA.
    The joint is assumed to move at a max speed of qdmax.
    In case of an endpoint
    """
    delta_time = 0.05
    # and end point with three segments
    if endpoint:
        Ta = 0.2
        Td = 0.2
        qdB = 0
    # Waypoint Case
    else:
        Ta = 0.2
        Td = 0.2
        qdB = qdmax
    Tcte = (np.abs(qB - qA) - 0.5 * (qdA + qdmax)*Ta - 0.5 * (qdB + qdmax)*Td) / qdmax
    # if the time at constant speed is negative, then clip to zero
    if Tcte <= 0.0:
        Tcte = 0.0
    # compute total time
    ttotal = Tcte + Ta + Td
    # round to next sample time
    n = np.floor(ttotal/delta_time)+1
    ttotal = delta_time*n
    return ttotal