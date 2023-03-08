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


def n_movements(p_current, p_target, vmax=1.0, delta_time=0.05):
    """
    Compute the number of points on the line, considering a very simple planning:
        - constant speed vmax.
        - simulation delta_time in Coppelia.
    """
    total_time = np.linalg.norm(np.array(p_target) - np.array(p_current)) / vmax
    n = total_time / delta_time
    n = np.ceil(n)
    return int(n)


def n_movements_slerp(Q_current, Q_target, wmax=3.0, delta_time=0.05):
    """
    Compute the number of points in orientation based on cosinus distance between quaternions
        - constant speed wmax.
        - simulation delta_time in Coppelia.
    """
    cth = Q_current.dot(Q_target)
    th = np.arccos(cth)
    total_time = th / wmax
    n = total_time / delta_time
    n = np.ceil(n)
    return int(n)


def generate_target_positions(p_current, p_target, n):
    """
    Generate n points between the current and target positions p_current and p_target
    """
    tt = np.linspace(0, 1, int(n))
    target_positions = []
    p_current = np.array(p_current)
    p_target = np.array(p_target)
    for t in tt:
        target_pos = t*p_target + (1-t)*p_current
        target_positions.append(target_pos)
    return target_positions


def generate_target_orientations(abc_current, abc_target, n):
    """
    Generate a set of interpolated orientations. The initial Euler angles are converted to Quaternions
    """
    Q1 = abc_current.Q()
    Q2 = abc_target.Q()
    tt = np.linspace(0, 1, int(n))
    target_orientations = []
    for t in tt:
        Q = slerp(Q1, Q2, t)
        target_orientations.append(Q.Euler())
    return target_orientations


def generate_target_orientations_Q(Q1, Q2, n):
    """
    Generate a set of n quaternions between Q1 and Q2. Use SLERP to find an interpolation between them.
    """
    Q1 = Q1.Q()
    Q2 = Q2.Q()
    tt = np.linspace(0, 1, int(n))
    target_orientations = []
    for t in tt:
        Q = slerp(Q1, Q2, t)
        target_orientations.append(Q)
    return target_orientations


def path_planning_line(current_position, current_orientation, target_position, target_orientation,
                       linear_speed=0.5, angular_speed=0.2):
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
    # b) if p_current != p_target--> compunte number of movements based on euclidean distance
    n1 = n_movements(p_current, p_target, linear_speed)
    n2 = n_movements_slerp(Qcurrent, Qtarget, angular_speed)

    n = max(n1, n2)
    # generate n target positions
    target_positions = generate_target_positions(p_current, p_target, n)
    # generating quaternions on the line. Use SLERP to interpolate between quaternions
    target_orientations = generate_target_orientations_Q(Qcurrent, Qtarget, n)
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
