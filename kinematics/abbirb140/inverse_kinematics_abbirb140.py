#!/usr/bin/env python
# encoding: utf-8
"""
The specific inverse kinematic function for the ABBIRB140 robot. This function is called from robots/abbirb140.py

@Authors: Arturo Gil
@Time: April 2026
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix


def inverse_kinematics_abbirb140(robot, target_position, target_orientation, extended=False, q0=None):
    """
    Inverse kinematic method for the ABB IRB140 robot.

    Please, beware that the ABB robot corresponds to a modified version of the original robot that is included in
    Coppelia. In particular, the movement direction of joint2 and joint3 have been reversed and now match the
    positive direction specified by the manufacturer (ABB).

    Generally, given an end effector position and orientation, 8 different solutions are provided for the inverse
    kinematic problem. If the extended option is enabled, some extra solutions are provided. These solutions exist,
    given that the joint ranges for q4 and q6 are [-400, 400] degrees.

    CAUTION: q0 is used here only on situations close to (or in) a singularity.
    In case of a wrist singularity (q5=0), q4 takes the last valid value in q0 and q6 takes q4+q6=C --> q6 = C-q4.
    """
    q = []
    # build Target matrix
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    # Remove Ttcp, so that T_end_effector is specified
    Tcp_inv = robot.Ttcp.inv()
    Ttarget = Ttarget * Tcp_inv
    # Compute wrist position in X0Y0Z0
    Pm = compute_Pm(robot, Ttarget)
    # solve for q1
    q1s = solve_q1(Pm, q0)
    # for each possible solution in q1, compute q2 and q3
    for i in range(len(q1s)):
        # for each q1 solve for q2, q3. Caution do not normalize q2 or q3
        q2, q3 = solve_for_theta23(robot, q1s[i], Pm)
        if np.isnan(np.sum(q2 + q3)):
            continue
        v1 = np.array([q1s[i], q2[0], q3[0]])
        v2 = np.array([q1s[i], q2[1], q3[1]])
        q.append(v1)
        q.append(v2)
    # make them real numbers! and transpose
    q = np.array(q)
    q = q.T
    if len(q) == 0:
        robot.error_print.print('inverse_kinematics_abbirb140 ERROR: KINEMATIC ERROR.', 'red')
        robot.error_print.print('ERROR: NO SOLUTIONS FOUND!! IS THE TARGET POINT REACHABLE?', 'red')
        return np.array([])
    n_solutions = q.shape[1]
    q_total = []
    # solve the last three joints, for each value for q1, q2 and q3
    for i in range(n_solutions):
        qi = q[:, i]
        # two different orientations with q4 in [-pi, pi] and q6 in [-pi, pi]
        # in the non extended version, two different solutions are provided
        qwa, qwb = solve_spherical_wrist(robot, qi, Ttarget, q0)
        if not extended:
            # append the two solutions to the last three joints
            q1 = np.concatenate((qi, qwa), axis=0)
            q2 = np.concatenate((qi, qwb), axis=0)
            q_total.append(q1)
            q_total.append(q2)
        # in the extended version, we consider the extended range in q4 and q6 (adding +-2pi combinations)
        else:
            # qwa, qwb = solve_spherical_wrist(robot, qi, Ttarget, q0)
            q1 = extend_solutions(qi, qwa)
            q2 = extend_solutions(qi, qwb)
            q_total.extend(q1)
            q_total.extend(q2)
    q_total = np.array(q_total)
    # transpose, solutions are arranged by columns
    q_total = q_total.T
    return q_total


def compute_Pm(robot, Ttarget):
    """
    Compute the center position of the wrist
    """
    # get the value from the robot class (last link length)
    L6 = robot.serial_parameters.transformations[5].d
    # Position of the end effector
    P = Ttarget.pos()
    # z6=z5
    z6 = np.array(Ttarget.array[0:3, 2])
    # Compute wrist center point
    Pm = P.T - L6 * z6.T
    return Pm


def solve_q1(Pm, q0):
    """
    q0 includes the last known joint position of the robot. This information is only used in the case that a
    Z0 singularity is found
    """
    d = np.linalg.norm(Pm[0:2])
    # print('Solve q1: ', d)
    # print(d)
    # check singularity
    if d < 0.01 and q0 is not None:
        # in this case, we are inside the singularity, and we decide to use the previous solution
        q1 = q0[0]
    else:
        # if q(1) is a solution, then q(1) +- pi is also a solution
        # this is the general case.
        q1 = np.arctan2(Pm[1], Pm[0])
    q1_a = q1 + np.pi
    q1_b = q1 - np.pi
    q1_a = np.arctan2(np.sin(q1_a), np.cos(q1_a))
    q1_b = np.arctan2(np.sin(q1_b), np.cos(q1_b))
    q1s = np.array([q1, q1_a, q1_b])
    # Find unique values within 7 decimals
    q1s, idx = np.unique(q1s.round(decimals=7), axis=0, return_index=True)
    q1s = q1s[idx]
    return q1s


def solve_for_theta23(robot, q1, Pm):
    # See arm geometry
    L2 = robot.serial_parameters.transformations[1].a
    L3 = robot.serial_parameters.transformations[3].d
    A01 = robot.serial_parameters.transformations[0].dh(q1)
    Pm = np.concatenate((Pm, [1]), axis=0)
    # Express     Pm in the     reference     system     1,    for convenience
    p1 = np.dot(A01.inv().toarray(), Pm.T)
    r = np.linalg.norm(np.array([p1[0], p1[1]]))
    beta = np.arctan2(-p1[1], p1[0])
    a = (L2 ** 2 + r ** 2 - L3 ** 2) / (2 * r * L2)
    b = (L2 ** 2 + L3 ** 2 - r ** 2) / (2 * L2 * L3)
    if np.abs(a) < 1.0:
        gamma = np.arccos(a)
    else:
        # print('WARNING: ONE OF THE INVERSE KINEMATIC SOLUTIONS IS NOT FEASIBLE (ABB IRB140 ROBOT). The point is out of the workspace')
        gamma = np.nan
    if np.abs(b) < 1.0:
        eta = np.arccos(b)
    else:
        # print('WARNING: ONE OF THE INVERSE KINEMATIC SOLUTIONS IS NOT FEASIBLE (ABB IRB140 ROBOT). The point is out of the workspace')
        eta = np.nan
    # elbow  up
    q2_1 = np.pi / 2 - beta - gamma
    # elbow  down
    q2_2 = np.pi / 2 - beta + gamma
    # elbow up
    q3_1 = np.pi / 2 - eta
    # elbow down
    q3_2 = eta - 3 * np.pi / 2
    # joint ranges are considered and we try to restrict the solution in that case.
    if q2_1 < robot.joint_parameters.joint_ranges[0, 1] or q2_1 > robot.joint_parameters.joint_ranges[1, 1]:
        q2_1 = np.arctan2(np.sin(q2_1), np.cos(q2_1))
    if q2_2 < robot.joint_parameters.joint_ranges[0, 1] or q2_2 > robot.joint_parameters.joint_ranges[1, 1]:
        q2_2 = np.arctan2(np.sin(q2_2), np.cos(q2_2))
    if q3_1 < robot.joint_parameters.joint_ranges[0, 2] or q3_1 > robot.joint_parameters.joint_ranges[1, 2]:
        q3_1 = np.arctan2(np.sin(q3_1), np.cos(q3_1))
    if q3_2 < robot.joint_parameters.joint_ranges[0, 2] or q3_2 > robot.joint_parameters.joint_ranges[1, 2]:
        q3_2 = np.arctan2(np.sin(q3_2), np.cos(q3_2))
    return np.array([q2_1, q2_2]), np.array([q3_1, q3_2])


def solve_spherical_wrist(robot, q, T, q0):
    """
    q store the values of the lower joints q1, q2 and q3
    T the orientation of the end effector
    q0 is used only in the case of a singularity.
    if q0 is not None and q5=0, then, q4 is obtained from q0 and, since q4+q6=C, q6=C-q4
    Solve robot's wrist using an algebraic solution
    % [s(q4)*s(q6)-c(q4)*c(q5)*c(q6), c(q6)*s(q4)+c(q4)*c(q5)*s(q6), -c(q4)*s(q5)]
    % [-c(q4)*s(q6)-c(q5)*c(q6)*s(q4), c(q5)*s(q4)*s(q6)-c(q4)*c(q6),-s(q4)*s(q5)]
    % [-c(q6)*s(q5),                  s(q5)*s(q6),                     c(q5)]

    % degenerate
    % [-cos(q4 + q6),np.sin(q4 + q6), 0, 0]
    % [-sin(q4 + q6), -cos(q4 + q6), 0, 0]
    % [0, 0, 1, 89 / 200]
    % [0, 0, 0, 1]
    """
    A01 = robot.serial_parameters.transformations[0].dh(q[0])
    A12 = robot.serial_parameters.transformations[1].dh(q[1])
    A23 = robot.serial_parameters.transformations[2].dh(q[2])
    # this allows to compute the value of A34*A45*A56
    Q = A23.inv() * A12.inv() * A01.inv() * T
    # detect the degenerate case when q(5) = 0, this leads to zeros   % in Q13, Q23, Q31 and Q32 and Q33 = 1
    # thresh = 0.000001
    thresh = 1e-6
    delta = 1.0 - abs(Q[2, 2])
    # degenerate solution
    if delta < thresh:
        # print(50 * '!')
        # print('Degenerate')
        # degenerate solution
        q5 = np.real(np.arccos(Q[2, 2]))
        q5_ = -q5
        # try to get the last valid solution of q4
        if q0 is None:
            # robot.wrist_sing.append(1)
            # find a solution by saying q4=0
            q4 = 0.0
            q4_ = np.pi
            q6 = np.arctan2(Q[0, 1], -Q[1, 1])
            q6_ = q6 - np.pi
        else:
            # robot.wrist_sing.append(2)
            # obtain the value from the last solution
            q4 = q0[3]
            q4_ = q4 + np.pi
            q6 = np.arctan2(Q[0, 1], -Q[1, 1]) - q4
            q6_ = np.arctan2(Q[0, 1], -Q[1, 1]) - q4_
    else:
        # standard solution
        # robot.wrist_sing.append(3)
        q5 = np.arccos(Q[2, 2])
        # alternate solution -q5
        q5_ = -q5
        s5 = np.sign(q5)
        s5_ = np.sign(q5_)
        q4 = np.arctan2(-s5 * Q[1, 2], -s5 * Q[0, 2])
        q4_ = np.arctan2(-s5_ * Q[1, 2], -s5_ * Q[0, 2])
        q6 = np.arctan2(s5 * Q[2, 1], -s5 * Q[2, 0])
        q6_ = np.arctan2(s5_ * Q[2, 1], -s5_ * Q[2, 0])
    # normalize
    q4 = np.arctan2(np.sin(q4), np.cos(q4))
    q4_ = np.arctan2(np.sin(q4_), np.cos(q4_))
    q6 = np.arctan2(np.sin(q6), np.cos(q6))
    q6_ = np.arctan2(np.sin(q6_), np.cos(q6_))
    wrist1 = [q4, q5, q6]
    wrist2 = [q4_, q5_, q6_]
    return np.array(wrist1), np.array(wrist2)


def extend_solutions(qi, qw):
    """
    Adds combinations of +-2pi to the solutions in wrist and concatenates
    """
    q_total = []
    combinations = np.array([[0, 0, 0],
                             [2*np.pi, 0, 0],
                             [-2*np.pi, 0, 0],
                             [0, 0, 2 * np.pi],
                             [0, 0, -2 * np.pi],
                             [2 * np.pi, 0, 2 * np.pi],
                             [-2 * np.pi, 0, -2 * np.pi],
                             [2 * np.pi, 0, -2 * np.pi],
                             [-2 * np.pi, 0, 2 * np.pi]])
    for i in range(len(combinations)):
        qwi = qw + np.array(combinations[i])
        qt = np.concatenate((qi, qwi))
        q_total.append(qt)
    return q_total


def check_can_be_reached_abbirb140(robot, target_positions, target_orientations):
    """
    Check that the targets are in range
    """
    ranges_ok = []
    for i in range(len(target_positions)):
        range_ok = check_can_be_reached_abbirb140_i(robot=robot,
                                                    target_position=target_positions[i],
                                                    target_orientation=target_orientations[i])
        ranges_ok.append(range_ok)
    # if any is False, then return that the trajectory contains errors.
    if False in ranges_ok:
        total = False
    else:
        total = True
    return total, ranges_ok


def check_can_be_reached_abbirb140_i(robot, target_position, target_orientation):
    """
    This computes if the specified target_position and target_orientation is reachable
    for the IRB140 robot.
    It actually looks for all
    """
    qinv = robot.inverse_kinematics(target_position=target_position,
                                    target_orientation=target_orientation)
    qi = robot.filter_joint_limits(qinv)
    qi = qi.squeeze()
    if len(qi):
        return True
    else:
        return False
