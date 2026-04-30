#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

    EXERCISE: SOLVE the inverse kinematics problem in a 7DOF redundant robot.
                as a secondary target, minimize the lateral function to avoid
                exceeding the joint limits.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import moore_penrose_damped
from artelib.euler import Euler
from artelib.path_planning import n_movements, generate_target_positions, generate_target_orientations_Q
from artelib.plottools import plot_vars
from artelib.tools import  compute_kinematic_errors
from sceneconfig.scene_configs_kukalbr import init_simulation_KUKALBR


def null_space_projector(J):
    n = J.shape[1]
    I = np.eye(n)
    # pseudo inverse moore-penrose
    # here, do not use pinv
    try:
        Jp = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        P = I - np.dot(Jp, J)
    except np.linalg.LinAlgError:
        P = I
    return P


def saturate_qi(q, qmin, qmax):
    deltaq = np.pi/20
    if q > (qmax - deltaq):
        q = qmax - deltaq
    elif q < (qmin + deltaq):
        q = qmin + deltaq
    return q


def diff_w_lateral(q, qmin, qmax):
    """
    The function computes the differential of the secondary function w_lateral
    """
    n = len(q)
    dw = []
    for i in range(n):
        qi = saturate_qi(q[i], qmin[i], qmax[i])
        dwi_num = (qmin[i] - qmax[i])*(-2*qi + qmax[i] + qmin[i])
        dwi_den = ((qi - qmax[i]) * (qi - qmin[i]))**2
        dwi = dwi_num/dwi_den
        if np.isinf(dwi) or np.isnan(dwi):
            dwi = 0.0
        dw.append(dwi)
    return np.array(dw)


def minimize_w_lateral(J, q, qmax, qmin):
    qd0 = diff_w_lateral(q, qmax, qmin)
    qd0 = np.dot(-1.0, qd0)
    P = null_space_projector(J)
    qdb = np.dot(P, qd0)
    norma = np.linalg.norm(qdb)
    if np.isnan(np.sum(qdb)):
        return np.zeros(len(q))
    if norma > 0.0001:
        return qdb / norma
    else:
        return qdb


def inversekinematics_line(robot, target_position, target_orientation, q0, vmax=0.5):
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    Ti = robot.directkinematics(q0)
    Qcurrent = Ti.Q()
    Qtarget = target_orientation.Q()
    p_current = Ti.pos()
    p_target = Ttarget.pos()
    n = n_movements(p_current, p_target, vmax)
    # generate n target positions
    target_positions = generate_target_positions(p_current, p_target, n)
    # generating quaternions on the line. Use SLERP to interpolate between quaternions
    target_orientations = generate_target_orientations_Q(Qcurrent, Qtarget, len(target_positions))
    q_path = []
    q = q0
    # now try to reach each target position on the line
    for i in range(0, len(target_positions)):
        q = inversekinematics_secondary(robot=robot, target_position=target_positions[i],
                                        target_orientation=target_orientations[i], q0=q)
        q_path.append(q)
    return q_path


def inversekinematics_secondary(robot, target_position, target_orientation, q0):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    q = q0
    max_iterations = 500
    qmin = robot.joint_ranges[0]
    qmax = robot.joint_ranges[1]
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.directkinematics(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        print('e: ', e)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        J, Jv, Jw = robot.manipulator_jacobian(q)
        # compute joint speed to achieve the reference
        qda = moore_penrose_damped(J, e)
        qdb = minimize_w_lateral(J, q, qmin=qmin, qmax=qmax)
        qdb = 0.2 * np.linalg.norm(qda) * qdb
        qd = qda + qdb
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
    return q


def pick_and_place(robot, step_number):
    target_positions = [[0.6, -0.2, 0.25],  # initial in front of conveyor
                        [0.6, 0.1, 0.25],  # pick the piece
                        [0.6, -0.1, 0.50],  # bring the piece up (and backwards)
                        [0.2, -0.7, 0.50],  # over the table
                        [0.2, -0.7, 0.35]]  # drop the piece on the table
    target_orientations = [[-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    # change target points for drop zone
    target_positions[3] = target_positions[3] + step_number * np.array([0, 0.06, 0])
    target_positions[4] = target_positions[4] + step_number * np.array([0, 0.06, 0])

    # initial arm position
    q0 = np.array([-np.pi / 8, 0, 0, -np.pi / 2, 0.1, 0.1, 0.1])
    # q0 = np.array([0,0,0,0,0,0,0])
    # plan trajectories
    q1_path = inversekinematics_line(robot=robot, target_position=target_positions[0],
                                     target_orientation=Euler(target_orientations[0]), q0=q0)
    q2_path = inversekinematics_line(robot=robot, target_position=target_positions[1],
                                     target_orientation=Euler(target_orientations[1]), q0=q1_path[-1])
    q3_path = inversekinematics_line(robot=robot, target_position=target_positions[2],
                                     target_orientation=Euler(target_orientations[2]), q0=q2_path[-1])
    q4_path = inversekinematics_line(robot=robot, target_position=target_positions[3],
                                     target_orientation=Euler(target_orientations[3]), q0=q3_path[-1])
    q5_path = inversekinematics_line(robot=robot, target_position=target_positions[4],
                                     target_orientation=Euler(target_orientations[4]), q0=q4_path[-1])

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=False)
    robot.wait(20)
    robot.open_gripper(precision=False)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, precision='last')
    robot.set_joint_target_trajectory(q2_path, precision='last')
    robot.close_gripper(precision=True)
    robot.set_joint_target_trajectory(q3_path, precision='last')
    robot.set_joint_target_trajectory(q4_path, precision='last')
    robot.set_joint_target_trajectory(q5_path, precision='last')
    robot.open_gripper(precision=True)
    robot.set_joint_target_trajectory(q5_path[::-1], precision='last')
    # # # back to initial
    # robot.set_joint_target_positions(q0, precision=False)
    # robot.wait(20)


def pallet_application():
    robot, _ = init_simulation_KUKALBR()
    for i in range(0, 6):
        pick_and_place(robot, i)
    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pallet_application()
