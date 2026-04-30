#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

    EXERCISE: SOLVE the inverse kinematics problem in a 7DOF redundant robot.
                as a secondary target, minimize the sum of square differences:
                    w = \sum_i (q[i] - qcentral[i])^2

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import moore_penrose_damped
from artelib.euler import Euler
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from artelib.tools import  compute_kinematic_errors
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def diff_w_central(q, qcentral, K):
    dw = []
    for i in range(0, len(qcentral)):
        dwi = K[i]*(q[i]-qcentral[i])
        dw.append(dwi)
    return np.array(dw)


def null_space_projector(J):
    n = J.shape[1]
    P = np.eye(n)-np.dot(np.linalg.pinv(J), J)
    return P


def minimize_w_central(J, q, qc, K):
    qd0 = diff_w_central(q, qc, K)
    qd0 = np.dot(-1.0, qd0)
    P = null_space_projector(J)
    qdb = np.dot(P, qd0)
    norma = np.linalg.norm(qdb)
    if norma > 0.0001:
        return qdb / norma
    else:
        return qdb


def inverse_kinematics(robot, target_position, target_orientation, q0):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    q = q0
    max_iterations = 500
    qc = [0, 0, 0, 0, 0, 0, 0]
    K = [1, 1, 1, 1, 1, 1, 1]
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.directkinematics(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        print('e: ', e)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        J, _, _ = robot.manipulator_jacobian(q)
        # compute joint speed to achieve the reference
        qda = moore_penrose_damped(J, e)
        qdb = minimize_w_central(J, q, qc, K)
        qdb = 0.5 * qdb
        na = np.linalg.norm(qda)
        nb = np.linalg.norm(qdb)
        print('Normas: ', na, nb)
        qd = qda + qdb
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
    return q


def pick(robot, gripper):
    target_positions = [[0.6, -0.1, 0.25],  # initial in front of conveyor
                        [0.6, 0.28, 0.25],  # pick the piece
                        [0.6, 0.0, 0.4]]  # drop the piece on the table
    target_orientations = [[-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2],
                           [-np.pi / 2, 0, -np.pi / 2]]
    close_gripper = [False, True, True]
    # initial arm position
    q0 = np.array([0, 0, 0, -np.pi/2, 0, np.pi/4, 0])
    robot.moveAbsJ(q_target=q0, precision=False)

    for i in range(len(target_positions)):
        q = inverse_kinematics(robot=robot,
                               target_position=target_positions[i],
                               target_orientation=Euler(target_orientations[i]), q0=q0)
        robot.moveAbsJ(q_target=q, precision=False)
        q0 = q
        if close_gripper[i]:
            gripper.close(precision=True)
        else:
            gripper.open(precision=True)


def place(robot, gripper, step_number):
    target_positions = [[0.2, -0.7, 0.4],  # over the table
                        [0.2, -0.7, 0.35]]  # drop the piece on the table
    target_orientations = [[-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    close_gripper = [True, False, False]
    # change target points for drop zone
    target_positions[0] = target_positions[0] + step_number * np.array([0, 0.06, 0])
    target_positions[1] = target_positions[1] + step_number * np.array([0, 0.06, 0])

    q0 = np.array([-np.pi/2, 0, 0, -np.pi / 2, 0, np.pi / 4, 0])
    robot.moveAbsJ(q_target=q0, precision=False)

    for i in range(len(target_positions)):
        q = inverse_kinematics(robot=robot,
                               target_position=target_positions[i],
                               target_orientation=Euler(target_orientations[i]), q0=q0)
        robot.moveAbsJ(q_target=q, precision=False)
        q0 = q
        if close_gripper[i]:
            gripper.close(precision=True)
        else:
            gripper.open(precision=True)


def pick_place():
    simulation = Simulation()
    simulation.start()
    robot = RobotKUKALBR(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start(name='/LBR_iiwa_14_R820/RG2/RG2_openCloseJoint')
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    for i in range(0, 6):
        pick(robot, gripper)
        place(robot, gripper, i)

    robot.plot_trajectories()
    simulation.stop()


if __name__ == "__main__":
    pick_place()

