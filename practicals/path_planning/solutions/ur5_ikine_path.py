#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import slerp, compute_kinematic_errors
from robots.grippers import GripperRG2
from robots.simulation import Simulation
from robots.ur5 import RobotUR5

DELTA_TIME = 50.0/1000.0


def n_movements(p_current, p_target, vmax):
    total_time = np.linalg.norm(np.array(p_target) - np.array(p_current)) / vmax
    n = total_time / DELTA_TIME
    n = np.ceil(n)
    return int(n)


def path_planning_p(p_current, p_target, n):
    tt = np.linspace(0, 1, int(n))
    target_positions = []
    p_current = np.array(p_current)
    p_target = np.array(p_target)
    for t in tt:
        target_pos = t * p_target + (1 - t) * p_current
        target_positions.append(target_pos)
    return target_positions


def path_planning_o(o_current, o_target, n):
    """
    Generate a set of n quaternions between Q1 and Q2. Use SLERP to find an interpolation between them.
    """
    Q1 = o_current.Q()
    Q2 = o_target.Q()
    tt = np.linspace(0, 1, int(n))
    target_orientations = []
    for t in tt:
        Q = slerp(Q1, Q2, t)
        target_orientations.append(Q)
    return target_orientations


def moore_penrose_damped(J, e):
    """
    Compute qd given J and v.
    If close to singularity, used damped version.
    """
    # abs values during singular and noisy configurations
    manip = np.abs(np.linalg.det(np.dot(J, J.T)))
    # print('Manip is: ', manip)
    # normal case --> just compute pseudo inverse if we are far from a singularity
    if manip > .01 ** 2:
        # print('Standard solution')
        # moore penrose pseudo inverse J^T(J*J^T)^{-1}
        iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        qd = 0.5*np.dot(iJ, e.T)
        return qd
    print('Close to singularity: implementing DAMPED Least squares solution')
    K = 0.01 * np.eye(np.min(J.shape))
    iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))
    qd = np.dot(iJ, e.T)
    return qd


def delta_q_transpose(J, e):
    alpha1 = np.dot(np.dot(np.dot(J, J.T), e), e)
    alpha2 = np.dot(np.dot(J, J.T), e)
    alpha2 = np.dot(alpha2, alpha2)
    alpha = alpha1/alpha2
    dq = alpha*np.dot(J.T, e)
    return dq


def inverse_kinematics(robot, target_position, target_orientation, q0):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion.
    """
    Ttarget = HomogeneousMatrix(target_position, target_orientation)
    q = q0
    max_iterations = 10000
    errs = []
    for i in range(max_iterations):
        # print('Iteration number: ', i)
        Ti = robot.directkinematics(q)
        J, Jv, Jw = robot.manipulator_jacobian(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        errs.append(error_dist)
        # print('vwref: ', e)
        # print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged in ', i, ' iterations!')
            print(errs)
            break
        # compute delta q for the step
        # EXERCISE: TEST THE THREE METHODS
        qd = moore_penrose_damped(J, e)
        # qd = delta_q_transpose(J, e)
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
    return q


def inversekinematics_path(robot, target_positions, target_orientations, q0):
    """
    Solve iteratively q for each of the target positions and orientation specified
    """
    q_path = []
    q = q0
    # now try to reach each target position on the line
    for i in range(len(target_positions)):
        q = inverse_kinematics(robot=robot, target_position=target_positions[i],
                               target_orientation=target_orientations[i], q0=q)
        q_path.append(q)
    return q_path


def pick_and_place():
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start(name='/UR5/RG2/RG2_openCloseJoint')
    target_positions = [[0.6, -0.3, 0.4],
                        [0.6, -0.2, 0.25], # initial in front of conveyor
                        [0.6, 0.1, 0.25], # pick the piece
                        [0.6, -0.1, 0.35], # bring the piece up
                        [0.4, -0.1, 0.35], # middle point
                        [0.2, -0.55, 0.4], # over the table
                        [0.2, -0.55, 0.3],
                        [0.2, -0.55, 0.4]] # drop the piece
    target_orientations = [[-np.pi/2, 0, 0],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi / 2, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    open_gripper = [False,
                    True,  # initial in front of conveyor
                    False,  # pick the piece
                    False,  # bring the piece up
                    False,  # middle point
                    False,  # over the table
                    True,
                    True]  # drop the piece
    q = np.array([-np.pi / 2, -np.pi / 8, np.pi / 2, -0.1, -0.1, -0.1])
    robot.moveAbsJ(q)
    robot.wait(20)
    for i in range(len(target_positions)-1):
        # robot.set_target_position_orientation(target_positions[i+1], target_orientations[i+1])
        n = n_movements(target_positions[i], target_positions[i+1], vmax=0.5)
        path_p = path_planning_p(target_positions[i], target_positions[i+1], n)
        path_o = path_planning_o(Euler(target_orientations[i]), Euler(target_orientations[i+1]), n)
        # you may also use the integrated function in the robot class
        # q_path = robot.inversekinematics_path(robot, path_p, path_o, q)
        q_path = inversekinematics_path(robot, path_p, path_o, q)
        if open_gripper[i]:
            gripper.open(precision=True)
        else:
            gripper.close(precision=True)
        # move the robot!
        for i in range(len(q_path)):
            robot.moveAbsJ(q_target=q_path[i], qdfactor=0.8, precision=False, endpoint=False)
        q = q_path[-1]

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place()

