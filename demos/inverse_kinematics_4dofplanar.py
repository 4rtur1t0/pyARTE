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
from artelib.plottools import plot_vars, plot
from artelib.tools import buildT, compute_w_between_R
from sceneconfig.scene_configs_misc import init_simulation_4dof_planar

DELTA_TIME = 50.0/1000.0


def compute_errors(Tcurrent, Ttarget):
    """
    Compute the error
    """
    # current position of the end effector and target position
    p_current = Tcurrent[0:3, 3]
    p_target = Ttarget[0:3, 3]
    e1 = np.array(p_target - p_current)
    error_dist = np.linalg.norm(e1)
    e2 = compute_w_between_R(Tcurrent, Ttarget, total_time=1)
    error_orient = np.linalg.norm(e2)
    e = np.hstack((e1, e2))
    return e, error_dist, error_orient


def delta_q_transpose(J, e):
    alpha1 = np.dot(np.dot(np.dot(J, J.T), e), e)
    alpha2 = np.dot(np.dot(J, J.T), e)
    alpha2 = np.dot(alpha2, alpha2)
    alpha = alpha1/alpha2
    dq = alpha*np.dot(J.T, e)
    return dq


def inversekinematics_transpose(robot, target_position, target_orientation, q0):
    """
    """
    Ttarget = buildT(target_position, target_orientation)
    q = q0
    max_iterations = 3000
    error_dists = []
    error_orients = []
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        e, error_dist, error_orient = compute_errors(Tcurrent=Ti, Ttarget=Ttarget)
        error_dists.append(np.linalg.norm(error_dist))
        error_orients.append(np.linalg.norm(error_orient))
        print('e: ', e)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.001 and error_orient < 0.001:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # adapt dimensions
        J = np.vstack((J[0:2], [J[-1]]))
        e = np.array([e[0], e[1], e[-1]])
        qd = delta_q_transpose(J, e)
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
    # plot(error_dists)
    # plot(error_orients)
    return q



def inversekinematics_moore_penrose(robot, target_position, target_orientation, q0, vmax=0.5):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = buildT(target_position, target_orientation)
    Ti = robot.direct_kinematics(q0)
    total_time = robot.compute_time(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax)
    total_time = 0.1*total_time
    q = q0
    max_iterations = 3000
    q_path = []
    qd_path = []
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax, total_time=total_time)
        vwref = robot.adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient, vmax=vmax)

        print(Ttarget - Ti)
        print('vwref: ', vwref)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # adapt dimensions
        J = np.vstack((J[0:2], [J[-1]]))
        vwref = np.array([vwref[0], vwref[1], vwref[-1]])
        # compute joint speed to achieve the reference
        qd = robot.moore_penrose_damped(J, vwref)
        [qd, _, _] = robot.check_speed(qd)
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
        q_path.append(q)
        qd_path.append(qd)
    return q_path, qd_path


def inverse_kin_techniques(robot):
    # target_positions = [[0.5, -0.2, 0.0]]  # drop the piece on the table
    target_positions = [[1.0, 0.0, 0.0]]  # drop the piece on the table
    target_orientations = [[0, 0, 0]]

    # initial arm position
    q0 = np.array([-np.pi, np.pi/2, -np.pi/2, np.pi/8])

    robot.set_target_position_orientation(target_positions[0], target_orientations[0])

    # eval number of steps to converge
    # eval smoothness
    q1 = inversekinematics_transpose(robot=robot, target_position=target_positions[0],
                                     target_orientation=target_orientations[0], q0=q0)

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    robot.set_joint_target_positions(q1, precision=True)
    robot.wait(20)


if __name__ == "__main__":
    robot = init_simulation_4dof_planar()

    inverse_kin_techniques(robot)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()
