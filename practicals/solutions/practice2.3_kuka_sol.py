#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.plottools import plot_vars
from artelib.tools import buildT
from sceneconfig.scene_configs import init_simulation_KUKALBR

DELTA_TIME = 50.0/1000.0


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


def inversekinematics2(robot, target_position, target_orientation, q0, vmax=0.5):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = buildT(target_position, target_orientation)
    Ti = robot.direct_kinematics(q0)
    total_time = robot.compute_time(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax)
    total_time = 0.1*total_time
    q = q0
    vrefs = []
    wrefs = []
    max_iterations = 500
    qc = [0, 0, 0, 0, 0, 0, 0]
    K = [0, 0, 0, 0, 0, 1, 0]
    #K = [1, 1, 1, 1, 1, 10, 1]
    q_path = []
    qd_path = []
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax, total_time=total_time)
        vwref = robot.adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient, vmax=vmax)
        vrefs.append(vwref[0:3])
        wrefs.append(vwref[3:6])
        print(Ttarget - Ti)
        print('vwref: ', vwref)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        qda = robot.moore_penrose_damped(J, vwref)

        #
        # CALCULE UNA VELOCIDAD ARTICULAR QUE MINIMICE W
        #
        qdb = minimize_w_central(J, q, qc, K)
        qdb = 0.5 * np.linalg.norm(qda) * qdb
        qd = qda + qdb
        [qd, _, _] = robot.check_speed(qd)
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        q = robot.apply_joint_limits(q)
        q_path.append(q)
        qd_path.append(qd)
    return q_path, qd_path


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
    q0 = np.array([-np.pi / 8, 0, 0, -np.pi / 2, 0, 0, 0])

    # plan trajectories
    [q1_path, _] = inversekinematics2(robot=robot, target_position=target_positions[0],
                                      target_orientation=target_orientations[0], q0=q0)
    [q2_path, _] = inversekinematics2(robot=robot, target_position=target_positions[1],
                                      target_orientation=target_orientations[1], q0=q1_path[-1])
    [q3_path, _] = inversekinematics2(robot=robot, target_position=target_positions[2],
                                      target_orientation=target_orientations[2], q0=q2_path[-1])
    [q4_path, _] = inversekinematics2(robot=robot, target_position=target_positions[3],
                                      target_orientation=target_orientations[3], q0=q3_path[-1])
    [q5_path, _] = inversekinematics2(robot=robot, target_position=target_positions[4],
                                      target_orientation=target_orientations[4], q0=q4_path[-1])

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)
    robot.open_gripper(wait=True)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, wait=False)
    robot.set_joint_target_trajectory(q2_path, wait=False)
    robot.close_gripper(wait=True)
    robot.set_joint_target_trajectory(q3_path, wait=False)
    robot.set_joint_target_trajectory(q4_path, wait=False)
    robot.set_joint_target_trajectory(q5_path, wait=False)
    robot.open_gripper(wait=True)
    robot.set_joint_target_trajectory(q5_path[::-1], wait=False)
    # # # back to initial
    robot.set_joint_target_positions(q0, wait=True)


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
