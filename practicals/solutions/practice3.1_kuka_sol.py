#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.
The demo represents a KUKA LBR IIWA robot trying to avoid collisions with a sphere.

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


def compute_repulsion(pe, ps):
    K = 0.01
    u = pe - ps
    r = np.linalg.norm(u)
    if r < 0.1:
        r = 0.1
    elif r > 0.5:
        K = 0.0
    u = u / r
    vrep = np.dot(K/r**2, u)
    vrep = np.hstack((vrep, np.array([0, 0, 0])))
    return vrep


def inversekinematics3(robot, sphere, target_position, target_orientation, q0, vmax=1.0):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = buildT(target_position, target_orientation)
    q = q0
    max_iterations = 500
    qc = [0, 0, 0, 0, 0, 0, 0]
    K = [0, 0, 0, 0, 0, 1, 0]
    q_path = []
    qd_path = []
    sphere_positions = []
    nvreps = []
    ndist = []
    for i in range(0, max_iterations):
        # move sphere to next position (randomly)
        sphere.next_position()
        ps = sphere.get_position()
        sphere_positions.append(ps)
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        pe = Ti[0:3, 3]
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax)
        vrep=compute_repulsion(pe=pe, ps=sphere.get_position())
        nvreps.append([np.linalg.norm(vrep)])
        ndist.append([np.linalg.norm(pe-ps)])
        print(Ttarget - Ti)
        vwref = vwref + vrep
        vwref = robot.adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient, vmax=vmax)
        print('vwref: ', vwref)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        qda = robot.moore_penrose_damped(J, vwref)
        qdb = minimize_w_central(J, q, qc, K)
        qdb = 0.5 * np.linalg.norm(qda) * qdb
        qd = qda + qdb
        [qd, _, _] = robot.check_speed(qd)
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        q = robot.apply_joint_limits(q)
        q_path.append(q)
        qd_path.append(qd)
    #plot_vars(nvreps, title='norma vrep')
    #plot_vars(ndist, title='distancia')
    return q_path, qd_path, sphere_positions


def plot_robot_and_sphere(robot, sphere, q_path, sphere_positions):
    for i in range(0, len(q_path)):
        robot.set_joint_target_positions(q_path[i], wait=False)
        sphere.set_object_position(position=sphere_positions[i])
    return


def pick_and_place(robot, sphere, step_number):
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
    [q1_path, _, sp1] = inversekinematics3(robot=robot, sphere=sphere, target_position=target_positions[0],
                                      target_orientation=target_orientations[0], q0=q0)
    [q2_path, _, sp2] = inversekinematics3(robot=robot, sphere=sphere, target_position=target_positions[1],
                                      target_orientation=target_orientations[1], q0=q1_path[-1])
    [q3_path, _, sp3] = inversekinematics3(robot=robot, sphere=sphere, target_position=target_positions[2],
                                      target_orientation=target_orientations[2], q0=q2_path[-1])
    [q4_path, _, sp4] = inversekinematics3(robot=robot, sphere=sphere, target_position=target_positions[3],
                                      target_orientation=target_orientations[3], q0=q3_path[-1])
    [q5_path, _, sp5] = inversekinematics3(robot=robot, sphere=sphere, target_position=target_positions[4],
                                      target_orientation=target_orientations[4], q0=q4_path[-1])

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)
    robot.open_gripper(wait=True)

    plot_robot_and_sphere(robot=robot, sphere=sphere, q_path=q1_path, sphere_positions=sp1)
    plot_robot_and_sphere(robot=robot, sphere=sphere, q_path=q2_path, sphere_positions=sp2)
    robot.close_gripper(wait=True)
    plot_robot_and_sphere(robot=robot, sphere=sphere, q_path=q3_path, sphere_positions=sp3)
    plot_robot_and_sphere(robot=robot, sphere=sphere, q_path=q4_path, sphere_positions=sp4)
    plot_robot_and_sphere(robot=robot, sphere=sphere, q_path=q5_path, sphere_positions=sp5)
    robot.open_gripper(wait=True)
    robot.set_joint_target_trajectory(q5_path[::-1], wait=False)
    # # # # back to initial
    robot.set_joint_target_positions(q0, wait=True)
    # set the target we are willing to reach on Coppelia
    # robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    # robot.set_joint_target_trajectory(q1_path, wait=False)
    # robot.set_joint_target_trajectory(q2_path, wait=False)
    #
    # robot.set_joint_target_trajectory(q3_path, wait=False)
    # robot.set_joint_target_trajectory(q4_path, wait=False)
    # robot.set_joint_target_trajectory(q5_path, wait=False)
    #
    #


def pallet_application():
    robot, sphere = init_simulation_KUKALBR()

    for i in range(0, 6):
        pick_and_place(robot, sphere, i)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pallet_application()
