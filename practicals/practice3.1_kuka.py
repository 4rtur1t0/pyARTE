#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820_1.ttt scene before running this script.
The demo represents a KUKA LBR IIWA robot trying to avoid collisions with a sphere.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.plottools import plot_vars, plot_xy
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


def potential0(r):
    K = 1.0
    p = K * (1 / r)
    return p


def potential(r):
    # EJERCICIO, calcular el módulo de la función de repulsión

    return p


def compute_repulsion(pe, ps):
    u = pe - ps
    r = np.linalg.norm(u)
    if r > 0.0:
        u = u / r

    # EJERCICIO: Calcular la velocidad de repulsíón considerando el vector
    # unitario u

    # EJERCICIO: note que la repulsión no implica un cambio en la orientación,
    # con lo que wref=(0,0,0)
    vrep = np.hstack((vrep, np.array([0, 0, 0])))
    return vrep


def inversekinematics3(robot, sphere, target_position, target_orientation, q0, vmax=0.5):
    """
    fine: whether to reach the target point with precision or not.
    vmax: linear velocity of the planner.
    """
    Ttarget = buildT(target_position, target_orientation)
    q = q0
    max_iterations = 1500
    qc = [0, 0, 0, 0, 0, 0, 0]
    K = [0, 1, 0, 0, 0, 0, 0]
    q_path = []
    qd_path = []
    ps = sphere.get_position()
    Ti = robot.direct_kinematics(q)
    total_time = robot.compute_time(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax)
    total_time = 0.2*total_time
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        pe = Ti[0:3, 3]
        # compute ATTRACTION
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax,
                                                                total_time=total_time)
        # compute REPULSION
        vrep = compute_repulsion(pe=pe, ps=ps)
        vwref = vwref + vrep
        vwref = robot.adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient, vmax=vmax)
        if error_dist < robot.max_error_dist_inversekinematics and \
                error_orient < robot.max_error_orient_inversekinematics:
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
        [q, _] = robot.apply_joint_limits(q)
        q_path.append(q)
        qd_path.append(qd)
    return q_path, qd_path


def plot_robot_and_sphere(robot, sphere, q_path, sphere_positions):
    for i in range(0, len(q_path)):
        robot.set_joint_target_positions(q_path[i], wait=False)
        sphere.set_object_position(position=sphere_positions[i])
    return


def follow_line_obstacle(robot, sphere):
    target_positions = [[0.5, 0.4, 0.5],  # initial in front of conveyor
                        [0.5, -0.4, 0.5]]  # drop the piece on the table
    target_orientations = [[0, np.pi/8, 0],
                           [0, np.pi/8, 0]]

    # initial arm position
    q0 = np.array([-np.pi / 8, 0, 0, -np.pi / 2, 0, 0, 0])

    # necesita cambiar la posición central
    sphere.set_object_position([0.55, 0.0, 0.45])
    #sphere.set_object_position([0.5, 0.0, 0.5])
    #sphere.set_object_position([0.5, 0.0, 0.4])
    #sphere.set_object_position([0.35, 0.0, 0.4])

    # plan trajectories
    [q1_path, _] = inversekinematics3(robot=robot, sphere=sphere, target_position=target_positions[0],
                                      target_orientation=target_orientations[0], q0=q0)
    [q2_path, _] = inversekinematics3(robot=robot, sphere=sphere, target_position=target_positions[1],
                                      target_orientation=target_orientations[1], q0=q1_path[-1])

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=False)
    robot.wait(15)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, precision='last')
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_trajectory(q2_path, precision='last')
    robot.wait(15)


if __name__ == "__main__":
    robot, sphere = init_simulation_KUKALBR()
    follow_line_obstacle(robot, sphere)
    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()
