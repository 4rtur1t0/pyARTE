#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.tools import buildT
from sceneconfig.scene_configs import init_simulation_UR5

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


def check_joints(robot, q):
    """
    Check that each joint is within range.
    Returns True if all joints are within range
    Returns False if not.
    Finally, an array with the valid indexes are returned
    """
    valid = True
    valid_indexes = []
    for i in range(0, len(q)):
        # greater than min and lower than max
        if (robot.joint_ranges[0, i] < q[i]) and (robot.joint_ranges[1, i] > q[i]):
            valid_indexes.append(True)
            continue
        else:
            print(30 * '*')
            print('JOINT ERROR: RANGE ERROR! Joint: q', i + 1, ' is out of range')
            print(30 * '*')
            valid = False
            valid_indexes.append(False)
    return valid, valid_indexes


def check_speed(robot, qd):
    """
    Checks that all joints speeds are within its limits.
    In addition, a corrected qd is returned that scales down the whole qd vector by a common constant.
    Please take into account that if qd is close to inf values, the returned vector will not meet any kinematic
    constrain.
    """
    # check that the array is finite
    check_nan = np.isnan(qd).any()
    check_inf = np.isinf(qd).any()
    if check_nan or check_inf:
        print(30 * '*')
        print('JOINT ERROR: SPEED IS INF OR NAN!')
        print('Setting speed to zero')
        print(30 * '*')
        return np.zeros(len(qd)), False, False
    print('Joint speed norm: ', np.linalg.norm(qd))
    #######################################################################################
    # EJERCICIO
    # La velocidad máxima de cada articulación está almacenada en robot.max_joint_speeds[i]
    # compruebe la velocidad comandada qd[i] con esta velocidad.
    # si la velocidad es menor --> OK
    # si la velocidad es mayor, debería escalarse el vector qd entero.
    #######################################################################################


    return qd_corrected


def moore_penrose_damped(J, vwref):
    """
    Considers a simple joint control to behave properly in the presence of a singularity
    """
    manip = np.linalg.det(np.dot(J, J.T))
    print('Manip is: ', manip)
    # we are far from a singularity
    if manip > .01 ** 2:
        # iJ = np.linalg.pinv(J)
        # moore penrose pseudo inverse J^T(J*J^T)^{-1}
        iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        qd = np.dot(iJ, vwref.T)
        return qd
    print('Close to singularity: implementing DAMPED Least squares solution')
    K = 0.01 * np.eye(np.min(J.shape))
    iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))
    qd = np.dot(iJ, vwref.T)
    return qd


def inverse_kinematics_line(robot, target_position, target_orientation, q0, vmax=1.0):
    """
    vmax: linear velocity of the planner.
    """
    max_iterations = 1500
    Ttarget = buildT(target_position, target_orientation)
    q_path = []
    qd_path = []
    q = q0
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=vmax)
        print(Ttarget - Ti)
        print('vwref: ', vwref)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        qd = moore_penrose_damped(J, vwref)
        # check joint speed and correct if necessary
        [qd, _, _] = check_speed(robot, qd)
        # integrate movement. Please check that Delta_time matches coppelia simulation time step
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        # check joints ranges
        check_joints(robot, q)
        ##################################################################################
        # EJERCICIO, guarde q y qd en q_path y qd_path
        #################################################################################
    return q_path, qd_path


def move_to_target():
    robot, scene = init_simulation_UR5()
    ################################################################
    # EJERCICIO:
    # A) COMPLETE LA FUNCIÓN inverse_kinematics_line
    ###############################################################
    # initial joint position q0
    q0 = np.array([-np.pi, 0.1, np.pi/2, 0.1, 0.1, 0.1])
    # a list of target positions
    target_positions = [[0.6, -0.2, 0.25],
                        [0.6, 0.1, 0.25]]
    # a list of target orientations (Euler angles alpha, beta, gamma in rad)
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2]]

    vmax = 0.5 # m/s

    # Compute inverse kinematics for each targe position/orientation
    [q1_path, _] = inverse_kinematics_line(robot=robot, target_position=target_positions[0],
                                           target_orientation=target_orientations[0], q0=q0, vmax=vmax)
    [q2_path, _] = inverse_kinematics_line(robot=robot, target_position=target_positions[1],
                                           target_orientation=target_orientations[1], q0=q1_path[-1], vmax=vmax)

    # NOW! execute trajectories on the simulated robot
    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)
    robot.open_gripper()
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_trajectory(q1_path, wait=False)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[1], target_orientations[1])
    robot.set_joint_target_trajectory(q2_path, wait=False)
    robot.set_joint_target_positions(q2_path[-1], wait=True)
    robot.close_gripper(wait=True)

    # stop the arm
    robot.stop_arm()
    # stop simulation
    scene.stop_simulation()
    # plot joints for each movement
    robot.plot_trajectories()


if __name__ == "__main__":
    move_to_target()

