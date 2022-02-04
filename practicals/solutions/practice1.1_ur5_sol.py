#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.orientation import Euler
from artelib.tools import buildT, compute_kinematic_errors
from sceneconfig.scene_configs import init_simulation_UR5

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


def moore_penrose_damped(J, vwref):
    """
    Compute qd given J and v.
    If close to singularity, used damped version.
    """
    manip = np.linalg.det(np.dot(J, J.T))
    print('Manip is: ', manip)
    # normal case --> just compute pseudo inverse
    # we are far from a singularity
    if manip > .01 ** 2:
        # moore penrose pseudo inverse J^T(J*J^T)^{-1}
        iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        qd = np.dot(iJ, vwref.T)
        return qd
    print('Close to singularity: implementing DAMPED Least squares solution')
    K = 0.01 * np.eye(np.min(J.shape))
    iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))
    qd = np.dot(iJ, vwref.T)
    return qd


def inverse_kinematics(robot, target_position, target_orientation, q0):
    """
    Find q that allows the robot to achieve the specified target position and orientaiton
    CAUTION: target_orientation must be specified as a quaternion.
    """
    Ttarget = buildT(target_position, target_orientation)
    q = q0
    max_iterations = 10000
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        J, Jv, Jw = robot.get_jacobian(q)
        e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
        print('vwref: ', e)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        # compute delta q for the step
        qd = moore_penrose_damped(J, e)
        q = q + qd
        [q, _] = robot.apply_joint_limits(q)
    return q


def pick_and_place():
    robot = init_simulation_UR5()
    target_positions = [[0.6, -0.2, 0.25], # initial in front of conveyor
                        [0.6, 0.1, 0.25], # pick the piece
                        [0.6, 0.1, 0.35], # bring the piece up
                        [0.4, -0.1, 0.35], # middle point
                        [0.2, -0.55, 0.4], # over the table
                        [0.2, -0.55, 0.3]] # drop the piece
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi / 2, 0, 0],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    q0 = np.array([-np.pi, -np.pi/8, np.pi/2, 0, 0, 0])

    # plan trajectories
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=Euler(target_orientations[0]), q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=Euler(target_orientations[1]), q0=q1)
    q3 = inverse_kinematics(robot=robot, target_position=target_positions[2],
                            target_orientation=Euler(target_orientations[2]), q0=q2)
    q4 = inverse_kinematics(robot=robot, target_position=target_positions[3],
                            target_orientation=Euler(target_orientations[3]), q0=q3)
    q5 = inverse_kinematics(robot=robot, target_position=target_positions[4],
                            target_orientation=Euler(target_orientations[4]), q0=q4)
    q6 = inverse_kinematics(robot=robot, target_position=target_positions[5],
                            target_orientation=Euler(target_orientations[5]), q0=q5)

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    robot.wait(15)
    robot.open_gripper(precision=True)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1, precision=True)
    robot.set_joint_target_positions(q2, precision=True)
    robot.close_gripper(precision=True)
    robot.set_joint_target_positions(q3, precision=True)
    robot.set_joint_target_positions(q4, precision=True)
    robot.set_joint_target_positions(q5, precision=True)
    robot.set_joint_target_positions(q6, precision=True)
    robot.open_gripper(precision=True)
    robot.set_joint_target_positions(q5)

    # Stop arm and simulation
    robot.stop_arm()
    robot.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place()

