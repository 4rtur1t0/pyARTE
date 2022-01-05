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
    """
    Ttarget = buildT(target_position, target_orientation)
    q = q0
    max_iterations = 1500
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        Ti = robot.direct_kinematics(q)
        J, Jv, Jw = robot.get_jacobian(q)
        vwref, error_dist, error_orient = robot.compute_actions(Tcurrent=Ti, Ttarget=Ttarget, vmax=1)
        print('vwref: ', vwref)
        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist < 0.01 and error_orient < 0.01:
            print('Converged!!')
            break
        # compute joint speed to achieve the reference
        qd = moore_penrose_damped(J, vwref)
        # integrate movement. Please check that Delta_time matches coppelia simulation time step
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
    return q


def pick_and_place():
    robot, scene = init_simulation_UR5()
    target_positions = [[0.6, -0.2, 0.25], # initial in front of conveyor
                        [0.6, 0.1, 0.25], # pick the piece
                        [0.6, 0.1, 0.35], # bring the piece up
                        [0.2, -0.6, 0.4], # over the table
                        [0.2, -0.6, 0.3]] # drop the piece
    target_orientations = [[-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi/2, 0, -np.pi/2],
                           [-np.pi, 0, 0],
                           [-np.pi, 0, 0]]
    q0 = np.array([-np.pi, 0, np.pi/2, 0, 0, 0])

    # plan trajectories
    q1 = inverse_kinematics(robot=robot, target_position=target_positions[0],
                            target_orientation=target_orientations[0], q0=q0)
    q2 = inverse_kinematics(robot=robot, target_position=target_positions[1],
                            target_orientation=target_orientations[1], q0=q1)
    q3 = inverse_kinematics(robot=robot, target_position=target_positions[2],
                            target_orientation=target_orientations[2], q0=q2)
    q4 = inverse_kinematics(robot=robot, target_position=target_positions[3],
                            target_orientation=target_orientations[3], q0=q3)
    q5 = inverse_kinematics(robot=robot, target_position=target_positions[4],
                            target_orientation=target_orientations[4], q0=q4)

    # NOW execute trajectories computed before.
    # set initial position of robot
    robot.set_joint_target_positions(q0, wait=True)
    robot.open_gripper(wait=True)
    # set the target we are willing to reach on Coppelia
    robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    robot.set_joint_target_positions(q1, wait=True)
    robot.set_joint_target_positions(q2, wait=True)
    robot.close_gripper(wait=True)
    robot.set_joint_target_positions(q3, wait=True)
    robot.set_joint_target_positions(q4, wait=True)
    robot.set_joint_target_positions(q5, wait=True)
    robot.open_gripper(wait=True)
    robot.set_joint_target_positions(q4)

    [image, resolution] = robot.get_image()
    robot.save_image(image=image, resolution=resolution, filename='test.png')

    # Stop arm and simulation
    robot.stop_arm()
    scene.stop_simulation()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place()

