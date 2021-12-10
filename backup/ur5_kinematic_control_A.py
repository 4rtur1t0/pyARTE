#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import time
import sim
import sys
import numpy as np
from artelib.ur5 import RobotUR5
from artelib.scene import Scene
import matplotlib.pyplot as plt

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


def init_simulation():
    # global pathpositions
    # Python connect to the V-REP client
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    if clientID != -1:
        print("Connected to remote API server")
        # stop previous simiulation
        sim.simxStopSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        time.sleep(3)
        sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
        # enable the synchronous mode
        sim.simxSynchronous(clientID=clientID, enable=True)
        # time.sleep(3)
    else:
        print("Connection not successful")
        sys.exit("Connection failed,program ended!")

    armjoints = []
    gripper = []
    objects = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'UR5', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)

    errorCode, gripper1 = sim.simxGetObjectHandle(clientID, 'MicoHand_fingers12_motor1', sim.simx_opmode_oneshot_wait)
    errorCode, gripper2 = sim.simxGetObjectHandle(clientID, 'MicoHand_fingers12_motor2', sim.simx_opmode_oneshot_wait)

    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)

    # errorCode, sphere = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)

    gripper.append(gripper1)
    gripper.append(gripper2)

    # objects.append(sphere)
    robot = RobotUR5(clientID=clientID, wheeljoints=[],
                        armjoints=armjoints, base=robotbase,
                        end_effector=end_effector, gripper=gripper, target=target)
    scene = Scene(clientID=clientID, objects=objects)
    return robot, scene


def plot_trajectories(q_rs):
    q_rs = np.array(q_rs)
    plt.figure()

    for i in range(0, 6):
        plt.plot(q_rs[:, i], label='q' + str(i + 1))
    plt.legend()
    plt.show(block=True)


def inverse_kinematic_control2(J, vwref):
    """
    Considers a simple joint control to behave properly in the presence of a singularity
    """
    manip = np.linalg.det(np.dot(J, J.T))
    print('Manip is: ', manip)
    # print('Manip v is: ', np.linalg.det(np.dot(Jv, Jv.T)))
    # print('Manip w is: ', np.linalg.det(np.dot(Jw, Jw.T)))
    # normal case --> just compute pseudo inverse
    # we are far from a singularity
    if manip > .01**2:
        # iJ = np.linalg.pinv(J)
        # moore penrose pseudo inverse
        iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        qd = np.dot(iJ, vwref.T)
        return qd
    print('Close to singularity: implementing DAMPED Least squares solution')
    K = 0.01*np.eye(np.min(J.shape))
    iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))
    qd = np.dot(iJ, vwref.T)
    return qd


def adjust_vwref(vwref, error_dist, error_orient):
    # linear speed
    vref = vwref[0:3]
    # angular speed
    wref = vwref[3:6]
    # ACTIVIDAD:
    if error_dist > .1:
        vmag = 0.3
    else:
        vmag = 3 * error_dist + 0.01
    if error_orient > .1:
        wmag = 0.8
    else:
        wmag = 8 * error_orient + 0.01
    vref = vmag * vref
    wref = wmag * wref
    vwref = np.hstack((vref, wref))
    return vwref


def main_loop():
    max_iterations = 1500
    total_error = 0.002
    robot, scene = init_simulation()

    # q = np.array([np.pi/8, np.pi/8, np.pi/8, np.pi/8, np.pi/8, np.pi/8])
    q = np.array([-np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi, np.pi])
    # q = np.dot(0.0, q)
    # set initial position of robot
    robot.set_arm_joint_target_positions(q)
    robot.wait_till_joint_position_is_met(q)

    # position, orientation = robot.get_end_effector_position_orientation()
    # R = euler2Rot(orientation)
    # print('position:', position)
    # print('alphabetagamma: ', orientation)
    # print('Rotation matrix R: ', R)

    # TRY TO REACH DIFFERENT TARGETS
    # ok damped
    # target_position = [0.5, 0.5, 0.5]
    # target_orientation = [0.0, 0.0, 0.0]
    # ok damped
    # target_position = [0.3, 0.3, 0.6]
    # target_orientation = [np.pi/2, 0.0, 0.0]
    # ok damped
    # target_position = [0.3, -0.3, 0.6]
    # target_orientation = [np.pi / 2, 0.0, 0.0]
    # fails with damped
    # target_position = [0.0, 0.0, 0.5]
    # target_orientation = [0.0, 0.0, 0.0]
    # pick
    target_position = [0.4, 0.5, 0.15]
    # target_orientation = [np.pi/2, np.pi/2, np.pi/2]
    target_orientation = [0, np.pi/2, 0]

    # draw current target on Coppelia
    robot.set_target_position_orientation(target_position, target_orientation)


    q_rs = []
    for i in range(0, max_iterations):
        print('Iteration number: ', i)
        vwref, vref, wref = robot.compute_vref_wref(target_position, target_orientation)
        error_dist, error_orient = robot.compute_target_error(target_position, target_orientation)
        vwref = adjust_vwref(vwref=vwref, error_dist=error_dist, error_orient=error_orient)

        print('errordist, error orient: ', error_dist, error_orient)
        if error_dist + error_orient < total_error:
            print('Converged!!')
            break
        # get current coordinates of the arm
        q = robot.get_arm_joint_positions()
        J, Jv, Jw = robot.get_jacobian(q)
        # compute joint speed to achieve the reference
        qd = inverse_kinematic_control2(J, vwref)
        # check joint speed and correct if necessary
        [qd, _, _] = robot.check_speed(qd)
        # integrate movement. Please check that Delta_time matches coppelia simulation time step
        qd = np.dot(DELTA_TIME, qd)
        q = q + qd
        # check joints ranges
        robot.check_joints(q)
        robot.set_arm_joint_target_positions(q)
        robot.wait_till_joint_position_is_met(q)
        q_rs.append(q)

    plot_trajectories(q_rs)
    robot.stop_arm()
    scene.stop_simulation()


if __name__ == "__main__":
    main_loop()
