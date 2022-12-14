#!/usr/bin/env python
# encoding: utf-8
"""
A series of functions that connect pyARTE and Coppelia.
Robot: Please include here the functions that interface with scenes with the UR5 robot.

Please, beware that the handles in Coppelia have specific names for each scene. In pyARTE, a series of scenes with known
names for each joint have been placed in pyARTE/scenes


@Authors: Arturo Gil
@Time: April 2021

@Authors: Arturo Gil
@Time: April 2021
"""
import sim
from robots.grippers import GripperRG2, GripperBarretHand, SuctionPad
from robots.ur5 import RobotUR5
from sceneconfig.scene_configs import init_sim


def init_simulation_UR5():
    clientID = init_sim()
    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'UR5', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)
    errorCode, gripper_joint1 = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)
    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    gripper = GripperRG2(clientID=clientID, joints=[gripper_joint1])

    robot = RobotUR5(clientID=clientID, wheeljoints=[],
                     armjoints=armjoints, base=robotbase,
                     end_effector=end_effector, gripper=gripper, target=target, camera=camera)
    return robot


def init_simulation_UR5BarrettHand():
    clientID = init_sim()
    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'UR5', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)
    errorCode, gripper_joint1 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint',
                                                        sim.simx_opmode_oneshot_wait)
    errorCode, gripper_joint2 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint0',
                                                        sim.simx_opmode_oneshot_wait)
    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    gripper = GripperBarretHand(clientID=clientID, joints=[gripper_joint1, gripper_joint2])

    robot = RobotUR5(clientID=clientID, wheeljoints=[],
                     armjoints=armjoints, base=robotbase,
                     end_effector=end_effector, gripper=gripper, target=target, camera=camera)
    return robot


