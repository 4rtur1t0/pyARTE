#!/usr/bin/env python
# encoding: utf-8
"""
A series of functions that connect pyARTE and Coppelia.
Robot: Please include here the functions that interface with scenes with the IRB140 robot.

Please, beware that the handles in Coppelia have specific names for each scene. In pyARTE, a series of scenes with known
names for each joint have been placed in pyARTE/scenes

@Authors: Arturo Gil
@Time: April 2021
"""
import sim
from sceneconfig.scene_configs import init_sim
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2, GripperBarretHand, SuctionPad
from robots.proxsensor import ProxSensor


def init_simulation_ABBIRB140():
    clientID = init_sim()

    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'IRB140', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'joint6', sim.simx_opmode_oneshot_wait)
    # get the handle of the Barret Hand
    errorCode, gripper_joint1 = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)

    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)

    errorCode, conveyor_sensor = sim.simxGetObjectHandle(clientID, 'conveyor__sensor', sim.simx_opmode_oneshot_wait)
    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    gripper = GripperRG2(clientID=clientID, joints=[gripper_joint1])

    robot = RobotABBIRB140(clientID=clientID, wheeljoints=[],
                           armjoints=armjoints, base=robotbase,
                           end_effector=end_effector, gripper=gripper,
                           target=target, camera=camera)

    conveyor_sensor = ProxSensor(clientID, conveyor_sensor)

    return robot, conveyor_sensor


def init_simulation_ABBIRB140_BarretHand():
    clientID = init_sim()

    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'IRB140', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'joint6', sim.simx_opmode_oneshot_wait)

    errorCode, gripper_joint1 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint',
                                                        sim.simx_opmode_oneshot_wait)
    errorCode, gripper_joint2 = sim.simxGetObjectHandle(clientID, 'Barrett_openCloseJoint0',
                                                        sim.simx_opmode_oneshot_wait)

    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)

    errorCode, conveyor_sensor = sim.simxGetObjectHandle(clientID, 'conveyor__sensor', sim.simx_opmode_oneshot_wait)
    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    gripper = GripperBarretHand(clientID=clientID, joints=[gripper_joint1, gripper_joint2])

    robot = RobotABBIRB140(clientID=clientID, wheeljoints=[],
                           armjoints=armjoints, base=robotbase,
                           end_effector=end_effector, gripper=gripper,
                           target=target, camera=camera)

    conveyor_sensor = ProxSensor(clientID, conveyor_sensor)

    return robot, conveyor_sensor


def init_simulation_ABBIRB140_suction_pad():
    clientID = init_sim()

    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'IRB140', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'joint6', sim.simx_opmode_oneshot_wait)

    # errorCode, gripper_joint1 = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)

    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)

    errorCode, conveyor_sensor = sim.simxGetObjectHandle(clientID, 'conveyor__sensor', sim.simx_opmode_oneshot_wait)
    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    # gripper = GripperRG2(clientID=clientID, joints=[gripper_joint1])
    gripper = SuctionPad(clientID=clientID, joints=None)

    robot = RobotABBIRB140(clientID=clientID, wheeljoints=[],
                           armjoints=armjoints, base=robotbase,
                           end_effector=end_effector, gripper=gripper,
                           target=target, camera=camera)

    conveyor_sensor = ProxSensor(clientID, conveyor_sensor)

    return robot, conveyor_sensor
