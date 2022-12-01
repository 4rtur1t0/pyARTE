#!/usr/bin/env python
# encoding: utf-8
"""
A series of functions that connect pyARTE and Coppelia.
Robot: Please include here the functions that interface with scenes with the KUKA LBR robot.

Please, beware that the handles in Coppelia have specific names for each scene. In pyARTE, a series of scenes with known
names for each joint have been placed in pyARTE/scenes


@Authors: Arturo Gil
@Time: April 2021
"""
import sim
from robots.grippers import GripperRG2, GripperBarretHand, SuctionPad
from robots.kukalbr import RobotKUKALBR
from artelib.scene import Sphere
import numpy as np
from sceneconfig.scene_configs import init_sim


def init_simulation_KUKALBR():
    clientID = init_sim()

    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint6', sim.simx_opmode_oneshot_wait)
    errorCode, q7 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint7', sim.simx_opmode_oneshot_wait)
    errorCode, gripper_joint1 = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)
    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)
    errorCode, sphere_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    armjoints.append(q7)
    gripper = GripperRG2(clientID=clientID, joints=[gripper_joint1])
    sphere = Sphere(clientID=clientID, handle=sphere_handle,
                    pa=np.array([1.0, -0.45, 0.25]),
                    pb=np.array([0.5, -0.45, 0.25]))
    robot = RobotKUKALBR(clientID=clientID, wheeljoints=[],
                         armjoints=armjoints, base=robotbase,
                         end_effector=end_effector, gripper=gripper,
                         target=target, camera=camera)
    return robot, sphere