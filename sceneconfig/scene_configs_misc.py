#!/usr/bin/env python
# encoding: utf-8
"""
A series of functions that connect pyARTE and Coppelia.
Robot: Please include here the functions that interface with scenes with other robots (miscelanea). Mobile robots.

Please, beware that the handles in Coppelia have specific names for each scene. In pyARTE, a series of scenes with known
names for each joint have been placed in pyARTE/scenes


@Authors: Arturo Gil
@Time: April 2021

@Authors: Arturo Gil
@Time: April 2021
"""
import sim
from robots.planar4dof import Planar4DOF
from robots.robot_dyor import RobotDyor
from sceneconfig.scene_configs import init_sim


def init_simulation_4dof_planar():
    clientID = init_sim()
    armjoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, '4dofplanar', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)
    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'joint4', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    robot = Planar4DOF(clientID=clientID, wheeljoints=[],
                       armjoints=armjoints, base=robotbase,
                       end_effector=end_effector, gripper=None, target=target, camera=None)
    return robot


def init_simulation_mobile_robot():
    clientID = init_sim()
    wheeljoints = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'ROBOT_DYOR', sim.simx_opmode_oneshot_wait)
    errorCode, wheel1 = sim.simxGetObjectHandle(clientID, 'motor_L', sim.simx_opmode_oneshot_wait)
    errorCode, wheel2 = sim.simxGetObjectHandle(clientID, 'motor_R', sim.simx_opmode_oneshot_wait)

    wheeljoints.append(wheel1)
    wheeljoints.append(wheel2)

    robot = RobotDyor(clientID=clientID, wheeljoints=wheeljoints,
                     armjoints=[], base=robotbase,
                     end_effector=None, gripper=None, target=None, camera=None)
    return robot