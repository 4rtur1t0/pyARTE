#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before using this class.

RobotUR5 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021

"""
import sim
import numpy as np
from artelib.robot import Robot
from kinematics.kinematics_ur5 import eval_symbolic_jacobian_UR5, eval_symbolic_T_UR5


class RobotUR5(Robot):
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target):
        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180])
        max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-360, -360, -360, -360, -360, -360],
                                 [360,   360,  360,  360,  360,  360]])
        joint_ranges = joint_ranges * np.pi / 180.0

        Robot.__init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target,
                       max_joint_speeds=max_joint_speeds, joint_ranges=joint_ranges)

    def open_gripper(self, wait=False):
        # sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.gripper[0],
        #                                 targetVelocity=0.02, operationMode=sim.simx_opmode_oneshot)
        # sim.simxSetJointMaxForce(clientID=self.clientID, jointHandle=self.gripper[0],
        #                                 force=1.0, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
                                       targetPosition=0.05, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointMaxForce(clientID=self.clientID, jointHandle=self.gripper[0],
                                 force=1.0, operationMode=sim.simx_opmode_oneshot)
        # sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[1],
        #                                targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)
        if wait:
            self.wait(10)

    def close_gripper(self, wait=False):
        # sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.gripper[0],
        #                                targetVelocity=-0.02, operationMode=sim.simx_opmode_oneshot)
        # sim.simxSetJointMaxForce(clientID=self.clientID, jointHandle=self.gripper[0],
        #                          force=-1.0, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.gripper[0],
                                        targetPosition=-0.05, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointMaxForce(clientID=self.clientID, jointHandle=self.gripper[0],
                                 force=-1.0, operationMode=sim.simx_opmode_oneshot)
        if wait:
            self.wait(10)

    def get_jacobian(self, q):
        J, Jv, Jw = eval_symbolic_jacobian_UR5(q)
        return J, Jv, Jw

    def direct_kinematics(self, q):
        T = eval_symbolic_T_UR5(q)
        return T
