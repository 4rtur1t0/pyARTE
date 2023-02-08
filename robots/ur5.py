#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before using this class.

RobotUR5 is a derived class of the Robot base class

@Authors: Arturo Gil
@Time: April 2021
"""
import sim
import numpy as np
from artelib import homogeneousmatrix
from artelib.seriallink import SerialRobot
from robots.robot import Robot
from kinematics.kinematics_ur5 import eval_symbolic_jacobian_UR5


class RobotUR5(Robot):
    def __init__(self, clientID):
        # init base class attributes
        Robot.__init__(self)
        self.clientID = clientID

        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-360, -360, -360, -360, -360, -360],
                                 [360,   360,  360,  360,  360,  360]])
        self.joint_ranges = joint_ranges * np.pi / 180.0
        self.max_iterations_inverse_kinematics = 1500
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        # self.ikmethod = 'transpose'
        self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True
        self.epsilonq = 0.005

        self.serialrobot = SerialRobot(n=6, T0=np.eye(4), TCP=np.eye(4), name='UR5')
        self.serialrobot.append(th=-np.pi/2, d=0.089159, a=0, alpha=np.pi/2)
        self.serialrobot.append(th=+np.pi/2, d=0,        a=0.425, alpha=0)
        self.serialrobot.append(th=0,        d=0,        a=0.39225, alpha=0)
        self.serialrobot.append(th=-np.pi/2, d=0.10915,  a=0, alpha=-np.pi/2)
        self.serialrobot.append(th=0,        d=0.09465,  a=0, alpha=np.pi/2)
        self.serialrobot.append(th=0,        d=0.0823,   a=0, alpha=0)

    def start(self, base_name='/UR5', joint_name='UR5_joint'):
        errorCode, robotbase = sim.simxGetObjectHandle(self.clientID, base_name, sim.simx_opmode_oneshot_wait)
        errorCode, q1 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '1',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q2 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '2',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q3 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '3',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q4 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '4',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q5 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '5',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q6 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '6',
                                                sim.simx_opmode_oneshot_wait)

        joints = []
        joints.append(q1)
        joints.append(q2)
        joints.append(q3)
        joints.append(q4)
        joints.append(q5)
        joints.append(q6)
        self.joints = joints

    def get_jacobian(self, q):
        J, Jv, Jw = eval_symbolic_jacobian_UR5(q)
        return J, Jv, Jw

    def directkinematics(self, q):
        T = self.serialrobot.directkinematics(q)
        return homogeneousmatrix.HomogeneousMatrix(T)
