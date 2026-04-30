#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/4dofplanar.ttt scene before using this class.

Planar4DOF is a derived class of the Robot base class that particularizes some details of the robot

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from robots.robot import Robot
from kinematics.kinematics_planar4dof import eval_symbolic_jacobian_planar_4dof, eval_symbolic_T_planar4dof
import sim


class Planar4DOF(Robot):
    def __init__(self, clientID):
        Robot.__init__(self)
        self.clientID = clientID

        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-90, -90, -90, -90],
                                 [90,   90,  90,  90]])
        self.joint_ranges = joint_ranges * np.pi / 180.0
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        self.max_iterations_inverse_kinematics = 1500
        # self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        self.ikmethod = 'transpose'
        self.epsilonq = 0.001

    def start(self, base_name='/4dofplanar', joint_name='joint'):
        armjoints = []
        # Get the handles of the relevant objects
        errorCode, robotbase = sim.simxGetObjectHandle(self.clientID, base_name, sim.simx_opmode_oneshot_wait)
        errorCode, q1 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '1', sim.simx_opmode_oneshot_wait)
        errorCode, q2 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '2', sim.simx_opmode_oneshot_wait)
        errorCode, q3 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '3', sim.simx_opmode_oneshot_wait)
        errorCode, q4 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '4', sim.simx_opmode_oneshot_wait)

        armjoints.append(q1)
        armjoints.append(q2)
        armjoints.append(q3)
        armjoints.append(q4)
        self.joints = armjoints

    def get_jacobian(self, q):
        J, Jv, Jw = eval_symbolic_jacobian_planar_4dof(q)
        return J, Jv, Jw

    def direct_kinematics(self, q):
        T = eval_symbolic_T_planar4dof(q)
        return T

