#!/usr/bin/env python
# encoding: utf-8
"""
Please open the practicasl/dynamics/scene/onedofrobot.ttt scene before using this class.

The OneDOFRobot is a simple robot with a single joint

@Authors: Arturo Gil
@Time: April 2024
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import filter_path, path_trapezoidal_i
from artelib.seriallink import SerialRobot
from robots.robot import Robot
import matplotlib.pyplot as plt


class OneDOFRobot(Robot):
    def __init__(self, simulation):
        # init base class attributes
        Robot.__init__(self, simulation)
        self.DOF = 1
        self.q_current = np.zeros((1, self.DOF))
        self.qd_current = np.zeros((1, self.DOF))
        self.qdd_current = np.zeros((1, self.DOF))
        self.q = []
        self.qd = []
        self.t = []

        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([500])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges. joints 4 and 6 can be configured as unlimited
        # default joint limits:
        # q1 (+-180), q2 (-90,110), q3 (-230, 50), q4 (+-200), q5 (+-115), q6 (+-400)
        # here, the joint range for q4 has been extended
        joint_ranges = np.array([[-180],
                                 [180]])
        self.pid_controllers = np.array([[0.8, 0, 0.2],
                                [0.1, 0, 0.1],
                                [0.2, 0, 0.1],
                                [0.1, 0, 0.1],
                                [0.1, 0, 0.1],
                                [0.1, 0, 0.1]])
        self.joint_ranges = joint_ranges * np.pi / 180.0
        self.max_iterations_inverse_kinematics = 15000
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        self.ikmethod = 'closed-equations'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True
        # Sum of squared errors in joints to finish precision=True instructions
        # self.epsilonq = 0.0002
        self.epsilonq = 0.02

        # DH parameters of the robot
        self.serialrobot = SerialRobot(n=1, T0=np.eye(4), name='OneDOFRobot')
        self.serialrobot.append(th=0, d=0.352, a=0.07, alpha=-np.pi / 2, link_type='R')

        self.m = 1
        self.g = 9.81
        self.L = 1
        self.I = (1 / 3)*self.m*self.L**2

    def start(self, base_name='/ROBOTBASE', joint_name='joint'):
        armjoints = []
        # Get the handles of the relevant objects
        # robotbase = self.simulation.sim.getObject(base_name)
        q1 = self.simulation.sim.getObject(base_name + '/' + joint_name + '1')
        armjoints.append(q1)
        # must store the joints
        self.joints = armjoints

    def set_torques(self, tau):
        for i in range(len(tau)):
            self.simulation.sim.setJointTargetForce(self.joints[i], tau[i])

    def get_state(self):
        # q = []
        # qd = []
        # qdd = []
        # return joint position, speed and acceleration
        # for i in range(len(self.joints)):
        qi = self.simulation.sim.getJointPosition(self.joints[0])
        qdi = self.simulation.sim.getJointVelocity(self.joints[0])
            # qddi = self.simulation.sim.getJointAcceleration(self.joints[i])
            # q.append(qi)
            # qd.append(qdi)
            # qdd.append(qddi)
        # q = np.array(q)
        # qd = np.array(qd)
        # qdd = np.array(qdd)
        return qi, qdi #, qdd

    def save_state(self):
        # return joint position, speed and acceleration
        for i in range(len(self.joints)):
            qi = self.simulation.sim.getJointPosition(self.joints[i])
            qdi = self.simulation.sim.getJointVelocity(self.joints[i])
            # qddi = self.simulation.sim.getJointAcceleration(self.joints[i])
            self.q.append(qi)
            self.qd.append(qdi)
            # qdd.append(qddi)
        ti = self.simulation.sim.getSimulationTime()
        self.t.append(ti)

    def inversedynamics(self, q, qd, qdd):
        I = self.I
        m = self.m
        g = self.g
        L = self.L
        # q, qd, qdd = self.get_state()
        tau = I*qdd + m*g*(L/2)*np.cos(q)
        return tau