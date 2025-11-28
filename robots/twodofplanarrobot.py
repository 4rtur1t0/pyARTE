#!/usr/bin/env python
# encoding: utf-8
"""
Please open the practicasl/control/twodofrobot.ttt scene before using this class.

The TwoDOFRobot is a simple two DOF planar robot with two rotational joints

@Authors: Arturo Gil
@Time: November 2025
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import filter_path, path_trapezoidal_i
from artelib.seriallink import SerialRobot
from robots.robot import Robot
import matplotlib.pyplot as plt


class TwoDOFRobot(Robot):
    def __init__(self, simulation):
        # init base class attributes
        Robot.__init__(self, simulation)
        self.DOF = 2
        self.q_current = np.zeros((1, self.DOF))
        self.qd_current = np.zeros((1, self.DOF))
        self.qdd_current = np.zeros((1, self.DOF))
        self.q = []
        self.qd = []
        self.tau = []
        self.t = []

        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([500, 500])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges. joints 4 and 6 can be configured as unlimited
        # default joint limits:
        # q1 (+-180), q2 (-90,110), q3 (-230, 50), q4 (+-200), q5 (+-115), q6 (+-400)
        # here, the joint range for q4 has been extended
        joint_ranges = np.array([[-180, -180],
                                 [180, 180]])
        #self.pid_controllers = np.array([[0.8, 0, 0.2],
        #                        [0.1, 0, 0.1],
        #                        [0.2, 0, 0.1],
        #                        [0.1, 0, 0.1],
        #                        [0.1, 0, 0.1],
        #                        [0.1, 0, 0.1]])

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
        self.serialrobot = SerialRobot(n=2, T0=np.eye(4), name='TwoDOFRobot')
        self.serialrobot.append(th=0, d=0.0, a=0.5, alpha=0, link_type='R')
        self.serialrobot.append(th=0, d=0.0, a=0.5, alpha=0, link_type='R')

        # dynamic parameters and simple notation for lengths
        self.m1 = 1.0
        self.m2 = 1.0
        self.g = 9.81
        # lengths
        self.L1 = 0.5
        self.L2 = 0.5
        #self.a = np.array([self.L[0]/2, self.L[1]/2])
        self.I = [(1 / 12)*self.m1*self.L1**2, (1 / 12)*self.m2*self.L2**2]

    def start(self, base_name='/ROBOTBASE', joint_name='joint'):
        armjoints = []
        # Get the handles of the relevant objects
        # robotbase = self.simulation.sim.getObject(base_name)
        q1 = self.simulation.sim.getObject(base_name + '/' + joint_name + '1')
        q2 = self.simulation.sim.getObject(base_name + '/' + joint_name + '2')

        armjoints.append(q1)
        armjoints.append(q2)

        # must store the joints
        self.joints = armjoints

    def set_torques(self, tau):
        for i in range(len(tau)):
            self.simulation.sim.setJointTargetForce(self.joints[i], tau[i])

    def get_state(self):
        q = []
        qd = []
        # qdd = []
        # return joint position, speed and acceleration
        for i in range(len(self.joints)):
            qi = self.simulation.sim.getJointPosition(self.joints[i])
            qdi = self.simulation.sim.getJointVelocity(self.joints[i])
            q.append(qi)
            qd.append(qdi)
        return q, qd #, qdd

    def save_state(self):
        # return joint position, speed and acceleration
        q = []
        qd = []
        tau = []
        for i in range(len(self.joints)):
            qi = self.simulation.sim.getJointPosition(self.joints[i])
            qdi = self.simulation.sim.getJointVelocity(self.joints[i])
            # qddi = self.simulation.sim.getJointAcceleration(self.joints[i])
            taui = self.simulation.sim.getJointForce(self.joints[i])
            q.append(qi)
            qd.append(qdi)
            tau.append(taui)
        self.q.append(q)
        self.qd.append(qd)
        self.tau.append(tau)
        # qdd.append(qddi)
        ti = self.simulation.sim.getSimulationTime()
        self.t.append(ti)

    def inversedynamics(self, q, qd, qdd):
        g = self.g
        I1 = self.I[0]
        I2 = self.I[1]
        m1 = self.m1
        m2 = self.m2
        l1 = self.L1
        l2 = self.L2
        a1 = l1/2
        a2 = l2/2
        # the state in shorthand
        q1 = q[0]
        q2 = q[1]
        qd1 = qd[0]
        qd2 = qd[1]
        qdd1 = qdd[0]
        qdd2 = qdd[1]
        cosq1 = np.cos(q1)
        cosq2 = np.cos(q2)
        sinq1 = np.sin(q1)
        sinq2 = np.sin(q2)
        cosq1q2 = np.cos(q1+q2)

        # tau = Mqdd+C()+G(q)
        M11 = I1 + I2 + m1 * a1 ** 2 + m2 * (l1 ** 2 + a2 ** 2 + 2 * l1 * a2 * cosq2)
        M12 = I2 + m2*(a2**2 + l1*a2*cosq2)
        M21 = M12
        M22 = I2 + m2*a2**2
        M = np.array([[M11, M12],
                    [M21, M22]])

        # Coriollis
        b = m2*l1*a2
        C1 = -b*sinq2*(2*qd1*qd2+qd2**2)
        C2 = b*sinq2*(qd1**2)
        C = np.array([[C1], [C2]])

        # Gravity
        G1 = (m1*a1 + m2*l1)*g*cosq1 + m2*a2*g*cosq1q2
        G2 = m2*a2*g*cosq1q2
        G = np.array([[G1], [G2]])
        qdd = np.array([qdd1, qdd2])
        tau = np.matmul(M, qdd.T) + C.T + G.T
        return tau[0]