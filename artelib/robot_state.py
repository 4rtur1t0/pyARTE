#!/usr/bin/env python
# encoding: utf-8
"""
Classes to store the robot state during simulation

@Authors: Arturo Gil
@Time: Febrer de 2026
"""
import numpy as np
import matplotlib.pyplot as plt


class RobotState():
    """
        Store the joint parameters. Joint ranges, max_joint_speeds, max_joint_accelerations
    """
    def __init__(self, robot):
        self.DOF = robot.DOF
        self.robot = robot
        # self.simulation = simulation
        # store a set of joint handlers
        self.joint_handlers = None
        self.q_current = np.zeros((1, self.DOF))
        self.qd_current = np.zeros((1, self.DOF))
        self.qdd_current = np.zeros((1, self.DOF))
        self.q = []
        self.qd = []
        self.tau = []
        self.t = []

    def get_state(self):
        q = []
        qd = []
        # qdd = []
        # return joint position, speed and acceleration
        for i in range(len(self.joint_handlers)):
            qi = self.robot.simulation.sim.getJointPosition(self.joint_handlers[i])
            qdi = self.robot.simulation.sim.getJointVelocity(self.joint_handlers[i])
            q.append(qi)
            qd.append(qdi)
        return q, qd #, qdd

    def save_state(self):
        # return joint position, speed and acceleration
        q = []
        qd = []
        tau = []
        # t = []
        for i in range(len(self.joint_handlers)):
            qi = self.robot.simulation.sim.getJointPosition(self.joint_handlers[i])
            qdi = self.robot.simulation.sim.getJointVelocity(self.joint_handlers[i])
            # qddi = self.simulation.sim.getJointAcceleration(self.joints[i])
            taui = self.robot.simulation.sim.getJointForce(self.joint_handlers[i])
            q.append(qi)
            qd.append(qdi)
            tau.append(taui)
        self.q.append(q)
        self.qd.append(qd)
        self.tau.append(tau)
        ti = self.robot.simulation.sim.getSimulationTime()
        self.t.append(ti)

    def get_state_time(self):
        return self.t

    def reset_state(self):
        self.q = []
        self.qd = []
        self.tau = []
        self.t = []

    def plot_states(self):
        q = np.array(self.q).T
        qd = np.array(self.qd).T
        # qref = np.array(self.qref).T
        # qdref = np.array(self.qdref).T
        # qddref = np.array(self.qddref).T
        tau = np.array(self.tau).T
        # joint position and references
        plt.figure()
        plt.title('JOINTS AND REFERENCES (rad)')
        for i in range(self.robot.DOF):
            plt.plot(self.t, q[i, :], label='q' + str(i + 1), linewidth=4)
        plt.legend()
        plt.show()

        plt.figure()
        plt.title('JOINTS SPEEDS AND JOINT SPEED REFERENCES (rad/s)')
        for i in range(self.robot.DOF):
            plt.plot(self.t, qd[i, :], label='qd' + str(i + 1), linewidth=4)
        plt.legend()
        plt.show()

        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, tau[i, :], label='tau' + str(i + 1), linewidth=4)
        plt.legend()
        plt.title('APPLIED JOINT TORQUES (Nm)')
        plt.show()
