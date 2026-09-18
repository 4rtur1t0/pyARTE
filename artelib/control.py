#!/usr/bin/env python
# encoding: utf-8
"""
The control class

@Authors: Arturo Gil
@Time: Febrer de 2026
"""
import numpy as np
import matplotlib.pyplot as plt


class Controller():
    """
        Includes method to follow the computed path
    """
    def __init__(self, robot, control_parameters):
        self.robot = robot
        # control parameters include kp, kd and ki
        self.control_parameters = control_parameters
        # save values
        self.qref = []
        self.qdref = []
        self.qddref = []
        self.q = []
        self.qd = []
        self.tau = []
        self.t = []
        # The integral of the error
        self.qerror_int = np.zeros(robot.DOF)
        # self.last_qref = np.zeros(robot.DOF)
        return

    def control(self, qref, qdref, qddref, plot=False):
        # reset the integral of the error
        self.qerror_int = np.zeros(self.robot.DOF)
        for i in range(qref.shape[1]):
            # Execute each control step
            self.control_step(qref[:, i], qdref[:, i], qddref[:, i]) #, t[i])
            # save robot state and control state
            self.robot.save_state()
        if plot:
            # self.plot_control_results()
            # self.plot_control_results_errors()
            self.plot_references()
        # reset the control state of this particular movement
        self.reset_values()
        # self.last_qref = qref[:, -1]
        return

    def control_step(self, qrefi, qdrefi, qddrefi):
        # get current state
        q, qd = self.robot.get_state()
        if self.control_parameters.control_type == 'open_loop':
            # Usually incorrect, compute tau for the reference only
            tau = self.robot.inversedynamics(qrefi, qdrefi, qddrefi)
        elif self.control_parameters.control_type == 'PD_precomputed':
            # precomputed torque with PD compensation (yes, qdd can be computed from reference)
            tau = self.robot.inversedynamics(q, qd, qddrefi)
            # Add a PD control action
            kp = self.control_parameters.kp
            kd = self.control_parameters.kd
            ki = self.control_parameters.ki
            self.qerror_int = self.qerror_int + qrefi - q
            tau = tau + np.matmul(kp, qrefi - q) + np.matmul(kd, qdrefi - qd) + \
                  np.matmul(ki, self.qerror_int)
        elif self.control_parameters.control_type == 'acc_compensation':
            kp = self.control_parameters.kp
            kd = self.control_parameters.kd
            ki = self.control_parameters.ki
            self.qerror_int = self.qerror_int + qrefi - q
            # the compensation on qdd (adding a compensation based on the error)
            qdd = qddrefi + np.matmul(kp, qrefi - q) + \
                  np.matmul(kd, qdrefi - qd) + \
                  np.matmul(ki, self.qerror_int)
            tau = self.robot.inverse_dynamics(q, qd, qdd)
        # print('Computed corrections: ', u_corr)
        # print('Error de posicion (rad): ', np.linalg.norm(qrefi - q))
        # print('Error de velocidad: (rad/s)', np.linalg.norm(qdrefi - qd))
        # print('Tau: ', tau)
        # print('Time: ', ti)
        # print('Errores de posicion: ', np.abs(qrefi - q), np.linalg.norm(qrefi - q))
        # print('Errores de velocidad: ', np.abs(qdrefi - qd), np.linalg.norm(qdrefi - qd))
        # apply the computed torques
        self.robot.apply_torques(tau)
        self.save_values(qrefi, qdrefi, qddrefi, q, qd, tau)
        # wait a simulation time step
        self.robot.simulation.wait(1)
        return

    def maintain_last_joint_position(self):
        # q, qd = self.robot.get_state()
        self.control_step(self.robot.last_qref, np.zeros(self.robot.DOF), np.zeros(self.robot.DOF), 0)

    def save_values(self, qrefi, qdrefi, qddrefi, qi, qdi, taui):
        self.qref.append(qrefi)
        self.qdref.append(qdrefi)
        self.qddref.append(qddrefi)
        self.q.append(qi)
        self.qd.append(qdi)
        self.tau.append(taui)
        ti = self.robot.simulation.sim.getSimulationTime()
        self.t.append(ti)

    def reset_values(self):
        self.qref = []
        self.qdref = []
        self.qddref = []
        self.q = []
        self.qd = []
        self.tau = []
        self.t = []
        # reset integral error
        self.qerror_int = np.zeros(self.robot.DOF)

    def plot_control_results(self):
        q = np.array(self.q).T
        qd = np.array(self.qd).T
        qref = np.array(self.qref).T
        qdref = np.array(self.qdref).T
        qddref = np.array(self.qddref).T
        tau = np.array(self.tau).T
        # joint position and references
        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, q[i, :], label='q' + str(i + 1), linewidth=4)
            plt.plot(self.t, qref[i, :], label='q' + str(i + 1) + ' reference', linewidth=4)
        plt.title('JOINTS AND REFERENCES')
        plt.legend()
        plt.show()

        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, qd[i, :], label='qd' + str(i + 1), linewidth=4)
            plt.plot(self.t, qdref[i, :], label='qd' + str(i + 1) + ' reference', linewidth=4)
        plt.legend()
        plt.title('JOINTS SPEEDS AND JOINT SPEED REFERENCES')
        plt.show()

        plt.figure()
        for i in range(self.robot.DOF):
            # not saving the actual acceleration
            # plt.plot(self.t, self.qdd[i, :], label='qdd' + str(i), linewidth=4)
            plt.plot(self.t, qddref[i, :], label='qdd' + str(i + 1) + ' reference', linewidth=4)
        plt.legend()
        plt.title('JOINT ACCELERATION REFERENCES')
        plt.show()

        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, tau[i, :], label='tau' + str(i + 1), linewidth=4)
        plt.legend()
        plt.title('APPLIED JOINT TORQUES (Nm)')
        plt.show()

    def plot_control_results_errors(self):
        q = np.array(self.q).T
        qd = np.array(self.qd).T
        qref = np.array(self.qref).T
        qdref = np.array(self.qdref).T

        # compute position error
        # joint position and references
        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, qref[i, :]- q[i, :], label='error q' + str(i + 1), linewidth=4)
            # plt.plot(self.t, qref[i, :], label='q' + str(i + 1) + ' reference', linewidth=4)
        plt.title('JOINT ERRORS')
        plt.legend()
        plt.show()

        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, qdref[i, :]-qd[i, :], label='error qd' + str(i + 1), linewidth=4)
            # plt.plot(self.t, qdref[i, :], label='qd' + str(i + 1) + ' reference', linewidth=4)
        plt.legend()
        plt.title('JOINTS SPEEDS ERRORS')
        plt.show()

    def plot_references(self):
        qref = np.array(self.qref).T
        qdref = np.array(self.qdref).T
        qddref = np.array(self.qddref).T

        # joint position and references
        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, qref[i, :], label='q' + str(i + 1) + ' reference', linewidth=4)
        plt.title('REFERENCES ON JOINT POSITIONS')
        plt.legend()
        plt.show()

        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, qdref[i, :], label='qd' + str(i + 1) + ' reference', linewidth=4)
        plt.legend()
        plt.title('REFERENCES ON JOINT SPEEDS')
        plt.show()

        plt.figure()
        for i in range(self.robot.DOF):
            plt.plot(self.t, qddref[i, :], label='qdd' + str(i + 1) + ' reference', linewidth=4)
        plt.legend()
        plt.title('REFERENCES ON JOINT ACCELERATIONS')
        plt.show()




