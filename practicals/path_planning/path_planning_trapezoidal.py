#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
import matplotlib.pyplot as plt
from artelib.parameters import JointParameters
from artelib.trajectory_planner import TrajectoryPlanner


def plot_trajectory(q, qd, qdd, t):
    plt.figure()
    for i in range(len(q)):
        plt.plot(t, q[i, :], label='q'+str(i+1), linewidth=4)
    plt.legend()
    plt.title('Joint trajectories')
    plt.show()

    plt.figure()
    for i in range(len(q)):
        plt.plot(t, qd[i, :], label='qd' + str(i + 1), linewidth=4)
    plt.legend()
    plt.title('Joint speeds')
    plt.show()

    plt.figure()
    for i in range(len(q)):
        plt.plot(t, qdd[i, :], label='qdd' + str(i + 1), linewidth=4)
    plt.legend()
    plt.title('Joint accelerations')
    plt.show()


if __name__ == "__main__":
    joint_parameters = JointParameters(joint_ranges=np.pi * np.array([[-180.0],
                                                                      [180.0]])/180.0,
                                       max_joint_speeds=np.pi * np.array([200.0])/180.0,
                                       max_joint_accelerations=np.pi * np.array([2000.0]) / 180.0,
                                       settle_time=0.1,
                                       max_linear_velocity=2.0,  # m/s
                                       max_linear_acceleration=10.0,  # m/s/s
                                       max_angular_velocity=3.0,  # rad/s
                                       max_angular_acceleration=20.0,  # rad/s/s
                                       DOF=1)
    delta_time = 0.01
    planner = TrajectoryPlanner(joint_parameters=joint_parameters, delta_time=delta_time,
                                settle_time=joint_parameters.settle_time)

    q0 = [-5.0]
    qf = [-10.0]
    qt, qdt, qddt, time = planner.trapezoidal_coordinated(q0, qf, joint_parameters.max_joint_speeds,
                                                          joint_parameters.max_joint_accelerations)
    plot_trajectory(qt, qdt, qddt, time)


