"""
Simple motor selection demo

"""
import numpy as np
from robots.simulation import Simulation
import matplotlib.pyplot as plt
from robots.twodofplanarrobot import TwoDOFRobot
import matplotlib
matplotlib.use('tkagg', force=True)
from helpers_torque_control import path_planning
from helpers_torque_control import plot_path

def motor_selection(robot, G):
    # errors in speed
    plt.figure()
    plt.plot(robot.t, np.array(robot.qd).T[0, :]*G[0]*60/(2*np.pi), label='qd1*G1 (rpm)', linewidth=4)
    plt.plot(robot.t, np.array(robot.qd).T[1, :]*G[1]*60/(2*np.pi), label='qd2*G2 (rpm)', linewidth=4)
    plt.xlabel('tiempo')
    plt.ylabel('rpm')
    plt.legend()
    plt.show(block=True)

    plt.figure()
    plt.title('Torques')
    plt.plot(robot.t, np.array(robot.tau).T[0, :]/G[0], label='Tau 1/G1')
    plt.plot(robot.t, np.array(robot.tau).T[1, :]/G[1], label='Tau 2/G2')
    plt.xlabel('tiempo')
    plt.ylabel('Tau (Nm)')
    plt.legend()
    plt.show(block=True)

def control_closed_loop(robot, q1, q2, total_time, algorithm):
    delta_t = robot.simulation.get_simulation_time_step()
    q1, qd1, qdd1, t = path_planning(q1, [0, 0], [0, 0], [0, total_time], delta_t)
    q2, qd2, qdd2, t = path_planning(q2, [0, 0], [0, 0], [0, total_time], delta_t)
    qref = np.vstack((q1, q2))
    qdref = np.vstack((qd1, qd2))
    qddref = np.vstack((qdd1, qdd2))
    plot_path(qref, qdref, qddref, t)
    # perform control
    for i in range(len(t)):
        control_step_closed_loop(robot, qref[:, i], qdref[:, i], qddref[:, i], algorithm=algorithm)
        robot.save_state()
    # robot.plot_results(qref, qdref, qddref, t)
    # robot.plot_errors(qref, qdref, qddref, t)
    return


def control_step_closed_loop(robot, qref, qdref, qddref, algorithm):
    # get current state
    q, qd = robot.get_state()
    if algorithm == 'PD':
        # precomputed torque with PD compensation (yes, qdd can be computed from the reference)
        # the precomputed torque is compensated via a vanilla PD
        # tau = robot.inversedynamics(q, qd, qddref)
        # Add a PD control action
        tau = 2.2 * (qdref - qd) + 10 * (qref - q)
    elif algorithm == 'PD_precomputed':
        # precomputed torque with PD compensation (yes, qdd can be computed from the reference)
        # the precomputed torque is compensated via a vanilla PD
        tau = robot.inversedynamics(q, qd, qddref)
        # Add a PD control action
        tau = tau + 0.3 * (qdref - qd) + 5.8 * (qref - q)
    elif algorithm == 'acc_compensation':
        # the compensation on qdd (adding a compensation based on the error)
        qdd = qddref + 3.5 * (qdref - qd) + 30.8 * (qref - q)
        tau = robot.inversedynamics(q, qd, qdd)
    # apply torques and continue
    robot.set_torques(tau)
    robot.wait(1)
    return


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = TwoDOFRobot(simulation=simulation)
    robot.I = [0.02146, 0.02146]
    robot.start()
    total_time = 2

    # Apply closed loop control PD-precomputed
    q1 = [0, np.pi / 2]
    q2 = [0, -np.pi / 2]
    control_closed_loop(robot=robot, q1=q1, q2=q2,
                        total_time=total_time, algorithm='acc_compensation')
    # the torques and speeds are stored in the robot variable
    G = [1, 1]
    motor_selection(robot, G)
    simulation.stop()