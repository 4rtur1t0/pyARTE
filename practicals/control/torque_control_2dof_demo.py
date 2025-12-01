"""
Simple torque based control 1

1 Plan a path (q1 to q2)
         position,
         speed
         acceleration.
2 At each time step, compute the inverse dynamics.

3 Apply torque along with a PID correction based on the error in position, speed and acceleration.

"""
import numpy as np
from robots.onedofrobot import OneDOFRobot
from robots.simulation import Simulation
import matplotlib.pyplot as plt
from robots.twodofplanarrobot import TwoDOFRobot
import matplotlib
matplotlib.use('tkagg', force=True)
from helpers_torque_control import path_planning
from helpers_torque_control import plot_path


def control_open_loop(robot, q1, q2, total_time):
    delta_t = robot.simulation.get_simulation_time_step()
    q1, qd1, qdd1, t = path_planning(q1, [0, 0], [0, 0], [0, total_time], delta_t)
    q2, qd2, qdd2, t = path_planning(q2, [0, 0], [0, 0], [0, total_time], delta_t)
    qref = np.vstack((q1, q2))
    qdref = np.vstack((qd1, qd2))
    qddref = np.vstack((qdd1, qdd2))
    plot_path(qref, qdref, qddref, t)
    for i in range(len(t)):
        tau = robot.inversedynamics(qref[:, i], qdref[:, i], qddref[:, i])
        robot.save_state()
        robot.set_torques(tau)
        robot.wait(1)
    robot.plot_results(qref, qdref, qddref, t)
    return


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
        robot.save_state()
        control_step_closed_loop(robot, qref[:, i], qdref[:, i], qddref[:, i], algorithm=algorithm)
    robot.plot_results(qref, qdref, qddref, t)
    robot.plot_errors(qref, qdref, qddref, t)
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


def apply_torques(robot, tau, total_time):
    delta_t = robot.simulation.get_simulation_time_step()
    n_steps = int(total_time/delta_t)
    for i in range(n_steps):
        robot.save_state()
        robot.set_torques(tau)
        robot.wait(1)
    robot.save_state()
    robot.plot_states()
    robot.reset_state()


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = TwoDOFRobot(simulation=simulation)
    robot.I = [0.02146, 0.02146]
    robot.start()
    total_time = 5

    # (A) Apply zero torques
    # apply_torques(robot, tau=[0, 0], total_time=total_time)
    # Apply constant torques
    # apply_torques(robot, tau=[5, 2], total_time=total_time)

    # (B) Apply open loop control, zero reference
    # q1 = [0, 0]
    # q2 = [0, 0]
    # control_open_loop(robot=robot, q1=q1, q2=q2, total_time=total_time)

    # (C) Apply open loop control
    # q1 = [0, np.pi/2]
    # q2 = [0, np.pi/8]
    # control_open_loop(robot=robot, q1=q1, q2=q2, total_time=total_time)

    # (D) Apply closed loop control PD
    # q1 = [0, np.pi / 2]
    # q2 = [0, np.pi / 8]
    # control_closed_loop(robot=robot, q1=q1, q2=q2,
    #                     total_time=total_time, algorithm='PD')
    # (E) Apply closed loop control PD compensation
    # q1 = [0, np.pi/2]
    # q2 = [0, np.pi/8]
    # control_closed_loop(robot=robot, q1=q1, q2=q2,
    #                     total_time=total_time, algorithm='PD_precomputed')
    # Apply closed loop control PD-precomputed
    q1 = [0, np.pi / 2]
    q2 = [0, np.pi / 8]
    control_closed_loop(robot=robot, q1=q1, q2=q2,
                        total_time=total_time, algorithm='acc_compensation')
    simulation.stop()