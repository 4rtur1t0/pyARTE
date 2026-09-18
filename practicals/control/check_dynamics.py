"""
Simple statics to check the dynamics model of the robot.
"""
import numpy as np
from helpers.print_errors import ErrorPrint
from robots.simulation import Simulation
import matplotlib.pyplot as plt
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.onedofrobot import OneDOFRobot
from robots.sphericalrobotRRR import SphericalRobotRRR
from robots.twodofplanarrobot import TwoDOFRobot


def plot_torques(robot, t, tau):
    tau = np.array(tau).T
    plt.figure()
    plt.title('Computed torques')
    for i in range(robot.DOF):
        plt.plot(t, tau[i, :], label='tau' + str(i + 1), linewidth=4)
        plt.legend()
        plt.show()


def check_dynamics(robot, total_time):
    """
    Check the dynamic part of the model. Allow the robot to move freely (zero torques), while monitoring
    position q, speed qd and computing acceleration at each time step.
    """
    delta_t = robot.simulation.get_simulation_time_step()
    n_steps = int(total_time / delta_t)
    tau = np.zeros(robot.DOF)
    # apply zero torques and save q, qd and compute qdd at each timestep
    # apply constant torques and observe the movement
    q = []
    qd = []
    # apply torques for the first time
    robot.apply_torques(tau)
    robot.simulation.wait()
    # then save the state
    for i in range(n_steps):
        robot.save_state()
        # also save state independently
        qi = robot.get_joint_positions()
        qdi = robot.get_joint_speeds()
        q.append(qi)
        qd.append(qdi)
        robot.simulation.wait()
    robot.plot_states()

    tau = []
    for i in range(n_steps-1):
        qi = q[i]
        qdi = qd[i]
        qdi1 = qd[i+1]
        qddi = (qdi1-qdi)/delta_t
        # tau should be zero at each time step!
        taui = robot.inverse_dynamics(qi, qdi, qddi)
        tau.append(taui)
    t = robot.get_state_time()
    plot_torques(robot, t[0:-1], tau)
    # reset state
    robot.reset_state()


def check_statics(robot, total_time):
    """
    Checks the static part of the dynamic model.
    a) Load the robot (onedof robot, twodof robot)
    b) Place the robot at an initial starting position with zero speeds
    c) Execute the function.
    """
    # check statics
    q = robot.get_joint_positions()
    qd = robot.get_joint_speeds()
    qdd = np.zeros(robot.DOF)
    if np.linalg.norm(qd) > 0.01:
        ErrorPrint.print('Error: testing statics with non-zero joint speeds', color='red')
        print('Please set the Coppelia Model to zero speed')
        return
    tau = robot.inverse_dynamics(q, qd, qdd)
    print(tau)
    delta_t = robot.simulation.get_simulation_time_step()
    n_steps = int(total_time/delta_t)
    # apply constant torques and observe the movement
    for i in range(n_steps):
        robot.apply_torques(tau)
        robot.save_state()
        # caution! do not use robot.wait() since it
        # automatically applies a controlled action
        robot.simulation.wait()
    robot.plot_states()
    # reset state
    robot.reset_state()


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Select different robots to check the dynamics
    #################################################
    # q = [np.pi / 4]
    # robot = OneDOFRobot(simulation=simulation)
    #################################################
    # q = [np.pi / 4, np.pi/4]
    # robot = TwoDOFRobot(simulation=simulation)
    #################################################
    # q = [np.pi / 4, np.pi/4, np.pi/4]
    # robot = SphericalRobotRRR(simulation=simulation)
    #################################################
    q = [np.pi/8, np.pi/8, np.pi/8, np.pi/8, np.pi/8, np.pi/8]
    robot = RobotABBIRB140(simulation=simulation)
    #################################################
    robot.start()
    # check statics and dynamics for a total time of secs
    # check for about 1-2 secs
    total_time = 1.5

    ##################################################
    # place the robot at zero speed and try to make it
    # stay still by applying the static torques.
    check_statics(robot=robot, total_time=total_time)
    ##################################################

    ##################################################
    # Now: move the robot to different place first.
    robot.moveAbsJ(q_target=q, plot=True, precision=True)
    # robot.moveQuintic(q_target=q, total_time=total_time, precision=True, plot=True)
    # caution, reset stat, so that we only test
    robot.reset_state()
    # and now, check the general dynamic equation for every time step.
    # tau = M*qdd + C(q, qd) + G(q)
    check_dynamics(robot=robot, total_time=total_time)
    ##################################################
    simulation.stop()