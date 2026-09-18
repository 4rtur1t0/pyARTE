"""
Simple motor selection demo

"""
import numpy as np
from robots.simulation import Simulation
import matplotlib.pyplot as plt
from robots.twodofplanarrobot import TwoDOFRobot
import matplotlib
matplotlib.use('tkagg', force=True)


def motor_selection(robot, G):
    t = robot.robot_state.t
    qd = robot.robot_state.qd
    tau = robot.robot_state.tau
    # Plot speeds considering G in rpm
    plt.figure()
    plt.plot(t, np.array(qd).T[0, :]*G[0]*60/(2*np.pi), label='qd1*G1 (rpm)', linewidth=4)
    plt.plot(t, np.array(qd).T[1, :]*G[1]*60/(2*np.pi), label='qd2*G2 (rpm)', linewidth=4)
    plt.xlabel('tiempo')
    plt.ylabel('rpm')
    plt.legend()
    plt.show(block=True)

    # plot torques
    plt.figure()
    plt.title('Torques')
    plt.plot(t, np.array(tau).T[0, :]/G[0], label='Tau 1/G1')
    plt.plot(t, np.array(tau).T[1, :]/G[1], label='Tau 2/G2')
    plt.xlabel('tiempo')
    plt.ylabel('Tau/G (Nm)')
    plt.legend()
    plt.show(block=True)


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = TwoDOFRobot(simulation=simulation)
    # robot.I = [0.02146, 0.02146]
    robot.start()
    total_time = 1.0

    # Perform a movement and then plot speeds an torque considering different G
    G = [100, 100]
    q1 = [np.pi / 4, np.pi / 4]
    robot.moveQuintic(q_target=q1, total_time=total_time, precision=True)
    motor_selection(robot, G)
    simulation.stop()