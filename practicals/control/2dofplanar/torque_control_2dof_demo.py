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
# from robots.onedofrobot import OneDOFRobot
from robots.simulation import Simulation
# import matplotlib.pyplot as plt
from robots.twodofplanarrobot import TwoDOFRobot
import matplotlib


def apply_torques(robot, tau, total_time):
    delta_t = robot.simulation.get_simulation_time_step()
    n_steps = int(total_time/delta_t)
    for i in range(n_steps):
        robot.save_state()
        robot.apply_torques(tau)
        robot.simulation.wait(1)
    robot.save_state()
    robot.plot_states()
    robot.reset_state()


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = TwoDOFRobot(simulation=simulation)
    robot.start()
    total_time = 5
    #################################################################
    # PART I: apply constant torques
    # (A) Apply zero torques
    # apply_torques(robot=robot, tau=[0, 0], total_time=total_time)
    # (B) Apply constant torques
    # apply_torques(robot=robot, tau=[5, 2], total_time=total_time)
    #################################################################

    #################################################################
    # PART II: control
    # Now experiment with different types of closed-loop control
    # (C) Apply open loop control, zero reference
    # (D) Apply closed loop control PD precomputed
    # (E) Apply closed loop control acc_compensation
    # Modify TwoDofRobot control parameters to open loop
    #################################################################
    # try to move robot with moveAbsJ (trapezoidal)
    q1 = [np.pi/4, np.pi/4]
    q2 = [0.0, 0.0]
    robot.moveQuintic(q_target=q1, total_time=total_time, precision=True, plot=True)
    robot.moveQuintic(q_target=q2, total_time=total_time, precision=True, plot=True)
    simulation.stop()